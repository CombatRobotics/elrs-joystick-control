// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package telemetry

import (
	"context"
	"errors"
	"fmt"
	"io"

	"github.com/kaack/elrs-joystick-control/pkg/crc"
	"github.com/kaack/elrs-joystick-control/pkg/crossfire"
	"go.bug.st/serial"
)

// indexOfFrameStart returns the offset of the first byte that looks like a
// CRSF frame start (0xEA / 0xEE / 0xC8), or -1 if none. Tiny inline replacement
// for slices.IndexFunc to avoid pulling in golang.org/x/exp.
func indexOfFrameStart(data []byte) int {
	for i, b := range data {
		if isTelemetryAddress(crossfire.Endpoint(b)) {
			return i
		}
	}
	return -1
}

// Reader pulls bytes from a serial port and emits decoded CRSF frames.
//
// Three architectural choices that matter for half-duplex CRSF on FTDI:
//
//  1. Parse-buffered-first. Each iteration of Next tries Split on already-
//     buffered data BEFORE blocking on Read. The previous structure always
//     read first, which let the kernel TTY buffer fill and drop bytes
//     whenever there were already complete frames sitting in memory.
//
//  2. 0xC8 frame-start. isTelemetryAddress recognises 0xC8 in addition to
//     0xEA/0xEE. Without this, host admin echoes (ping, model-id, parameter
//     write) carry an inner 0xEE that the parser would mis-latch onto,
//     producing 241-byte phantom frames that always CRC-fail.
//
//  3. Echoes silently skipped. Frames whose first byte is 0xEE or 0xC8 are
//     host-self echoes; Unmarshal returns nil for them. The recv loop never
//     logs echoes — only frames addressed to the host (0xEA).
type Reader struct {
	Buffer []uint8
	Port   serial.Port

	start           int
	end             int
	currentCapacity int
	initialCapacity int
	err             error
	eof             bool

	// Diagnostic counters. Zero-valued at init; printed by recv-loop heartbeat.
	BytesRead     uint64
	ReadCalls     uint64
	ZeroReads     uint64
	FramesGood    uint64
	FramesEcho    uint64 // 0xEE / 0xC8 host-self frames (silently skipped)
	FramesUnknown uint64 // valid CRC but unrecognised addr/type
	FramesCrcBad  uint64
}

func NewReader(port serial.Port) *Reader {
	capacity := 1024
	return &Reader{
		Buffer:          make([]uint8, capacity),
		Port:            port,
		initialCapacity: capacity,
		currentCapacity: capacity,
	}
}

// Next returns the next decoded telemetry frame, or an error.
//
// Cancellation: pass a ctx that gets cancelled on shutdown. Next checks
// ctx.Err() between Read calls and returns *InterruptedError when the context
// is done. Callers should treat that as a clean exit, not a hard error.
//
// The serial port should have a non-zero read timeout (e.g. 20ms) so Read
// returns periodically when the line is silent and Next can notice ctx
// cancellation.
func (s *Reader) Next(ctx context.Context) (TelemType, error) {
	var err error
	var skip int
	var count int
	var frame *[]uint8
	var newCap int
	var newBuf []uint8
	var tmp TelemType

	for {

		if s.start >= s.end {
			s.start = 0
			s.end = 0
			if s.currentCapacity > s.initialCapacity {
				s.Buffer = make([]uint8, s.initialCapacity)
				s.currentCapacity = s.initialCapacity
			}
		}

		// Try to extract a frame from already-buffered data BEFORE blocking
		// on another Read. Crucial for keeping up with the kernel TTY buffer
		// when bytes are arriving in bursts.
		if s.start < s.end {
			skip, frame, err = Split(s.Buffer[s.start:s.end], s.eof)
			s.start += skip
			if frame != nil {
				goto handleFrame
			}
			if err != nil {
				s.FramesCrcBad++
				return nil, err
			}
		}

		count = 0
		for count == 0 {
			if ctx.Err() != nil {
				return nil, &InterruptedError{}
			}

			if count, err = s.Port.Read(s.Buffer[s.end:]); err != nil {
				s.err = err
				s.eof = true
				break
			}
			s.ReadCalls++
			if count == 0 {
				s.ZeroReads++
			}
		}
		s.BytesRead += uint64(count)

		if s.start >= s.end && err != nil {
			return nil, err
		}

		s.end += count
		if s.end == s.currentCapacity {
			newCap = s.currentCapacity * 2
			newBuf = make([]uint8, newCap)
			copy(newBuf, s.Buffer)
			s.Buffer = newBuf
			s.currentCapacity = newCap
		}

		skip, frame, err = Split(s.Buffer[s.start:s.end], s.eof)
		s.start += skip

	handleFrame:

		if frame != nil {
			s.FramesGood++
			if tmp, err = Unmarshal(*frame); err != nil {
				return nil, err
			} else if tmp == nil {
				// Either an echoed host-write frame or an unrecognised
				// addr/type. Distinguish for diagnostics; silently retry
				// either way.
				if len(*frame) > 0 && ((*frame)[0] == 0xEE || (*frame)[0] == 0xC8) {
					s.FramesEcho++
				} else {
					s.FramesUnknown++
				}
				continue
			}

			return tmp, nil
		}

		if err != nil {
			s.FramesCrcBad++
			return nil, err
		}
	}

}

// Split locates and validates a single CRSF frame at the start of `data`.
// Returns:
//
//	advance: how many bytes to skip past in the input
//	token:   the extracted frame, or nil if not enough data / no sync byte found
//	err:     non-nil only on CRC mismatch (frame found but corrupt)
//
// On CRC mismatch, advance = frameStart+1 so the caller resyncs byte-by-byte.
func Split(data []byte, atEOF bool) (advance int, token *[]byte, err error) {
	dataLen := int32(len(data))

	if atEOF && len(data) == 0 {
		return 0, nil, io.EOF
	}

	frameStart := int32(indexOfFrameStart(data))

	if frameStart < 0 {
		return len(data), nil, nil
	}

	if frameStart+1 == dataLen {
		if atEOF {
			return 0, nil, fmt.Errorf("incomplete frame, stopped at frame id %x", data[frameStart])
		}
		return 0, nil, nil
	}

	frameLength := int32(data[frameStart+1])
	remainingBytes := (dataLen - 1) - (frameStart + 1)

	if remainingBytes < frameLength {
		if atEOF {
			return 0, nil, fmt.Errorf("incomplete frame, need %d bytes but only %d remain", frameLength, remainingBytes)
		}
		return 0, nil, nil
	}

	if frameLength == 0 {
		return int(frameStart) + 1, nil, nil
	}

	frame := data[frameStart : frameStart+1+frameLength+1]
	frameCrc8 := frame[len(frame)-1]
	computedCrc8 := crc.D5(frame[2 : len(frame)-1])
	if computedCrc8 != frameCrc8 {
		// Only log CRC mismatches on 0xEA-start frames — those represent
		// real firmware-originated telemetry that got corrupted on the wire
		// and is genuinely useful to surface.
		//
		// 0xEE and 0xC8 starts are either echoed host frames or random
		// noise that happened to contain that byte — common when the TX
		// module is unpowered (line floats) or during line contention.
		// Logging those drowns out the real signal.
		//
		// The byte-by-byte resync (skip = frameStart+1) still happens, and
		// FramesCrcBad still counts these so they show up in the heartbeat.
		if frame[0] == 0xEA {
			head := frame
			if len(head) > 6 {
				head = head[:6]
			}
			fmt.Printf("(reader) CRC mismatch first=%x len=%d head=%x got=%x want=%x\n",
				frame[0], len(frame), head, frameCrc8, computedCrc8)
		}
		return int(frameStart) + 1, nil, errors.New("frame crc mismatch")
	}

	skip := frameStart + int32(len(frame))
	return int(skip), &frame, nil
}

// Unmarshal converts a CRC-validated raw frame into a typed TelemType.
//
// Returns nil, nil for:
//   - host-echo frames (first byte 0xEE or 0xC8)
//   - frames addressed to the host that we don't decode (DeviceInfo,
//     DeviceSettingsEntry, Battery, Attitude, GPS, etc.)
//
// Returns a non-nil TelemType for:
//   - SyncExtFrame   (RADIO_ID frame with OPENTX_SYNC ext-type)
//   - LinkStatsFrame (LINK_STATISTICS)
//   - StatusExtFrame (ELRS_STATUS)
func Unmarshal(data []byte) (TelemType, error) {
	if len(data) < 3 {
		return nil, errors.New("cannot unmarshal as telemetry frame: length too small")
	}

	fAddr := crossfire.Endpoint(data[0])
	fType := crossfire.FrameType(data[2])

	// Frames originating from the host (echoed back over half-duplex). Skip
	// silently — they are expected, not noise.
	if fAddr == crossfire.ModuleEndpoint || fAddr == crossfire.Endpoint(crossfire.UartSyncFrame) {
		return nil, nil
	}

	// Only frames addressed to the host (0xEA) are interesting.
	if fAddr != crossfire.HandsetEndpoint {
		return nil, nil
	}

	switch fType {
	case crossfire.LinkStatsFrame:
		if len(data) < LinkStatsFrameSize {
			return nil, fmt.Errorf("link-stats frame too small: got %d, want %d", len(data), LinkStatsFrameSize)
		}
		return &LinkStatsFrame{RawData: data}, nil

	case crossfire.RadioFrame:
		// 0x3A. Extended frame; the OPENTX_SYNC sub-type lives at offset 5.
		if len(data) < 14 {
			return nil, fmt.Errorf("radio frame too small: got %d", len(data))
		}
		extType := crossfire.FrameType(data[5])
		if extType == crossfire.OpenTxSyncFrame {
			return &SyncExtFrame{RawData: data}, nil
		}
		// Any other RADIO_ID sub-type — not relevant to us.
		return nil, nil

	case crossfire.StatusFrame:
		// 0x2E ELRS status reply.
		if len(data) < MinStatusExtFrameSize {
			return nil, fmt.Errorf("status frame too small: got %d, want >=%d", len(data), MinStatusExtFrameSize)
		}
		return &StatusExtFrame{RawData: data}, nil
	}

	// Recognised CRSF frame addressed to the host but a type we deliberately
	// don't decode (DeviceInfo 0x29, ParameterSettingsEntry 0x2B, Battery
	// 0x08, Attitude 0x1E, etc.). Counted as FramesUnknown by caller.
	return nil, nil
}
