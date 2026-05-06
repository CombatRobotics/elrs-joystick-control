// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package crossfire

import (
	"github.com/kaack/elrs-joystick-control/pkg/crc"
	"github.com/kaack/elrs-joystick-control/pkg/util"
	"time"
)

func CreateModelIDFrame(modelId uint8) []uint8 {
	frame := []uint8{
		/* 0: */ uint8(UartSyncFrame),
		/* 1: */ 8,
		/* 2: */ uint8(CommandFrame),
		/* 3: */ uint8(ModuleEndpoint),
		/* 4: */ uint8(HandsetEndpoint),
		/* 5: */ uint8(SubcommandFrame),
		/* 6: */ uint8(CmdModelSelectFrame),
		/* 7: */ modelId, //model id
		/* 8: */ 0, //crc BA
		/* 9: */ 0, //crc D5
	}

	frame[8] = crc.BA(frame[2:8])
	frame[9] = crc.D5(frame[2:9])
	return frame
}

// CreatePR100FFrame returns exact raw ELRS command bytes for Packet Rate = 100Hz Full.
func CreatePR100FFrame() []uint8 {
	return []uint8{0xEE, 0x06, 0x2D, 0xEE, 0xEA, 0x01, 0x01, 0xE5}
}

// CreateTLMOffFrame returns exact raw ELRS command bytes for Telemetry = Off.
func CreateTLMOffFrame() []uint8 {
	return []uint8{0xEE, 0x06, 0x2D, 0xEE, 0xEA, 0x02, 0x01, 0xF8}
}

// CreateSW8CHFrame returns exact raw ELRS command bytes for Switch mode = 8ch.
func CreateSW8CHFrame() []uint8 {
	return []uint8{0xEE, 0x06, 0x2D, 0xEE, 0xEA, 0x03, 0x00, 0x26}
}

// CreateLMNormFrame returns exact raw ELRS command bytes for Link mode = Normal.
func CreateLMNormFrame() []uint8 {
	return []uint8{0xEE, 0x06, 0x2D, 0xEE, 0xEA, 0x04, 0x00, 0x17}
}

// CreateMMOnFrame returns exact raw ELRS command bytes for Model Match = On.
func CreateMMOnFrame() []uint8 {
	return []uint8{0xEE, 0x06, 0x2D, 0xEE, 0xEA, 0x05, 0x01, 0xC9}
}

func CreatePingDevicesFrame() []uint8 {
	frame := []uint8{
		/* 0: */ uint8(UartSyncFrame),
		/* 1: */ 5,
		/* 2: */ uint8(PingDevicesFrame),
		/* 3: */ uint8(AllEndpoint),
		/* 4: */ uint8(LuaEndpoint),
		/* 5: */ 0, //crc BA
		/* 6: */ 0, //crc D5
	}

	frame[5] = crc.BA(frame[2:5])
	frame[6] = crc.D5(frame[2:6])
	return frame
}

func CreateParameterSettingsReadFrame(deviceId uint8, fieldId uint8, fieldChunk uint8) []uint8 {
	frame := []uint8{
		/* 0: */ uint8(UartSyncFrame),
		/* 1: */ 7,
		/* 2: */ uint8(ParameterSettingsReadFrame),
		/* 3: */ deviceId,
		/* 4: */ uint8(LuaEndpoint),
		/* 5: */ fieldId,
		/* 6: */ fieldChunk,
		/* 7: */ 0, //crc BA
		/* 8: */ 0, //crc D5
	}

	frame[7] = crc.BA(frame[2:7])
	frame[8] = crc.D5(frame[2:8])
	return frame
}

func CreateParameterSettingWriteFrameUint8(deviceId uint8, fieldId uint8, fieldValue uint8) []uint8 {
	frame := []uint8{
		/* 0: */ uint8(UartSyncFrame),
		/* 1: */ 7,
		/* 2: */ uint8(ParameterSettingsWriteFrame),
		/* 3: */ deviceId,
		/* 4: */ uint8(LuaEndpoint),
		/* 5: */ fieldId,
		/* 6: */ fieldValue,
		/* 7: */ 0, //crc BA
		/* 8: */ 0, //crc D5
	}

	frame[7] = crc.BA(frame[2:7])
	frame[8] = crc.D5(frame[2:8])
	//fmt.Printf("%x\n", frame)
	return frame
}

// CreateParameterSettingWriteFrameUint16
// not sure if this would work, ELRS seems to throw away the least-significant-byte of the fieldValue
func CreateParameterSettingWriteFrameUint16(deviceId uint8, fieldId uint8, fieldValue uint16) []uint8 {
	frame := []uint8{
		/* 0: */ uint8(UartSyncFrame),
		/* 1: */ 8,
		/* 2: */ uint8(ParameterSettingsWriteFrame),
		/* 3: */ deviceId,
		/* 4: */ uint8(LuaEndpoint),
		/* 5: */ fieldId,
		/* 6: */ uint8((fieldValue >> 8) & 0x0F),
		/* 7: */ uint8(fieldValue & 0x0F),
		/* 8: */ 0, //crc BA
		/* 9: */ 0, //crc D5
	}

	frame[8] = crc.BA(frame[2:8])
	frame[9] = crc.D5(frame[2:9])
	return frame
}

// GetRefreshRate returns a sensible initial channel-pack cadence for a given
// UART baud rate.
//
// The host send cadence is set by the firmware's RequestedRCpacketInterval
// (default 5000us = 200Hz), NOT by the UART baud. Higher baud only makes each
// burst shorter on the wire; it does not change how often the firmware
// expects a packet. Sending faster than firmware expects causes host bursts
// to collide with the firmware's half-duplex reply window, which prevents
// OPENTX_SYNC packets from ever arriving and the link never bootstraps.
//
// The previous logic returned MinRefreshRate (500us) for any baud >= 921600,
// ~10x faster than the firmware ever wants. That broke bootstrap.
func GetRefreshRate(baudRate int32) time.Duration {
	if baudRate <= 115200 {
		// Slow handsets (legacy / non-ELRS) — 16ms is conventional.
		return 16 * 1000 * time.Microsecond
	}
	// 400k and up: match firmware default 200Hz. The firmware will refine
	// this via OPENTX_SYNC packets once a clean RX is established.
	return 5 * 1000 * time.Microsecond
}

func PackChannels(channels *[16]util.CRSFValue) (result []byte) {
	var CrossfireChBits uint8 = 11
	var buf [26]byte
	var offset uint8 = 0
	var bits util.CRSFValue
	var bitsAvailable uint8 = 0

	buf[offset] = 0xEE
	offset += 1

	buf[offset] = 24 // 1(ID) + 22 + 1(CRC)
	offset += 1

	buf[offset] = 0x16
	offset += 1

	for i := 0; i < 16; i++ {
		var val = channels[i]
		shifted := val << bitsAvailable
		bits |= shifted
		bitsAvailable += CrossfireChBits
		for bitsAvailable >= 8 {
			buf[offset] = byte(bits)
			offset += 1
			bits >>= 8
			bitsAvailable -= 8
		}
	}

	buf[25] = crc.D5(buf[2:25])
	//fmt.Printf("%d\n", channels)
	//fmt.Printf("%x\n", buf)
	return buf[:]
}

// SyncPeriods converts a CRSF OPENTX_SYNC payload into a (steadyPeriod, nextPeriod)
// pair. Both `rate` and `offset` are in 0.1 us units (firmware convention).
//
//   - steadyPeriod: the period the host should run at long-term (== rate).
//   - nextPeriod:   the period to use for the very next tick only, so that
//     subsequent ticks land in the firmware's expected RX window. Computed as
//     `rate - offset` (a positive offset means the host arrived AFTER the
//     firmware's expected slot, so the next tick must come SOONER).
//
// Both values are clamped to [MinRefreshRate, MaxRefreshRate] for sanity.
//
// Replaces the old AdjustSendRate which incorrectly returned (rate+offset)/10
// as a permanent new period — that conflated rate and offset and applied a
// one-shot phase correction as a permanent rate change, causing the period
// to drift and never converge.
func SyncPeriods(rate int32, offset int32) (steadyPeriod time.Duration, nextPeriod time.Duration) {
	steadyPeriod = time.Duration(rate/10) * time.Microsecond
	nextPeriod = time.Duration((rate-offset)/10) * time.Microsecond

	if steadyPeriod < MinRefreshRate {
		steadyPeriod = MinRefreshRate
	} else if steadyPeriod > MaxRefreshRate {
		steadyPeriod = MaxRefreshRate
	}

	if nextPeriod < MinRefreshRate {
		nextPeriod = MinRefreshRate
	} else if nextPeriod > MaxRefreshRate {
		nextPeriod = MaxRefreshRate
	}

	return steadyPeriod, nextPeriod
}
