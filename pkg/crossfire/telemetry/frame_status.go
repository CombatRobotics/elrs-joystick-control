// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package telemetry

import (
	"encoding/binary"
	"fmt"
	"github.com/kaack/elrs-joystick-control/pkg/crossfire"
)

// MinStatusExtFrameSize is the minimum on-the-wire byte count of a Status
// frame (empty message string):
//
//	[0xEA][len][0x2E][0xEA][0xEE][pktsBad:1][pktsGood:2 BE][flags:1]['\0'][crc] = 11 bytes
const MinStatusExtFrameSize int = 11

// LUA flag bit positions (must match firmware lib/LUA/lua.h:7-19).
const (
	LuaFlagConnected       = 0
	LuaFlagStatus1         = 1
	LuaFlagModelMatch      = 2 // ⚠ set when MISMATCHED (warning bit)
	LuaFlagIsArmed         = 3
	LuaFlagWarning1        = 4
	LuaFlagErrorConnected  = 5
	LuaFlagErrorBaudRate   = 6
	LuaFlagCriticalWarn2   = 7
)

// StatusExtFrame is sent by the firmware in response to a host PARAMETER_WRITE
// with parameterIndex=0 (the "ELRS status request"). Payload is
// tagLuaElrsParams from lua.h:
//
//	[5]   pktsBad   (uint8)
//	[6:8] pktsGood  (uint16 BE)
//	[8]   flags     (uint8 bitfield indexed by lua_Flags enum)
//	[9..] msg       (null-terminated string)
type StatusExtFrame struct {
	RawData []uint8
}

func (t *StatusExtFrame) Addr() crossfire.Endpoint {
	return crossfire.Endpoint(t.RawData[0])
}

func (t *StatusExtFrame) Type() crossfire.FrameType {
	return crossfire.FrameType(t.RawData[2])
}

func (t *StatusExtFrame) Dst() crossfire.Endpoint {
	return crossfire.Endpoint(t.RawData[3])
}

func (t *StatusExtFrame) Src() crossfire.Endpoint {
	return crossfire.Endpoint(t.RawData[4])
}

func (t *StatusExtFrame) Data() []uint8 {
	return t.RawData[5:]
}

func (t *StatusExtFrame) PktsBad() uint8 {
	return t.RawData[5]
}

func (t *StatusExtFrame) PktsGood() uint16 {
	return binary.BigEndian.Uint16(t.RawData[6:8])
}

func (t *StatusExtFrame) Flags() uint8 {
	return t.RawData[8]
}

// Connected returns true when the firmware reports an active link to an RX
// (LUA_FLAG_CONNECTED bit set).
func (t *StatusExtFrame) Connected() bool {
	return (t.Flags()>>LuaFlagConnected)&1 == 1
}

// ModelMismatched returns true when the firmware is flagging a model mismatch.
// LUA_FLAG_MODEL_MATCH is a warning bit: set means mismatch, clear means no
// mismatch warning.
func (t *StatusExtFrame) ModelMismatched() bool {
	return (t.Flags()>>LuaFlagModelMatch)&1 == 1
}

// ModelMatched reports whether the status frame is free of the mismatch warning.
// Callers that want "currently connected and matched" should combine this with a
// live-link signal such as recent LinkStats.
func (t *StatusExtFrame) ModelMatched() bool {
	return !t.ModelMismatched()
}

// Armed returns true when the firmware reports an armed state (AUX1 high).
func (t *StatusExtFrame) Armed() bool {
	return (t.Flags()>>LuaFlagIsArmed)&1 == 1
}

// Message returns the null-terminated warning message string, if any.
func (t *StatusExtFrame) Message() string {
	if len(t.RawData) <= 9 {
		return ""
	}
	// payload starts at [9], runs up to but not including the trailing CRC.
	body := t.RawData[9 : len(t.RawData)-1]
	// strip trailing nulls
	for i, b := range body {
		if b == 0 {
			return string(body[:i])
		}
	}
	return string(body)
}

func (t *StatusExtFrame) String() string {
	return fmt.Sprintf(
		"(status-frame) status_connected_bit=%v model_mismatched_bit=%v model_matched_bit=%v armed=%v pktsGood=%d pktsBad=%d flags=0x%02x msg=%q",
		t.Connected(), t.ModelMismatched(), t.ModelMatched(), t.Armed(),
		t.PktsGood(), t.PktsBad(), t.Flags(), t.Message(),
	)
}
