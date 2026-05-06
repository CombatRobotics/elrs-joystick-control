// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package telemetry

import (
	"encoding/binary"
	"fmt"
	"github.com/kaack/elrs-joystick-control/pkg/crossfire"
	"time"
)

// SyncExtFrame carries an OPENTX_SYNC payload from the firmware to the host.
// On the wire:
//
//	0xEA <len> 0x3A 0xEA 0xEE <ext_type=0x10> <rate:4 BE> <offset:4 BE> <crc>
//
// Both rate and offset are signed int32 in 0.1us units (firmware convention).
type SyncExtFrame struct {
	RawData []uint8
}

func (t *SyncExtFrame) Addr() crossfire.Endpoint {
	return crossfire.Endpoint(t.RawData[0])
}

func (t *SyncExtFrame) Type() crossfire.FrameType {
	return crossfire.FrameType(t.RawData[2])
}

func (t *SyncExtFrame) Dst() crossfire.Endpoint {
	return crossfire.Endpoint(t.RawData[3])
}

func (t *SyncExtFrame) Src() crossfire.Endpoint {
	return crossfire.Endpoint(t.RawData[4])
}

func (t *SyncExtFrame) ExtType() crossfire.FrameType {
	return crossfire.FrameType(t.RawData[5])
}

func (t *SyncExtFrame) Data() []uint8 {
	return t.RawData[6:]
}

// Rate returns the firmware's requested host send period in 0.1us units.
// Divide by 10 for microseconds.
func (t *SyncExtFrame) Rate() int32 {
	return int32(binary.BigEndian.Uint32(t.RawData[6:10]))
}

// Offset returns the phase correction the host should apply, in 0.1us units.
// A positive value means the host's last packet arrived AFTER the firmware's
// expected slot; the host should make its next tick come SOONER by this much.
func (t *SyncExtFrame) Offset() int32 {
	return int32(binary.BigEndian.Uint32(t.RawData[10:14]))
}

func (t *SyncExtFrame) String() string {
	return fmt.Sprintf("(sync-frame) rate: %v, offset: %v",
		time.Duration(t.Rate()/10)*time.Microsecond,
		time.Duration(t.Offset()/10)*time.Microsecond)
}
