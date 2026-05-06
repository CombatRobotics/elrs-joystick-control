// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package telemetry

import (
	"github.com/kaack/elrs-joystick-control/pkg/crossfire"
)

// TelemType is implemented by all frames the reader can decode.
// Frames addressed to the host (0xEA) implement this interface.
type TelemType interface {
	Addr() crossfire.Endpoint
	Type() crossfire.FrameType
	Data() []uint8
}

// TelemExtType is implemented by extended-header frames (frame_type >= 0x28),
// which carry sender/destination bytes after the type byte.
type TelemExtType interface {
	TelemType
	Dst() crossfire.Endpoint
	Src() crossfire.Endpoint
}
