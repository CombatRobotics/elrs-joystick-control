// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package telemetry

import (
	"github.com/kaack/elrs-joystick-control/pkg/crossfire"
)

// isTelemetryAddress recognises the three frame-start bytes that can appear
// on a half-duplex CRSF bus:
//
//	0xEA HandsetEndpoint   — firmware → host (telemetry replies)
//	0xEE ModuleEndpoint    — host → firmware (channel-pack echo)
//	0xC8 UartSyncFrame     — host → firmware (admin echoes: ping, model-id,
//	                         parameter-read/write). These admin frames carry
//	                         the device id (often 0xEE for the TX module) at
//	                         offset 3; without recognising 0xC8 as a valid
//	                         start byte the parser mis-aligns on the internal
//	                         0xEE and produces 241-byte phantom "frames" that
//	                         always CRC-fail.
func isTelemetryAddress(c crossfire.Endpoint) bool {
	return c == crossfire.HandsetEndpoint ||
		c == crossfire.ModuleEndpoint ||
		c == crossfire.Endpoint(crossfire.UartSyncFrame)
}
