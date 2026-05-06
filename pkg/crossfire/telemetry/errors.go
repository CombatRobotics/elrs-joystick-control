// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package telemetry

// InterruptedError is returned by Reader.Next when its context is cancelled.
// Callers should treat it as a clean shutdown signal, not a real error.
type InterruptedError struct{}

func (e *InterruptedError) Error() string {
	return "reader interrupted"
}
