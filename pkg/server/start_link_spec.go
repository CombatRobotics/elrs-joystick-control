// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package server

import (
	"fmt"
	"strconv"
	"strings"
)

const startLinkModelIDSuffix = "|model_id="

func ParseStartLinkPortSpec(portSpec string) (string, uint8, error) {
	defaultModelID := uint8(0)
	if portSpec == "" {
		return "", defaultModelID, nil
	}

	index := strings.LastIndex(portSpec, startLinkModelIDSuffix)
	if index < 0 {
		return strings.TrimSpace(portSpec), defaultModelID, nil
	}

	portName := strings.TrimSpace(portSpec[:index])
	if portName == "" {
		return "", defaultModelID, fmt.Errorf("port name is empty in start link spec")
	}

	rawModelID := strings.TrimSpace(portSpec[index+len(startLinkModelIDSuffix):])
	if rawModelID == "" {
		return "", defaultModelID, fmt.Errorf("model id is empty in start link spec")
	}

	modelID, err := strconv.ParseUint(rawModelID, 10, 8)
	if err != nil {
		return "", defaultModelID, fmt.Errorf("model id is not a valid integer: %w", err)
	}

	if modelID > 63 {
		return "", defaultModelID, fmt.Errorf("model id %d is out of range [0..63]", modelID)
	}

	return portName, uint8(modelID), nil
}
