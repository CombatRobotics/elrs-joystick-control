// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package link

type ChannelRequest int32

const (
	SendModelId ChannelRequest = iota
	PingDevices ChannelRequest = iota
)

const (
	ModelIDMin uint8 = 0
	ModelIDMax uint8 = 63
)

type ReadDeviceFieldsRequest struct {
	deviceId   uint8
	fieldId    uint8
	fieldChunk uint8
}

type WriteDeviceFieldRequestUint8 struct {
	deviceId   uint8
	fieldId    uint8
	fieldValue uint8
}

type WriteDeviceFieldRequestUint16 struct {
	deviceId   uint8
	fieldId    uint8
	fieldValue uint16
}

type PortState int32

const (
	PortUnknown      PortState = iota
	PortDisconnected PortState = iota
	PortConnected    PortState = iota
)

type SupervisorState int32

//goland:noinspection GoUnusedConst
const (
	SupervisorUnknown  SupervisorState = iota
	SupervisorInactive SupervisorState = iota
	SupervisorActive   SupervisorState = iota
)
