// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package link

import (
	"errors"
	"fmt"
	"github.com/kaack/elrs-joystick-control/pkg/crossfire"
	"github.com/kaack/elrs-joystick-control/pkg/crossfire/settings"
	"github.com/kaack/elrs-joystick-control/pkg/proto/generated/pb"
	sc "github.com/kaack/elrs-joystick-control/pkg/serial"
	"github.com/kaack/elrs-joystick-control/pkg/util"
	"gopkg.in/tomb.v2"
	"strings"
	"sync"
	"time"
)

type Controller struct {
	serialCtl *sc.Controller

	channels [16]uint16

	portState       PortState
	supervisorState SupervisorState

	sentPacketsCount  uint64
	recvPacketsCount  uint64
	errorPacketsCount uint64

	supervisorTomb *tomb.Tomb
	sendLoopTomb   *tomb.Tomb
	recvLoopTomb   *tomb.Tomb
	portLoopTomb   *tomb.Tomb

	TelemetryBroadcaster    *TelemetryBroadcaster
	DeviceInfoBroadcaster   *TelemetryBroadcaster
	DeviceFieldBroadcaster  *TelemetryBroadcaster
	DeviceStatusBroadcaster *TelemetryBroadcaster

	sendChan chan any
	recvChan chan any

	linkCfgMu      sync.RWMutex
	activePortName string
	activeBaudRate int32

	channelSourceMu   sync.RWMutex
	channelSourceMode ChannelSourceMode
	ros2TopicName     string

	ros2SubscriberTomb *tomb.Tomb

	ros2DataMu            sync.RWMutex
	ros2MessageCount      uint64
	ros2ParseErrorCount   uint64
	ros2WriteCount        uint64
	ros2LastRawLeftRPM    int32
	ros2LastRawRightRPM   int32
	ros2LastLeftCRSF      util.CRSFValue
	ros2LastRightCRSF     util.CRSFValue
	ros2LastRecvAt        time.Time
	ros2LastWriteAt       time.Time
	ros2LastWritePort     string
	ros2LastSubscriberErr string

	modelIDMu               sync.RWMutex
	modelID                 uint8
	modelIDTriggerCount     uint64
	modelIDSentCount        uint64
	modelIDSendErrorCount   uint64
	lastModelIDTriggerAt    time.Time
	lastModelIDSentAt       time.Time
	lastModelIDSendError    string
	lastModelIDTriggerDebug string
}

func NewCtl(sc *sc.Controller) *Controller {
	linkCtl := &Controller{
		portState:               PortUnknown,
		supervisorState:         SupervisorInactive,
		serialCtl:               sc,
		TelemetryBroadcaster:    NewTelemetryBroadcaster(),
		DeviceInfoBroadcaster:   NewTelemetryBroadcaster(),
		DeviceFieldBroadcaster:  NewTelemetryBroadcaster(),
		DeviceStatusBroadcaster: NewTelemetryBroadcaster(),
		modelID:                 0,
		channelSourceMode:       ChannelSourceROS2,
		ros2TopicName:           "WheelRPM",
	}
	err := linkCtl.Init()

	if err != nil {
		linkCtl.Quit()
		panic(err)
	}
	return linkCtl
}

func (c *Controller) Init() (err error) {

	return err
}

func (c *Controller) Quit() {

}

func (c *Controller) GetLinkState(state *pb.LinkState) *pb.LinkState {
	if state == nil {
		state = &pb.LinkState{}
	}

	state.SupervisorState = pb.SupervisorState(c.supervisorState)
	state.PortState = pb.PortState(c.portState)
	state.ReceivedPacketsCount = c.recvPacketsCount
	state.SentPacketsCount = c.sentPacketsCount
	state.ErrorPacketsCount = c.errorPacketsCount

	return state
}

func (c *Controller) SetModelID(modelID uint8) error {
	if modelID < ModelIDMin || modelID > ModelIDMax {
		return errors.New(fmt.Sprintf("model id must be in range [%d..%d], but got %d", ModelIDMin, ModelIDMax, modelID))
	}

	c.modelIDMu.Lock()
	defer c.modelIDMu.Unlock()
	c.modelID = modelID

	fmt.Printf("(link) configured model id: %d\n", modelID)
	return nil
}

func (c *Controller) GetModelID() uint8 {
	c.modelIDMu.RLock()
	defer c.modelIDMu.RUnlock()
	return c.modelID
}

func (c *Controller) SetChannelSourceMode(mode string) error {
	normalized := strings.ToLower(strings.TrimSpace(mode))
	if normalized == "" {
		normalized = string(ChannelSourceConfig)
	}

	switch ChannelSourceMode(normalized) {
	case ChannelSourceConfig, ChannelSourceROS2:
		c.channelSourceMu.Lock()
		c.channelSourceMode = ChannelSourceMode(normalized)
		c.channelSourceMu.Unlock()
		fmt.Printf("(link) configured channel source mode: %s\n", normalized)
		return nil
	default:
		return errors.New(fmt.Sprintf("invalid channel source mode %q, expected one of [%s,%s]", mode, ChannelSourceConfig, ChannelSourceROS2))
	}
}

func (c *Controller) GetChannelSourceMode() ChannelSourceMode {
	c.channelSourceMu.RLock()
	defer c.channelSourceMu.RUnlock()
	if c.channelSourceMode == "" {
		return ChannelSourceConfig
	}
	return c.channelSourceMode
}

func (c *Controller) SetROS2TopicName(topic string) {
	normalized := strings.TrimSpace(topic)
	if normalized == "" {
		normalized = "WheelRPM"
	}
	c.channelSourceMu.Lock()
	c.ros2TopicName = normalized
	c.channelSourceMu.Unlock()
	fmt.Printf("(link) configured ros2 topic: %s\n", normalized)
}

func (c *Controller) GetROS2TopicName() string {
	c.channelSourceMu.RLock()
	defer c.channelSourceMu.RUnlock()
	if c.ros2TopicName == "" {
		return "WheelRPM"
	}
	return c.ros2TopicName
}

func (c *Controller) capToCRSFValue(raw int32) util.CRSFValue {
	if raw < int32(util.CRSFMinValue) {
		return util.CRSFValue(util.CRSFMinValue)
	}
	if raw > int32(util.CRSFMaxValue) {
		return util.CRSFValue(util.CRSFMaxValue)
	}
	return util.CRSFValue(raw)
}

func (c *Controller) RecordROS2Message(rawLeftRPM int32, rawRightRPM int32) (util.CRSFValue, util.CRSFValue) {
	leftCRSF := c.capToCRSFValue(rawLeftRPM)
	rightCRSF := c.capToCRSFValue(rawRightRPM)

	c.ros2DataMu.Lock()
	defer c.ros2DataMu.Unlock()

	c.ros2MessageCount += 1
	c.ros2LastRawLeftRPM = rawLeftRPM
	c.ros2LastRawRightRPM = rawRightRPM
	c.ros2LastLeftCRSF = leftCRSF
	c.ros2LastRightCRSF = rightCRSF
	c.ros2LastRecvAt = time.Now()

	return leftCRSF, rightCRSF
}

func (c *Controller) RecordROS2ParseError(err error, line string) {
	c.ros2DataMu.Lock()
	defer c.ros2DataMu.Unlock()

	c.ros2ParseErrorCount += 1
	c.ros2LastSubscriberErr = fmt.Sprintf("line=%q err=%v", line, err)
}

func (c *Controller) RecordROS2Write(port string, leftCRSF util.CRSFValue, rightCRSF util.CRSFValue) {
	c.ros2DataMu.Lock()
	defer c.ros2DataMu.Unlock()

	c.ros2WriteCount += 1
	c.ros2LastWriteAt = time.Now()
	c.ros2LastWritePort = port
	c.ros2LastLeftCRSF = leftCRSF
	c.ros2LastRightCRSF = rightCRSF
}

func (c *Controller) SetROS2LastError(err string) {
	c.ros2DataMu.Lock()
	defer c.ros2DataMu.Unlock()
	c.ros2LastSubscriberErr = err
}

func (c *Controller) GetROS2ChannelsSnapshot() ([16]util.CRSFValue, util.CRSFValue, util.CRSFValue) {
	c.ros2DataMu.RLock()
	defer c.ros2DataMu.RUnlock()

	var channels [16]util.CRSFValue
	channels[0] = c.ros2LastLeftCRSF
	channels[1] = c.ros2LastRightCRSF
	return channels, c.ros2LastLeftCRSF, c.ros2LastRightCRSF
}

func (c *Controller) GetROS2DebugString() string {
	c.ros2DataMu.RLock()
	defer c.ros2DataMu.RUnlock()

	return fmt.Sprintf(
		"topic=%s msgs=%d parse_errors=%d writes=%d last_raw_left=%d last_raw_right=%d last_left_crsf=%d last_right_crsf=%d last_recv=%s last_write=%s last_write_port=%s last_err=%s",
		c.GetROS2TopicName(),
		c.ros2MessageCount,
		c.ros2ParseErrorCount,
		c.ros2WriteCount,
		c.ros2LastRawLeftRPM,
		c.ros2LastRawRightRPM,
		c.ros2LastLeftCRSF,
		c.ros2LastRightCRSF,
		c.ros2LastRecvAt.Format(time.RFC3339Nano),
		c.ros2LastWriteAt.Format(time.RFC3339Nano),
		c.ros2LastWritePort,
		c.ros2LastSubscriberErr,
	)
}

func (c *Controller) IsSupervisorActive() bool {
	return c.supervisorState == SupervisorActive && c.supervisorTomb != nil && c.supervisorTomb.Alive()
}

func (c *Controller) SetActiveLinkConfig(portName string, baudRate int32) {
	c.linkCfgMu.Lock()
	defer c.linkCfgMu.Unlock()

	c.activePortName = portName
	c.activeBaudRate = baudRate
}

func (c *Controller) ClearActiveLinkConfig() {
	c.linkCfgMu.Lock()
	defer c.linkCfgMu.Unlock()

	c.activePortName = ""
	c.activeBaudRate = 0
}

func (c *Controller) GetActiveLinkConfig() (string, int32) {
	c.linkCfgMu.RLock()
	defer c.linkCfgMu.RUnlock()

	return c.activePortName, c.activeBaudRate
}

func (c *Controller) TriggerModelIDSend(source string) error {
	if c.sendChan == nil || !c.IsSupervisorActive() {
		return errors.New("link is not active, model id frame was not queued")
	}

	c.RecordModelIDTrigger(fmt.Sprintf("source=%s", source))
	select {
	case c.sendChan <- SendModelId:
		fmt.Printf("(link) queued model id send request (source=%s, %s)\n", source, c.GetModelIDDebugString())
		return nil
	case <-time.After(250 * time.Millisecond):
		return errors.New("timeout while queuing model id frame")
	}
}

func (c *Controller) RecordModelIDTrigger(debug string) {
	c.modelIDMu.Lock()
	defer c.modelIDMu.Unlock()

	c.modelIDTriggerCount += 1
	c.lastModelIDTriggerAt = time.Now()
	c.lastModelIDTriggerDebug = debug
}

func (c *Controller) RecordModelIDSendOK() {
	c.modelIDMu.Lock()
	defer c.modelIDMu.Unlock()

	c.modelIDSentCount += 1
	c.lastModelIDSentAt = time.Now()
	c.lastModelIDSendError = ""
}

func (c *Controller) RecordModelIDSendError(err error) {
	c.modelIDMu.Lock()
	defer c.modelIDMu.Unlock()

	c.modelIDSendErrorCount += 1
	c.lastModelIDSendError = err.Error()
}

func (c *Controller) GetModelIDDebugString() string {
	c.modelIDMu.RLock()
	defer c.modelIDMu.RUnlock()

	return fmt.Sprintf("model_id=%d, triggers=%d, sent=%d, send_errors=%d, last_trigger=%s, last_send=%s, last_trigger_info=%s, last_send_error=%s",
		c.modelID,
		c.modelIDTriggerCount,
		c.modelIDSentCount,
		c.modelIDSendErrorCount,
		c.lastModelIDTriggerAt.Format(time.RFC3339Nano),
		c.lastModelIDSentAt.Format(time.RFC3339Nano),
		c.lastModelIDTriggerDebug,
		c.lastModelIDSendError,
	)
}

func (c *Controller) GetCRSFDevices() ([]*pb.CRSFDeviceInfoData, error) {

	if c.sendChan == nil || c.supervisorState != SupervisorActive {
		return nil, errors.New("link is not active, start RF Link to use this function")
	}

	luaChan := c.DeviceInfoBroadcaster.Subscribe()
	defer c.DeviceInfoBroadcaster.Unsubscribe(luaChan)

	devicesMap := map[uint32]*pb.CRSFDeviceInfoData{}
	var devicesProtoList []*pb.CRSFDeviceInfoData

	// sometimes the module does not reply with any information
	// when pinging devices, so to work around this
	// we repeatedly ask the module to ping all devices, and only
	// exit when there are multiple collisions

	collisions := 0
	for collisions < 3 {

	RequestDevices:
		//fmt.Printf("requesting field: %d, chunk: %d\n", fieldIndex, chunkIndex)
		c.sendChan <- PingDevices
	WaitForDevices:
		for {
			select {
			case telem := <-luaChan:
				deviceInfo := telem.GetDeviceInfo()

				if deviceInfo == nil {
					// no device info received
					goto WaitForDevices
				} else if _, ok := devicesMap[deviceInfo.GetId()]; ok {
					// already seen this device
					collisions += 1
					break WaitForDevices
				}

				devicesMap[deviceInfo.GetId()] = deviceInfo
				devicesProtoList = append(devicesProtoList, deviceInfo)

				goto WaitForDevices
			case <-time.After(50 * time.Millisecond):
				goto RequestDevices
			}
		}
	}

	return devicesProtoList, nil
}

var fieldConstructors = map[pb.CRSFDeviceFieldType]func(id uint32, parentId uint32, data []uint8) settings.FieldType{
	pb.CRSFDeviceFieldType_TEXT_SELECT: settings.NewTextSelectField,
	pb.CRSFDeviceFieldType_FOLDER:      settings.NewFolderField,
	pb.CRSFDeviceFieldType_COMMAND:     settings.NewCommandField,
	pb.CRSFDeviceFieldType_STRING:      settings.NewStringField,
	pb.CRSFDeviceFieldType_INFO:        settings.NewInfoField,
	pb.CRSFDeviceFieldType_INT8:        settings.NewInt8Field,
	pb.CRSFDeviceFieldType_INT16:       settings.NewInt16Field,
	pb.CRSFDeviceFieldType_INT32:       settings.NewInt32Field,
	pb.CRSFDeviceFieldType_UINT8:       settings.NewUint8Field,
	pb.CRSFDeviceFieldType_UINT16:      settings.NewUint16Field,
	pb.CRSFDeviceFieldType_UINT32:      settings.NewUint32Field,
}

func (c *Controller) GetCRSFDeviceFields(deviceInfo *pb.CRSFDeviceInfoData) ([]*pb.CRSFDeviceFieldData, error) {
	if c.sendChan == nil || c.supervisorState != SupervisorActive {
		return nil, errors.New("link is not active, start RF Link to use this function")
	}

	var fieldProtoList []*pb.CRSFDeviceFieldData
	var fieldId uint32
	var fieldProto *pb.CRSFDeviceFieldData
	var err error

	for fieldId = 1; fieldId < deviceInfo.GetFieldCount(); fieldId++ {
		if fieldProto, err = c.GetCRSFDeviceField(deviceInfo, fieldId, 5*time.Second); err != nil {
			return nil, err
		} else if fieldProto != nil {
			fieldProtoList = append(fieldProtoList, fieldProto)
		}
	}

	return fieldProtoList, nil
}

func (c *Controller) GetCRSFDeviceField(deviceInfo *pb.CRSFDeviceInfoData, fieldId uint32, timeout time.Duration) (*pb.CRSFDeviceFieldData, error) {
	if c.sendChan == nil || c.supervisorState != SupervisorActive {
		return nil, errors.New("link is not active, start RF Link to use this function")
	}

	if deviceInfo == nil {
		return nil, errors.New("device is required")
	}

	if fieldId > deviceInfo.GetFieldCount() {
		return nil, errors.New("field id is not valid")
	}

	luaChan := c.DeviceFieldBroadcaster.Subscribe()
	defer c.DeviceFieldBroadcaster.Unsubscribe(luaChan)

	handledChunksMap := map[uint32]bool{}

	var fieldData []byte
	var firstFieldEntry *pb.CRSFDeviceFieldEntryData
	var fieldProto *pb.CRSFDeviceFieldData

	chunkIndex := 0

	chunkRequestsCount := uint32(0)           // how many times the chunk has been requested
	maxRequestsPerChunk := uint32(5)          // how many times to request chunk, before giving up
	chunkWaitTimeout := 50 * time.Millisecond // how long to wait for a chunk, before requesting it again

	if timeout > time.Duration(maxRequestsPerChunk)*chunkWaitTimeout {
		maxRequestsPerChunk = uint32(timeout / chunkWaitTimeout)
	}

RequestChunk:
	//fmt.Printf("requesting field: %d, chunk: %d\n", fieldIndex, chunkIndex)
	c.sendChan <- &ReadDeviceFieldsRequest{
		deviceId:   uint8(deviceInfo.Id),
		fieldId:    uint8(fieldId),
		fieldChunk: uint8(chunkIndex),
	}
WaitForChunk:
	for {
		select {
		case telem := <-luaChan:
			fieldEntry := telem.GetDeviceFieldEntry()

			if fieldEntry == nil {
				//fmt.Printf("field entry null, will wait for chunk ...\n")
				goto WaitForChunk
			} else if fieldEntry.GetId() != fieldId {
				//fmt.Printf("field entry id mismatch, got id: %d, expected: %d, will wait for chunk ...\n", fieldEntry.GetId(), fieldId)
				goto WaitForChunk
			}

			if _, ok := handledChunksMap[fieldEntry.GetChunksRemaining()]; !ok {
				handledChunksMap[fieldEntry.GetChunksRemaining()] = true
				var chunk []byte
				if chunkIndex == 0 {
					// first chunk header is: (id, chunks remaining, parent id, data type) which is 4 bytes
					chunk = fieldEntry.GetBuffer()[4 : len(fieldEntry.GetBuffer())-1]
				} else {
					// other chunks header is: (id, chunks remaining) which is 2 bytes
					chunk = fieldEntry.GetBuffer()[2 : len(fieldEntry.GetBuffer())-1]
				}

				//fmt.Printf("appending chunk %d for field: %d\n", chunkIndex, fieldId)
				//fmt.Printf("chunk %x\n", string(chunk))
				fieldData = append(fieldData, chunk...)
				chunkRequestsCount = 0

				if chunkIndex == 0 {
					firstFieldEntry = fieldEntry //save this, because the data type only comes on first chunk
				}
			} else {
				goto RequestChunk
			}

			//check to see if there are more chunks needed
			if fieldEntry.GetChunksRemaining() > 0 {
				//fmt.Printf("there are %d more chunks for field %d\n", fieldEntry.GetChunksRemaining(), fieldId)
				chunkIndex += 1
				goto RequestChunk
			}

			if constructor, ok := fieldConstructors[firstFieldEntry.GetDataType()]; ok {
				field := constructor(firstFieldEntry.GetId(),
					firstFieldEntry.GetParentId(),
					fieldData)
				proto := field.Proto()
				fieldProto = proto
				fmt.Printf("%s\n", proto)
			}

			//fmt.Printf("all %d chunks received for field %d\n", chunkIndex, fieldIndex)
			break WaitForChunk
		case <-time.After(chunkWaitTimeout):
			chunkRequestsCount += 1
			//fmt.Printf("timeout(%v/%v) while waiting for chunk %d on field %d\n", chunkRequestsCount, maxRequestsPerChunk, chunkIndex, fieldId)
			if chunkRequestsCount >= maxRequestsPerChunk {
				break WaitForChunk
			} else {
				goto RequestChunk
			}
		}
	}

	return fieldProto, nil
}

func (c *Controller) SetCRSFDeviceField(device *pb.CRSFDeviceInfoData, field *pb.CRSFDeviceFieldData) (*pb.CRSFDeviceFieldData, error) {
	if device == nil {
		return nil, errors.New("device is required")
	}

	if field == nil {
		return nil, errors.New("field is required")
	}

	fieldData := field.GetData()
	var fieldDataRes *pb.CRSFDeviceFieldData
	var err error

	switch data := fieldData.(type) {
	case *pb.CRSFDeviceFieldData_Int8:
		min := data.Int8.GetMin()
		max := data.Int8.GetMax()
		value := data.Int8.GetValue()
		if !(int8(value) >= int8(min) && int8(value) <= int8(max)) {
			return nil, errors.New("int8 value is out of range")
		}
		c.sendChan <- &WriteDeviceFieldRequestUint8{
			deviceId:   uint8(device.GetId()),
			fieldId:    uint8(data.Int8.GetId()),
			fieldValue: uint8(value),
		}

		return field, nil

	case *pb.CRSFDeviceFieldData_Uint8:
		min := data.Uint8.GetMin()
		max := data.Uint8.GetMax()
		value := data.Uint8.GetValue()

		if !(uint8(value) >= uint8(min) && uint8(value) <= uint8(max)) {
			return nil, errors.New("uint8 value is out of range")
		}
		c.sendChan <- &WriteDeviceFieldRequestUint8{
			deviceId:   uint8(device.GetId()),
			fieldId:    uint8(data.Uint8.GetId()),
			fieldValue: uint8(value),
		}
		return field, nil
	case *pb.CRSFDeviceFieldData_Int16:
		min := data.Int16.GetMin()
		max := data.Int16.GetMax()
		value := data.Int16.GetValue()

		if !(int16(value) >= int16(min) && int16(value) <= int16(max)) {
			return nil, errors.New("int16 value is out of range")
		}
		c.sendChan <- &WriteDeviceFieldRequestUint16{
			deviceId:   uint8(device.GetId()),
			fieldId:    uint8(data.Int16.GetId()),
			fieldValue: uint16(value),
		}
		return field, nil
	case *pb.CRSFDeviceFieldData_Uint16:
		min := data.Uint16.GetMin()
		max := data.Uint16.GetMax()
		value := data.Uint16.GetValue()

		if !(uint16(value) >= uint16(min) && uint16(value) <= uint16(max)) {
			return nil, errors.New("uint16 value is out of range")
		}
		c.sendChan <- &WriteDeviceFieldRequestUint16{
			deviceId:   uint8(device.GetId()),
			fieldId:    uint8(data.Uint16.GetId()),
			fieldValue: uint16(value),
		}
		return field, nil
	case *pb.CRSFDeviceFieldData_TextSelect:
		min := data.TextSelect.GetMin()
		max := data.TextSelect.GetMax()
		value := data.TextSelect.GetValue()

		if !(value >= min && value <= max) {
			return nil, errors.New("selection value is out of range")
		}
		for i := 0; i < 5; i++ {
			//send the request multiple times, module sometimes ignores it
			c.sendChan <- &WriteDeviceFieldRequestUint8{
				deviceId:   uint8(device.GetId()),
				fieldId:    uint8(data.TextSelect.GetId()),
				fieldValue: uint8(value),
			}
			time.Sleep(250 * time.Millisecond)
		}

		return field, nil
	case *pb.CRSFDeviceFieldData_Command:
		step := data.Command.GetStep()

		// allowed steps
		if !(step == pb.CRSFDeviceFieldCommandStep_CLICK ||
			step == pb.CRSFDeviceFieldCommandStep_CANCEL ||
			step == pb.CRSFDeviceFieldCommandStep_CONFIRMED ||
			step == pb.CRSFDeviceFieldCommandStep_QUERY) {
			return nil, errors.New("requested step not allowed")
		}

		if fieldDataRes, err = c.RunCRSFCommand(device, data.Command, 5*time.Second); err != nil {
			return nil, err
		}
	default:
		return nil, errors.New(fmt.Sprintf("field cannot be set"))
	}

	return fieldDataRes, nil
}

func (c *Controller) RunCRSFCommand(device *pb.CRSFDeviceInfoData, field *pb.CRSFDeviceFieldCommand, timeout time.Duration) (*pb.CRSFDeviceFieldData, error) {
	if c.sendChan == nil || c.supervisorState != SupervisorActive {
		return nil, errors.New("link is not active, start RF Link to use this function")
	}

	if device == nil {
		return nil, errors.New("device is required")
	}

	if field == nil {
		return nil, errors.New("field is required")
	}

	if field.GetId() > device.GetFieldCount() {
		return nil, errors.New("field id is not valid")
	}

	luaChan := c.DeviceFieldBroadcaster.Subscribe()
	defer c.DeviceFieldBroadcaster.Unsubscribe(luaChan)

	handledChunksMap := map[uint32]bool{}

	var fieldData []byte
	var firstFieldEntry *pb.CRSFDeviceFieldEntryData
	var fieldProto *pb.CRSFDeviceFieldData

	chunkIndex := 0

	chunkRequestsCount := uint32(0)            // how many times the chunk has been requested
	maxRequestsPerChunk := uint32(5)           // how many times to request chunk, before giving up
	chunkWaitTimeout := 150 * time.Millisecond // how long to wait for a chunk, before requesting it again

	if timeout > time.Duration(maxRequestsPerChunk)*chunkWaitTimeout {
		maxRequestsPerChunk = uint32(timeout / chunkWaitTimeout)
	}

	//fmt.Printf("(cmd) running command %s (id: %v)\n", field.GetName(), field.GetId())
	c.sendChan <- &WriteDeviceFieldRequestUint8{
		deviceId:   uint8(device.GetId()),
		fieldId:    uint8(field.GetId()),
		fieldValue: uint8(field.GetStep()),
	}

	goto WaitForChunk

RequestChunk:
	//fmt.Printf("(cmd) requesting field: %d, chunk: %d\n", field.GetId(), chunkIndex)
	c.sendChan <- &WriteDeviceFieldRequestUint8{
		deviceId:   uint8(device.GetId()),
		fieldId:    uint8(field.GetId()),
		fieldValue: uint8(settings.StepQuery),
	}
WaitForChunk:
	for {
		//fmt.Printf("(cmd) waiting on chunks for field %d (expecting chunk: %d)\n", field.GetId(), chunkIndex)
		select {
		case telem := <-luaChan:
			fieldEntry := telem.GetDeviceFieldEntry()

			if fieldEntry == nil {
				//fmt.Printf("field entry null, will wait for chunk ...\n")
				goto WaitForChunk
			} else if fieldEntry.GetId() != field.GetId() {
				//fmt.Printf("field entry id mismatch, got id: %d, expected: %d, will wait for chunk ...\n", fieldEntry.GetId(), fieldId)
				goto WaitForChunk
			}

			//fmt.Printf("(cmd) received chunks for field %d (expecting chunk: %d)\n", field.GetId(), chunkIndex)

			if _, ok := handledChunksMap[fieldEntry.GetChunksRemaining()]; !ok {
				handledChunksMap[fieldEntry.GetChunksRemaining()] = true
				var chunk []byte
				if chunkIndex == 0 {
					// first chunk header is: (id, chunks remaining, parent id, data type) which is 4 bytes
					chunk = fieldEntry.GetBuffer()[4 : len(fieldEntry.GetBuffer())-1]
				} else {
					// other chunks header is: (id, chunks remaining) which is 2 bytes
					chunk = fieldEntry.GetBuffer()[2 : len(fieldEntry.GetBuffer())-1]
				}

				//fmt.Printf("(cmd) appending chunk %d for field: %d\n", chunkIndex, field.GetId())
				//fmt.Printf("(cmd) chunk %x\n", string(chunk))
				fieldData = append(fieldData, chunk...)
				chunkRequestsCount = 0

				if chunkIndex == 0 {
					firstFieldEntry = fieldEntry //save this, because the data type only comes on first chunk
				}
			} else {
				goto RequestChunk
			}

			//check to see if there are more chunks needed
			if fieldEntry.GetChunksRemaining() > 0 {
				//fmt.Printf("(cmd) there are %d more chunks for field %d\n", fieldEntry.GetChunksRemaining(), field.GetId())
				chunkIndex += 1
				goto RequestChunk
			}

			if constructor, ok := fieldConstructors[firstFieldEntry.GetDataType()]; ok {
				field := constructor(firstFieldEntry.GetId(),
					firstFieldEntry.GetParentId(),
					fieldData)
				proto := field.Proto()
				fieldProto = proto
				fmt.Printf("%s\n", proto)
			}

			//fmt.Printf("(cmd) all %d chunks received for field %d\n", chunkIndex, field.GetId())
			break WaitForChunk
		case <-time.After(chunkWaitTimeout):
			chunkRequestsCount += 1
			//fmt.Printf("(cmd) timeout(%v/%v) while waiting for chunk %d on field %d\n", chunkRequestsCount, maxRequestsPerChunk, chunkIndex, field.GetId())
			if chunkRequestsCount >= maxRequestsPerChunk {
				break WaitForChunk
			} else {
				goto RequestChunk
			}
		}
	}

	return fieldProto, nil
}

func (c *Controller) GetCRSFDeviceLinkStatus(timeout time.Duration) (*pb.CRSFDeviceLinkStatusData, error) {
	if c.sendChan == nil || c.supervisorState != SupervisorActive {
		return nil, errors.New("link is not active, start RF Link to use this function")
	}

	luaChan := c.DeviceStatusBroadcaster.Subscribe()
	defer c.DeviceStatusBroadcaster.Unsubscribe(luaChan)

	var fieldProto *pb.CRSFDeviceLinkStatusData

	statusRequestsCount := uint32(0)                  // how many times the status has been requested
	maxStatusRequests := uint32(5)                    // how many times to request status, before giving up
	statusRequestWaitTimeout := 50 * time.Millisecond // how long to wait for a status, before requesting it again

	if timeout > time.Duration(maxStatusRequests)*statusRequestWaitTimeout {
		maxStatusRequests = uint32(timeout / statusRequestWaitTimeout)
	}

RequestStatus:
	//fmt.Printf("(status) requesting status frame\n")
	c.sendChan <- &WriteDeviceFieldRequestUint8{
		deviceId:   uint8(crossfire.ModuleEndpoint),
		fieldId:    uint8(0x00),
		fieldValue: uint8(0x00),
	}
WaitForStatus:
	for {
		//fmt.Printf("(status) waiting on status frame\n")
		select {
		case telem := <-luaChan:
			//fmt.Printf("(status) received telemetry frame\n")
			status := telem.GetDeviceLinkStatus()

			if status == nil {
				//fmt.Printf("(status) field status is null, will wait again ...\n")
				goto WaitForStatus
			}

			fieldProto = status
			//fmt.Printf("(status) received status frame\n")
			break WaitForStatus
		case <-time.After(statusRequestWaitTimeout):
			statusRequestsCount += 1
			//fmt.Printf("timeout(%v/%v) while waiting for status frame\n", statusRequestsCount, maxStatusRequests)
			if statusRequestsCount >= maxStatusRequests {
				break WaitForStatus
			} else {
				goto RequestStatus
			}
		}
	}

	return fieldProto, nil
}

func (c *Controller) ClearCRSFDeviceLinkCriticalFlags() error {

	if c.sendChan == nil || c.supervisorState != SupervisorActive {
		return errors.New("link is not active, start RF Link to use this function")
	}

	c.sendChan <- &WriteDeviceFieldRequestUint8{
		deviceId:   uint8(crossfire.ModuleEndpoint),
		fieldId:    uint8(0x2E), //special field to request clearing flag
		fieldValue: uint8(0x00),
	}

	return nil
}
