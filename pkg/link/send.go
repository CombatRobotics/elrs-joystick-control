// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package link

import (
	"errors"
	"fmt"
	crsf "github.com/kaack/elrs-joystick-control/pkg/crossfire"
	telem "github.com/kaack/elrs-joystick-control/pkg/crossfire/telemetry"
	"github.com/kaack/elrs-joystick-control/pkg/serial"
	"github.com/kaack/elrs-joystick-control/pkg/util"
	"gopkg.in/tomb.v2"
	"time"
)

func (c *Controller) StartSendLoop(port *serial.Port, sendChan chan any, recvChan chan any) error {

	if c.sendLoopTomb != nil && c.sendLoopTomb.Alive() {
		return errors.New("send loop is already active")
	}

	c.sendLoopTomb = &tomb.Tomb{}
	c.sendLoopTomb.Go(func() error {
		return c.SendLoop(port, sendChan, recvChan)
	})

	return nil
}

func (c *Controller) StopSendLoop() error {
	if c.sendLoopTomb == nil || !c.sendLoopTomb.Alive() {
		return nil
	}

	c.sendLoopTomb.Kill(nil)
	if err := c.sendLoopTomb.Wait(); err != nil {
		return err
	}
	return nil
}

//goland:noinspection GoUnusedParameter
func (c *Controller) SendLoop(port *serial.Port, sendChan chan any, recvChan chan any) error {
	const fixedChannelSendPeriod = 10 * time.Millisecond
	fmt.Printf("(send-loop) starting, fixed channel refresh period %v (independent of baud)\n", fixedChannelSendPeriod)

	var err error
	ticker := time.NewTicker(fixedChannelSendPeriod)

	c.sentPacketsCount = 0
	configModeWarningPrinted := false
	unknownModeWarningPrinted := false
	zeroChannels := [16]util.CRSFValue{}

Loop:
	for {
		select {
		case <-c.sendLoopTomb.Dying():
			break Loop
		case chData := <-sendChan:
			switch data := (chData).(type) {
			case ChannelRequest:
				if data == SendModelId {
					modelID := c.GetModelID()
					modelIDFrame := crsf.CreateModelIDFrame(modelID)
					fmt.Printf("(send-loop) writing model id frame (model_id=%d frame=% X)\n", modelID, modelIDFrame)
					var written int32
					if written, err = port.Write(modelIDFrame); err != nil {
						c.errorPacketsCount += 1
						c.RecordModelIDSendError(err)
						fmt.Printf("(send-loop) could not write model id frame on port %s. %s\n", port.Name, err.Error())
						break
					}
					fmt.Printf("[MODEL_MATCH_TX_PACKET] time=%s port=%s model_id=%d bytes_written=%d packet=% X\n",
						time.Now().Format(time.RFC3339Nano), port.Name, modelID, written, modelIDFrame,
					)
					c.RecordModelIDSendOK()
					fmt.Printf("(send-loop) model id frame write ok (%s)\n", c.GetModelIDDebugString())
					continue
				} else if data == PingDevices {
					fmt.Printf("(send-loop) pinging devices\n")
					if _, err = port.Write(crsf.CreatePingDevicesFrame()); err != nil {
						c.errorPacketsCount += 1
						fmt.Printf("(send-loop) could not write ping devices frame on port %s. %s\n", port.Name, err.Error())
						break
					}
				}
			case *ReadDeviceFieldsRequest:
				if _, err = port.Write(crsf.CreateParameterSettingsReadFrame(data.deviceId, data.fieldId, data.fieldChunk)); err != nil {
					c.errorPacketsCount += 1
					fmt.Printf("(send-loop) could not write \"parameters-settings-read\" frame on port %s. %s\n", port.Name, err.Error())
					break
				}
			case *WriteDeviceFieldRequestUint8:
				fmt.Printf("(send-loop) setting device field (deviceId: %v, fieldId: %v, value(uint8): %v)\n", data.deviceId, data.fieldId, data.fieldValue)
				if _, err = port.Write(crsf.CreateParameterSettingWriteFrameUint8(data.deviceId, data.fieldId, data.fieldValue)); err != nil {
					c.errorPacketsCount += 1
					fmt.Printf("(send-loop) could not write \"parameters-settings-write-uint8\" frame on port %s. %s\n", port.Name, err.Error())
					break
				}
			case *WriteDeviceFieldRequestUint16:
				fmt.Printf("(send-loop) setting device field (deviceId: %v, fieldId: %v, value(uint16): %v)\n", data.deviceId, data.fieldId, data.fieldValue)
				if _, err = port.Write(crsf.CreateParameterSettingWriteFrameUint16(data.deviceId, data.fieldId, data.fieldValue)); err != nil {
					c.errorPacketsCount += 1
					fmt.Printf("(send-loop) could not write \"parameters-settings-write-uint16\" frame on port %s. %s\n", port.Name, err.Error())
					break
				}
			case *telem.TelemSyncType:
				// Intentionally ignored: channel send period is fixed to 10ms.
				// Keep consuming this type so recv->send sync events don't appear as unknown requests.
			default:
				fmt.Printf("(send-loop) unknown channel request\n")
			}

		case <-ticker.C:
			switch c.GetChannelSourceMode() {
			case ChannelSourceROS2:
				ros2Channels, leftCRSF, rightCRSF := c.GetROS2ChannelsSnapshot()
				if _, err = port.Write(crsf.PackChannels(&ros2Channels)); err != nil {
					fmt.Printf("(send-loop) could not write ros2 channels on port %s. %s\n", port.Name, err.Error())
					break Loop
				}

				c.sentPacketsCount += 1
				c.RecordROS2Write(port.Name, leftCRSF, rightCRSF)
				fmt.Printf("(send-loop) Written Left: %d, Right: %d (port=%s)\n", leftCRSF, rightCRSF, port.Name)

			case ChannelSourceConfig:
				if !configModeWarningPrinted {
					fmt.Printf("(send-loop) channel source 'config' is disabled in this ROS2-only build. writing zero channels\n")
					configModeWarningPrinted = true
				}
				if _, err = port.Write(crsf.PackChannels(&zeroChannels)); err != nil {
					fmt.Printf("(send-loop) could not write fallback zero channels on port %s. %s\n", port.Name, err.Error())
					break Loop
				}
				c.sentPacketsCount += 1

			default:
				if !unknownModeWarningPrinted {
					fmt.Printf("(send-loop) unknown channel source mode %q. writing zero channels\n", c.GetChannelSourceMode())
					unknownModeWarningPrinted = true
				}
				if _, err = port.Write(crsf.PackChannels(&zeroChannels)); err != nil {
					fmt.Printf("(send-loop) could not write fallback zero channels on port %s. %s\n", port.Name, err.Error())
					break Loop
				}
				c.sentPacketsCount += 1
			}
		}
	}

	fmt.Println("(send-loop): exiting send loop ...")

	return nil
}

//goland:noinspection GoUnusedFunction
func printChannels(channels *[16]util.CRSFValue) {
	fmt.Printf("(send-loop) r: %04d, p: %04d, t: %04d, y: %04d, arm: %04d, pre: %04d, mode: %04d\n", channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], channels[6])
}
