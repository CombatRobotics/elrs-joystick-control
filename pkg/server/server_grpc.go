// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package server

import (
	"context"
	"fmt"
	"github.com/kaack/elrs-joystick-control/pkg/http"
	lc "github.com/kaack/elrs-joystick-control/pkg/link"
	"github.com/kaack/elrs-joystick-control/pkg/proto/generated/pb"
	sc "github.com/kaack/elrs-joystick-control/pkg/serial"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
	"time"
)

type GRPCServer struct {
	pb.UnimplementedJoystickControlServer
	SerialCtl *sc.Controller
	LinkCtl   *lc.Controller
	HTTPCtl   *http.Controller
}

func (s *GRPCServer) GetGamepads(context.Context, *pb.Empty) (*pb.GetGamepadsRes, error) {
	return nil, status.Error(codes.Unimplemented, "gamepad support disabled in ROS2-only mode")
}

func (s *GRPCServer) GetTransmitters(context.Context, *pb.Empty) (*pb.GetTransmitterRes, error) {
	ports, err := s.SerialCtl.GetSerialPorts()
	if err != nil {
		return nil, err
	}

	var res pb.GetTransmitterRes
	for _, port := range ports {

		res.Transmitters = append(res.Transmitters, &pb.Transmitter{
			Port: port.Name,
			Name: port.Product,
		})
	}

	return &res, nil
}

func (s *GRPCServer) GetConfig(context.Context, *pb.Empty) (*pb.GetConfigRes, error) {
	return nil, status.Error(codes.Unimplemented, "config graph support disabled in ROS2-only mode")
}

func (s *GRPCServer) SetConfig(_ context.Context, req *pb.SetConfigReq) (*pb.Empty, error) {
	return nil, status.Error(codes.Unimplemented, "config graph support disabled in ROS2-only mode")
}

func (s *GRPCServer) StartHTTP(context.Context, *pb.Empty) (*pb.Empty, error) {
	if err := s.HTTPCtl.Start(); err != nil {
		return nil, status.Error(codes.Internal, err.Error())
	}
	return &pb.Empty{}, nil
}

func (s *GRPCServer) StopHTTP(context.Context, *pb.Empty) (*pb.Empty, error) {
	if err := s.HTTPCtl.Stop(); err != nil {
		return nil, status.Error(codes.Internal, err.Error())
	}
	return &pb.Empty{}, nil
}

func (s *GRPCServer) StartLink(_ context.Context, req *pb.StartLinkReq) (*pb.Empty, error) {

	if req.GetPort() == "" {
		return nil, status.Error(codes.InvalidArgument, "port_name is required")
	}

	if req.GetBaudRate() <= 0 {
		return nil, status.Error(codes.InvalidArgument, "baud_rate is required")
	}

	portName, modelID, err := ParseStartLinkPortSpec(req.GetPort())
	if err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("invalid port/model id payload. %s", err.Error()))
	}
	if portName == "" {
		return nil, status.Error(codes.InvalidArgument, "port_name is required")
	}

	if s.LinkCtl.IsSupervisorActive() {
		activePort, activeBaudRate := s.LinkCtl.GetActiveLinkConfig()
		if err := s.LinkCtl.SetModelID(modelID); err != nil {
			return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("invalid model_id. %s", err.Error()))
		}

		if portName != activePort || req.GetBaudRate() != activeBaudRate {
			fmt.Printf(
				"(grpc) startLink active-update: requested port=%q baud=%d differs from active port=%q baud=%d. treating as model-id-only update\n",
				portName,
				req.GetBaudRate(),
				activePort,
				activeBaudRate,
			)
		}
		fmt.Printf(
			"(grpc) startLink active-update: active_port=%q active_baud=%d requested_model_id=%d\n",
			activePort,
			activeBaudRate,
			modelID,
		)
		if err := s.LinkCtl.TriggerModelIDSend("grpc_startlink_active_update"); err != nil {
			return nil, status.Error(codes.Internal, fmt.Sprintf("could not queue model id frame. %s", err.Error()))
		}
		return &pb.Empty{}, nil
	}

	if err := s.LinkCtl.SetModelID(modelID); err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("invalid model_id. %s", err.Error()))
	}

	fmt.Printf("(grpc) startLink request: raw_port=%q parsed_port=%q baud=%d model_id=%d\n",
		req.GetPort(),
		portName,
		req.GetBaudRate(),
		modelID,
	)

	if err := s.LinkCtl.StartSupervisor(portName, req.GetBaudRate()); err != nil {
		return nil, status.Error(codes.Internal, err.Error())
	}
	return &pb.Empty{}, nil
}

func (s *GRPCServer) StopLink(context.Context, *pb.Empty) (*pb.Empty, error) {
	fmt.Printf("(grpc) stopLink request (%s)\n", s.LinkCtl.GetModelIDDebugString())
	if err := s.LinkCtl.StopSupervisor(); err != nil {
		return nil, status.Error(codes.Internal, err.Error())
	}
	return &pb.Empty{}, nil
}

func (s *GRPCServer) GetGamepadStream(req *pb.GetGamepadStreamReq, server pb.JoystickControl_GetGamepadStreamServer) error {
	return status.Error(codes.Unimplemented, "gamepad stream disabled in ROS2-only mode")
}

func (s *GRPCServer) GetTransmitterStream(req *pb.GetTransmitterStreamReq, server pb.JoystickControl_GetTransmitterStreamServer) error {
	return status.Error(codes.Unimplemented, "transmitter config stream disabled in ROS2-only mode")
}

func (s *GRPCServer) GetEvalStream(_ *pb.Empty, server pb.JoystickControl_GetEvalStreamServer) error {
	return status.Error(codes.Unimplemented, "eval stream disabled in ROS2-only mode")
}

func (s *GRPCServer) GetLinkStream(_ *pb.Empty, server pb.JoystickControl_GetLinkStreamServer) error {

	var err error

	ticker := time.NewTicker(500 * time.Millisecond)
	state := s.LinkCtl.GetLinkState(nil)

	if err = s.StreamLinkState(state, server); err != nil {
		return err
	}

	for {
		select {
		case <-ticker.C:
			if err = s.StreamLinkState(state, server); err != nil {
				return err
			}
		}
	}

}

func (s *GRPCServer) GetTelemetryStream(_ *pb.Empty, server pb.JoystickControl_GetTelemetryStreamServer) error {

	var err error
	var telemetry *pb.Telemetry

	telemetryChan := s.LinkCtl.TelemetryBroadcaster.Subscribe()
	defer s.LinkCtl.TelemetryBroadcaster.Unsubscribe(telemetryChan)

	for {
		telemetry = <-telemetryChan
		if err = server.Send(telemetry); err != nil {
			return err
		}
	}

}

func (s *GRPCServer) GetAppInfo(_ context.Context, _ *pb.Empty) (*pb.GetAppInfoRes, error) {

	var info *VersionInfo
	var err error

	if info, err = GetVersionInfo(); err != nil {
		return nil, status.Error(codes.Internal, fmt.Sprintf("could not unmarshal version file. %s", err.Error()))
	}

	return &pb.GetAppInfoRes{
		ReleaseTag: info.ReleaseTag,
		CommitHash: info.CommitHash,
		BranchName: info.BranchName,
	}, nil
}

func (s *GRPCServer) GetCRSFDevices(_ context.Context, _ *pb.Empty) (*pb.GetCRSFDevicesRes, error) {

	luaChan := s.LinkCtl.DeviceInfoBroadcaster.Subscribe()
	defer s.LinkCtl.DeviceInfoBroadcaster.Unsubscribe(luaChan)

	var err error
	var devicesList []*pb.CRSFDeviceInfoData
	if devicesList, err = s.LinkCtl.GetCRSFDevices(); err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("could not get devices. %s", err.Error()))
	}

	res := pb.GetCRSFDevicesRes{
		Devices: devicesList,
	}

	return &res, nil
}

func (s *GRPCServer) GetCRSFDeviceFields(_ context.Context, req *pb.GetCRSFDeviceFieldsReq) (*pb.GetCRSFDeviceFieldsRes, error) {
	if req == nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("request payload required"))
	}

	deviceInfo := req.GetDevice()

	if deviceInfo == nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("device info required"))
	}

	var err error
	var deviceFields []*pb.CRSFDeviceFieldData

	if deviceFields, err = s.LinkCtl.GetCRSFDeviceFields(deviceInfo); err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("could not get device fields. %s", err.Error()))
	}

	for _, field := range deviceFields {
		FixOptionsArrows(field)
	}

	res := pb.GetCRSFDeviceFieldsRes{
		Fields: deviceFields,
	}

	return &res, nil
}

func (s *GRPCServer) GetCRSFDeviceField(_ context.Context, req *pb.GetCRSFDeviceFieldReq) (*pb.GetCRSFDeviceFieldRes, error) {

	if req == nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("request payload required"))
	}

	deviceInfo := req.GetDevice()

	if deviceInfo == nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("device is required"))
	}

	var err error
	var deviceField *pb.CRSFDeviceFieldData

	if deviceField, err = s.LinkCtl.GetCRSFDeviceField(deviceInfo, req.GetFieldId(), time.Second); err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("could get device field. %s", err.Error()))
	}

	if deviceField == nil {
		return nil, status.Error(codes.NotFound, fmt.Sprintf("could not get device field. %s", err.Error()))
	}

	FixOptionsArrows(deviceField)

	res := pb.GetCRSFDeviceFieldRes{
		Field: deviceField,
	}

	return &res, nil
}

func (s *GRPCServer) SetCRSFDeviceField(_ context.Context, req *pb.SetCRSFDeviceFieldReq) (*pb.SetCRSFDeviceFieldRes, error) {
	if req == nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("request payload required"))
	}

	var err error
	var deviceField *pb.CRSFDeviceFieldData

	if deviceField, err = s.LinkCtl.SetCRSFDeviceField(req.GetDevice(), req.GetField()); err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("could not set device field. %s", err.Error()))
	}

	res := pb.SetCRSFDeviceFieldRes{
		Field: deviceField,
	}

	return &res, nil
}

func (s *GRPCServer) GetCRSFDeviceLinkStatus(_ context.Context, _ *pb.Empty) (*pb.GetCRSFDeviceLinkStatusRes, error) {

	var err error
	var deviceLinkStatus *pb.CRSFDeviceLinkStatusData

	if deviceLinkStatus, err = s.LinkCtl.GetCRSFDeviceLinkStatus(5 * time.Second); err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("could not get device link status. %s", err.Error()))
	}

	res := pb.GetCRSFDeviceLinkStatusRes{
		LinkStatus: deviceLinkStatus,
	}

	return &res, nil
}

func (s *GRPCServer) ClearCRSFDeviceLinkCriticalFlags(_ context.Context, _ *pb.Empty) (*pb.Empty, error) {

	var err error

	if err = s.LinkCtl.ClearCRSFDeviceLinkCriticalFlags(); err != nil {
		return nil, status.Error(codes.InvalidArgument, fmt.Sprintf("could not clear device link critical flags. %s", err.Error()))
	}

	return &pb.Empty{}, nil
}
