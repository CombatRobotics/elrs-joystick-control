// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package server

import (
	"fmt"
	pb2 "github.com/kaack/elrs-joystick-control/pkg/proto/generated/pb"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/status"
)

func (s *GRPCServer) StreamLinkState(state *pb2.LinkState, server pb2.JoystickControl_GetLinkStreamServer) error {
	var err error

	state = s.LinkCtl.GetLinkState(state)

	if err = server.Send(state); err != nil {
		return status.Error(codes.Aborted, fmt.Sprintf("error sending link state. %s", err.Error()))
	}

	return nil
}
