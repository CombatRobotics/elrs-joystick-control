// SPDX-FileCopyrightText: © 2023 ZhouYixun 291028775@qq.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package client

import (
	"context"
	"fmt"
	"github.com/kaack/elrs-joystick-control/pkg/proto/generated/pb"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
	"time"
)

func Init(txServerPortName string, txServerPortBaudRate, grpcPort int, disableWebUI bool) {
	if (len(txServerPortName) != 0 && txServerPortBaudRate != 0) || disableWebUI {
		ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
		defer cancel()

		var err error
		var conn *grpc.ClientConn
		if conn, err = grpc.DialContext(ctx, fmt.Sprintf("localhost:%d", grpcPort), grpc.WithTransportCredentials(insecure.NewCredentials())); err != nil {
			panic(err)
		}
		defer conn.Close()

		client := pb.NewJoystickControlClient(conn)
		var res *pb.Empty

		if len(txServerPortName) != 0 && txServerPortBaudRate != 0 {
			if res, err = client.StartLink(ctx, &pb.StartLinkReq{
				Port:     txServerPortName,
				BaudRate: int32(txServerPortBaudRate),
			}); err != nil {
				panic(err)
			}

			fmt.Printf("%v", res)
		}

		if disableWebUI {
			if res, err = client.StopHTTP(ctx, &pb.Empty{}); err != nil {
				panic(err)
			}

			fmt.Printf("%v", res)
		}
	}
}
