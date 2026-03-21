// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package link

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	"gopkg.in/tomb.v2"
	"io"
	"os/exec"
	"strconv"
	"strings"
	"time"
)

func (c *Controller) StartROS2Subscriber() error {
	if c.GetChannelSourceMode() != ChannelSourceROS2 {
		return nil
	}

	if c.ros2SubscriberTomb != nil && c.ros2SubscriberTomb.Alive() {
		return nil
	}

	c.ros2SubscriberTomb = &tomb.Tomb{}
	c.ros2SubscriberTomb.Go(func() error {
		return c.ROS2SubscriberLoop()
	})
	return nil
}

func (c *Controller) StopROS2Subscriber() error {
	if c.ros2SubscriberTomb == nil || !c.ros2SubscriberTomb.Alive() {
		return nil
	}

	c.ros2SubscriberTomb.Kill(nil)
	if err := c.ros2SubscriberTomb.Wait(); err != nil {
		return err
	}
	fmt.Printf("(ros2) subscriber stopped (%s)\n", c.GetROS2DebugString())
	return nil
}

func (c *Controller) ROS2SubscriberLoop() error {
	fmt.Printf("(ros2) subscriber loop starting (topic=%s)\n", c.GetROS2TopicName())
	defer fmt.Printf("(ros2) subscriber loop exiting\n")

	restartDelay := time.Second

Loop:
	for {
		select {
		case <-c.ros2SubscriberTomb.Dying():
			break Loop
		default:
		}

		topic := c.GetROS2TopicName()
		cmdCtx, cancel := context.WithCancel(context.Background())
		cmd := exec.CommandContext(cmdCtx, "ros2", "topic", "echo", topic)

		stdout, err := cmd.StdoutPipe()
		if err != nil {
			cancel()
			c.SetROS2LastError(err.Error())
			fmt.Printf("(ros2) could not capture stdout for topic %s. %s\n", topic, err.Error())
			if !c.waitROS2RestartDelay(restartDelay) {
				break Loop
			}
			continue
		}

		stderr, err := cmd.StderrPipe()
		if err != nil {
			cancel()
			c.SetROS2LastError(err.Error())
			fmt.Printf("(ros2) could not capture stderr for topic %s. %s\n", topic, err.Error())
			if !c.waitROS2RestartDelay(restartDelay) {
				break Loop
			}
			continue
		}

		if err = cmd.Start(); err != nil {
			cancel()
			c.SetROS2LastError(err.Error())
			fmt.Printf("(ros2) could not start ros2 subscriber command for topic %s. %s\n", topic, err.Error())
			if !c.waitROS2RestartDelay(restartDelay) {
				break Loop
			}
			continue
		}

		fmt.Printf("(ros2) subscriber process started (topic=%s pid=%d)\n", topic, cmd.Process.Pid)

		stopWatch := make(chan struct{})
		go func() {
			select {
			case <-c.ros2SubscriberTomb.Dying():
				cancel()
			case <-stopWatch:
			}
		}()

		go c.consumeROS2Stderr(stderr)
		readErr := c.consumeROS2Stdout(stdout)
		close(stopWatch)
		waitErr := cmd.Wait()
		cancel()

		if readErr != nil {
			c.SetROS2LastError(readErr.Error())
			fmt.Printf("(ros2) subscriber stdout read error. %s\n", readErr.Error())
		}
		if waitErr != nil {
			c.SetROS2LastError(waitErr.Error())
			fmt.Printf("(ros2) subscriber command exited with error. %s\n", waitErr.Error())
		}

		select {
		case <-c.ros2SubscriberTomb.Dying():
			break Loop
		default:
		}

		fmt.Printf("(ros2) subscriber restarting in %s\n", restartDelay)
		if !c.waitROS2RestartDelay(restartDelay) {
			break Loop
		}
		if restartDelay < 5*time.Second {
			restartDelay *= 2
		}
	}

	return nil
}

func (c *Controller) waitROS2RestartDelay(delay time.Duration) bool {
	select {
	case <-time.After(delay):
		return true
	case <-c.ros2SubscriberTomb.Dying():
		return false
	}
}

func (c *Controller) consumeROS2Stdout(reader io.Reader) error {
	scanner := bufio.NewScanner(reader)

	var rawLeftRPM int32
	var rawRightRPM int32
	var haveLeft bool
	var haveRight bool

	flushMessage := func() {
		if !haveLeft && !haveRight {
			return
		}
		if !haveLeft || !haveRight {
			err := errors.New("incomplete WheelRPM message")
			line := fmt.Sprintf("left_present=%v right_present=%v", haveLeft, haveRight)
			c.RecordROS2ParseError(err, line)
			fmt.Printf("(ros2) incomplete WheelRPM message ignored (%s)\n", line)
			haveLeft = false
			haveRight = false
			return
		}

		leftCRSF, rightCRSF := c.RecordROS2Message(rawLeftRPM, rawRightRPM)
		fmt.Printf("(ros2) WheelRPM raw left=%d right=%d capped left=%d right=%d\n", rawLeftRPM, rawRightRPM, leftCRSF, rightCRSF)
		haveLeft = false
		haveRight = false
	}

	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}

		switch {
		case line == "---":
			flushMessage()
		case strings.HasPrefix(line, "left_rpm:"):
			value, err := parseROS2FieldInt32("left_rpm:", line)
			if err != nil {
				c.RecordROS2ParseError(err, line)
				fmt.Printf("(ros2) parse error on left_rpm line=%q err=%s\n", line, err.Error())
				continue
			}
			rawLeftRPM = value
			haveLeft = true
		case strings.HasPrefix(line, "right_rpm:"):
			value, err := parseROS2FieldInt32("right_rpm:", line)
			if err != nil {
				c.RecordROS2ParseError(err, line)
				fmt.Printf("(ros2) parse error on right_rpm line=%q err=%s\n", line, err.Error())
				continue
			}
			rawRightRPM = value
			haveRight = true
		default:
			// ignore header and all other fields
		}
	}

	flushMessage()
	return scanner.Err()
}

func (c *Controller) consumeROS2Stderr(reader io.Reader) {
	scanner := bufio.NewScanner(reader)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		c.SetROS2LastError(line)
		fmt.Printf("(ros2)(stderr) %s\n", line)
	}
}

func parseROS2FieldInt32(prefix string, line string) (int32, error) {
	raw := strings.TrimSpace(strings.TrimPrefix(line, prefix))
	if raw == "" {
		return 0, errors.New("missing numeric value")
	}
	value, err := strconv.ParseInt(raw, 10, 32)
	if err != nil {
		return 0, err
	}
	return int32(value), nil
}
