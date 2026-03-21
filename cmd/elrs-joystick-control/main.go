// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package main

import (
	"bufio"
	"context"
	"errors"
	"fmt"
	crsf "github.com/kaack/elrs-joystick-control/pkg/crossfire"
	"github.com/kaack/elrs-joystick-control/pkg/util"
	"go.bug.st/serial"
	"gopkg.in/yaml.v3"
	"io"
	"os"
	"os/exec"
	"os/signal"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	configFilePath  = "config.yaml"
	minChannelIndex = 0
	maxChannelIndex = 15
	absoluteCRSFMax = 2047
	minModelID      = 0
	maxModelID      = 63
)

type SerialConfig struct {
	TXPortName string `yaml:"tx_port_name"`
	TXBaudRate int    `yaml:"tx_baud_rate"`
}

type ROS2Config struct {
	TopicName string `yaml:"topic_name"`
}

type LinkConfig struct {
	ChannelSendPeriodMS int `yaml:"channel_send_period_ms"`
}

type ModelMatchConfig struct {
	ModelID int `yaml:"model_id"`
}

type MappingConfig struct {
	LeftRPMChannel       int `yaml:"left_rpm_channel"`
	RightRPMChannel      int `yaml:"right_rpm_channel"`
	OtherChannelsDefault int `yaml:"other_channels_default"`
}

type LimitsConfig struct {
	CRSFMin int `yaml:"crsf_min"`
	CRSFMax int `yaml:"crsf_max"`
}

type LoggingConfig struct {
	ROS2RX           bool `yaml:"ros2_rx"`
	TXWrites         bool `yaml:"tx_writes"`
	SubscriberErrors bool `yaml:"subscriber_errors"`
}

type Config struct {
	Serial     SerialConfig     `yaml:"serial"`
	ROS2       ROS2Config       `yaml:"ros2"`
	Link       LinkConfig       `yaml:"link"`
	ModelMatch ModelMatchConfig `yaml:"model_match"`
	Mapping    MappingConfig    `yaml:"mapping"`
	Limits     LimitsConfig     `yaml:"limits"`
	Logging    LoggingConfig    `yaml:"logging"`
}

func defaultConfig() Config {
	return Config{
		Serial: SerialConfig{
			TXPortName: "/dev/ttyUSB0",
			TXBaudRate: 400000,
		},
		ROS2: ROS2Config{
			TopicName: "/WheelRPM",
		},
		Link: LinkConfig{
			ChannelSendPeriodMS: 10,
		},
		ModelMatch: ModelMatchConfig{
			ModelID: 0,
		},
		Mapping: MappingConfig{
			LeftRPMChannel:       0,
			RightRPMChannel:      1,
			OtherChannelsDefault: 0,
		},
		Limits: LimitsConfig{
			CRSFMin: 0,
			CRSFMax: 2047,
		},
		Logging: LoggingConfig{
			ROS2RX:           true,
			TXWrites:         true,
			SubscriberErrors: true,
		},
	}
}

func loadConfig(path string) (Config, error) {
	cfg := defaultConfig()
	data, err := os.ReadFile(path)
	if err != nil {
		return cfg, fmt.Errorf("could not read %s: %w", path, err)
	}

	if err = yaml.Unmarshal(data, &cfg); err != nil {
		return cfg, fmt.Errorf("could not parse YAML config %s: %w", path, err)
	}

	if err = cfg.validate(); err != nil {
		return cfg, err
	}
	return cfg, nil
}

func (c *Config) validate() error {
	c.Serial.TXPortName = strings.TrimSpace(c.Serial.TXPortName)
	if c.Serial.TXPortName == "" {
		return errors.New("serial.tx_port_name is required")
	}
	if c.Serial.TXBaudRate <= 0 {
		return fmt.Errorf("serial.tx_baud_rate must be > 0, got %d", c.Serial.TXBaudRate)
	}

	c.ROS2.TopicName = strings.TrimSpace(c.ROS2.TopicName)
	if c.ROS2.TopicName == "" {
		return errors.New("ros2.topic_name is required")
	}

	if c.Link.ChannelSendPeriodMS <= 0 {
		return fmt.Errorf("link.channel_send_period_ms must be > 0, got %d", c.Link.ChannelSendPeriodMS)
	}

	if c.ModelMatch.ModelID < minModelID || c.ModelMatch.ModelID > maxModelID {
		return fmt.Errorf("model_match.model_id must be in [%d..%d], got %d", minModelID, maxModelID, c.ModelMatch.ModelID)
	}

	if c.Mapping.LeftRPMChannel < minChannelIndex || c.Mapping.LeftRPMChannel > maxChannelIndex {
		return fmt.Errorf("mapping.left_rpm_channel must be in [%d..%d], got %d", minChannelIndex, maxChannelIndex, c.Mapping.LeftRPMChannel)
	}
	if c.Mapping.RightRPMChannel < minChannelIndex || c.Mapping.RightRPMChannel > maxChannelIndex {
		return fmt.Errorf("mapping.right_rpm_channel must be in [%d..%d], got %d", minChannelIndex, maxChannelIndex, c.Mapping.RightRPMChannel)
	}
	if c.Mapping.LeftRPMChannel == c.Mapping.RightRPMChannel {
		return fmt.Errorf("mapping.left_rpm_channel and mapping.right_rpm_channel must be different, both are %d", c.Mapping.LeftRPMChannel)
	}

	if c.Limits.CRSFMin < 0 || c.Limits.CRSFMin > absoluteCRSFMax {
		return fmt.Errorf("limits.crsf_min must be in [0..%d], got %d", absoluteCRSFMax, c.Limits.CRSFMin)
	}
	if c.Limits.CRSFMax < 0 || c.Limits.CRSFMax > absoluteCRSFMax {
		return fmt.Errorf("limits.crsf_max must be in [0..%d], got %d", absoluteCRSFMax, c.Limits.CRSFMax)
	}
	if c.Limits.CRSFMin > c.Limits.CRSFMax {
		return fmt.Errorf("limits.crsf_min (%d) cannot be greater than limits.crsf_max (%d)", c.Limits.CRSFMin, c.Limits.CRSFMax)
	}

	if c.Mapping.OtherChannelsDefault < c.Limits.CRSFMin || c.Mapping.OtherChannelsDefault > c.Limits.CRSFMax {
		return fmt.Errorf("mapping.other_channels_default must be in [%d..%d], got %d", c.Limits.CRSFMin, c.Limits.CRSFMax, c.Mapping.OtherChannelsDefault)
	}

	return nil
}

type wheelState struct {
	mu sync.RWMutex

	messageCount    uint64
	parseErrorCount uint64
	lastRawLeftRPM  int32
	lastRawRightRPM int32
	lastLeftCRSF    util.CRSFValue
	lastRightCRSF   util.CRSFValue
	lastRecvAt      time.Time
	lastError       string
}

func (w *wheelState) update(rawLeft int32, rawRight int32, minValue util.CRSFValue, maxValue util.CRSFValue) (util.CRSFValue, util.CRSFValue) {
	left := capToCRSFValue(rawLeft, minValue, maxValue)
	right := capToCRSFValue(rawRight, minValue, maxValue)

	w.mu.Lock()
	defer w.mu.Unlock()
	w.messageCount += 1
	w.lastRawLeftRPM = rawLeft
	w.lastRawRightRPM = rawRight
	w.lastLeftCRSF = left
	w.lastRightCRSF = right
	w.lastRecvAt = time.Now()
	return left, right
}

func (w *wheelState) parseError(err error, line string) {
	w.mu.Lock()
	defer w.mu.Unlock()
	w.parseErrorCount += 1
	w.lastError = fmt.Sprintf("line=%q err=%v", line, err)
}

func (w *wheelState) setError(err string) {
	w.mu.Lock()
	defer w.mu.Unlock()
	w.lastError = err
}

func (w *wheelState) snapshot() (util.CRSFValue, util.CRSFValue) {
	w.mu.RLock()
	defer w.mu.RUnlock()
	return w.lastLeftCRSF, w.lastRightCRSF
}

func (w *wheelState) debugString(topic string) string {
	w.mu.RLock()
	defer w.mu.RUnlock()
	return fmt.Sprintf(
		"topic=%s msgs=%d parse_errors=%d last_raw_left=%d last_raw_right=%d last_left=%d last_right=%d last_recv=%s last_err=%s",
		topic,
		w.messageCount,
		w.parseErrorCount,
		w.lastRawLeftRPM,
		w.lastRawRightRPM,
		w.lastLeftCRSF,
		w.lastRightCRSF,
		w.lastRecvAt.Format(time.RFC3339Nano),
		w.lastError,
	)
}

func capToCRSFValue(raw int32, minValue util.CRSFValue, maxValue util.CRSFValue) util.CRSFValue {
	if raw < int32(minValue) {
		return minValue
	}
	if raw > int32(maxValue) {
		return maxValue
	}
	return util.CRSFValue(raw)
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

func consumeROS2Stderr(ctx context.Context, reader io.Reader, state *wheelState, logErrors bool) {
	scanner := bufio.NewScanner(reader)
	for scanner.Scan() {
		select {
		case <-ctx.Done():
			return
		default:
		}
		line := strings.TrimSpace(scanner.Text())
		if line == "" {
			continue
		}
		state.setError(line)
		if logErrors {
			fmt.Printf("(ros2)(stderr) %s\n", line)
		}
	}
}

func consumeROS2Stdout(
	ctx context.Context,
	reader io.Reader,
	state *wheelState,
	minValue util.CRSFValue,
	maxValue util.CRSFValue,
	logRX bool,
	logErrors bool,
) error {
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
			state.parseError(err, line)
			if logErrors {
				fmt.Printf("(ros2) incomplete WheelRPM message ignored (%s)\n", line)
			}
			haveLeft = false
			haveRight = false
			return
		}

		left, right := state.update(rawLeftRPM, rawRightRPM, minValue, maxValue)
		if logRX {
			fmt.Printf("(ros2) WheelRPM raw left=%d right=%d capped left=%d right=%d\n", rawLeftRPM, rawRightRPM, left, right)
		}
		haveLeft = false
		haveRight = false
	}

	for scanner.Scan() {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

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
				state.parseError(err, line)
				if logErrors {
					fmt.Printf("(ros2) parse error on left_rpm line=%q err=%s\n", line, err.Error())
				}
				continue
			}
			rawLeftRPM = value
			haveLeft = true
		case strings.HasPrefix(line, "right_rpm:"):
			value, err := parseROS2FieldInt32("right_rpm:", line)
			if err != nil {
				state.parseError(err, line)
				if logErrors {
					fmt.Printf("(ros2) parse error on right_rpm line=%q err=%s\n", line, err.Error())
				}
				continue
			}
			rawRightRPM = value
			haveRight = true
		default:
			// ignore header and unknown fields
		}
	}

	flushMessage()
	return scanner.Err()
}

func runROS2Subscriber(
	ctx context.Context,
	topic string,
	state *wheelState,
	minValue util.CRSFValue,
	maxValue util.CRSFValue,
	logCfg LoggingConfig,
) error {
	if logCfg.SubscriberErrors {
		fmt.Printf("(ros2) subscriber loop starting (topic=%s)\n", topic)
		defer fmt.Printf("(ros2) subscriber loop exiting (%s)\n", state.debugString(topic))
	}

	restartDelay := time.Second

	for {
		select {
		case <-ctx.Done():
			return nil
		default:
		}

		cmdCtx, cancel := context.WithCancel(ctx)
		cmd := exec.CommandContext(cmdCtx, "ros2", "topic", "echo", topic)

		stdout, err := cmd.StdoutPipe()
		if err != nil {
			cancel()
			state.setError(err.Error())
			if logCfg.SubscriberErrors {
				fmt.Printf("(ros2) could not capture stdout for topic %s. %s\n", topic, err.Error())
			}
		} else {
			stderr, err := cmd.StderrPipe()
			if err != nil {
				cancel()
				state.setError(err.Error())
				if logCfg.SubscriberErrors {
					fmt.Printf("(ros2) could not capture stderr for topic %s. %s\n", topic, err.Error())
				}
			} else {
				if err = cmd.Start(); err != nil {
					cancel()
					state.setError(err.Error())
					if logCfg.SubscriberErrors {
						fmt.Printf("(ros2) could not start ros2 subscriber command for topic %s. %s\n", topic, err.Error())
					}
				} else {
					if logCfg.SubscriberErrors {
						fmt.Printf("(ros2) subscriber process started (topic=%s pid=%d)\n", topic, cmd.Process.Pid)
					}

					go consumeROS2Stderr(cmdCtx, stderr, state, logCfg.SubscriberErrors)
					readErr := consumeROS2Stdout(cmdCtx, stdout, state, minValue, maxValue, logCfg.ROS2RX, logCfg.SubscriberErrors)
					waitErr := cmd.Wait()
					cancel()

					if readErr != nil {
						state.setError(readErr.Error())
						if logCfg.SubscriberErrors {
							fmt.Printf("(ros2) subscriber stdout read error. %s\n", readErr.Error())
						}
					}
					if waitErr != nil {
						state.setError(waitErr.Error())
						if logCfg.SubscriberErrors {
							fmt.Printf("(ros2) subscriber command exited with error. %s\n", waitErr.Error())
						}
					}
				}
			}
		}

		select {
		case <-ctx.Done():
			return nil
		case <-time.After(restartDelay):
			if logCfg.SubscriberErrors {
				fmt.Printf("(ros2) subscriber restarting in %s\n", restartDelay)
			}
		}

		if restartDelay < 5*time.Second {
			restartDelay *= 2
		}
	}
}

func sendModelIDFrame(port serial.Port, modelID uint8, source string, logWrites bool) error {
	frame := crsf.CreateModelIDFrame(modelID)
	written, err := port.Write(frame)
	if err != nil {
		return fmt.Errorf("(model-id) write failed source=%s model_id=%d err=%w", source, modelID, err)
	}

	if logWrites {
		fmt.Printf(
			"[MODEL_MATCH_TX_PACKET] time=%s source=%s model_id=%d bytes_written=%d packet=% X\n",
			time.Now().Format(time.RFC3339Nano),
			source,
			modelID,
			written,
			frame,
		)
	}
	return nil
}

func main() {
	cfg, err := loadConfig(configFilePath)
	if err != nil {
		fmt.Printf("(app) failed to load config. %s\n", err.Error())
		os.Exit(1)
	}

	minValue := util.CRSFValue(cfg.Limits.CRSFMin)
	maxValue := util.CRSFValue(cfg.Limits.CRSFMax)
	otherDefault := util.CRSFValue(cfg.Mapping.OtherChannelsDefault)

	fmt.Printf("(app) loaded config=%s\n", configFilePath)
	fmt.Printf(
		"(app) starting ROS2->CRSF pipeline port=%s baud=%d model_id=%d topic=%s period_ms=%d left_ch=%d right_ch=%d other_default=%d limits=[%d..%d]\n",
		cfg.Serial.TXPortName,
		cfg.Serial.TXBaudRate,
		cfg.ModelMatch.ModelID,
		cfg.ROS2.TopicName,
		cfg.Link.ChannelSendPeriodMS,
		cfg.Mapping.LeftRPMChannel,
		cfg.Mapping.RightRPMChannel,
		cfg.Mapping.OtherChannelsDefault,
		cfg.Limits.CRSFMin,
		cfg.Limits.CRSFMax,
	)

	serialPort, err := serial.Open(cfg.Serial.TXPortName, &serial.Mode{
		BaudRate: cfg.Serial.TXBaudRate,
		Parity:   serial.NoParity,
		DataBits: 8,
		StopBits: serial.OneStopBit,
	})
	if err != nil {
		fmt.Printf("(app) failed to open serial port %s. %s\n", cfg.Serial.TXPortName, err.Error())
		os.Exit(1)
	}
	defer func() {
		if closeErr := serialPort.Close(); closeErr != nil {
			fmt.Printf("(app) serial close error on %s. %s\n", cfg.Serial.TXPortName, closeErr.Error())
		}
	}()
	fmt.Printf("(app) serial port opened %s @ %d baud\n", cfg.Serial.TXPortName, cfg.Serial.TXBaudRate)

	if err = sendModelIDFrame(serialPort, uint8(cfg.ModelMatch.ModelID), "startup", cfg.Logging.TXWrites); err != nil {
		fmt.Printf("%s\n", err.Error())
	}

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt)
	defer cancel()

	state := &wheelState{}

	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		if subErr := runROS2Subscriber(ctx, cfg.ROS2.TopicName, state, minValue, maxValue, cfg.Logging); subErr != nil {
			fmt.Printf("(ros2) subscriber fatal error. %s\n", subErr.Error())
			cancel()
		}
	}()

	channelTicker := time.NewTicker(time.Duration(cfg.Link.ChannelSendPeriodMS) * time.Millisecond)
	defer channelTicker.Stop()

	for {
		select {
		case <-ctx.Done():
			fmt.Printf("(app) shutdown requested\n")
			wg.Wait()
			fmt.Printf("(app) shutdown complete\n")
			return
		case <-channelTicker.C:
			left, right := state.snapshot()

			channels := [16]util.CRSFValue{}
			for idx := range channels {
				channels[idx] = otherDefault
			}
			channels[cfg.Mapping.LeftRPMChannel] = left
			channels[cfg.Mapping.RightRPMChannel] = right

			if _, err = serialPort.Write(crsf.PackChannels(&channels)); err != nil {
				fmt.Printf("(send-loop) could not write channels on port %s. %s\n", cfg.Serial.TXPortName, err.Error())
				cancel()
				continue
			}
			if cfg.Logging.TXWrites {
				fmt.Printf(
					"(send-loop) written left=%d(ch=%d) right=%d(ch=%d) port=%s baud=%d\n",
					left,
					cfg.Mapping.LeftRPMChannel,
					right,
					cfg.Mapping.RightRPMChannel,
					cfg.Serial.TXPortName,
					cfg.Serial.TXBaudRate,
				)
			}
		}
	}
}
