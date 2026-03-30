// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package main

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	crsf "github.com/kaack/elrs-joystick-control/pkg/crossfire"
	"github.com/kaack/elrs-joystick-control/pkg/util"
	"go.bug.st/serial"
	"gopkg.in/yaml.v3"
	"io"
	"net"
	"os"
	"os/exec"
	"os/signal"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	configFilePath           = "config.yaml"
	defaultControlSocketPath = "/tmp/tota_elrs_bridge_model_id.sock"
	minChannelIndex          = 0
	maxChannelIndex          = 15
	absoluteCRSFMax          = 2047
	minModelID               = 0
	maxModelID               = 63
	modelIDRetries           = 5
)

const modelIDRetryDelay = 250 * time.Millisecond

const (
	initConfigRetries   = 3
	initConfigSendDelay = 50 * time.Millisecond
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
	ScalingFactor        int `yaml:"scaling_factor"`
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

type modelIDChangeRequest struct {
	ModelID  int
	Source   string
	Response chan error
}

type setModelIDIPCRequest struct {
	ModelID int `json:"model_id"`
}

type setModelIDIPCResponse struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
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
			ScalingFactor:        1,
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
	if c.Mapping.ScalingFactor <= 0 {
		return fmt.Errorf("mapping.scaling_factor must be > 0, got %d", c.Mapping.ScalingFactor)
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
	lastRawValues   map[string]int32
	lastCRSFValues  map[string]util.CRSFValue
	lastRecvAt      time.Time
	lastError       string
}

func newWheelState() *wheelState {
	return &wheelState{
		lastRawValues:  make(map[string]int32),
		lastCRSFValues: make(map[string]util.CRSFValue),
	}
}

func (w *wheelState) updateFields(rawValues map[string]int32, scalingFactor int, minValue util.CRSFValue, maxValue util.CRSFValue) map[string]util.CRSFValue {
	capped := make(map[string]util.CRSFValue, len(rawValues))
	for field, raw := range rawValues {
		capped[field] = capToCRSFValue(raw, scalingFactor, minValue, maxValue)
	}

	w.mu.Lock()
	defer w.mu.Unlock()

	w.messageCount += 1
	w.lastRecvAt = time.Now()
	for field, raw := range rawValues {
		w.lastRawValues[field] = raw
	}
	for field, value := range capped {
		w.lastCRSFValues[field] = value
	}

	return capped
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

func (w *wheelState) snapshotFieldValue(field string) (util.CRSFValue, bool) {
	w.mu.RLock()
	defer w.mu.RUnlock()
	value, ok := w.lastCRSFValues[field]
	return value, ok
}

func (w *wheelState) debugString(topic string) string {
	w.mu.RLock()
	defer w.mu.RUnlock()
	fields := make([]string, 0, len(w.lastCRSFValues))
	for field, value := range w.lastCRSFValues {
		fields = append(fields, fmt.Sprintf("%s=%d", field, value))
	}
	sort.Strings(fields)
	return fmt.Sprintf(
		"topic=%s msgs=%d parse_errors=%d last_fields=[%s] last_recv=%s last_err=%s",
		topic,
		w.messageCount,
		w.parseErrorCount,
		strings.Join(fields, ","),
		w.lastRecvAt.Format(time.RFC3339Nano),
		w.lastError,
	)
}

type channelRoutes map[string]int

var crsfChannelRoutes = channelRoutes{}

func set_channel(field string, channel int) error {
	field = strings.TrimSpace(field)
	if field == "" {
		return errors.New("field name is required")
	}
	if channel < minChannelIndex || channel > maxChannelIndex {
		return fmt.Errorf("channel must be in [%d..%d], got %d", minChannelIndex, maxChannelIndex, channel)
	}

	for existingField, existingChannel := range crsfChannelRoutes {
		if existingField != field && existingChannel == channel {
			return fmt.Errorf("channel %d is already assigned to field %q", channel, existingField)
		}
	}
	crsfChannelRoutes[field] = channel
	return nil
}

func formatRouteSummary(routes channelRoutes) string {
	if len(routes) == 0 {
		return ""
	}
	parts := make([]string, 0, len(routes))
	for field, channel := range routes {
		parts = append(parts, fmt.Sprintf("%s->ch%d", field, channel))
	}
	sort.Strings(parts)
	return strings.Join(parts, ", ")
}

func capToCRSFValue(raw int32, scalingFactor int, minValue util.CRSFValue, maxValue util.CRSFValue) util.CRSFValue {
	scaled := int64(raw) * int64(scalingFactor)
	if scaled < int64(minValue) {
		return minValue
	}
	if scaled > int64(maxValue) {
		return maxValue
	}
	return util.CRSFValue(scaled)
}

func parseROS2FieldLine(line string) (string, string, bool) {
	index := strings.Index(line, ":")
	if index < 0 {
		return "", "", false
	}
	field := strings.TrimSpace(line[:index])
	rawValue := strings.TrimSpace(line[index+1:])
	if field == "" || rawValue == "" {
		return "", "", false
	}
	return field, rawValue, true
}

func clearRawFieldMap(values map[string]int32) {
	for field := range values {
		delete(values, field)
	}
}

func formatRoutedUpdate(rawValues map[string]int32, cappedValues map[string]util.CRSFValue, routes channelRoutes) string {
	if len(rawValues) == 0 {
		return ""
	}
	parts := make([]string, 0, len(rawValues))
	for field, raw := range rawValues {
		channel, ok := routes[field]
		if !ok {
			continue
		}
		parts = append(parts, fmt.Sprintf("%s(raw=%d,capped=%d,ch=%d)", field, raw, cappedValues[field], channel))
	}
	sort.Strings(parts)
	return strings.Join(parts, " ")
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
	scalingFactor int,
	minValue util.CRSFValue,
	maxValue util.CRSFValue,
	routes channelRoutes,
	logRX bool,
	logErrors bool,
) error {
	scanner := bufio.NewScanner(reader)
	rawFields := make(map[string]int32, len(routes))

	flushMessage := func() {
		if len(rawFields) == 0 {
			return
		}
		capped := state.updateFields(rawFields, scalingFactor, minValue, maxValue)
		if logRX {
			fmt.Printf("(ros2) routed update %s\n", formatRoutedUpdate(rawFields, capped, routes))
		}
		clearRawFieldMap(rawFields)
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
		default:
			field, rawValue, ok := parseROS2FieldLine(line)
			if !ok {
				continue
			}
			if _, routed := routes[field]; !routed {
				continue
			}

			value, err := strconv.ParseInt(rawValue, 10, 32)
			if err != nil {
				parseErr := fmt.Errorf("field %s expects int32 value: %w", field, err)
				state.parseError(parseErr, line)
				if logErrors {
					fmt.Printf("(ros2) parse error on %s line=%q err=%s\n", field, line, parseErr.Error())
				}
				continue
			}
			rawFields[field] = int32(value)
		}
	}

	flushMessage()
	return scanner.Err()
}

func runROS2Subscriber(
	ctx context.Context,
	topic string,
	state *wheelState,
	scalingFactor int,
	minValue util.CRSFValue,
	maxValue util.CRSFValue,
	routes channelRoutes,
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
					readErr := consumeROS2Stdout(cmdCtx, stdout, state, scalingFactor, minValue, maxValue, routes, logCfg.ROS2RX, logCfg.SubscriberErrors)
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

type namedInitPacket struct {
	name  string
	bytes []byte
}

func sendRawInitPacket(port serial.Port, packet namedInitPacket, attempt int, retries int, logWrites bool) error {
	written, err := port.Write(packet.bytes)
	if err != nil {
		return fmt.Errorf("(init-config) write failed packet=%s attempt=%d/%d err=%w", packet.name, attempt, retries, err)
	}

	if logWrites {
		fmt.Printf("[ELRS_INIT_PACKET] time=%s packet=%s attempt=%d/%d bytes_written=%d payload=% X\n",
			time.Now().Format(time.RFC3339Nano),
			packet.name,
			attempt,
			retries,
			written,
			packet.bytes,
		)
	}
	return nil
}

// sendELRSInitConfigSequence sends exact raw ELRS setup packets after Model ID write.
// Each packet is sent 3 times with 50ms delay to improve reliability over one-way serial links.
func sendELRSInitConfigSequence(port serial.Port, logWrites bool) error {
	sequence := []namedInitPacket{
		{name: "PR100F", bytes: crsf.CreatePR100FFrame()},
		{name: "TLMOff", bytes: crsf.CreateTLMOffFrame()},
		{name: "SW8CH", bytes: crsf.CreateSW8CHFrame()},
		{name: "LMNorm", bytes: crsf.CreateLMNormFrame()},
		{name: "MMOn", bytes: crsf.CreateMMOnFrame()},
	}

	for _, pkt := range sequence {
		success := 0
		var lastErr error
		for attempt := 1; attempt <= initConfigRetries; attempt++ {
			err := sendRawInitPacket(port, pkt, attempt, initConfigRetries, logWrites)
			if err != nil {
				lastErr = err
				fmt.Printf("%s\n", err.Error())
			} else {
				success++
			}
			time.Sleep(initConfigSendDelay)
		}

		if success == 0 {
			return fmt.Errorf("(init-config) all attempts failed packet=%s last_error=%w", pkt.name, lastErr)
		}
		fmt.Printf("(init-config) packet complete name=%s success=%d/%d\n", pkt.name, success, initConfigRetries)
	}

	return nil
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

func sendModelIDFrameWithRetry(port serial.Port, modelID uint8, retries int, delay time.Duration, logWrites bool) error {
	if retries < 1 {
		retries = 1
	}

	successCount := 0
	var lastErr error

	for attempt := 1; attempt <= retries; attempt++ {
		source := fmt.Sprintf("startup burst=%d/%d", attempt, retries)
		err := sendModelIDFrame(port, modelID, source, logWrites)
		if err != nil {
			lastErr = err
			fmt.Printf("(model-id) burst attempt=%d/%d failed model_id=%d err=%s\n", attempt, retries, modelID, err.Error())
		} else {
			successCount++
		}

		if attempt < retries {
			time.Sleep(delay)
		}
	}

	if successCount == 0 {
		return fmt.Errorf("(model-id) all burst attempts failed model_id=%d attempts=%d last_error=%w", modelID, retries, lastErr)
	}

	fmt.Printf("(model-id) startup burst complete model_id=%d success=%d/%d\n", modelID, successCount, retries)
	return nil
}

func controlSocketPath() string {
	path := strings.TrimSpace(os.Getenv("TOTA_ELRS_MODEL_ID_SOCKET"))
	if path == "" {
		return defaultControlSocketPath
	}
	return path
}

func writeSetModelIDIPCResponse(conn net.Conn, response setModelIDIPCResponse) {
	if err := json.NewEncoder(conn).Encode(response); err != nil {
		fmt.Printf("(model-id-ipc) failed to write response. %s\n", err.Error())
	}
}

func handleSetModelIDConn(ctx context.Context, conn net.Conn, modelIDRequests chan<- modelIDChangeRequest) {
	defer conn.Close()

	var request setModelIDIPCRequest
	if err := json.NewDecoder(conn).Decode(&request); err != nil {
		writeSetModelIDIPCResponse(conn, setModelIDIPCResponse{
			Success: false,
			Message: fmt.Sprintf("invalid request payload: %s", err.Error()),
		})
		return
	}

	if request.ModelID < minModelID || request.ModelID > maxModelID {
		writeSetModelIDIPCResponse(conn, setModelIDIPCResponse{
			Success: false,
			Message: fmt.Sprintf("model_id must be in [%d..%d], got %d", minModelID, maxModelID, request.ModelID),
		})
		return
	}

	resultCh := make(chan error, 1)
	changeRequest := modelIDChangeRequest{
		ModelID:  request.ModelID,
		Source:   "ros2 service",
		Response: resultCh,
	}

	select {
	case modelIDRequests <- changeRequest:
	case <-ctx.Done():
		writeSetModelIDIPCResponse(conn, setModelIDIPCResponse{
			Success: false,
			Message: "bridge is shutting down",
		})
		return
	}

	select {
	case writeErr := <-resultCh:
		if writeErr != nil {
			writeSetModelIDIPCResponse(conn, setModelIDIPCResponse{
				Success: false,
				Message: writeErr.Error(),
			})
			return
		}
		writeSetModelIDIPCResponse(conn, setModelIDIPCResponse{
			Success: true,
			Message: fmt.Sprintf("model_id packet sent: %d", request.ModelID),
		})
	case <-ctx.Done():
		writeSetModelIDIPCResponse(conn, setModelIDIPCResponse{
			Success: false,
			Message: "bridge is shutting down",
		})
	}
}

func startModelIDIPCServer(ctx context.Context, socketPath string, modelIDRequests chan<- modelIDChangeRequest) (func(), error) {
	if err := os.Remove(socketPath); err != nil && !errors.Is(err, os.ErrNotExist) {
		return nil, fmt.Errorf("could not clear stale model-id socket %s: %w", socketPath, err)
	}

	listener, err := net.Listen("unix", socketPath)
	if err != nil {
		return nil, fmt.Errorf("could not listen on model-id socket %s: %w", socketPath, err)
	}

	cleanup := func() {
		_ = listener.Close()
		if err := os.Remove(socketPath); err != nil && !errors.Is(err, os.ErrNotExist) {
			fmt.Printf("(model-id-ipc) could not remove socket %s. %s\n", socketPath, err.Error())
		}
	}

	go func() {
		<-ctx.Done()
		cleanup()
	}()

	go func() {
		for {
			conn, acceptErr := listener.Accept()
			if acceptErr != nil {
				if ctx.Err() != nil || errors.Is(acceptErr, net.ErrClosed) {
					return
				}
				fmt.Printf("(model-id-ipc) accept failed on %s. %s\n", socketPath, acceptErr.Error())
				continue
			}

			go handleSetModelIDConn(ctx, conn, modelIDRequests)
		}
	}()

	fmt.Printf("(model-id-ipc) listening socket=%s\n", socketPath)
	return cleanup, nil
}

func main() {
	cfg, err := loadConfig(configFilePath)
	if err != nil {
		fmt.Printf("(app) failed to load config. %s\n", err.Error())
		os.Exit(1)
	}

	if err = set_channel("left_rpm", cfg.Mapping.LeftRPMChannel); err != nil {
		fmt.Printf("(app) invalid route for left_rpm. %s\n", err.Error())
		os.Exit(1)
	}
	if err = set_channel("right_rpm", cfg.Mapping.RightRPMChannel); err != nil {
		fmt.Printf("(app) invalid route for right_rpm. %s\n", err.Error())
		os.Exit(1)
	}
	// Add more field routes here. Example: set_channel("led_cmd", 4)

	minValue := util.CRSFValue(cfg.Limits.CRSFMin)
	maxValue := util.CRSFValue(cfg.Limits.CRSFMax)
	otherDefault := util.CRSFValue(cfg.Mapping.OtherChannelsDefault)
	fmt.Printf("(app) loaded config=%s\n", configFilePath)
	fmt.Printf(
		"(app) starting ROS2->CRSF pipeline port=%s baud=%d model_id=%d topic=%s period_ms=%d routes=%s scaling_factor=%d other_default=%d limits=[%d..%d]\n",
		cfg.Serial.TXPortName,
		cfg.Serial.TXBaudRate,
		cfg.ModelMatch.ModelID,
		cfg.ROS2.TopicName,
		cfg.Link.ChannelSendPeriodMS,
		formatRouteSummary(crsfChannelRoutes),
		cfg.Mapping.ScalingFactor,
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

	if err = sendModelIDFrameWithRetry(serialPort, uint8(cfg.ModelMatch.ModelID), modelIDRetries, modelIDRetryDelay, cfg.Logging.TXWrites); err != nil {
		fmt.Printf("%s\n", err.Error())
	}
	if err = sendELRSInitConfigSequence(serialPort, cfg.Logging.TXWrites); err != nil {
		fmt.Printf("%s\n", err.Error())
	}

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt)
	defer cancel()

	modelIDRequests := make(chan modelIDChangeRequest)
	socketPath := controlSocketPath()
	cleanupIPC, err := startModelIDIPCServer(ctx, socketPath, modelIDRequests)
	if err != nil {
		fmt.Printf("(model-id-ipc) startup failed. %s\n", err.Error())
		os.Exit(1)
	}
	defer cleanupIPC()

	state := newWheelState()

	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		if subErr := runROS2Subscriber(ctx, cfg.ROS2.TopicName, state, cfg.Mapping.ScalingFactor, minValue, maxValue, crsfChannelRoutes, cfg.Logging); subErr != nil {
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
		case request := <-modelIDRequests:
			writeErr := sendModelIDFrame(serialPort, uint8(request.ModelID), request.Source, cfg.Logging.TXWrites)
			if writeErr != nil {
				fmt.Printf("%s\n", writeErr.Error())
			}
			request.Response <- writeErr
		case <-channelTicker.C:
			channels := [16]util.CRSFValue{}
			for idx := range channels {
				channels[idx] = otherDefault
			}

			routedFields := make([]string, 0, len(crsfChannelRoutes))
			for field, channel := range crsfChannelRoutes {
				value, ok := state.snapshotFieldValue(field)
				if !ok {
					continue
				}
				channels[channel] = value
				routedFields = append(routedFields, fmt.Sprintf("%s=%d(ch=%d)", field, value, channel))
			}
			sort.Strings(routedFields)

			if _, err = serialPort.Write(crsf.PackChannels(&channels)); err != nil {
				fmt.Printf("(send-loop) could not write channels on port %s. %s\n", cfg.Serial.TXPortName, err.Error())
				cancel()
				continue
			}
			if cfg.Logging.TXWrites {
				fmt.Printf(
					"(send-loop) written routed=[%s] port=%s baud=%d\n",
					strings.Join(routedFields, " "),
					cfg.Serial.TXPortName,
					cfg.Serial.TXBaudRate,
				)
			}
		}
	}
}
