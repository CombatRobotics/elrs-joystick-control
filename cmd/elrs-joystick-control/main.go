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
	telem "github.com/kaack/elrs-joystick-control/pkg/crossfire/telemetry"
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
	defaultChannelSocketPath = "/tmp/tota_elrs_bridge_channels.sock"
	minChannelIndex          = 0
	maxChannelIndex          = 15
	absoluteCRSFMax          = 2047
	minModelID               = 0
	maxModelID               = 63
)

// modelIDResendInterval is how often the desired model id is re-sent while
// the TX has not yet confirmed it (see the convergence state in the run loop).
const modelIDResendInterval = 1 * time.Second

// txSilentRestartAfter: if the TX module stays completely silent this long
// despite an open serial port, exit so the supervisor respawns us with a
// fresh serial session — the only remaining recovery lever for a wedged module.
const txSilentRestartAfter = 30 * time.Second

const (
	initConfigRetries   = 3
	initConfigSendDelay = 50 * time.Millisecond
)

const (
	// adminQuietWindow is how long to suppress channel-pack writes after an
	// admin frame (model-id change, status request) so the firmware's
	// half-duplex reply lands on a quiet line. The firmware's largest typical
	// reply at 400k baud is ~2ms; 4ms gives a safe margin at any baud.
	adminQuietWindow = 4 * time.Millisecond

	// recvReadTimeout is the per-Read timeout on the serial port, used by
	// the recv goroutine. Read returns periodically with no data so the
	// goroutine can notice ctx cancel.
	recvReadTimeout = 20 * time.Millisecond

	// txAliveTimeout is how long we wait without any inbound TX-originated
	// frame (sync, status, forwarded linkstats) before declaring the TX dead.
	txAliveTimeout = 2 * time.Second

	// txAliveCheckInterval is how often we poll for the tx-alive timeout above.
	txAliveCheckInterval = 500 * time.Millisecond

	// statusPollInterval is how often we send a PARAMETER_WRITE(0xEE, 0, 0)
	// to ask the firmware for an ELRS status reply (which carries the model-
	// match flag). 1s is plenty fresh and only one admin frame per second.
	statusPollInterval = 1 * time.Second

	// recvHeartbeatInterval throttles the (recv-loop) heartbeat log line.
	recvHeartbeatInterval = 1 * time.Second
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

type setChannelsIPCRequest struct {
	Channels []int `json:"channels"`
}

type channelFrameState struct {
	mu       sync.RWMutex
	channels [16]util.CRSFValue
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

func controlSocketPath() string {
	path := strings.TrimSpace(os.Getenv("TOTA_ELRS_MODEL_ID_SOCKET"))
	if path == "" {
		return defaultControlSocketPath
	}
	return path
}

func channelSocketPath() string {
	path := strings.TrimSpace(os.Getenv("TOTA_ELRS_CHANNEL_SOCKET"))
	if path == "" {
		return defaultChannelSocketPath
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

func newChannelFrameState(defaultValue util.CRSFValue) *channelFrameState {
	state := &channelFrameState{}
	state.setDisconnected(defaultValue)
	return state
}

func (s *channelFrameState) setDisconnected(defaultValue util.CRSFValue) {
	s.mu.Lock()
	defer s.mu.Unlock()
	for idx := range s.channels {
		s.channels[idx] = defaultValue
	}
}

func (s *channelFrameState) snapshot() [16]util.CRSFValue {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.channels
}

func (s *channelFrameState) setFromRequest(request setChannelsIPCRequest) error {
	if len(request.Channels) != 16 {
		return fmt.Errorf("channels payload must contain 16 values, got %d", len(request.Channels))
	}

	next := [16]util.CRSFValue{}
	for idx, value := range request.Channels {
		if value < 0 || value > absoluteCRSFMax {
			return fmt.Errorf("channels[%d] must be in [0..%d], got %d", idx, absoluteCRSFMax, value)
		}
		next[idx] = util.CRSFValue(value)
	}

	s.mu.Lock()
	defer s.mu.Unlock()
	s.channels = next
	return nil
}

func handleChannelFrameStreamConn(conn net.Conn, channelFrames *channelFrameState) error {
	defer conn.Close()

	decoder := json.NewDecoder(conn)
	for {
		var request setChannelsIPCRequest
		if err := decoder.Decode(&request); err != nil {
			if errors.Is(err, io.EOF) {
				return nil
			}
			return fmt.Errorf("decode failed: %w", err)
		}

		if err := channelFrames.setFromRequest(request); err != nil {
			fmt.Printf("(channel-ipc) invalid frame ignored. %s\n", err.Error())
		}
	}
}

func startChannelFrameIPCServer(
	ctx context.Context,
	socketPath string,
	defaultValue util.CRSFValue,
	channelFrames *channelFrameState,
) (func(), error) {
	if err := os.Remove(socketPath); err != nil && !errors.Is(err, os.ErrNotExist) {
		return nil, fmt.Errorf("could not clear stale channel socket %s: %w", socketPath, err)
	}

	listener, err := net.Listen("unix", socketPath)
	if err != nil {
		return nil, fmt.Errorf("could not listen on channel socket %s: %w", socketPath, err)
	}

	cleanup := func() {
		_ = listener.Close()
		if err := os.Remove(socketPath); err != nil && !errors.Is(err, os.ErrNotExist) {
			fmt.Printf("(channel-ipc) could not remove socket %s. %s\n", socketPath, err.Error())
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
				fmt.Printf("(channel-ipc) accept failed on %s. %s\n", socketPath, acceptErr.Error())
				continue
			}

			fmt.Printf("(channel-ipc) client connected socket=%s\n", socketPath)
			if streamErr := handleChannelFrameStreamConn(conn, channelFrames); streamErr != nil {
				fmt.Printf("(channel-ipc) stream ended with error. %s\n", streamErr.Error())
			}
			channelFrames.setDisconnected(defaultValue)
			fmt.Printf("(channel-ipc) client disconnected, fallback to default channels\n")
		}
	}()

	fmt.Printf("(channel-ipc) listening socket=%s\n", socketPath)
	return cleanup, nil
}

func formatNonZeroChannels(channels [16]util.CRSFValue) string {
	parts := make([]string, 0, len(channels))
	for idx, value := range channels {
		if value == 0 {
			continue
		}
		parts = append(parts, fmt.Sprintf("ch%d=%d", idx, value))
	}
	if len(parts) == 0 {
		return "all_zero"
	}
	return strings.Join(parts, " ")
}

// runRecvLoop continuously decodes frames from the serial port and pushes
// link-relevant events to the main loop via three buffered channels:
//
//   - syncEvents: every OPENTX_SYNC frame (used to phase-correct the channel
//     ticker)
//   - linkStatsEvents: every LinkStatistics arrival (used as another sign that
//     the TX is actively forwarding telemetry)
//   - statusEvents: every ELRS Status reply (used for the model-mismatch flag)
//
// Each channel is buffered=1 with non-blocking sends — if the main loop is
// momentarily slow, we drop the older event rather than block the recv
// goroutine. Sync arrives at 5/sec, LinkStats at ~4/sec, Status at 1/sec —
// dropping a single one is harmless.
//
// Sync and LinkStats frames are logged directly here. Status frames are passed
// to the main loop so the authoritative state print can include tx_alive and
// is_model_mismatch.
func runRecvLoop(
	ctx context.Context,
	port serial.Port,
	syncEvents chan<- *telem.SyncExtFrame,
	linkStatsEvents chan<- struct{},
	statusEvents chan<- *telem.StatusExtFrame,
) {
	reader := telem.NewReader(port)
	lastHeartbeat := time.Now()
	fmt.Printf("(recv-loop) starting read_timeout=%s\n", recvReadTimeout)

	for {
		if ctx.Err() != nil {
			fmt.Printf("(recv-loop) exiting (ctx cancelled)\n")
			return
		}

		frame, err := reader.Next(ctx)
		if err != nil {
			if _, ok := err.(*telem.InterruptedError); ok {
				fmt.Printf("(recv-loop) exiting (interrupted)\n")
				return
			}
			// Most errors here are "frame crc mismatch" — already logged by
			// the reader. Just keep going.
			continue
		}

		switch f := frame.(type) {
		case *telem.SyncExtFrame:
			fmt.Printf("(recv-loop) [SYNC] %s\n", f)
			select {
			case syncEvents <- f:
			default:
			}
		case *telem.LinkStatsFrame:
			fmt.Printf("(recv-loop) [RX-LINK] %s\n", f)
			select {
			case linkStatsEvents <- struct{}{}:
			default:
			}
		case *telem.StatusExtFrame:
			select {
			case statusEvents <- f:
			default:
			}
		}

		if time.Since(lastHeartbeat) >= recvHeartbeatInterval {
			fmt.Printf("(recv-loop) heartbeat bytesRead=%d readCalls=%d zeroReads=%d framesGood=%d framesEcho=%d framesUnknown=%d framesCrcBad=%d\n",
				reader.BytesRead, reader.ReadCalls, reader.ZeroReads,
				reader.FramesGood, reader.FramesEcho, reader.FramesUnknown, reader.FramesCrcBad)
			lastHeartbeat = time.Now()
		}
	}
}

// sendStatusRequest writes a CRSF_FRAMETYPE_PARAMETER_WRITE with
// parameterIndex=0, value=0 to the TX module. The firmware treats this as
// the "ELRS status request" (lua.cpp:363-370) and replies with a Status
// frame carrying the LUA_FLAG_CONNECTED / LUA_FLAG_MODEL_MATCH flags.
func sendStatusRequest(port serial.Port) error {
	frame := crsf.CreateParameterSettingWriteFrameUint8(uint8(crsf.ModuleEndpoint), 0, 0)
	_, err := port.Write(frame)
	return err
}

func main() {
	cfg, err := loadConfig(configFilePath)
	if err != nil {
		fmt.Printf("(app) failed to load config. %s\n", err.Error())
		os.Exit(1)
	}
	fmt.Printf("(app) loaded config=%s\n", configFilePath)
	fmt.Printf(
		"(app) starting ROS2->CRSF pipeline port=%s baud=%d model_id=%d period_ms=%d channel_input=ipc\n",
		cfg.Serial.TXPortName,
		cfg.Serial.TXBaudRate,
		cfg.ModelMatch.ModelID,
		cfg.Link.ChannelSendPeriodMS,
	)

	// For visibility only — show the firmware-recommended cadence next to the
	// YAML-configured one. They should be the same (5ms at any baud >= 400k).
	suggested := crsf.GetRefreshRate(int32(cfg.Serial.TXBaudRate))
	configured := time.Duration(cfg.Link.ChannelSendPeriodMS) * time.Millisecond
	fmt.Printf("(app) channel cadence: configured=%s suggested=%s (firmware default 5ms/200Hz)\n", configured, suggested)
	if configured < suggested {
		fmt.Printf("(app) WARNING: configured cadence %s is faster than firmware default %s — host bursts may collide with firmware replies\n", configured, suggested)
	}

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
	// Required for the recv goroutine: bounded blocking on Read so it can
	// notice ctx cancellation and exit cleanly. The send path is unaffected
	// (Write doesn't honour this timeout).
	if err = serialPort.SetReadTimeout(recvReadTimeout); err != nil {
		fmt.Printf("(app) failed to set serial read timeout. %s\n", err.Error())
		os.Exit(1)
	}
	fmt.Printf("(app) serial port opened %s @ %d baud\n", cfg.Serial.TXPortName, cfg.Serial.TXBaudRate)

	if err = sendELRSInitConfigSequence(serialPort, cfg.Logging.TXWrites); err != nil {
		fmt.Printf("%s\n", err.Error())
	}

	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt)
	defer cancel()

	modelIDRequests := make(chan modelIDChangeRequest)
	modelIDSocket := controlSocketPath()
	cleanupModelIDIPC, err := startModelIDIPCServer(ctx, modelIDSocket, modelIDRequests)
	if err != nil {
		fmt.Printf("(model-id-ipc) startup failed. %s\n", err.Error())
		os.Exit(1)
	}
	defer cleanupModelIDIPC()

	defaultChannelValue := util.CRSFValue(0)
	channelFrames := newChannelFrameState(defaultChannelValue)
	channelSocket := channelSocketPath()
	cleanupChannelIPC, err := startChannelFrameIPCServer(ctx, channelSocket, defaultChannelValue, channelFrames)
	if err != nil {
		fmt.Printf("(channel-ipc) startup failed. %s\n", err.Error())
		os.Exit(1)
	}
	defer cleanupChannelIPC()

	// Recv side: spawn a goroutine that decodes frames continuously and
	// pushes events back through three buffered channels.
	syncEvents := make(chan *telem.SyncExtFrame, 1)
	linkStatsEvents := make(chan struct{}, 1)
	statusEvents := make(chan *telem.StatusExtFrame, 1)
	go runRecvLoop(ctx, serialPort, syncEvents, linkStatsEvents, statusEvents)

	// Channel-pack ticker. YAML configures the initial period (config.yaml
	// channel_send_period_ms — set this to 5 for ELRS at any baud >= 400k).
	// SyncPeriods adjusts steadyPeriod on every OPENTX_SYNC arrival.
	steadyPeriod := time.Duration(cfg.Link.ChannelSendPeriodMS) * time.Millisecond
	channelTicker := time.NewTicker(steadyPeriod)
	defer channelTicker.Stop()
	phaseShiftPending := false

	// quietUntil suppresses channel-pack writes for adminQuietWindow after
	// any admin frame (model-id change, status request) so the firmware's
	// reply lands on a quiet line.
	var quietUntil time.Time

	var lastTxActivityAt time.Time
	txAlive := false
	modelMismatch := false

	// Model-id convergence: re-send the desired id until a status reply that
	// arrives after a send reports no mismatch — proof the admin path is live
	// and the write took. Re-arms whenever the TX reappears after silence or
	// a mismatch shows up later, so the id is re-asserted without any caller
	// having to retry.
	desiredModelID := uint8(cfg.ModelMatch.ModelID)
	modelIDPending := true
	modelIDSentSincePending := false
	modelIDResendTicker := time.NewTicker(modelIDResendInterval)
	defer modelIDResendTicker.Stop()
	loopStartedAt := time.Now()

	sendDesiredModelID := func(source string) {
		if writeErr := sendModelIDFrame(serialPort, desiredModelID, source, cfg.Logging.TXWrites); writeErr != nil {
			fmt.Printf("%s\n", writeErr.Error())
		} else {
			modelIDSentSincePending = true
		}
		quietUntil = time.Now().Add(adminQuietWindow)
	}
	sendDesiredModelID("startup")

	// Periodic status request to surface model-match state.
	statusPollTicker := time.NewTicker(statusPollInterval)
	defer statusPollTicker.Stop()
	txAliveCheckTicker := time.NewTicker(txAliveCheckInterval)
	defer txAliveCheckTicker.Stop()

	// Last observed effective link state, for state-change-only logging.
	lastStatusValid := false
	lastStatusArmed := false
	lastStatusPktsGood := uint16(0)
	lastStatusPktsBad := uint8(0)
	lastStatusMessage := ""
	lastTxAlive := false
	lastModelMismatch := false
	lastFlagsText := "stale"

	logStatus := func(flagsText string, source string, armed bool, pktsGood uint16, pktsBad uint8, msg string) {
		fmt.Printf("(recv-loop) [STATUS] tx_alive=%v is_model_mismatch=%v armed=%v pktsGood=%d pktsBad=%d flags=%s msg=%q",
			txAlive,
			modelMismatch,
			armed,
			pktsGood,
			pktsBad,
			flagsText,
			msg,
		)
		if source != "" {
			fmt.Printf(" source=%s", source)
		}
		fmt.Printf("\n")

		if !lastStatusValid ||
			txAlive != lastTxAlive ||
			modelMismatch != lastModelMismatch ||
			armed != lastStatusArmed ||
			pktsGood != lastStatusPktsGood ||
			pktsBad != lastStatusPktsBad ||
			msg != lastStatusMessage ||
			flagsText != lastFlagsText {
			fmt.Printf("(link) state changed: tx_alive=%v is_model_mismatch=%v armed=%v pktsGood=%d pktsBad=%d flags=%s msg=%q\n",
				txAlive,
				modelMismatch,
				armed,
				pktsGood,
				pktsBad,
				flagsText,
				msg,
			)
			lastTxAlive = txAlive
			lastModelMismatch = modelMismatch
			lastStatusArmed = armed
			lastStatusPktsGood = pktsGood
			lastStatusPktsBad = pktsBad
			lastStatusMessage = msg
			lastFlagsText = flagsText
			lastStatusValid = true
		}
	}

	for {
		select {
		case <-ctx.Done():
			fmt.Printf("(app) shutdown requested\n")
			fmt.Printf("(app) shutdown complete\n")
			return
		case request := <-modelIDRequests:
			desiredModelID = uint8(request.ModelID)
			modelIDPending = true
			modelIDSentSincePending = false
			writeErr := sendModelIDFrame(serialPort, desiredModelID, request.Source, cfg.Logging.TXWrites)
			if writeErr != nil {
				fmt.Printf("%s\n", writeErr.Error())
			} else {
				modelIDSentSincePending = true
			}
			// Even on write error, set the quiet window — if the write
			// partially succeeded the firmware may still reply.
			quietUntil = time.Now().Add(adminQuietWindow)
			request.Response <- writeErr

		case <-modelIDResendTicker.C:
			if modelIDPending {
				sendDesiredModelID("converge")
			}

		case sync := <-syncEvents:
			lastTxActivityAt = time.Now()
			if !txAlive {
				modelIDPending = true
				modelIDSentSincePending = false
			}
			txAlive = true

			// One-shot phase shift: schedule the next channel tick so it
			// lands in the firmware's expected RX window. Subsequent ticks
			// revert to the steady period.
			newSteady, nextTick := crsf.SyncPeriods(sync.Rate(), sync.Offset())
			steadyPeriod = newSteady
			channelTicker.Reset(nextTick)
			phaseShiftPending = true

		case <-linkStatsEvents:
			now := time.Now()
			lastTxActivityAt = now
			if !txAlive {
				modelIDPending = true
				modelIDSentSincePending = false
			}
			txAlive = true

		case <-txAliveCheckTicker.C:
			now := time.Now()
			if txAlive && !lastTxActivityAt.IsZero() && now.Sub(lastTxActivityAt) > txAliveTimeout {
				txAlive = false
				modelMismatch = false
				if lastStatusValid {
					logStatus("stale", "tx-timeout", lastStatusArmed, lastStatusPktsGood, lastStatusPktsBad, lastStatusMessage)
				}
			}
			if !txAlive {
				silentSince := lastTxActivityAt
				if silentSince.IsZero() {
					silentSince = loopStartedAt
				}
				if now.Sub(silentSince) > txSilentRestartAfter {
					fmt.Printf("(app) TX module silent for %s despite open serial port — exiting for a fresh serial session\n", txSilentRestartAfter)
					cancel()
				}
			}

		case <-statusPollTicker.C:
			if writeErr := sendStatusRequest(serialPort); writeErr != nil {
				fmt.Printf("(status-poll) write error: %s\n", writeErr.Error())
				continue
			}
			quietUntil = time.Now().Add(adminQuietWindow)

		case s := <-statusEvents:
			lastTxActivityAt = time.Now()
			if !txAlive {
				modelIDPending = true
				modelIDSentSincePending = false
			}
			txAlive = true
			modelMismatch = s.ModelMismatched()
			if modelMismatch {
				modelIDPending = true
			} else if modelIDPending && modelIDSentSincePending && s.Connected() {
				// Only a live RX link with no mismatch proves the id took;
				// an unlinked TX never clears the mismatch flag.
				modelIDPending = false
				fmt.Printf("(model-id) confirmed: rx connected, model match, model_id=%d\n", desiredModelID)
			}
			logStatus(
				fmt.Sprintf("0x%02x", s.Flags()),
				"",
				s.Armed(),
				s.PktsGood(),
				s.PktsBad(),
				s.Message(),
			)

		case <-channelTicker.C:
			// Suppress channel-pack writes during the post-admin quiet window
			// so the firmware's reply has a clean line.
			if !quietUntil.IsZero() && time.Now().Before(quietUntil) {
				if phaseShiftPending {
					channelTicker.Reset(steadyPeriod)
					phaseShiftPending = false
				}
				continue
			}

			channels := channelFrames.snapshot()

			if _, err = serialPort.Write(crsf.PackChannels(&channels)); err != nil {
				fmt.Printf("(send-loop) could not write channels on port %s. %s\n", cfg.Serial.TXPortName, err.Error())
				cancel()
				continue
			}
			if cfg.Logging.TXWrites {
				fmt.Printf(
					"(send-loop) written channels=[%s] port=%s baud=%d\n",
					formatNonZeroChannels(channels),
					cfg.Serial.TXPortName,
					cfg.Serial.TXBaudRate,
				)
			}

			// If a one-shot phase-correction tick just fired, restore the
			// steady period for subsequent ticks.
			if phaseShiftPending {
				channelTicker.Reset(steadyPeriod)
				phaseShiftPending = false
			}
		}
	}
}
