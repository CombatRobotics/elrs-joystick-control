package main

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"net"
	"sync"
	"time"

	"go.bug.st/serial"
)

const vtxLossTimeout = 3 * time.Second
const wfbStatsAddress = "127.0.0.1:8103"

var errVTXUnavailable = errors.New("VTX video link unavailable for 3 seconds; ELRS writes inhibited")

// vtxGuard uses authenticated video reception, never RSSI, a cached session,
// cumulative packet counts alone, or the presence of a stats TCP connection.
// time.Time retains its monotonic clock for the transmission deadline.
type vtxGuard struct {
	mu            sync.Mutex
	lastGood      time.Time
	lastTimestamp float64
}

func (g *vtxGuard) observe(line []byte, now time.Time) bool {
	var report struct {
		Type      string                 `json:"type"`
		ID        string                 `json:"id"`
		Timestamp float64                `json:"timestamp"`
		Session   map[string]interface{} `json:"session"`
		Packets   map[string][]int64     `json:"packets"`
	}
	if json.Unmarshal(line, &report) != nil || report.Type != "rx" || report.ID != "video rx" || len(report.Session) == 0 {
		return false
	}
	counts := report.Packets["dec_ok"]
	if len(counts) != 2 || counts[0] <= 0 || counts[1] < counts[0] {
		return false
	}
	// Reject queued/replayed history and future-dated reports. The stats producer
	// runs on this same host; wall-clock validation is only at ingestion.
	age := float64(now.UnixNano())/1e9 - report.Timestamp
	if age < -0.25 || age > 1.5 {
		return false
	}
	g.mu.Lock()
	defer g.mu.Unlock()
	if report.Timestamp <= g.lastTimestamp {
		return false
	}
	g.lastTimestamp = report.Timestamp
	// Account for transport delay without giving an old report a fresh lease.
	if age > 0 {
		now = now.Add(-time.Duration(age * float64(time.Second)))
	}
	g.lastGood = now
	return true
}

func (g *vtxGuard) allowed(now time.Time) bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	return !g.lastGood.IsZero() && now.Sub(g.lastGood) < vtxLossTimeout
}

// Every serial write, including setup/model/status frames, passes this final
// boundary. A caller cannot bypass the interlock by using another send path.
type vtxGuardedPort struct {
	serial.Port
	guard *vtxGuard
	now   func() time.Time
}

func (p *vtxGuardedPort) Write(data []byte) (int, error) {
	if !p.guard.allowed(p.now()) {
		return 0, errVTXUnavailable
	}
	return p.Port.Write(data)
}

func monitorVTX(ctx context.Context, address string, guard *vtxGuard) {
	for ctx.Err() == nil {
		conn, err := (&net.Dialer{Timeout: time.Second}).DialContext(ctx, "tcp", address)
		if err == nil {
			scanner := bufio.NewScanner(conn)
			scanner.Buffer(make([]byte, 4096), 256*1024)
			for ctx.Err() == nil {
				conn.SetReadDeadline(time.Now().Add(time.Second))
				if !scanner.Scan() {
					break
				}
				guard.observe(scanner.Bytes(), time.Now())
			}
			conn.Close()
		}
		select {
		case <-ctx.Done():
			return
		case <-time.After(250 * time.Millisecond):
		}
	}
}

func waitForVTX(ctx context.Context, guard *vtxGuard) bool {
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	for {
		if guard.allowed(time.Now()) {
			return true
		}
		select {
		case <-ctx.Done():
			return false
		case <-ticker.C:
		}
	}
}
