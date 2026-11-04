package main

import (
	"context"
	"errors"
	"fmt"
	"net"
	"sync"
	"testing"
	"time"

	"go.bug.st/serial"
)

func report(now time.Time, stream string, recent, total int) []byte {
	return []byte(fmt.Sprintf(`{"type":"rx","id":%q,"timestamp":%.6f,"session":{"epoch":0},"packets":{"dec_ok":[%d,%d]}}`, stream, float64(now.UnixNano())/1e9, recent, total))
}

func TestVTXTimeoutAndRecovery(t *testing.T) {
	g := &vtxGuard{}
	now := time.Unix(1700000000, 0)
	if g.allowed(now) {
		t.Fatal("startup must be inhibited")
	}
	if !g.observe(report(now, "video rx", 10, 100), now) {
		t.Fatal("valid video rejected")
	}
	if !g.allowed(now.Add(2999 * time.Millisecond)) {
		t.Fatal("cut off before 3 seconds")
	}
	if g.allowed(now.Add(3 * time.Second)) {
		t.Fatal("not inhibited at 3 seconds")
	}
	next := now.Add(4 * time.Second)
	if !g.observe(report(next, "video rx", 10, 110), next) || !g.allowed(next) {
		t.Fatal("fresh video did not recover")
	}
}

func TestNonVideoAndInvalidStatsCannotKeepELRSAlive(t *testing.T) {
	now := time.Unix(1700000000, 0)
	cases := [][]byte{
		report(now, "mavlink rx", 10, 100), report(now, "video rx", 0, 100),
		report(now, "video rx", -1, 100), report(now, "video rx", 10, 1),
		[]byte(`{bad`), []byte(`{"type":"rx","id":"video rx","session":null}`),
		report(now.Add(-2*time.Second), "video rx", 10, 100),
		report(now.Add(time.Second), "video rx", 10, 100),
	}
	for _, data := range cases {
		g := &vtxGuard{}
		if g.observe(data, now) || g.allowed(now) {
			t.Fatalf("unsafe report accepted: %s", data)
		}
	}
	g := &vtxGuard{}
	g.observe(report(now, "video rx", 1, 100), now)
	for i := 1; i <= 4; i++ {
		later := now.Add(time.Duration(i) * time.Second)
		g.observe(report(later, "video rx", 0, 100), later)
		g.observe(report(later, "mavlink rx", 100, 1000), later)
	}
	if g.allowed(now.Add(4 * time.Second)) {
		t.Fatal("unrelated traffic extended video lease")
	}
}

func TestDuplicateAndDelayedStatsDoNotExtendDeadline(t *testing.T) {
	g := &vtxGuard{}
	now := time.Unix(1700000000, 0)
	data := report(now, "video rx", 1, 1)
	g.observe(data, now)
	if g.observe(data, now.Add(time.Second)) {
		t.Fatal("duplicate accepted")
	}
	if g.allowed(now.Add(3 * time.Second)) {
		t.Fatal("duplicate extended deadline")
	}
	g = &vtxGuard{}
	if !g.observe(data, now.Add(time.Second)) {
		t.Fatal("recent delayed sample rejected")
	}
	if g.allowed(now.Add(3 * time.Second)) {
		t.Fatal("transport delay extended deadline")
	}
}

type capturePort struct {
	serial.Port
	writes int
}

func (p *capturePort) Write(data []byte) (int, error) { p.writes++; return len(data), nil }

func TestAllSerialFrameTypesAreBlocked(t *testing.T) {
	now := time.Unix(1700000000, 0)
	g := &vtxGuard{}
	raw := &capturePort{}
	p := &vtxGuardedPort{Port: raw, guard: g, now: func() time.Time { return now }}
	attempts := []func() error{
		func() error { _, err := p.Write([]byte{0xc8, 22, 0x16}); return err },
		func() error { return sendModelIDFrame(p, 1, "test", false) },
		func() error { return sendStatusRequest(p) },
		func() error {
			return sendRawInitPacket(p, namedInitPacket{name: "test", bytes: []byte{1}}, 1, 1, false)
		},
	}
	for _, attempt := range attempts {
		if !errors.Is(attempt(), errVTXUnavailable) {
			t.Fatal("write did not fail closed")
		}
	}
	if raw.writes != 0 {
		t.Fatal("blocked packets reached serial")
	}
	g.observe(report(now, "video rx", 1, 1), now)
	for _, attempt := range attempts {
		if err := attempt(); err != nil {
			t.Fatal(err)
		}
	}
	if raw.writes != 4 {
		t.Fatal("healthy link did not transmit")
	}
	now = now.Add(3 * time.Second)
	for _, attempt := range attempts {
		if !errors.Is(attempt(), errVTXUnavailable) {
			t.Fatal("deadline bypassed")
		}
	}
	if raw.writes != 4 {
		t.Fatal("packet transmitted after deadline")
	}
}

func TestStatsMonitorReadsRealTCPAndExpiresAfterDisconnect(t *testing.T) {
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	defer listener.Close()
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	g := &vtxGuard{}
	done := make(chan struct{})
	go func() { monitorVTX(ctx, listener.Addr().String(), g); close(done) }()
	conn, err := listener.Accept()
	if err != nil {
		t.Fatal(err)
	}
	now := time.Now()
	fmt.Fprintf(conn, "%s\n", report(now, "video rx", 1, 1))
	limit := time.Now().Add(time.Second)
	for !g.allowed(time.Now()) && time.Now().Before(limit) {
		time.Sleep(time.Millisecond)
	}
	if !g.allowed(time.Now()) {
		t.Fatal("monitor did not accept video stats")
	}
	conn.Close()
	if g.allowed(now.Add(4 * time.Second)) {
		t.Fatal("silent monitor kept link alive")
	}
	cancel()
	select {
	case <-done:
	case <-time.After(2 * time.Second):
		t.Fatal("monitor did not stop")
	}
}

func TestWaitForVTXCanBeCancelledWithoutOpeningSerial(t *testing.T) {
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	if waitForVTX(ctx, &vtxGuard{}) {
		t.Fatal("cancelled startup enabled serial")
	}
}

func TestGuardConcurrentObservationAndWrites(t *testing.T) {
	g := &vtxGuard{}
	var wg sync.WaitGroup
	for i := 0; i < 4; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 100; j++ {
				now := time.Now()
				g.observe(report(now, "video rx", 1, j+1), now)
				g.allowed(now)
			}
		}()
	}
	wg.Wait()
}
