package main

import (
	"encoding/binary"
	"testing"
)

// Expected frames come from BrainCo's brainco-hand-sdk
// tools/canfd-diagnostics/direct/frame_reference.txt (same Modbus payload as RS485).
func TestBrainCoPercentTargetFrames(t *testing.T) {
	cases := []struct {
		name    string
		slave   byte
		targets [brainCoNumFingers]int
		want    []byte
	}{
		{"left open", 126, [6]int{0, 0, 0, 0, 0, 0},
			[]byte{0x7E, 0x10, 0x04, 0x2E, 0x00, 0x03, 0x06, 0, 0, 0, 0, 0, 0, 0x38, 0x10}},
		{"left close", 126, [6]int{50, 50, 100, 100, 100, 100},
			[]byte{0x7E, 0x10, 0x04, 0x2E, 0x00, 0x03, 0x06, 0x32, 0x32, 0x64, 0x64, 0x64, 0x64, 0x70, 0x02}},
		{"right close", 127, [6]int{50, 50, 100, 100, 100, 100},
			[]byte{0x7F, 0x10, 0x04, 0x2E, 0x00, 0x03, 0x06, 0x32, 0x32, 0x64, 0x64, 0x64, 0x64, 0x72, 0x83}},
	}
	for _, c := range cases {
		req := writeRegistersRequest(c.slave, brainCoRegPercentTargets, packPercentTargets(c.targets))
		got := binary.LittleEndian.AppendUint16(req, modbusCRC(req))
		if string(got) != string(c.want) {
			t.Errorf("%s: got % x, want % x", c.name, got, c.want)
		}
	}
}

func TestPackPercentTargetsOrderAndClamp(t *testing.T) {
	// Finger 0 in the low byte, finger 1 in the high byte; values clamp to 0..100.
	got := packPercentTargets([6]int{1, 2, -5, 150, 30, 40})
	want := []uint16{0x0201, 0x6400, 0x281E}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("word %d: got %#04x, want %#04x", i, got[i], want[i])
		}
	}
}

func TestModbusCRC(t *testing.T) {
	if got := modbusCRC([]byte("123456789")); got != 0x4B37 {
		t.Errorf("got %#04x, want 0x4b37", got)
	}
}
