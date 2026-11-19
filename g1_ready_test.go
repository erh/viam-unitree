package main

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

// fakeLoco records the sport commands runReadyToMove sends. Like the firmware, it accepts
// Regular mode (200) only from StandUp (4): from elsewhere the request succeeds and is ignored.
type fakeLoco struct {
	fsm    int
	fsmErr error
	calls  []string
	// ignoreStarts is how many Start requests the robot ignores while its
	// stand-up is still finishing.
	ignoreStarts int
}

func (f *fakeLoco) GetFsmID() (int, error) { return f.fsm, f.fsmErr }

func (f *fakeLoco) SetFsmID(id int) error {
	f.calls = append(f.calls, fmt.Sprintf("fsm %d", id))
	if id == FsmStart {
		if f.fsm != FsmStandUp {
			return nil
		}
		if f.ignoreStarts > 0 {
			f.ignoreStarts--
			return nil
		}
	}
	f.fsm = id
	f.fsmErr = nil
	return nil
}

func TestReadyToMove(t *testing.T) {
	defer func(sleep func(context.Context, time.Duration) error) { readyToMoveSleep = sleep }(readyToMoveSleep)
	readyToMoveSleep = func(context.Context, time.Duration) error { return nil }

	full := []string{"fsm 1", "fsm 4", "fsm 200"}
	for _, tc := range []struct {
		name   string
		fsm    int
		fsmErr error
		force  bool
		want   []string
		end    int
	}{
		{"regular mode", FsmStart, nil, false, nil, FsmStart},
		// Leaving Running mode takes Damp, which drops a standing robot: only with force.
		{"running mode", FsmRun, nil, false, nil, FsmRun},
		{"standing", FsmStandUp, nil, false, []string{"fsm 200"}, FsmStart},
		{"damped", FsmDamp, nil, false, full, FsmStart},
		{"zero torque", FsmZeroTorque, nil, false, full, FsmStart},
		{"sitting", FsmSit, nil, false, full, FsmStart},
		{"unreadable", 0, errors.New("rpc timeout"), false, full, FsmStart},
		// A walk mode reported, but limp after a fault: force runs everything.
		{"forced in regular mode", FsmStart, nil, true, full, FsmStart},
		{"forced in running mode", FsmRun, nil, true, full, FsmStart},
	} {
		loco := &fakeLoco{fsm: tc.fsm, fsmErr: tc.fsmErr}
		note, err := runReadyToMove(context.Background(), loco, logging.NewTestLogger(t), tc.force)
		if err != nil {
			t.Fatalf("%s: %v", tc.name, err)
		}
		if !slices.Equal(loco.calls, tc.want) || loco.fsm != tc.end || note == "" {
			t.Errorf("%s: sent %v, ended in FSM %d with note %q; want %v, FSM %d", tc.name, loco.calls, loco.fsm, note, tc.want, tc.end)
		}
		if (tc.name == "running mode") != strings.Contains(note, "Running mode") {
			t.Errorf("%s: note %q", tc.name, note)
		}
	}

	// The robot ignores Start until its stand-up has finished: keep asking.
	slow := &fakeLoco{fsm: FsmDamp, ignoreStarts: 3}
	if _, err := runReadyToMove(context.Background(), slow, logging.NewTestLogger(t), false); err != nil {
		t.Fatalf("slow stand-up: %v", err)
	}
	if want := []string{"fsm 1", "fsm 4", "fsm 200", "fsm 200", "fsm 200", "fsm 200"}; !slices.Equal(slow.calls, want) || slow.fsm != FsmStart {
		t.Errorf("slow stand-up: sent %v, ended in FSM %d; want %v, FSM %d", slow.calls, slow.fsm, want, FsmStart)
	}

	// A robot that never takes Start is reported, not taken for walking.
	stuck := &ignoresStart{fakeLoco: fakeLoco{fsm: FsmStandUp}}
	_, err := runReadyToMove(context.Background(), stuck, logging.NewTestLogger(t), false)
	if err == nil || !strings.Contains(err.Error(), "didn't enter Regular mode") || len(stuck.calls) != 0 || stuck.starts != startAttempts {
		t.Errorf("ignored Start: err %v after %d Start requests, want an error after %d", err, stuck.starts, startAttempts)
	}
}

// ignoresStart accepts every request but never enters Regular mode.
type ignoresStart struct {
	fakeLoco
	starts int
}

func (f *ignoresStart) SetFsmID(id int) error {
	if id == FsmStart {
		f.starts++
		return nil
	}
	return f.fakeLoco.SetFsmID(id)
}

// TestStandAndWalkKeepArms checks that moving between the standing states leaves engaged arms
// alone: releasing them would drop whatever they hold.
func TestStandAndWalkKeepArms(t *testing.T) {
	defer func(sleep func(context.Context, time.Duration) error) { readyToMoveSleep = sleep }(readyToMoveSleep)
	readyToMoveSleep = func(context.Context, time.Duration) error { return nil }

	a := &armSDK{gravityScale: 1, weight: 1, target: 1, writer: 1}
	armSDKMu.Lock()
	armSDKInstance = a
	armSDKMu.Unlock()
	defer func() {
		armSDKMu.Lock()
		armSDKInstance = nil
		armSDKMu.Unlock()
	}()
	engaged := func(what string) {
		t.Helper()
		if a.target != 1 || a.weight != 1 || a.writer != 1 {
			t.Errorf("%s: arm_sdk target %v, weight %v, writer %d; want engaged", what, a.target, a.weight, a.writer)
		}
	}
	// ready_to_move: already walking (either mode), or standing and entering walk mode.
	for fsm, want := range map[int][]string{FsmStart: nil, FsmRun: nil, FsmStandUp: {"fsm 200"}} {
		loco := &fakeLoco{fsm: fsm}
		if _, err := runReadyToMove(context.Background(), loco, logging.NewTestLogger(t), false); err != nil {
			t.Fatal(err)
		}
		if !slices.Equal(loco.calls, want) {
			t.Errorf("ready_to_move from FSM %d sent %v, want %v", fsm, loco.calls, want)
		}
		engaged(fmt.Sprintf("ready_to_move from FSM %d", fsm))
	}
	// stand: from walk mode it stands without dropping the arms; already standing, it does nothing.
	for fsm, want := range map[int][]string{FsmStart: {"fsm 4"}, FsmStandUp: nil} {
		loco := &fakeLoco{fsm: fsm}
		if err := standUp(context.Background(), loco, logging.NewTestLogger(t)); err != nil {
			t.Fatal(err)
		}
		if !slices.Equal(loco.calls, want) || loco.fsm != FsmStandUp {
			t.Errorf("stand from FSM %d sent %v and ended in %d, want %v and %d", fsm, loco.calls, loco.fsm, want, FsmStandUp)
		}
		engaged(fmt.Sprintf("stand from FSM %d", fsm))
	}
	if !armsStayEngaged(FsmStandUp) || !armsStayEngaged(FsmStart) || armsStayEngaged(FsmDamp) ||
		armsStayEngaged(FsmZeroTorque) || armsStayEngaged(FsmSit) || armsStayEngaged(FsmRun) {
		t.Error("armsStayEngaged: only StandUp and Start keep the arms")
	}
}
