package main

import (
	"context"
	"errors"
	"fmt"
	"slices"
	"testing"
	"time"

	"go.viam.com/rdk/logging"
)

// fakeLoco records the sport commands runReadyToMove sends.
type fakeLoco struct {
	fsm    int
	fsmErr error
	calls  []string
}

func (f *fakeLoco) GetFsmID() (int, error) { return f.fsm, f.fsmErr }

func (f *fakeLoco) SetFsmID(id int) error {
	f.calls = append(f.calls, fmt.Sprintf("fsm %d", id))
	f.fsm = id
	return nil
}

func (f *fakeLoco) SetBalanceMode(mode int) error {
	f.calls = append(f.calls, fmt.Sprintf("balance %d", mode))
	return nil
}

func TestReadyToMoveSkipsDampWhenStanding(t *testing.T) {
	defer func(sleep func(context.Context, time.Duration) error) { readyToMoveSleep = sleep }(readyToMoveSleep)
	readyToMoveSleep = func(context.Context, time.Duration) error { return nil }

	full := []string{"fsm 1", "fsm 4", "balance 1", "fsm 802"}
	for _, tc := range []struct {
		name   string
		fsm    int
		fsmErr error
		force  bool
		want   []string
	}{
		{"walking", FsmRun, nil, false, nil},
		{"standing", FsmStandUp, nil, false, []string{"balance 1", "fsm 802"}},
		{"started", FsmStart, nil, false, []string{"balance 1", "fsm 802"}},
		{"damped", FsmDamp, nil, false, full},
		{"zero torque", FsmZeroTorque, nil, false, full},
		{"sitting", FsmSit, nil, false, full},
		{"unreadable", 0, errors.New("rpc timeout"), false, full},
		// Walk mode reported, but limp after a fault: force runs everything.
		{"forced while walking", FsmRun, nil, true, full},
	} {
		loco := &fakeLoco{fsm: tc.fsm, fsmErr: tc.fsmErr}
		if err := runReadyToMove(context.Background(), loco, logging.NewTestLogger(t), tc.force); err != nil {
			t.Fatalf("%s: %v", tc.name, err)
		}
		if !slices.Equal(loco.calls, tc.want) {
			t.Errorf("%s: sent %v, want %v", tc.name, loco.calls, tc.want)
		}
	}
}
