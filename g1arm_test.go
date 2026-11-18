package main

import (
	"context"
	"strings"
	"testing"

	"go.viam.com/rdk/referenceframe"
)

// TestCommandPositionsRefusesEarly checks that bad moves are refused before anything reads joint
// state or limits: these g1Arms have no armSDK, so reaching that code would panic.
func TestCommandPositionsRefusesEarly(t *testing.T) {
	ctx := context.Background()
	body := &g1Arm{side: sideBody, indices: make([]int, 15)}
	err := body.MoveToJointPositions(ctx, make([]referenceframe.Input, 15), nil)
	if err == nil || !strings.Contains(err.Error(), "read-only") {
		t.Errorf("body move: got %v, want read-only error", err)
	}
	left := &g1Arm{side: sideLeft, indices: make([]int, G1ArmDoF)}
	for _, n := range []int{G1ArmDoF - 1, G1ArmDoF + 1} {
		if err := left.MoveToJointPositions(ctx, make([]referenceframe.Input, n), nil); err == nil {
			t.Errorf("arm accepted %d positions", n)
		}
	}
}

func TestConfigValidate(t *testing.T) {
	gains := make([]float64, G1ArmDoF)
	if _, _, err := (&G1ArmConfig{Kp: gains, Kd: gains}).Validate(""); err != nil {
		t.Errorf("arm with %d gains: %v", G1ArmDoF, err)
	}
	if _, _, err := (&G1ArmConfig{Kp: gains[1:]}).Validate(""); err == nil {
		t.Errorf("arm accepted %d kp entries", G1ArmDoF-1)
	}

	// The body is read-only: it takes model_path but no gains, of any length.
	if _, _, err := (&G1BodyConfig{ModelPath: "body.urdf"}).Validate(""); err != nil {
		t.Errorf("body without gains: %v", err)
	}
	for _, c := range []*G1BodyConfig{{Kp: gains}, {Kd: make([]float64, 15)}} {
		if _, _, err := c.Validate(""); err == nil {
			t.Errorf("body accepted gains %+v", *c)
		}
	}
}
