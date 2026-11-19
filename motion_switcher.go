package main

import (
	"encoding/json"
	"fmt"
)

// ApiMotionSwitcherCheckMode is the motion_switcher service's CheckMode API ID,
// from unitree_sdk2's motion_switcher_api.hpp.
const ApiMotionSwitcherCheckMode = 1001

// MotionSwitcherClient reads Unitree's motion_switcher service, which reports
// the robot's running motion controller (the sport service that balances,
// walks and stands). It only reads: releasing and re-selecting the controller
// to restart it without a reboot left a G1 with no controller when tried after
// a motor fault, so recovery from a fault is a power cycle.
type MotionSwitcherClient struct {
	rpc *RPCClient
}

func NewMotionSwitcherClient() (*MotionSwitcherClient, error) {
	rpc, err := NewRPCClient("motion_switcher", true)
	if err != nil {
		return nil, err
	}
	return &MotionSwitcherClient{rpc: rpc}, nil
}

func (m *MotionSwitcherClient) Close() {
	m.rpc.Close()
}

// CheckMode returns the active motion controller's form and name; name is
// empty when none is running.
func (m *MotionSwitcherClient) CheckMode() (form, name string, err error) {
	data, _, err := m.rpc.Call(ApiMotionSwitcherCheckMode, "{}", 5000)
	if err != nil {
		return "", "", err
	}
	var r map[string]interface{}
	if err := json.Unmarshal([]byte(data), &r); err != nil {
		return "", "", fmt.Errorf("parse motion mode %q: %w", data, err)
	}
	str := func(v interface{}) string {
		if v == nil {
			return ""
		}
		return fmt.Sprint(v)
	}
	return str(r["form"]), str(r["name"]), nil
}
