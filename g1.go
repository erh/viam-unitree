package main

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var g1Model = resource.NewModel("erh", "viam-unitree", "g1")

type G1Config struct {
	NetworkInterface string `json:"network_interface"`
}

func (c *G1Config) Validate(path string) ([]string, error) {
	return nil, nil
}

func init() {
	resource.RegisterComponent(generic.API, g1Model, resource.Registration[resource.Resource, *G1Config]{
		Constructor: newG1,
	})
}

type g1 struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	loco   *LocoClient
}

func newG1(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*G1Config](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}

	logger.Infof("Initializing G1 with network interface: %s", networkInterface)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	loco, err := NewLocoClient()
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("loco client: %w", err)
	}

	logger.Info("G1 initialized successfully")

	return &g1{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		loco:   loco,
	}, nil
}

// readyToMove runs the post-boot sequence to get the G1 into a state where
// it can accept locomotion commands: stand up from the ground, wait for the
// motion to settle, then transition the FSM into the "start" (locomotion)
// state.
func (g *g1) readyToMove(ctx context.Context) error {
	g.logger.Info("readyToMove: issuing stand_up")
	if _, err := g.loco.StandUp(); err != nil {
		return fmt.Errorf("stand_up: %w", err)
	}

	// Give the robot time to reach the standing pose before transitioning
	// into the locomotion FSM.
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(5 * time.Second):
	}

	g.logger.Info("readyToMove: issuing start")
	if _, err := g.loco.Start(); err != nil {
		return fmt.Errorf("start: %w", err)
	}

	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-time.After(2 * time.Second):
	}

	g.logger.Info("readyToMove: complete")
	return nil
}

func (g *g1) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cmdStr, ok := cmd["command"].(string)
	if !ok {
		return map[string]interface{}{"error": "missing 'command' field"}, nil
	}

	switch cmdStr {
	case "ready_to_move":
		if err := g.readyToMove(ctx); err != nil {
			return map[string]interface{}{"rc": -1.0, "error": err.Error()}, nil
		}
		return map[string]interface{}{"rc": 0.0}, nil
	default:
		return map[string]interface{}{"error": "unknown command: " + cmdStr}, nil
	}
}

func (g *g1) Close(ctx context.Context) error {
	if g.loco != nil {
		g.loco.Close()
	}
	ShutdownDDS()
	return nil
}
