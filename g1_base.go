package main

import (
	"context"
	"fmt"
	"math"
	"sync"
	"sync/atomic"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var g1BaseModel = resource.NewModel("erh", "viam-unitree", "g1-base")

type G1BaseConfig struct {
	NetworkInterface string `json:"network_interface"`
}

func (c *G1BaseConfig) Validate(path string) ([]string, error) {
	return nil, nil
}

func init() {
	resource.RegisterComponent(base.API, g1BaseModel, resource.Registration[base.Base, *G1BaseConfig]{
		Constructor: newG1Base,
	})
}

type g1Base struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	loco   *LocoClient

	mu        sync.Mutex
	moving    atomic.Bool
	cancelCtx context.Context
	cancelFn  context.CancelFunc
}

func newG1Base(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (base.Base, error) {
	cfg, err := resource.NativeConfig[*G1BaseConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}

	logger.Infof("Initializing G1Base with network interface: %s", networkInterface)

	if err := InitChannel(0, networkInterface); err != nil {
		return nil, fmt.Errorf("channel init: %w", err)
	}

	loco, err := NewLocoClient()
	if err != nil {
		return nil, fmt.Errorf("loco client create: %w", err)
	}
	if err := loco.Init(); err != nil {
		loco.Close()
		return nil, fmt.Errorf("loco client init: %w", err)
	}
	loco.SetTimeout(10.0)

	cancelCtx, cancelFn := context.WithCancel(context.Background())

	logger.Info("G1Base initialized successfully")

	return &g1Base{
		Named:     conf.ResourceName().AsNamed(),
		logger:    logger,
		loco:      loco,
		cancelCtx: cancelCtx,
		cancelFn:  cancelFn,
	}, nil
}

func (b *g1Base) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64, extra map[string]interface{}) error {
	if distanceMm == 0 || mmPerSec == 0 {
		return nil
	}

	speedMps := math.Abs(mmPerSec) / 1000.0
	durationSec := math.Abs(float64(distanceMm)) / math.Abs(mmPerSec)
	direction := 1.0
	if distanceMm < 0 {
		direction = -1.0
	}
	vx := float32(direction * speedMps)

	b.mu.Lock()
	// Create a new cancel context for this movement.
	b.cancelFn()
	moveCtx, moveFn := context.WithCancel(context.Background())
	b.cancelCtx = moveCtx
	b.cancelFn = moveFn
	b.mu.Unlock()

	b.moving.Store(true)
	defer b.moving.Store(false)

	deadline := time.Now().Add(time.Duration(durationSec * float64(time.Second)))
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for time.Now().Before(deadline) {
		select {
		case <-ctx.Done():
			b.loco.StopMove()
			return ctx.Err()
		case <-moveCtx.Done():
			return nil
		case <-ticker.C:
			b.loco.Move(vx, 0, 0)
		}
	}

	b.loco.StopMove()
	return nil
}

func (b *g1Base) Spin(ctx context.Context, angleDeg, degsPerSec float64, extra map[string]interface{}) error {
	if angleDeg == 0 || degsPerSec == 0 {
		return nil
	}

	durationSec := math.Abs(angleDeg) / math.Abs(degsPerSec)
	direction := 1.0
	if angleDeg < 0 {
		direction = -1.0
	}
	vyaw := float32(direction * math.Abs(degsPerSec) * math.Pi / 180.0)

	b.mu.Lock()
	b.cancelFn()
	moveCtx, moveFn := context.WithCancel(context.Background())
	b.cancelCtx = moveCtx
	b.cancelFn = moveFn
	b.mu.Unlock()

	b.moving.Store(true)
	defer b.moving.Store(false)

	deadline := time.Now().Add(time.Duration(durationSec * float64(time.Second)))
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for time.Now().Before(deadline) {
		select {
		case <-ctx.Done():
			b.loco.StopMove()
			return ctx.Err()
		case <-moveCtx.Done():
			return nil
		case <-ticker.C:
			b.loco.Move(0, 0, vyaw)
		}
	}

	b.loco.StopMove()
	return nil
}

func (b *g1Base) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	const maxLinearVel = 1.5
	const maxAngularVel = 1.0

	vx := float32(linear.X * maxLinearVel)
	vy := float32(linear.Y * maxLinearVel)
	vyaw := float32(angular.Z * maxAngularVel)

	b.moving.Store(vx != 0 || vy != 0 || vyaw != 0)
	b.loco.Move(vx, vy, vyaw)
	return nil
}

func (b *g1Base) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	// Viam: linear in mm/s, angular in deg/s.
	// Unitree: linear in m/s, angular in rad/s.
	vx := float32(linear.X / 1000.0)
	vy := float32(linear.Y / 1000.0)
	vyaw := float32(angular.Z * math.Pi / 180.0)

	b.moving.Store(vx != 0 || vy != 0 || vyaw != 0)
	b.loco.Move(vx, vy, vyaw)
	return nil
}

func (b *g1Base) Stop(ctx context.Context, extra map[string]interface{}) error {
	b.mu.Lock()
	b.cancelFn()
	b.mu.Unlock()

	b.loco.StopMove()
	b.moving.Store(false)
	return nil
}

func (b *g1Base) IsMoving(ctx context.Context) (bool, error) {
	return b.moving.Load(), nil
}

func (b *g1Base) Properties(ctx context.Context, extra map[string]interface{}) (base.Properties, error) {
	return base.Properties{
		WidthMeters: 0.45,
	}, nil
}

func (b *g1Base) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}

func (b *g1Base) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	cmdStr, ok := cmd["command"].(string)
	if !ok {
		return map[string]interface{}{}, nil
	}

	var rc int
	switch cmdStr {
	case "stand_up":
		rc = b.loco.StandUp()
	case "sit":
		rc = b.loco.Sit()
	case "squat":
		rc = b.loco.Squat()
	case "high_stand":
		rc = b.loco.HighStand()
	case "low_stand":
		rc = b.loco.LowStand()
	case "balance_stand":
		rc = b.loco.BalanceStand()
	case "damp":
		rc = b.loco.Damp()
	case "zero_torque":
		rc = b.loco.ZeroTorque()
	case "wave_hand":
		rc = b.loco.WaveHand()
	case "start":
		rc = b.loco.Start()
	case "stop_move":
		rc = b.loco.StopMove()
	default:
		return map[string]interface{}{"error": "unknown command: " + cmdStr}, nil
	}

	return map[string]interface{}{"rc": float64(rc)}, nil
}

func (b *g1Base) Close(ctx context.Context) error {
	b.mu.Lock()
	b.cancelFn()
	b.mu.Unlock()

	b.loco.StopMove()
	b.loco.Close()
	return nil
}
