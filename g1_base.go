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

func (c *G1BaseConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
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

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	loco, err := NewLocoClient()
	if err != nil {
		return nil, fmt.Errorf("loco client: %w", err)
	}

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
			sent, err := b.refreshMove(moveCtx, vx, 0, 0)
			if !sent {
				return nil
			}
			if err != nil {
				b.loco.StopMove()
				return moveError(err)
			}
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
			sent, err := b.refreshMove(moveCtx, 0, 0, vyaw)
			if !sent {
				return nil
			}
			if err != nil {
				b.loco.StopMove()
				return moveError(err)
			}
		}
	}

	b.loco.StopMove()
	return nil
}

func (b *g1Base) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	const maxLinearVel = 1.5
	const maxAngularVel = 2.0 // rad/s at full angular power; 1.0 felt dead below ~full stick

	vx := float32(linear.X * maxLinearVel)
	vy := float32(linear.Y * maxLinearVel)
	vyaw := float32(angular.Z * maxAngularVel)

	return b.startContinuousMove(vx, vy, vyaw)
}

func (b *g1Base) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	// Viam: linear in mm/s, angular in deg/s.
	// Unitree: linear in m/s, angular in rad/s.
	vx := float32(linear.X / 1000.0)
	vy := float32(linear.Y / 1000.0)
	vyaw := float32(angular.Z * math.Pi / 180.0)

	return b.startContinuousMove(vx, vy, vyaw)
}

// startContinuousMove keeps re-issuing Move at ~10Hz until Stop or another
// motion command cancels it. Move itself only lasts 1s, so a single call is
// not enough for Viam's continuous SetPower/SetVelocity semantics.
func (b *g1Base) startContinuousMove(vx, vy, vyaw float32) error {
	b.mu.Lock()
	b.cancelFn()
	moveCtx, moveFn := context.WithCancel(context.Background())
	b.cancelCtx = moveCtx
	b.cancelFn = moveFn
	b.mu.Unlock()

	if vx == 0 && vy == 0 && vyaw == 0 {
		b.loco.StopMove()
		b.moving.Store(false)
		return nil
	}

	// Bail if Stop already cancelled us before the first Move.
	select {
	case <-moveCtx.Done():
		b.moving.Store(false)
		return nil
	default:
	}

	// Issue the first Move synchronously so callers see failures, and so we
	// can re-StopMove if Stop raced while the RPC was in flight.
	if err := b.loco.Move(vx, vy, vyaw); err != nil {
		b.moving.Store(false)
		return moveError(err)
	}

	select {
	case <-moveCtx.Done():
		// Stop may have run StopMove before our Move completed; a late Move
		// would restart motion, so halt again.
		b.loco.StopMove()
		b.moving.Store(false)
		return nil
	default:
	}

	b.moving.Store(true)
	// Stop may have cancelled between the check above and Store(true);
	// don't leave IsMoving() stuck true and don't start the refresh loop.
	select {
	case <-moveCtx.Done():
		b.loco.StopMove()
		b.moving.Store(false)
		return nil
	default:
	}

	go func() {
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()

		for {
			select {
			case <-moveCtx.Done():
				return
			case <-ticker.C:
				sent, err := b.refreshMove(moveCtx, vx, vy, vyaw)
				if !sent {
					return
				}
				if err != nil {
					b.logger.Warnf("continuous move failed: %v", moveError(err))
				}
			}
		}
	}()
	return nil
}

func (b *g1Base) Stop(ctx context.Context, extra map[string]interface{}) error {
	// StopMove goes out under b.mu, so no refresh can follow it (see refreshMove).
	b.mu.Lock()
	b.cancelFn()
	b.loco.StopMove()
	b.mu.Unlock()

	b.moving.Store(false)
	return nil
}

// refreshMove re-sends a walk command for a move in progress, unless the move
// was cancelled. A select with both the cancellation and the ticker ready picks
// either, and a Move sent after Stop's StopMove restarts walking for a second
// (each command lasts 1 s). Checking and sending under b.mu, which Stop holds
// while it cancels and stops, rules that out.
func (b *g1Base) refreshMove(moveCtx context.Context, vx, vy, vyaw float32) (sent bool, err error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	if moveCtx.Err() != nil {
		return false, nil
	}
	return true, b.loco.Move(vx, vy, vyaw)
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

	var err error
	switch cmdStr {
	case "stand_up":
		_, err = b.loco.StandUp()
	case "sit":
		_, err = b.loco.Sit()
	case "squat":
		_, err = b.loco.Squat()
	case "high_stand":
		_, err = b.loco.HighStand()
	case "low_stand":
		_, err = b.loco.LowStand()
	case "balance_stand":
		_, err = b.loco.BalanceStand()
	case "damp":
		_, err = b.loco.Damp()
	case "zero_torque":
		_, err = b.loco.ZeroTorque()
	case "wave_hand":
		_, err = b.loco.WaveHand()
	case "start":
		_, err = b.loco.Start()
	case "stop_move":
		err = b.loco.StopMove()
	default:
		return map[string]interface{}{"error": "unknown command: " + cmdStr}, nil
	}

	result := map[string]interface{}{"rc": 0.0}
	if err != nil {
		result["rc"] = -1.0
		result["error"] = err.Error()
	}
	return result, nil
}

func (b *g1Base) Close(ctx context.Context) error {
	b.mu.Lock()
	b.cancelFn()
	b.mu.Unlock()

	b.loco.StopMove()
	b.loco.Close()
	ShutdownDDS()
	return nil
}

// moveError explains a refused walk command when arm_sdk is engaged: Running
// mode (FSM 802) doesn't walk while arm_sdk holds the arms.
func moveError(err error) error {
	if armSDKEngaged() {
		return fmt.Errorf("walk command failed while the arms are engaged through arm_sdk. "+
			"Walking with the arms engaged needs Regular mode (FSM 200, which ready_to_move enters), "+
			"not Running mode (802); or release the arms with an arm component's \"release\" command: %w", err)
	}
	return err
}
