package main

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"

	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var g1MovementSensorModel = resource.NewModel("erh", "viam-unitree", "g1-movementsensor")

// G1MovementSensorConfig configures the G1 odometer movementsensor.
//
// See https://support.unitree.com/home/en/G1_developer/odometer_service_interface
type G1MovementSensorConfig struct {
	NetworkInterface string `json:"network_interface"`
	// Topic defaults to "rt/odommodestate" (500Hz). Use "rt/lf/odommodestate"
	// for the 20Hz stream. Requires Unitree State Estimator >= 1.0.0.1.
	Topic string `json:"topic"`
}

func (c *G1MovementSensorConfig) Validate(path string) ([]string, error) {
	return nil, nil
}

func init() {
	resource.RegisterComponent(movementsensor.API, g1MovementSensorModel,
		resource.Registration[movementsensor.MovementSensor, *G1MovementSensorConfig]{
			Constructor: newG1MovementSensor,
		})
}

type g1MovementSensor struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	odom   *OdometerClient

	mu sync.Mutex
}

func newG1MovementSensor(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (movementsensor.MovementSensor, error) {
	cfg, err := resource.NativeConfig[*G1MovementSensorConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}
	topic := "rt/odommodestate"
	if cfg.Topic != "" {
		topic = cfg.Topic
	}

	logger.Infof("Initializing G1MovementSensor (interface=%s topic=%s)", networkInterface, topic)

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	odom, err := NewOdometerClient(topic)
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("odometer subscribe: %w", err)
	}

	logger.Info("G1MovementSensor initialized")
	return &g1MovementSensor{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		odom:   odom,
	}, nil
}

func (s *g1MovementSensor) client() (*OdometerClient, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.odom == nil {
		return nil, fmt.Errorf("movementsensor closed")
	}
	return s.odom, nil
}

func (s *g1MovementSensor) Position(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
	// Odometer reports local Cartesian meters, not GPS lat/lng.
	return nil, 0, movementsensor.ErrMethodUnimplementedPosition
}

func (s *g1MovementSensor) LinearVelocity(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	odom, err := s.client()
	if err != nil {
		return r3.Vector{}, err
	}
	st, err := odom.Latest()
	if err != nil {
		return r3.Vector{}, err
	}
	return r3.Vector{
		X: float64(st.Velocity[0]),
		Y: float64(st.Velocity[1]),
		Z: float64(st.Velocity[2]),
	}, nil
}

func (s *g1MovementSensor) AngularVelocity(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
	odom, err := s.client()
	if err != nil {
		return spatialmath.AngularVelocity{}, err
	}
	st, err := odom.Latest()
	if err != nil {
		return spatialmath.AngularVelocity{}, err
	}
	// Viam expects deg/s. Prefer full IMU gyro; fall back to yaw_speed on Z.
	return spatialmath.AngularVelocity{
		X: float64(st.Gyro[0]) * 180.0 / math.Pi,
		Y: float64(st.Gyro[1]) * 180.0 / math.Pi,
		Z: float64(st.YawSpeed) * 180.0 / math.Pi,
	}, nil
}

func (s *g1MovementSensor) LinearAcceleration(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	odom, err := s.client()
	if err != nil {
		return r3.Vector{}, err
	}
	st, err := odom.Latest()
	if err != nil {
		return r3.Vector{}, err
	}
	return r3.Vector{
		X: float64(st.Accel[0]),
		Y: float64(st.Accel[1]),
		Z: float64(st.Accel[2]),
	}, nil
}

func (s *g1MovementSensor) CompassHeading(ctx context.Context, extra map[string]interface{}) (float64, error) {
	return 0, movementsensor.ErrMethodUnimplementedCompassHeading
}

func (s *g1MovementSensor) Orientation(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
	odom, err := s.client()
	if err != nil {
		return nil, err
	}
	st, err := odom.Latest()
	if err != nil {
		return nil, err
	}
	// Unitree quaternion is w, x, y, z.
	q := &spatialmath.Quaternion{
		Real: float64(st.Quaternion[0]),
		Imag: float64(st.Quaternion[1]),
		Jmag: float64(st.Quaternion[2]),
		Kmag: float64(st.Quaternion[3]),
	}
	return q, nil
}

func (s *g1MovementSensor) Properties(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
	return &movementsensor.Properties{
		PositionSupported:           false, // Cartesian odom, not GPS — see Readings
		OrientationSupported:        true,
		CompassHeadingSupported:     false,
		LinearVelocitySupported:     true,
		AngularVelocitySupported:    true,
		LinearAccelerationSupported: true,
	}, nil
}

func (s *g1MovementSensor) Accuracy(ctx context.Context, extra map[string]interface{}) (*movementsensor.Accuracy, error) {
	return movementsensor.UnimplementedOptionalAccuracies(), nil
}

func (s *g1MovementSensor) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	readings, err := movementsensor.DefaultAPIReadings(ctx, s, extra)
	if err != nil {
		return nil, err
	}

	odom, err := s.client()
	if err != nil {
		return nil, err
	}
	st, err := odom.Latest()
	if err != nil {
		return nil, err
	}
	readings["position_meters"] = map[string]interface{}{
		"x": float64(st.Position[0]),
		"y": float64(st.Position[1]),
		"z": float64(st.Position[2]),
	}
	readings["body_height_m"] = float64(st.BodyHeight)
	readings["rpy_rad"] = map[string]interface{}{
		"roll":  float64(st.RPY[0]),
		"pitch": float64(st.RPY[1]),
		"yaw":   float64(st.RPY[2]),
	}
	readings["yaw_speed_rad_s"] = float64(st.YawSpeed)
	return readings, nil
}

func (s *g1MovementSensor) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *g1MovementSensor) Close(ctx context.Context) error {
	s.mu.Lock()
	odom := s.odom
	s.odom = nil
	s.mu.Unlock()

	if odom == nil {
		return nil // already closed; don't double-release the InitDDS ref
	}
	odom.Close()
	ShutdownDDS()
	return nil
}
