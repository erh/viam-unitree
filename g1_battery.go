package main

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/powersensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// g1-battery exposes the G1's battery management state as a power_sensor.
// It reads the hg BmsState_ message the G1 publishes on "rt/lf/bmsstate"
// (the hg LowState has no battery field). Voltage/Current/Power and state of
// charge (BatteryPercent) come through the native power_sensor API;
// SOH/temperature/cycle come through Readings.
var g1BatteryModel = resource.NewModel("erh", "viam-unitree", "g1-battery")

type g1BatteryConfig struct {
	NetworkInterface string `json:"network_interface"`
	// Topic defaults to "rt/lf/bmsstate".
	Topic string `json:"topic"`
}

func (c *g1BatteryConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

func init() {
	resource.RegisterComponent(powersensor.API, g1BatteryModel,
		resource.Registration[powersensor.PowerSensor, *g1BatteryConfig]{
			Constructor: newG1Battery,
		})
}

type g1Battery struct {
	resource.Named
	resource.AlwaysRebuild

	logger logging.Logger
	bms    *BmsClient
}

func newG1Battery(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (powersensor.PowerSensor, error) {
	cfg, err := resource.NativeConfig[*g1BatteryConfig](conf)
	if err != nil {
		return nil, err
	}

	networkInterface := "eth0"
	if cfg.NetworkInterface != "" {
		networkInterface = cfg.NetworkInterface
	}
	topic := "rt/lf/bmsstate"
	if cfg.Topic != "" {
		topic = cfg.Topic
	}

	if err := InitDDS(0, networkInterface); err != nil {
		return nil, fmt.Errorf("DDS init: %w", err)
	}

	bms, err := NewBmsClient(topic)
	if err != nil {
		ShutdownDDS()
		return nil, fmt.Errorf("bms client: %w", err)
	}

	return &g1Battery{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
		bms:    bms,
	}, nil
}

// Voltage returns pack voltage in volts (DC).
func (b *g1Battery) Voltage(ctx context.Context, extra map[string]interface{}) (float64, bool, error) {
	st, err := b.bms.Latest()
	if err != nil {
		return 0, false, err
	}
	return st.Voltage, false, nil
}

// Current returns pack current in amps (DC); negative when discharging.
func (b *g1Battery) Current(ctx context.Context, extra map[string]interface{}) (float64, bool, error) {
	st, err := b.bms.Latest()
	if err != nil {
		return 0, false, err
	}
	return st.Current, false, nil
}

// Power returns instantaneous pack power in watts (voltage × current).
func (b *g1Battery) Power(ctx context.Context, extra map[string]interface{}) (float64, error) {
	st, err := b.bms.Latest()
	if err != nil {
		return 0, err
	}
	return st.Voltage * st.Current, nil
}

// BatteryPercent returns the remaining battery charge as a percentage from 0 to 100.
func (b *g1Battery) BatteryPercent(ctx context.Context, extra map[string]interface{}) (float64, error) {
	st, err := b.bms.Latest()
	if err != nil {
		return 0, err
	}
	return float64(st.SOC), nil
}

// Properties reports that this power sensor supports battery percent (state of charge).
func (b *g1Battery) Properties(ctx context.Context, extra map[string]interface{}) (*powersensor.Properties, error) {
	return &powersensor.Properties{IsBatteryPercentSupported: true}, nil
}

// Readings surfaces the fields the power_sensor API can't: state of charge,
// state of health, cycle count, and pack temperature.
func (b *g1Battery) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	st, err := b.bms.Latest()
	if err != nil {
		return nil, err
	}
	return map[string]interface{}{
		"soc_percent": st.SOC,
		"soh_percent": st.SOH,
		"voltage_v":   st.Voltage,
		"current_a":   st.Current,
		"power_w":     st.Voltage * st.Current,
		"cycle":       st.Cycle,
		"temp_max_c":  st.MaxTempC,
		"temp_min_c":  st.MinTempC,
	}, nil
}

func (b *g1Battery) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (b *g1Battery) Close(ctx context.Context) error {
	if b.bms != nil {
		b.bms.Close()
		b.bms = nil
	}
	ShutdownDDS()
	return nil
}
