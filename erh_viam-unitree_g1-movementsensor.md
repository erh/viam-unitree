# Model erh:viam-unitree:g1-movementsensor

Movement sensor backed by the Unitree G1 [odometer service](https://support.unitree.com/home/en/G1_developer/odometer_service_interface). Subscribes to a `SportModeState_` DDS topic and exposes orientation, linear/angular velocity, linear acceleration, and Cartesian position (via Readings).

Requires State Estimator >= 1.0.0.1.

## Configuration

```json
{
  "network_interface": "eth0",
  "topic": "rt/lf/odommodestate"
}
```

### Attributes

| Name                | Type   | Inclusion | Description                                                                 |
|---------------------|--------|-----------|-----------------------------------------------------------------------------|
| `network_interface` | string | Optional  | Network interface for DDS (default: eth0)                                   |
| `topic`             | string | Optional  | DDS topic (default: `rt/lf/odommodestate` at 20Hz; use `rt/odommodestate` for 500Hz) |

## Data

| Method / Reading        | Source                                      | Notes                                      |
|-------------------------|---------------------------------------------|--------------------------------------------|
| Orientation             | `imu_state.quaternion` (w,x,y,z)            |                                            |
| LinearVelocity          | `velocity` (m/s, world frame)               |                                            |
| AngularVelocity         | gyro + `yaw_speed`                          | Reported in deg/s                          |
| LinearAcceleration      | `imu_state.accelerometer`                   |                                            |
| Readings.position_meters| `position` (m, world frame x/y/z)           | Local odom; GPS `Position` is unimplemented|
| Readings.rpy_rad        | `imu_state.rpy`                             | roll/pitch/yaw in radians                  |
| Readings.yaw_speed_rad_s| `yaw_speed`                                 | Body-frame yaw rate                        |
| CompassHeading / Position (GPS) | —                                   | Unimplemented                              |
