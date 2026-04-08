# Model erh:viam-unitree:g1-base

Base component for Unitree G1 humanoid robot locomotion. Wraps the Unitree SDK2 LocoClient to provide move, spin, and velocity control through the Viam Base API.

## Configuration

```json
{
  "network_interface": "eth0"
}
```

### Attributes

| Name                | Type   | Inclusion | Description                                              |
|---------------------|--------|-----------|----------------------------------------------------------|
| `network_interface` | string | Optional  | Network interface for DDS communication (default: eth0)  |

## DoCommand

Control G1 stance and special actions via DoCommand:

```json
{
  "command": "stand_up"
}
```

### Supported commands

| Command          | Description                   |
|------------------|-------------------------------|
| `stand_up`       | Stand up from ground          |
| `sit`            | Sit down                      |
| `squat`          | Squat position                |
| `high_stand`     | High standing position        |
| `low_stand`      | Low standing position         |
| `balance_stand`  | Balanced standing position    |
| `damp`           | Zero torque / damping mode    |
| `zero_torque`    | Zero all motor torques        |
| `wave_hand`      | Wave hand gesture             |
| `start`          | Start locomotion system       |
| `stop_move`      | Stop all movement             |
