# Models erh:viam-unitree:g1-left-arm and erh:viam-unitree:g1-right-arm

Viam `arm` components for the Unitree G1's 7-DoF arms. They publish low-level
joint commands on the `rt/arm_sdk` DDS topic and read measured joint states
from `rt/lowstate`. The two models share the same underlying `arm_sdk`
publisher (singleton) and use disjoint motor-index slices: indices 15..21 for
the left arm, 22..28 for the right.

When at least one arm component is configured, motor 29's "weight" field is
held at 1.0 so `arm_sdk` fully overrides sport-mode arm motion. The DoCommand
`release` resets it to 0 to hand the arms back to the sport controller.

## Configuration

```json
{
  "network_interface": "eth0"
}
```

### Attributes

| Name                | Type      | Inclusion | Description                                                                          |
|---------------------|-----------|-----------|--------------------------------------------------------------------------------------|
| `network_interface` | string    | Optional  | Network interface for DDS communication (default: `eth0`).                           |
| `model_path`        | string    | Optional  | Override the embedded kinematics (`.urdf` or `.json`). Joint names must be G1 joint names (e.g. `left_elbow_joint`). |
| `kp`                | float[7]  | Optional  | Per-joint position gains. Defaults: `[60, 60, 60, 60, 30, 30, 30]`.                  |
| `kd`                | float[7]  | Optional  | Per-joint velocity gains. Defaults: `[1.5, 1.5, 1.5, 1.5, 1.0, 1.0, 1.0]`.           |

Kinematics come from the G1 29-DoF URDF ([isri-aist/g1_description](https://github.com/isri-aist/g1_description)),
embedded in the module. Each arm's model starts at `torso_link` and ends at the
palm (`left_rubber_hand` / `right_rubber_hand`), so to place the arms on the
robot, set their frame parent to a [`g1-body`](erh_viam-unitree_g1-body.md)
component with no offset.

### Joint order

Both arms expose 7 DoF in shoulder-to-wrist order:

| Index | Joint                |
|-------|----------------------|
| 0     | Shoulder Pitch       |
| 1     | Shoulder Roll        |
| 2     | Shoulder Yaw         |
| 3     | Elbow                |
| 4     | Wrist Roll           |
| 5     | Wrist Pitch          |
| 6     | Wrist Yaw            |

## DoCommand

| Command       | Description                                                                  |
|---------------|------------------------------------------------------------------------------|
| `engage`      | Set arm_sdk weight to 1.0 (arm_sdk fully owns the arms).                     |
| `release`     | Set arm_sdk weight to 0.0 (sport mode reclaims the arms).                    |
| `set_weight`  | Set arm_sdk weight to a custom value 0..1; takes a `weight` numeric arg.     |

## Notes / Caveats

- The CDR layouts for `unitree_hg::msg::dds_::LowCmd_` / `LowState_` are
  derived from the public Unitree SDK2 documentation. They have been written
  to match the on-the-wire format used by the G1 `rt/arm_sdk` topic, but
  hardware verification is recommended for any deployment.
- Tune `kp` / `kd` for the specific G1 variant in use.
- For pre-recorded arm gestures (wave, hug, hands-up, etc.) use the generic
  `erh:viam-unitree:g1` component's DoCommand interface — those run on the
  G1's built-in arm-action library and don't need arm_sdk.
