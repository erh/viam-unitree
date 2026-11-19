# Models erh:viam-unitree:g1-left-arm and erh:viam-unitree:g1-right-arm

Viam `arm` components for the Unitree G1's 7-DoF arms. They publish low-level
joint commands on the `rt/arm_sdk` DDS topic and read measured joint states
from `rt/lowstate`. The two models share the same underlying `arm_sdk`
publisher (singleton) and use disjoint motor-index slices: indices 15..21 for
the left arm, 22..28 for the right.

### Who controls the arms

`arm_sdk` starts released, leaving the whole robot to the sport controller.
The first arm move engages it: motor 29's "weight" ramps to 1.0 over a second
and `arm_sdk` takes over the upper body, following Unitree's
`g1_arm7_sdk_dds_example`:

- **Arms** go where the arm components command them.
- **Waist** (yaw, roll, pitch) holds the pose it had when `arm_sdk` engaged.
- **Legs** are not commanded, so the balance controller keeps them.

Engaging always starts from the measured pose, so the arms don't jump to old
targets. Sport-service commands that need the arms or change mode release
`arm_sdk` first: gestures, `ready_to_move`, `set_fsm_id`, `set_speed_mode`,
`set_balance_mode`, `try_fsm` and the `g1-mode` switch. The weight ramps to 0,
then the `rt/arm_sdk` writer closes, because the firmware rejects mode changes
while it's open. The next arm move engages again.

Base moves don't release `arm_sdk`. If the robot refuses to walk while the
arms are engaged, the base returns an error saying so; send `release` first.

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
| `model_path`        | string    | Optional  | Override the embedded kinematics (`.urdf` or `.json`). It must have exactly that arm's 7 joints, named as in the G1 URDF (e.g. `left_elbow_joint`), in any order. |
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
| `engage`      | Ramp arm_sdk weight to 1.0, holding the current pose (arm moves do this automatically). |
| `release`     | Ramp arm_sdk weight to 0.0 and close the writer (the sport controller reclaims the upper body). |
| `set_weight`  | Set arm_sdk weight to a custom value 0..1 at once; takes a `weight` numeric arg. |

## Notes / Caveats

- The CDR layouts for `unitree_hg::msg::dds_::LowCmd_` / `LowState_` are
  derived from the public Unitree SDK2 documentation. They have been written
  to match the on-the-wire format used by the G1 `rt/arm_sdk` topic, but
  hardware verification is recommended for any deployment.
- Tune `kp` / `kd` for the specific G1 variant in use.
- For pre-recorded arm gestures (wave, hug, hands-up, etc.) use the generic
  `erh:viam-unitree:g1` component's DoCommand interface — those run on the
  G1's built-in arm-action library and don't need arm_sdk. They release
  arm_sdk first, so the arms end wherever the gesture leaves them.
- Leaving the legs to the balance controller while `arm_sdk` drives the upper
  body is Unitree's documented design, but it has not yet been confirmed on
  this firmware. Test with the robot on a gantry or harness: it should keep
  balancing (small leg corrections, recovering from a gentle push) while an
  arm moves.
