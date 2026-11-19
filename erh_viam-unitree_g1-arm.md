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
targets. Commands that need the arms, or that take torque away from the robot,
release `arm_sdk` first: gestures, and mode changes to anything but the two
standing states (damp, zero torque, sit, the start-up sequence, and
`set_speed_mode`, `set_balance_mode`, `try_fsm`). The weight ramps to 0, then
the `rt/arm_sdk` writer closes. The next arm move engages again. The module
releases the same way when it shuts down.

Changes between the standing states keep the arms engaged, so they go on
holding their pose and anything in the hands: stand (FSM 4) to walk (200) and
back, through the `g1-mode` switch, `ready_to_move` or `set_fsm_id`. If the
robot doesn't take such a change with the arms engaged, the module releases
them and asks again, and logs that it did.

### Walking with the arms engaged

Base moves don't release `arm_sdk`, so the robot can walk while the arms hold a
pose, for example to carry something. That needs the right walk mode:

| Mode | FSM | Walks with the arms engaged |
|------|-----|-----------------------------|
| Regular (Unitree's "main operation control") | 200 | Yes |
| Running | 802 | No: it ignores walk commands until the arms are released |

`ready_to_move` and the `g1-mode` switch's "walk" position enter Regular mode
(from power-up: Damp → StandUp → Start). The robot ignores Start until its
stand-up has finished, so they ask for up to about 16 s and return an error if
it never takes it. If the robot is already in Regular mode they do nothing and
leave the arms engaged. On this firmware Regular mode
is only accepted from StandUp, and leaving Running mode takes Damp, which
drops a standing robot. So for a robot in Running mode (set from the remote,
or by an older version of this module), `ready_to_move` does nothing and
says so in its reply; with the robot supported, `{"command": "ready_to_move",
"force": true}` runs the full sequence and ends in Regular mode.

This matches Unitree's teleoperation documentation (arm control alongside
walking supports only Regular mode) and was confirmed on a G1: walking with
`arm_sdk` engaged works in FSM 200 and not in 802.

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
| `gravity_compensation` | Read the gravity compensation scale and each upper-body joint's feedforward torque at the current pose (`torque_nm`). Pass `scale` (0 off, 1 full model, up to 1.5) to change it. Shared by both arms. |

## Gravity compensation

A position hold only pushes back once a joint has moved off its target, so
weight shows up as droop: two arms reaching forward (7 kg) lean the waist about
15° at kp 60, and the shoulders sag below their targets. While `arm_sdk` is
engaged, each waist and arm command also carries the torque gravity puts on
that joint, as feedforward (`tau`), so the joints hold their targets.

The torque comes from the masses and centers of mass in the embedded URDF
(16.9 kg above the waist), the measured joint angles, and the pelvis tilt from
the IMU in `rt/lowstate`, recomputed every 20 ms. It's capped at half of each
motor's rated torque from the URDF: waist roll and pitch 17.5 N·m, waist yaw 44,
shoulders, elbows and wrist roll 12.5, wrist pitch and yaw 2.5. Anything the
hands hold isn't in the model, so heavy objects still droop.

### Safety limits

While `arm_sdk` drives the upper body, the module also:

- **Releases if the robot tips:** if the pelvis tilts more than 15° from upright
  (falling, or hanging in a harness), `arm_sdk` fades out over 0.2 s and hands
  the upper body back to the sport controller. It won't engage while tilted.
- **Lets the waist give way:** if a waist joint is pushed more than 0.15 rad off
  its hold, the hold follows it, capping the position hold at about 9 N·m
  instead of fighting harder. (A waist roll motor, rated 35 N·m, faulted when
  the robot hung tilted in a harness with the waist held rigid.)
- **Logs heavy commands:** any upper-body motor commanded above 80% of its
  rated torque is logged with the hold and gravity parts.

It's on by default (scale 1). On a robot running it for the first time, set
`{"command": "gravity_compensation", "scale": 0.5}` before moving an arm, check on
the gantry that the torso and arms hold better rather than worse, then go to 1.
`scale: 0` turns it off. The scale resets to 1 when the module restarts.

## Troubleshooting

- The module logs when `arm_sdk` engages and releases, gravity scale changes,
  any change in a motor's error code from `rt/lowstate` (with its temperature
  and estimated torque), and gaps of over a second in `rt/lowstate`. Check
  these logs first when the robot goes limp.
- `{"command": "motor_status"}` on the `erh:viam-unitree:g1` component (read-only)
  returns every motor's angle, estimated torque, temperatures and error code,
  the IMU attitude, and `arm_sdk`'s weight, writer and gravity scale.
- If the robot reports walk mode but is limp, `ready_to_move` sees walk mode and
  does nothing; send `{"command": "ready_to_move", "force": true}` to run the
  full stand-up sequence.
- If the base doesn't move while the arms are engaged, the robot is probably in
  Running mode (FSM 802; check with `get_modes`). See
  [Walking with the arms engaged](#walking-with-the-arms-engaged).
- `{"command": "motion_mode"}` on the `g1` component (read-only) shows which
  motion controller is running. An empty name means none, and the sport service
  won't answer until one is.
- A motor fault (an error code in `motor_status`) disables every motor and stays
  latched; power-cycle the robot to clear it, then `ready_to_move`.

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
  body is Unitree's documented design. On a G1 it stood and, in Regular mode,
  walked with the arms engaged; how well it recovers from pushes or carries a
  load in that state hasn't been tested.
