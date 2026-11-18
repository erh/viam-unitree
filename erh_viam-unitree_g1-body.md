# Model erh:viam-unitree:g1-body

Read-only Viam `arm` component that puts the G1's pelvis, legs and waist into
the frame system, with live joint angles from `rt/lowstate`. Its kinematics
come from the G1 29-DoF URDF ([isri-aist/g1_description](https://github.com/isri-aist/g1_description)),
embedded in the module. The component's frame is the pelvis and its end
position is `torso_link`, where the arms attach.

It is an `arm` because only arm, gantry and gripper components can serve a
kinematic model to the frame system. All move commands return an error: the
legs belong to the locomotion controller.

## Full robot in the frame system

Give the body a frame, and make it the frame parent of both arms with no
offset. The arm models start at `torso_link`.

```json
{
  "components": [
    { "name": "body", "api": "rdk:component:arm", "model": "erh:viam-unitree:g1-body",
      "attributes": {}, "frame": { "parent": "world" } },
    { "name": "left-arm", "api": "rdk:component:arm", "model": "erh:viam-unitree:g1-left-arm",
      "attributes": {}, "frame": { "parent": "body" } },
    { "name": "right-arm", "api": "rdk:component:arm", "model": "erh:viam-unitree:g1-right-arm",
      "attributes": {}, "frame": { "parent": "body" } }
  ]
}
```

The head sensors are fixed links on `torso_link` in the URDF, so they can also use `body` as their frame parent:
`d435_link` sits at (57.6, 17.5, 419.9) mm, pitched 0.831 rad, and `mid360_link` sits at
(0.3, 0.0, 406.2) mm, pitched 0.040 rad. These are link frames (x forward), not
camera optical frames.

## Configuration

| Name                | Type   | Inclusion | Description                                                              |
|---------------------|--------|-----------|--------------------------------------------------------------------------|
| `network_interface` | string | Optional  | Network interface for DDS communication (default: `eth0`).               |
| `model_path`        | string | Optional  | Replace the embedded kinematics (`.json` or `.urdf`). Joint names must be G1 joint names. |

## Joint order

15 joints, in the order the model lists them (breadth-first from the pelvis):
left_hip_pitch, right_hip_pitch, waist_yaw, left_hip_roll, right_hip_roll,
waist_roll, left_hip_yaw, right_hip_yaw, waist_pitch, left_knee, right_knee,
left_ankle_pitch, right_ankle_pitch, left_ankle_roll, right_ankle_roll.

## Caveats

- Collision geometry is one box per link (body and arms), sized to the bounds of the upstream
  meshes. The meshes themselves (about 25 MB of STL) are not embedded, so the 3D view shows boxes.
- The motion planner ignores shape pairs that already overlap at the start of a move, so an arm
  resting against the torso box may pass through it during that move.
- Motion planning: plan arm goals in the `body` frame. A goal in `world` puts the body's
  joints in the planner's chain, and the body refuses to move.
