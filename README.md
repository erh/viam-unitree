# Module viam-unitree

Viam module for the Unitree G1 humanoid robot.

## Models

- [`erh:viam-unitree:g1-base`](erh_viam-unitree_g1-base.md) - Base component for G1 locomotion (move, spin, velocity control)
- [`erh:viam-unitree:g1-camera`](erh_viam-unitree_g1-camera.md) - Camera component for G1 video capture
- `erh:viam-unitree:g1-lidar` - Lidar component (point clouds) for G1. Default backend is Livox-SDK2 Mid-360; set `"source": "dds"` for Unitree DDS topics.
- `erh:viam-unitree:g1` - Generic component with high-level commands (incl. pre-built arm gestures)
- [`erh:viam-unitree:g1-left-arm`](erh_viam-unitree_g1-arm.md) - Left arm joint control via arm_sdk
- [`erh:viam-unitree:g1-right-arm`](erh_viam-unitree_g1-arm.md) - Right arm joint control via arm_sdk
- [`erh:viam-unitree:g1-body`](erh_viam-unitree_g1-body.md) - Read-only pelvis/legs/waist kinematics; parent of the arms in the frame system
- [`erh:viam-unitree:g1-movementsensor`](erh_viam-unitree_g1-movementsensor.md) - Odometer movement sensor (pose, velocity, orientation)
- [`erh:viam-unitree:g1-hand`](erh_viam-unitree_g1-hand.md) - BrainCo Revo2 hand as a gripper: open, grab, named poses and per-finger control over RS485

## Example fragment

[`g1_fragment_example.json`](g1_fragment_example.json) configures every model with its frame,
so the whole robot shows up connected in the frame system:

```
world
└── base                 ground under the pelvis
    ├── body             pelvis, 793 mm up (legs straight); its end is torso_link
    │   ├── left-arm     torso_link → left palm
    │   ├── right-arm    torso_link → right palm
    │   ├── camera       head camera (d435_link), optical frame: z out of the lens, x right
    │   └── lidar        mid360_link
    └── odometry         at the pelvis
```

The `g1` generic component has no frame. Offsets come from the URDF; the base-to-pelvis height
assumes straight legs, so the feet sit a little below the ground in a bent-knee stance.
