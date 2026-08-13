# Module viam-unitree

Viam module for the Unitree G1 humanoid robot.

## Models

- [`erh:viam-unitree:g1-base`](erh_viam-unitree_g1-base.md) - Base component for G1 locomotion (move, spin, velocity control)
- [`erh:viam-unitree:g1-camera`](erh_viam-unitree_g1-camera.md) - Camera component for G1 video capture
- `erh:viam-unitree:g1-lidar` - Lidar component (point clouds) for G1. Default backend is Livox-SDK2 Mid-360; set `"source": "dds"` for Unitree DDS topics.
- `erh:viam-unitree:g1` - Generic component with high-level commands (incl. pre-built arm gestures)
- [`erh:viam-unitree:g1-left-arm`](erh_viam-unitree_g1-arm.md) - Left arm joint control via arm_sdk
- [`erh:viam-unitree:g1-right-arm`](erh_viam-unitree_g1-arm.md) - Right arm joint control via arm_sdk
- [`erh:viam-unitree:g1-movementsensor`](erh_viam-unitree_g1-movementsensor.md) - Odometer movement sensor (pose, velocity, orientation)
