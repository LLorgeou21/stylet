# stylet_description

URDF/xacro robot model: a UR5e arm plus a custom needle-insertion end effector.

## Overview

`urdf/robot.urdf.xacro` imports the UR5e macro from `ur_description` and adds:

- **`needle_base`** / **`needle_tcp`**: the needle itself and its tool-center-point frame.
- **`needle_stage1_joint` / `needle_stage2_joint` / `needle_joint`**: three prismatic joints in series that drive needle insertion. The needle is too short to reach some targets by moving the arm alone (see the top-level `ARCHITECTURE.md`), so a dedicated linear actuator - decoupled from the arm - pushes it forward instead. `needle_stage1_link`/`needle_stage2_link` are purely visual (no collision, no physical role): they exist only so the render never shows a floating gap between the wrist and the needle as it extends (a discrete approximation of a telescoping tube).
- A force/torque sensor (`needle_ft_sensor`) attached to `needle_joint`, bridged to `/stylet/haptics/wrench` in simulation.
- A `collide_bitmask` on `needle_base` that disables *physical* contact with the target in Gazebo (no tissue model exists yet - see Phase 4 / `stylet_haptics`).

This package only describes the robot; it does not launch a simulation or add `<ros2_control>` tags itself - `stylet_moveit_config`'s `stylet.urdf.xacro` combines this file with the ros2_control hardware interface used by both Gazebo and MoveIt.

## Launch files

| File | Purpose |
|---|---|
| `launch/display.launch.py` | Standalone viewer: `robot_state_publisher` + `joint_state_publisher_gui` (manual joint sliders) + RViz. No Gazebo, no MoveIt - just for inspecting the model. |

## Usage

```bash
ros2 launch stylet_description display.launch.py
```

## Dependencies

`xacro`, `ur_description` (UR5e macro), `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`.
