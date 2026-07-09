# stylet_bringup

Single-launch entry point for the complete Phase 1-3 demo.

## Overview

`launch/full_demo.launch.py` starts every piece of the system in the right order with no manual steps:

1. **Immediately**: `stylet_simulation`'s `sim.launch.py` (Gazebo, robot, controllers, sensor bridges) and all 3 `stylet_perception` nodes (`point_cloud_merger`, `cloud_preprocessor`, `surface_registration`) - the perception nodes already retry internally on TF/topics that aren't ready yet, so no explicit wait is needed here.
2. **After an 8s delay**: `stylet_moveit_config`'s `move_group.launch.py` - gives Gazebo and the `ros2_control` controllers time to activate before `move_group` looks for a valid robot state.
3. **After a 10s delay**: `moveit_rviz.launch.py` (RViz, with the MoveIt plugin and the `stylet_ui` panel pre-loaded) and `stylet_planning`'s `procedure_planner.launch.py`.

This mirrors the order used manually throughout Phase 3 development. If the delays turn out to be too short on a given machine (heavier simulation load, slower startup), increase the `TimerAction` periods in `full_demo.launch.py`.

## Usage

```bash
ros2 launch stylet_bringup full_demo.launch.py
```

Then, once Gazebo and RViz are up and the LiDAR scan has converged (watch `/stylet/perception/fitness_score` or the aligned debug cloud in RViz): click "Set entry point" and pick a point on the target in the 3D view, then click "Launch operation" in the panel.

## Dependencies

`stylet_simulation`, `stylet_perception`, `stylet_moveit_config`, `stylet_planning` (all runtime-only - this package has no source of its own beyond the launch file).
