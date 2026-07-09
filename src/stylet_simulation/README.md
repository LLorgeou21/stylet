# stylet_simulation

Gazebo (Harmonic) simulation: world, target object, sensor bridges, and controller spawning.

## Overview

`launch/sim.launch.py` is the single source of truth for `/robot_description` (shared between Gazebo and MoveIt - see `stylet_moveit_config`, ADR-030/ADR-033) and starts everything needed for the robot to exist and move inside Gazebo:

1. Starts Gazebo with `worlds/stylet.world`, running (not paused - required for `gz_ros2_control` to tick).
2. Publishes `robot_description` via `robot_state_publisher` (`use_sim_time=True`).
3. Spawns the robot into Gazebo.
4. Spawns the `ros2_control` controllers hosted inside Gazebo: `joint_state_broadcaster`, `arm_controller` (6-DOF arm), `needle_insertion_controller` (3-stage needle actuator).
5. Bridges Gazebo topics to ROS 2: `/clock` (simulation time), the force/torque sensor (`/stylet/haptics/wrench`), and 5 LiDAR point clouds (`/lidar_1..5/points`).
6. Publishes static TF for the 5 fixed LiDAR sensor frames (defined only in SDF, never published as TF otherwise).

## World / models

| Path | Purpose |
|---|---|
| `worlds/stylet.world` | Gazebo world: ground, lighting, 5 `gpu_lidar` sensors around the workspace |
| `models/target/` | The asymmetric target object (mesh-based collision, `collide_bitmask` tuned to not physically collide with the needle - no tissue model yet) |

## Topics published (via bridges)

| Topic | Type | Source |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo sim time |
| `/lidar_1..5/points` | `sensor_msgs/msg/PointCloud2` | 5 LiDAR sensors |
| `/stylet/haptics/wrench` | `geometry_msgs/msg/WrenchStamped` | Force/torque sensor on `needle_joint` |

## Usage

```bash
ros2 launch stylet_simulation sim.launch.py
```

Starts Gazebo with the robot and controllers active, but no perception, planning, or UI - see `stylet_bringup` for the full stack.

## Dependencies

`ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `tf2_ros`, `xacro`, `controller_manager`, `stylet_description`, `stylet_moveit_config`.
