# stylet_planning

The procedure planner: plans and executes the approach + needle insertion, with force-triggered replanning.

## Overview

`procedure_planner` is a single node that hosts the `execute_procedure` action server (`stylet_msgs/action/ExecuteProcedure`). Given an entry point (published by `stylet_ui`'s picking tool) and a fixed target point (a parameter, transformed via `target_frame` published by `stylet_perception`), it:

1. **Retracts the needle actuator to 0** and moves the arm through the SRDF `"up"` named state (a known, target-far configuration - mitigates an intermittent Gazebo/MoveIt clock-synchronization issue that can otherwise make `move_group` report success from a stale robot state).
2. **Plans and executes the approach**: a pose 5cm back from the entry point along the entry→target axis, with orientation built from that axis alone (a single vector only fixes 2 of 3 rotational degrees of freedom - the third is completed via an arbitrary reference vector and cross products). Independently re-verifies the real `needle_tcp` position via TF after execution - MoveIt's own reported "success" is not trusted blindly.
3. **Inserts the needle**: the arm does not move during this phase (see the top-level `ARCHITECTURE.md` for why - a purely arm-driven Cartesian insertion was the original design, abandoned when it required the wrist to enter the target). Instead, `/needle_insertion_controller`'s 3 joints are commanded directly, computed from real TF each attempt. `/stylet/haptics/wrench` is monitored continuously; if the axial force exceeds a threshold, the trajectory is cancelled and replanned from the real (re-measured) position, up to `replanning_max_attempts` times.
4. Reports rich metrics on completion (position/alignment error, insertion depth, max force observed, attempts, duration) in the action result.

Also maintains the target's MoveIt collision object (the real mesh, not a bounding box - a box was tried and rejected, it covered too much legitimately-empty space around the asymmetric target) and its allowed-collision-matrix exemption for the needle (fetched, extended, and reapplied in full - a partial diff was tried first and broke the SRDF's own self-collision exemptions, since MoveIt replaces rather than merges an ACM diff).

## Node

| Node | Purpose |
|---|---|
| `procedure_planner` | See above. Constructs its own `MoveGroupInterface` for the "arm" group; talks to `arm_controller` and `needle_insertion_controller` directly via `FollowJointTrajectory` action clients (not through `MoveGroupInterface::execute()`, which isn't safe to call concurrently with `stop()` from another thread). |

## Topics / actions

| Name | Type | Direction |
|---|---|---|
| `/procedure/entry_point` | `geometry_msgs/msg/PointStamped` | sub |
| `/stylet/haptics/wrench` | `geometry_msgs/msg/WrenchStamped` | sub |
| `/procedure/target_point` | `geometry_msgs/msg/PointStamped` | pub (transient_local, published once) |
| `/procedure/entry_pose_robot`, `/procedure/target_pose_robot` | `geometry_msgs/msg/PoseStamped` | pub |
| `/stylet/system/state` | `std_msgs/msg/String` | pub (transient_local; `READY`/`APPROACHING`/`INSERTING`/`COMPLETED`/`ERROR`) |
| `/procedure/progress` | `std_msgs/msg/Float32` | pub |
| `execute_procedure` | `stylet_msgs/action/ExecuteProcedure` | action server |
| `/arm_controller/follow_joint_trajectory`, `/needle_insertion_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | action clients |

## Key parameters

| Parameter | Default | Purpose |
|---|---|---|
| `target_point_target_frame` | `[0.150045, 0.13125, 0.1071]` | Fixed target point, in the target mesh's own local frame |
| `approach_retreat_distance_m` | `0.05` | Retreat distance for the approach pose |
| `force_stop_threshold_n` | `5.0` | Axial force that triggers a stop + replan |
| `replanning_max_attempts` | `3` | Max replanning attempts before aborting |
| `needle_length_m` | `0.15` | Fixed needle length (must match `robot.urdf.xacro`) |
| `insertion_speed_m_s` | `0.01` | Needle advance speed |

## Dependencies

`rclcpp`, `rclcpp_action`, `geometry_msgs`, `std_msgs`, `moveit_msgs`, `moveit_ros_planning_interface`, `tf2_ros`, `tf2_geometry_msgs`, `stylet_msgs`, `eigen`, `geometric_shapes`, `shape_msgs`, `ament_index_cpp`, `control_msgs`; runtime: `moveit_configs_utils`, `stylet_moveit_config`, `stylet_description`.

## Usage

Not standalone - requires `move_group` and the simulation's controllers to be running. See `stylet_bringup`.
