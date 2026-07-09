# stylet_ui

RViz 2 plugin: pick the needle entry point in 3D and drive/monitor the procedure from a dockable panel.

## Overview

Two plugins, registered via `plugins_description.xml`, pre-loaded by `stylet_moveit_config`'s `config/moveit.rviz`:

| Plugin | Type | Purpose |
|---|---|---|
| `EntryPointTool` | `rviz_common::Tool` | Activated from the panel's "Set entry point" button (or shortcut `e`). Left-click in the 3D view picks the 3D point under the cursor, transforms it into `target_frame`, and publishes it on `/procedure/entry_point`. Forwards all other mouse events to the current view controller so camera navigation keeps working while active. |
| `TargetDefinitionPanel` | `rviz_common::Panel` | Shows entry/target coordinates, `needle_tcp`'s live position (TF, polled 5x/s), the current named step and progress of a running procedure (from the `ExecuteProcedure` action's own feedback - finer-grained than the `/stylet/system/state` topic), and the final result (metrics on success, the precise failure reason otherwise). The "Launch operation" button sends the `ExecuteProcedure` goal. |

The entry-point marker is a plain `visualization_msgs/msg/Marker` published on a topic (`/stylet_ui/entry_point_marker`, `transient_local`), colored by procedure state - **not** an `InteractiveMarker`. An earlier version used `interactive_markers::InteractiveMarkerServer`, but since the server was hosted inside RViz's own process (via the panel's shared node abstraction), RViz's own `InteractiveMarkers` display calling its `get_interactive_markers` service synchronously self-deadlocked (the thread waiting for the reply was the same one that would need to run to produce it). A plain topic publish has no such round-trip.

## Topics / actions

| Name | Type | Direction |
|---|---|---|
| `/procedure/entry_point` | `geometry_msgs/msg/PointStamped` | pub (`EntryPointTool`) / sub (`TargetDefinitionPanel`) |
| `/procedure/target_point` | `geometry_msgs/msg/PointStamped` | sub (transient_local - must match the publisher's QoS to catch its one-time publish) |
| `/stylet/system/state` | `std_msgs/msg/String` | sub (transient_local) |
| `/procedure/progress` | `std_msgs/msg/Float32` | sub |
| `/stylet_ui/entry_point_marker` | `visualization_msgs/msg/Marker` | pub (transient_local) |
| `execute_procedure` | `stylet_msgs/action/ExecuteProcedure` | action client |

## Dependencies

`rclcpp`, `rclcpp_action`, `stylet_msgs`, `rviz_common`, `rviz_rendering`, `pluginlib`, `qtbase5-dev`, `geometry_msgs`, `std_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs`.

## Usage

Loaded automatically by `stylet_moveit_config/launch/moveit_rviz.launch.py` (part of `stylet_bringup`'s full demo). To add it manually to another RViz session: Panels → Add New Panel → `stylet_ui/TargetDefinitionPanel`, then add a `Marker` display on `/stylet_ui/entry_point_marker`.
