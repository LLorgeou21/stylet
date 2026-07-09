# stylet_perception

LiDAR perception pipeline: point cloud fusion, filtering, and GICP surface registration against a reference model of the target.

## Pipeline

```
/lidar_1..5/points ──▶ point_cloud_merger ──▶ /stylet/perception/merged_cloud
                                                        │
                                                        ▼
                                              cloud_preprocessor
                                                        │
                                                        ▼
                                      /stylet/perception/filtered_cloud
                                                        │
                                                        ▼
                                              surface_registration
                                          ┌─────────────┼──────────────┐
                                          ▼             ▼              ▼
                          /stylet/perception/   /stylet/perception/   TF
                              target_pose        fitness_score    world → target_frame
```

## Nodes

| Node | Purpose |
|---|---|
| `point_cloud_merger` | Subscribes to all 5 LiDAR feeds, synchronizes them (`message_filters::ApproximateTime`, tunable via `sync_max_interval_ms`), transforms each into `world` via TF2, concatenates them into one cloud. |
| `cloud_preprocessor` | Passthrough crop (region of interest) + voxel downsampling (2mm) + statistical outlier removal (PCL). |
| `surface_registration` | PCA-based coarse initialization (all 4 valid axis-sign combinations tried, to avoid the ~90° failures a single PCA sign guess produces on partial views) followed by GICP refinement. Tracks frame-to-frame with a 2-candidate warm start once converged, falling back to the full 4-start search if fitness degrades. Publishes the estimated target pose, a fitness score, a debug cloud (reference transformed by the estimated pose, for visual comparison in RViz), and broadcasts the `world → target_frame` TF that the rest of the system (`stylet_planning`) builds on. |

## Topics

| Topic | Type | Node |
|---|---|---|
| `/stylet/perception/merged_cloud` | `sensor_msgs/msg/PointCloud2` | published by `point_cloud_merger` |
| `/stylet/perception/filtered_cloud` | `sensor_msgs/msg/PointCloud2` | published by `cloud_preprocessor` |
| `/stylet/perception/target_pose` | `geometry_msgs/msg/PoseStamped` | published by `surface_registration` |
| `/stylet/perception/fitness_score` | `std_msgs/msg/Float64` | published by `surface_registration` |
| `/stylet/perception/aligned_reference_debug` | `sensor_msgs/msg/PointCloud2` | published by `surface_registration` (visual debug only) |

## Scripts

| Script | Purpose |
|---|---|
| `scripts/generate_reference_pcd.py` | One-time offline generation of `config/target_reference.pcd` (25,000 points uniformly sampled from `target.stl`, via Open3D) - not run at launch time. |
| `scripts/validate_registration.py` | Standalone accuracy check: subscribes to `/stylet/perception/target_pose`, compares 50 samples against the known ground-truth pose, writes a CSV and a histogram (`config/registration_errors.{csv,png}`). Measured result: ~0.015mm translation error, ~0° rotation error, 50/50 samples within tolerance. |

## Dependencies

`rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `tf2_sensor_msgs`, `message_filters`, `pcl_conversions`, `ament_index_cpp`, PCL (filters, registration, common, io).

## Usage

Requires the 5 LiDAR topics and TF tree to already be published (i.e. run alongside `stylet_simulation`) - not a standalone package.
