# Stylet — Architecture & Validation

This document is the technical deep-dive behind [README.md](README.md): why the system is built the way it is, what was tried and abandoned, how it was validated, and what's explicitly out of scope for now. Package-level details (nodes, topics, launch order) live in each package's own README, linked from the top-level one.

## 1. System overview

The goal is to reproduce, in simulation, the core loop of image-guided percutaneous needle insertion: locate a target purely from sensor data (no ground-truth pose given to the planner), plan a safe approach, and insert with force feedback rather than blind open-loop motion.

```
5x LiDAR ─▶ fusion ─▶ filtering ─▶ GICP registration ─▶ target pose (TF)
                                                              │
                              operator picks entry point ─────┤
                                                              ▼
                                              approach planning (MoveIt)
                                                              │
                                                              ▼
                                    needle insertion (force-monitored,
                                    prismatic actuator, replanning)
```

Every stage publishes its result on a topic or TF frame that the next stage consumes - there is no single monolithic node. This was a deliberate choice: it means each stage (perception, planning, UI) can be tested and iterated on independently, and it mirrors how a real system would separate a perception subsystem from a planning subsystem.

## 2. Perception: locating the target

**Fusion.** Five simulated LiDAR sensors surround the workspace (one node reaching full coverage of an asymmetric target from a single viewpoint proved insufficient - occluded faces never appear in any single scan). `point_cloud_merger` synchronizes all five feeds (`message_filters::ApproximateTime`) and transforms each into a common `world` frame before concatenating.

**Registration.** `surface_registration` aligns the live (partial, noisy) point cloud against a pre-sampled reference model of the target using GICP (Generalized ICP), not vanilla ICP - GICP's plane-to-plane cost is markedly more robust on the target's flat asymmetric faces. ICP-family algorithms are local optimizers: they need a coarse initial alignment or they converge to the wrong local minimum. The natural choice - align the two clouds' principal axes via PCA - has a known ambiguity: PCA extracts axes, not their signs, so an axis can point either way with no way to tell from the eigendecomposition alone.

The first fix attempted was picking the sign via the cloud's skewness (asymmetric shapes have a discernible "heavy" and "light" side). This still failed sporadically (~90° misalignments) on partial views, where noise could flip skewness's sign call. The fix that actually worked: try all 4 valid sign combinations (of 8 possible, the 4 that keep the axis triad a proper rotation rather than a reflection), run GICP from each, and keep the best-fitness result. Validated externally (outside ROS, a standalone comparison across 10 point-cloud density levels): single-sign-guess initialization degraded from 10/10 to 4/10 successful registrations as density dropped; multi-start stayed 10/10 throughout.

Once a good registration is found, subsequent frames don't repeat the full 4-start search - the scene is static, so the previous transform is already close to correct. Two candidates are tried instead (the previous transform directly, and a fresh PCA with the previously-winning sign), falling back to the full search only if fitness degrades unexpectedly (tracking "lost").

**Result**: ~0.015mm translation error, ~0° rotation error, 50/50 successful registrations in a dedicated validation run (`stylet_perception/scripts/validate_registration.py`, comparing against the known simulation ground truth).

## 3. Simulation integration

Getting Gazebo (Harmonic) and MoveIt 2 to agree on the robot's state took more work than expected, mostly because their default assumptions don't match:

- **Hardware interface**: switched from `mock_components/GenericSystem` (a fake, no-physics hardware plugin) to `gz_ros2_control/GazeboSimSystem`, so the `controller_manager` lives inside Gazebo and controllers move a physically simulated robot rather than a kinematic mock.
- **A single `/robot_description`**: earlier, the simulation launch file and the MoveIt demo launch file each ran their own `robot_state_publisher` and `ros2_control_node` independently - a straightforward source of conflicts. `stylet_moveit_config/config/stylet.urdf.xacro` now combines the robot model with the `<ros2_control>` tags once, and every downstream launch file consumes that single description.
- **Clock synchronization**: with `use_sim_time` not propagated everywhere (`robot_state_publisher`, `move_group`, every controller), MoveIt's `current_state_monitor` compares a sim-time-stamped `/joint_states` against the wall clock and rejects the robot's current state as stale - producing planning and even *execution* failures that have nothing to do with the actual motion. Setting `use_sim_time: true` uniformly closed most of this gap, but not entirely (see §6, Known Limitations).
- **Collision awareness**: MoveIt initially had no idea the target physically existed - it lives only in Gazebo's world, never in MoveIt's planning scene - so the default planner (RRTConnect) happily planned straight through it, and only Gazebo's real physics stopped the actual robot, producing "successful" plans that were physically stuck. Fixed by registering the target as a real MoveIt collision object, using its actual mesh rather than an axis-aligned bounding box (a box was tried first and rejected: the target is deliberately asymmetric, so its bounding box covers a lot of legitimately empty space, and a collision-aware IK/planning check against that box wrongly marked large swaths of genuinely reachable space as "unreachable").

## 4. The needle-insertion redesign

This is the most significant mid-project pivot, worth walking through in full.

**The original plan** drove insertion the same way as the approach: a Cartesian path for the arm, computed with `computeCartesianPath()`, from the approach pose through the entry point to the target point.

**What went wrong**: for some entry/target geometries, this path requires the wrist to travel further than the needle itself is long. The needle (10cm) plus the ~5cm retreat distance can add up to well over the needle's own length depending on where the entry point is picked - and finishing a Cartesian path that goes "through" the needle's tip necessarily drags the wrist along with it. Observed directly: `computeCartesianPath()` stalling at ~73% completion, tracing back to a genuine collision between the wrist and the target, not a bug in the collision-checking logic.

**The fix mirrors how real image-guided needle robots are built**: the arm positions and orients a needle holder *once*, then a separate linear actuator - mechanically decoupled from the arm - pushes the needle in. Concretely:

- `needle_joint` in the URDF changed from a fixed joint to **prismatic**, and is now driven directly (not through MoveIt's planner - it's excluded from the "arm" planning group entirely, since including a prismatic joint in a 6-DOF-target IK chain would add a redundant degree of freedom with an unpredictable resting value).
- To avoid a visual side effect of "the needle detaching from the wrist" as it extends, the joint is split into **3 equal-travel stages in series**, with two purely-visual intermediate segments (no collision geometry, no physical role) each drawn extended backward far enough to always overlap their parent - a discrete approximation of a telescoping tube.
- `procedure_planner`'s insertion logic no longer touches the arm at all: each attempt reads the real `tool0`/`needle_tcp` TF, computes the needle joint's target value from that geometry, and sends it directly to a dedicated `needle_insertion_controller`. The same force-monitoring/cancel/replan loop that was designed for the arm-driven version carries over unchanged.
- Since the arm never moves during insertion, it never risks re-entering the target - the collision-avoidance problem that motivated switching away from `avoid_collisions=false` (see below) mostly disappears for this phase, though the target still needs a persistent, correctly-merged collision exemption for the needle so *later* replanning (e.g. retreating) doesn't itself get blocked by a "target overlaps needle" false self-collision.

**A collision-matrix lesson learned along the way**: allowing the needle to pass through the target during insertion needs an entry in MoveIt's Allowed Collision Matrix. A first attempt sent a small *diff* (`PlanningScene.is_diff = true`, 3 entry names) - which turned out to *replace* the whole matrix rather than extend it (MoveIt's diff-merge semantics for the ACM aren't a true merge). The robot's own SRDF-derived, always-adjacent-link self-collision exemptions vanished as a result, and the arm started reporting collisions with itself at rest. The fix: fetch the *complete* current matrix via the `/get_planning_scene` service, add only the new pair to it, and send the whole thing back.

## 5. Concurrency and reliability

A few issues only surfaced once the system had a real UI a person could click on, rather than a single scripted CLI test per run:

- **`MoveGroupInterface::asyncExecute()`/`stop()` from separate threads segfaults.** `MoveGroupInterface` runs its own internal executor thread to process an in-flight execution; calling `stop()` concurrently from another thread raced with it. Root-caused via `gdb`/`coredumpctl`. Fixed by talking to the trajectory controllers directly (`rclcpp_action::Client<FollowJointTrajectory>`), which only ever touches the node's own single-threaded executor.
- **Double-clicking "Launch operation" corrupted state.** With no guard, two goals could be accepted concurrently, both driving the same `MoveGroupInterface` from separate threads - the same category of problem as above, now reachable from ordinary UI use rather than only from a race in test scripts. Fixed with a `std::atomic<bool>` tracking whether a goal is already in progress, checked (and set) in the action server's goal-acceptance callback.
- **RViz's own display/panel reliability.** `moveit_configs_utils`'s auto-generated RViz launch function does not pass `robot_description`/`robot_description_semantic` to the `rviz2` node in this MoveIt release - only the planning pipeline and kinematics config - leaving both the 3D robot model and the planning-scene display empty. Fixed the same way `move_group.launch.py` already was: pass the full config dictionary explicitly. Separately, an `InteractiveMarkerServer` hosted inside RViz's own process (from a custom panel) made RViz's own topic picker hang indefinitely for that one topic - a same-process client/server deadlock. Since the marker in question was never actually interactive (no drag controls), it was replaced with a plain `Marker` publish, which needs no synchronous round-trip.

## 6. Validation summary

| Stage | Method | Result |
|---|---|---|
| GICP registration | `validate_registration.py`: 50 live samples vs. known ground truth | ~0.015mm translation, ~0° rotation, 50/50 within tolerance (<1mm, <0.5°) |
| Approach planning | Post-execution TF re-check of `needle_tcp` vs. the intended approach pose, multiple entry points on different sides of the target | 0.86–2.5mm position error, <0.4° alignment error |
| Needle insertion | Post-execution TF re-check vs. target, multiple entry points, insertion depths up to ~245mm | 0.76–1.62mm position error, 0.03–0.4° alignment error, succeeded on the first attempt in every recorded run |
| End-to-end | Full pipeline via `stylet_bringup`, recorded video + `ros2 bag` (2026-07-09) | Scan → registration converges → operator-picked entry point → approach → insertion → metrics reported, all in one launch |

A recurring design pattern across every planning stage: **never trust a "success" status alone.** MoveIt's own reported success and the controller's own goal-acceptance are both re-verified independently by reading the actual TF position after the fact, specifically because of the clock-synchronization issue in §3 - under load, `move_group` has been observed to report success while the robot's real position was tens of centimeters off target or physically stuck against the object.

## 7. Known limitations

- **Clock-synchronization issue mitigated, not fixed.** `use_sim_time` propagation closed most of the gap, but occasional false successes under heavy system load are still possible in principle. Mitigated with a mandatory pass through a known "up" configuration before every approach, plus the independent post-execution position check described above - not a root-cause fix of the underlying race.
- **No tissue model yet.** Physical contact between the needle and the target is currently disabled in Gazebo (`collide_bitmask`) because the default rigid-body contact produces unrealistic forces (hundreds of Newtons on first touch) with no tissue compliance to absorb them. Force-triggered replanning is therefore currently exercised with a placeholder threshold rather than a physically grounded one - Phase 4's job.
- **Insertion-actuator specification is a placeholder.** The prismatic joint's travel range, effort, and velocity limits are not derived from any real linear-actuator datasheet - no such hardware has been chosen or sized.
- **Static target only.** The target's pose is registered once per scene and assumed static; Phase 5 (respiratory motion) will require the registration pipeline to track a moving target and the insertion logic to gate on a breathing cycle.
