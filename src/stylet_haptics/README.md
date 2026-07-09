# stylet_haptics

**Placeholder - not yet implemented.** Reserved for Phase 4.

## Planned scope

A layered spring-damper tissue model that produces a physically plausible insertion force profile, replacing the current temporary workaround: `robot.urdf.xacro` presently disables *physical* collision between the needle and the target (`collide_bitmask`) because Gazebo's default rigid contact produces several hundred Newtons on first touch - unusable for testing the force-triggered stop/replan logic in `stylet_planning`, and not representative of real tissue anyway.

Expected inputs: `/stylet/haptics/wrench` (the raw force/torque sensor bridge from Gazebo, currently unused downstream) and the needle's insertion depth (from `stylet_planning`/TF). Expected output: a per-layer force profile published back for `procedure_planner` to react to, and/or a real-time force visualization in the RViz panel.

## Current state

No source files, no nodes, no launch files - `CMakeLists.txt`/`package.xml` only.
