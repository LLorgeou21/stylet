# stylet_msgs

Custom action definition shared across the Stylet packages.

## Contents

| File | Purpose |
|---|---|
| `action/ExecuteProcedure.action` | Triggers and monitors one needle-insertion procedure (approach + insertion) |

```
# Goal (empty - the entry point is already received on /procedure/entry_point,
# and the target point is a fixed procedure_planner parameter; this goal only
# triggers execution)
---
# Result
bool success
string message
---
# Feedback
float32 progress       # 0.0 - 1.0
string current_step    # e.g. "Moving through the 'up' position", "Insertion in progress"
```

## Who uses this

- **Server**: `stylet_planning`'s `procedure_planner` node hosts the `execute_procedure` action server.
- **Client**: `stylet_ui`'s `TargetDefinitionPanel` sends the goal from its "Launch operation" button and displays the live feedback/result.

## Dependencies

`rosidl_default_generators` (build), `rosidl_default_runtime` (runtime) - standard ROS 2 interface package, no other code.
