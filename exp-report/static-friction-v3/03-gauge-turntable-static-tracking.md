# Task matrix 03: GAUGE Turntable static tracking

| Field | Definition |
| --- | --- |
| Physical mechanism | Static friction on a rotating, non-inertial support |
| Setup | GAUGE Turntable task-1 (2 rad/s co-rotation) and task-3 (reversal) |
| Environment condition | Use GAUGE's measured asset properties and mocap initial state |
| Expected outcome | The cube co-rotates with the turntable without sustained relative slip |
| Real-world reference | GAUGE repeated mocap trajectories |
| Task-specific metrics | None |

This task extends static support to persistent tangential demand and direction
changes. Use the shared trajectory metrics as the authoritative real-world
score. Report support-relative displacement and velocity only when support
kinematics are available; GAUGE's published engine score itself is position
RMSE/DTW.
