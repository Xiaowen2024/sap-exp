# Static-Friction Benchmark v3

This folder separates the static-friction benchmark into a shared protocol and
task-specific matrices.

| Task matrix | Role | Task-specific evaluation |
| --- | --- | --- |
| [01 Atomic three-box hold](01-atomic-three-box-hold.md) | Controlled sub-Coulomb reference | None beyond the shared protocol |
| [02 GAUGE slope static hold](02-gauge-slope-static-hold.md) | Real-world single-contact validation | None beyond the shared protocol |
| [03 GAUGE turntable tracking](03-gauge-turntable-static-tracking.md) | Real-world rotating-support validation | None beyond the shared protocol |
| [04 Simple pinch cube](04-simple-pinch-cube.md) | Robot grasp / lift extension | Minimum hold grip force; grip-force error; lift-height error |
| [05 Middle-box ejection](05-middle-box-ejection.md) | Multi-contact task outcome extension | Expected-outcome agreement; false-retention/ejection rate |

All tasks use the definitions and mechanism-aware evaluation protocol in
[shared-protocol.md](shared-protocol.md).
