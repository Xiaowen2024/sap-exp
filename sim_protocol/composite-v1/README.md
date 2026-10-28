# Composite contact tasks v1

This folder collects task-level, contact-rich evaluations in which geometry, articulated dynamics, contact, actuation, and numerical choices are coupled.

Composite tasks are not used to declare a universal engine winner. Each task defines its own:

1. physical-fidelity metrics, when paired physical data exist;
2. simulation-consistency diagnostics; and
3. mechanism-aware evaluation.

No metric, calibration budget, or consistency diagnostic is assumed to be universal across these tasks.

## Task registry

| Task | Primary coupled mechanisms | Real-world reference |
|---|---|---|
| [Cassie landing](task-metrics/cassie-landing.md) | impact, closed linkages, multi-foot contact, articulated dynamics | Yes |
| [Peg-in-hole](task-metrics/peg-in-hole.md) | clearance, contact manifold, friction, geometric validity | Not in v1 |
| [DENSE EZGripper grasp](task-metrics/dense-ezgripper-grasp.md) | underactuated tendon-driven grasp, frictional retention, contact switching | Planned physical data |
