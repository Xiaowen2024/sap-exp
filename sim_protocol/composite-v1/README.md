# Composite contact tasks v1

This protocol covers task-level, contact-rich systems in which contact, geometry, articulated dynamics, control, and numerical choices interact. It is deliberately separate from the atomic benchmarks:

- static friction;
- dynamic friction;
- dynamic impact; and
- sustained contact.

A composite task is not used to declare a universal engine winner. Its role is to test whether conclusions from the atomic regimes remain useful once several mechanisms are coupled.

## Shared rules

### 1. Model and controller parity

Document the robot/object geometry, mass and inertia, collision representation, joint and actuator model, environment parameters, controller interface, update rate, torque limits, timestep, and all frozen native engine settings.

Physical environment parameters (for example mass, geometry, support pose, and independently calibrated friction) are fixed across engines. Native contact and solver settings may differ, but are selected only on training conditions and frozen before held-out scoring.

For closed-loop tasks, distinguish:

- **real-data replay:** initialize from a measured state and replay the measured command/torque trace; this supports real-world fidelity claims;
- **closed-loop stress test:** run the same controller architecture and limits in each simulator; this is task robustness, not automatically real-world accuracy.

### 2. Real-world fidelity

Use a real-world metric only when a synchronized reference exists. The task file defines the observable state and fixed evaluation window. Report held-out mean, SD, and 95% CI.

There is intentionally no universal composite-task trajectory score. A pelvis state, an insertion pose, and a grasped-object state have different physically meaningful observables.

### 3. Simulation consistency

Each task declares only the internal diagnostics that are meaningful for its mechanism. Penetration is a **physical-validity diagnostic or gate**, not a universal accuracy ranking: compliant models can permit finite overlap, while excessive overlap can enable an invalid task outcome.

Do not compare raw contact-point counts or unaggregated contact-force traces across engines as universal metrics.

### 4. Mechanism-aware usability

A budget unit is one complete native parameter-configuration evaluation over all training rollouts.

| Stage | Budget | Rule |
|---|---:|---|
| Default | \(B=1\) | Documented/default contact and solver settings; no trajectory fit. |
| First-principles calibration | \(B\leq5\) | Predeclared mechanism-motivated native candidates selected on training conditions only. |
| Brute-force upper bound | \(B\approx100\) | Disclosed native grid on training conditions only; an oracle ceiling, not a usability claim. |

For every composite task, freeze the selected engine configuration before held-out scoring. Then report:

| Check | Definition |
|---|---|
| **Bounded tuning** | \(\mathrm{Pass}@B\) and \(B_{\rm first\ pass}\) for a preregistered criterion; compare first-principles calibration with the same-budget brute-force baseline where feasible. |
| **Setup-variant robustness** | Freeze native settings, vary meaningful task conditions (e.g. object mass, geometry, surface, initial pose, or command), and report whether the conclusion transfers. |
| **Task-specific stability boundary** | For a task with a well-defined transition, map the boundary using the task's physical controls. This is a characterization of controllability, not a native-parameter fit. For a grasp, it is the hold/slip boundary over grip force, load, and object condition. |
| **Numerical local robustness** | With physics and tuning frozen, perturb timestep, initial state, and seed under a predeclared distribution. Report per-condition mean/SD across \(R\geq20\) draws. |

Repeated identical deterministic replays are reproducibility checks, not robustness samples.

## Task registry

| Task | Primary coupled mechanisms | Real-world reference |
|---|---|---|
| [Cassie landing](task-metrics/cassie-landing.md) | impact, closed linkages, multi-foot contact, articulated dynamics | Yes |
| [Peg-in-hole](task-metrics/peg-in-hole.md) | clearance, contact manifold, friction, geometric validity | Not in v1 |
| [DENSE EZGripper grasp](task-metrics/dense-ezgripper-grasp.md) | underactuated tendon-driven grasp, frictional retention, contact switching | Planned physical data |
