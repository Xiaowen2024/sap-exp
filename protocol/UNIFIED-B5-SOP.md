# Unified B=5 comparison SOP (2026-10-27)

This SOP governs new MuJoCo CPU, MuJoCo Warp, and SAP Warp comparisons in
`protocol/` and the task SOPs under `tasks/`. Older B=2, B=30, or B=100 artifacts remain historical and
must not be relabeled as B=5. SAP internal source stays outside the public
SIM-EXP workspace; only numerical artifacts and non-proprietary setup details
may be copied here.

## 1. What one budget unit means

- **B=1** is one *literal native-default numerical configuration*. Keep the
  task's mass, geometry, friction, gravity, timestep, actuator/input trace,
  contact-pair selection, and collision representation fixed. Omit numerical
  contact/solver overrides. Record the compiled/runtime values, not only the
  XML/YAML text.
- **B=5** is at most five complete native-parameter configurations, including
  B=1. Declare candidates 2–5 before scoring. Each candidate is evaluated on
  the same **selection-only** physical cases with **R=20** independent,
  predeclared *small initial-state perturbations*. These perturbations do not
  change solver/contact parameters and do not consume extra B units.
- Select the B=5 configuration by a task-specific, predeclared pass gate,
  then primary loss, then wall time. A configuration failing contact-capacity,
  finite-state, force-readout, or task-validity checks cannot win by having a
  numerically small trajectory error.
- Freeze the winner. Evaluate it on physical cases not used for selection,
  including the task's dt and setup-variant grids. Every evaluated condition
  uses R=20 independent small perturbations. Report raw rollout rows plus
  mean, SD, denominator, and uncertainty interval. Deterministic exact repeats
  are not counted as independent perturbations.

## 2. Alignment and audit rules

1. Lock the same physical task across engines: dimensions, inertias/mass,
   friction coefficient, initial pose/velocity distribution, input force or
   controller trajectory, runtime, sampling clock, and geometry representation.
   Native solver and contact-law parameters need **not** have equal numbers.
2. Keep *parameter sweep* and *local robustness* separate. An order-of-
   magnitude change in `ke`, `tau`, or `solref` is a new B candidate, never a
   micro-perturbation. Perturbation widths and RNG seeds are frozen across
   candidates and engines wherever their state representations permit it.
3. For every run, save engine name/version/commit, device, candidate index,
   **actual** compiled pair/contact values, `dt`, geometry hash, seed,
   physical condition, runtime, and metric/health traces. For SAP, distinguish
   shape-level from combined pair-level `ke` and `tau`.
4. Selection cases and held-out cases must have disjoint identifiers. Report
   real-data fidelity, analytic/simulation consistency, and usability/budget
   results separately; do not average incompatible reference types.
5. No contact force means force-based metrics are *missing*, not zero. Record
   contact duty/count, capacity truncation, force validity, NaNs, and final
   velocity so a failed normal-contact regime cannot masquerade as a friction
   result. Prefer current-state collision detection; never reuse an initial
   collision state for moving bodies.
6. Before a full run, assert the intended initial **world-frame linear**
   velocity and first few position increments from the actual simulator state.
   A free-joint qd index can represent angular rather than linear velocity;
   a nonzero velocity trace paired with zero travel is an invalid setup, not a
   friction-law result.

## 3. Task registry and selection rule

| Protocol | Task | Selection-only condition | Held-out evaluation | Primary B=5 selection criterion |
| --- | --- | --- | --- | --- |
| Static friction v2 | Three-box push | 3.5-N top-box hold | middle/top force grid, mass/friction/dt variants | valid normal gate, then low tail creep |
| Static friction v2 | Simple pinch | near-boundary reference mass/grip | mass, grip threshold, friction/dt variants | valid preload, then retention/height error |
| Dynamic friction v2 | GAUGE slope slider | wood task-2 + metal task-3 | plastic task-2 + wood task-1, analytic slope/horizontal cases | finite/valid contact, then real-data nRMSE |
| Dynamic friction v2 | MIT Push | recorded plywood hex trial | recorded plywood rect1 trial | valid contact, then final pose and wrench/twist loss |
| Dynamic impact v1 | GAUGE bounce, cube toss, Cassie landing | protocol-declared training samples | disjoint sample/trial IDs and dt perturbations | task-declared impact loss plus health gate |
| Sustained contact v1 | Three-plate stack | 4-mm nominal stack | thickness/dt/pose variants | correct order, overlap/force gate, then tail motion |
| Sustained contact v1 | Cube-toss tail | protocol-declared training samples | disjoint long-tail windows | contact continuity and settling/pose loss |
| Energy conservation v1 | Pendulum, Newton cradle | protocol-declared calibration cycles | held-out amplitudes/cycles/dt | energy-envelope error plus finite/contact health |

The task-specific README remains the source of metric formulas and outcome
thresholds. If a task has no valid real reference or cannot preserve the same
physical setup across engines, label it a **mechanism-only** result rather than
silently including it in real-world fidelity.

## 4. Reporting checklist

For each task and engine: list five ordered candidates and their motivation;
the B=1 compiled-default audit; selection-case metrics for all candidates
(R=20 each); the frozen winner and why; held-out metrics per condition
(R=20 each); contact/force validity and capacity; timing; and any deviations
from this SOP. Mark missing combinations **not run**, not failure or zero.
