# Real-task playback v1 — shared protocol

**Status: draft for review (2026-11-16). Nothing has been run under this protocol.**

New runs follow the [Unified B=5 SOP](../UNIFIED-B5-SOP.md) and the
[task runner SOP](../task_runner_sop.md). Execution order and gates are in the
[real-playback runner SOP](../../tasks/robot-playback/rh20t-dish-on-rack/runner-sop.md).

This regime replays the **recorded motion of a real robot hand** against
**free simulated objects** and asks one question per episode: *does the task
end the way it ended in reality?* The primary result is a task outcome, not a
trajectory error.

## 0. What is inherited and what is different

Inherited without change: a manifest written before any run; B=1 literal
default and B≤5 declared native candidates; selection cases disjoint from
held-out cases; R=20 small perturbations per cell with matched seeds; the same
physical world and asset files for both engines; health gates; `not run`,
`not discriminative` and `complete-negative` as legitimate results.

Three declared differences:

| | Other regimes | Real-task playback |
|---|---|---|
| Reference | recorded trajectory or analytic label | the episode's **real outcome label** (success / task failure) |
| Scene | fully released or analytic | partly **completed** by us; every completed quantity has a range, and the ranges are axes of the frozen matrix |
| Primary result | fidelity metric or labelled accuracy | **binary task outcome**, the stage at which it failed, and anomaly flags; finer numbers are diagnostics only |

## 1. Playback episode

An episode is one real demonstration with an outcome label.

- **Replayed:** the tool-centre-point (TCP) pose trace and the gripper width
  trace. The arm is not simulated. The gripper base is a free body pulled
  along the recorded TCP pose (§2.5), so no arm model, joint data, or robot
  controller enters the comparison.
- **Free:** every object the task acts on. Fixtures (table, goal fixture) are
  static unless the audit shows they move in the recording.
- **Never replayed:** object poses, forces, or anything derived from the real
  outcome.

## 2. Scene completion protocol

Public demonstration datasets are recorded to train visuomotor policies. They
give the robot's motion and images, not object geometry, pose, mass, or
friction. This section fixes how the missing quantities are supplied.

### 2.1 Evidence classes

| Class | Source | Example | Range comes from |
|---|---|---|---|
| **E1 recorded** | a dataset field | TCP pose, gripper width, wrist force | sensor resolution, time-base jitter |
| **E2 measured** | calibrated multi-view RGB-D of the episode's own frames | plate diameter, rack pose, table height, initial plate pose | spread across views and frames, plus calibration residual |
| **E3 trajectory-conditioned** | an event in the robot trace | rim thickness = stalled gripper width; payload weight = wrist-force step after lift | trace resolution |
| **E4 catalogue** | manufacturer datasheet or CAD | gripper pad size, stroke, grip force range | datasheet tolerance |
| **E5 assumed** | declared plausible interval with a cited basis | friction coefficients, plate wall profile away from the grasp point | the interval itself |

### 2.2 Rules

1. **Completion table.** Every quantity the simulator needs appears in one
   table with value, range, class, source frames or file, and method. No
   silent defaults.
2. **Highest class wins.** A lower class never overrides a higher one. When
   two classes disagree by more than their ranges, the episode is flagged and
   reviewed before it is admitted.
3. **Trajectory-conditioned quantities cost a stage.** An E3 quantity names
   the event it came from. The stage that event belongs to is then *not
   scored* for that episode; every later stage is.
4. **The goal fixture is never trajectory- or outcome-conditioned.** Its
   geometry and pose must be E1 or E2. Placing the goal where the robot
   released, or where the object ended up, is prohibited: it would decide the
   outcome by construction.
5. **Blind and frozen.** Completion is done from the real data only, hashed,
   and frozen before any rollout of that episode. A later change is a new
   revision; the earlier one is kept.
6. **One scene for both engines.** Same asset files, same completed values,
   same drive.
7. **Predicate check on reality.** The goal predicate (§3.1) is evaluated on
   the E2 estimate of the real final frame of every admitted episode. It must
   return the recorded real label. If it does not, the predicate or the
   completion is wrong and nothing is run.

### 2.3 Admission (margin) test

Not knowing a position exactly is harmless when the task has room to spare and
decisive when it does not. Each episode is tested before it is admitted:

- For every scored stage, compute the **geometric margin** *m*: how far the
  object or fixture could be displaced and the recorded motion would still
  plausibly complete that stage (for example slot width minus plate thickness,
  or pad opening minus rim thickness at approach).
- Compute the **combined uncertainty** *u* of the completed quantities that
  stage depends on (root-sum-square of their half-ranges).
- **Admit** the episode only if *m* ≥ 3 *u* for every scored stage. Otherwise
  record `not admitted: margin` with both numbers.

This is what keeps tight-clearance tasks out of this regime. They belong in a
regime with a released or analytic geometry.

### 2.4 Assets

- Objects and fixtures are **parametric primitives fitted to E2
  measurements**, built from convex pieces (as `assets/round_socket_exact`
  is), not scanned meshes. Depth noise is of the order of a plate rim, so a
  scan would add false precision and an uncontrolled convex decomposition.
- The asset builder is a script; the manifest records the parameters and the
  output hash.
- Shape detail that cannot be measured (E5) is covered by declared **variants**
  of the same family, which become matrix cells.

### 2.5 Drive

- **Gripper base.** A free rigid body coupled to the recorded TCP pose by a
  declared 6-DoF spring–damper with a force and torque cap, plus gravity
  feed-forward on the gripper's own mass. The runner computes the wrench from
  the simulated state and applies it as an external body wrench, so the
  drive is literally the same code for both engines.
  *Why not a rigid kinematic drive:* the recorded pose already contains the
  real contact compliance. A rigid drive combined with a fixture that is
  mis-placed by a few millimetres would press the object into the fixture with
  unbounded force and manufacture penetration or ejection that neither engine
  is responsible for.
- **Fingers.** A two-state machine derived from the recorded width trace by a
  declared rule: in `OPEN` segments the fingers track the recorded width; in
  `CLOSED` segments each finger receives a constant closing force (E4, with a
  range). The stalled width is then an output that can be compared with the
  recorded one.
- **Time base.** The recorded poses are interpolated to the simulation step
  (cubic in position, spherical in orientation) on the dataset's native
  timestamps.
- **Drive gate.** With no objects in the scene, the base must follow the
  recorded pose within the declared tolerance in both engines. Drive
  parameters may be adjusted only in this object-free test.

## 3. Outcome, stages, anomaly flags

### 3.1 Task outcome (primary, binary)

A geometric **goal predicate** evaluated at the end of a settle window after
the gripper has released and withdrawn. It is stated in terms of the goal
fixture's geometry (object held by the fixture, inside its envelope, at rest),
never as a distance to the real final pose. The task document fixes it.

### 3.2 Stage ladder (where it failed)

Every task declares an ordered list of stages, each with a geometric test.
A rollout reports the first stage it fails. For a pick-and-place task:

`acquired → carried → placed → retained`

This is what makes a failure readable: missed the grasp, dropped in transit,
did not reach the fixture, or fell out after release.

### 3.3 Anomaly flags (per rollout)

| Flag | Meaning | Effect |
|---|---|---|
| `invalid` | non-finite state, contact-capacity truncation, or tunnelling (object passes through a pad, fixture member, or the table) | counted separately; never correct |
| `deep_penetration` | peak overlap on a declared interface exceeds 0.5 × the local feature thickness | reported beside the outcome |
| `penetration_assisted` | success **and** `deep_penetration` on the interface that holds the object at that stage | validity warning; outcome is shown but marked |
| `ejected` | object speed exceeds the declared bound for the phase, or the object leaves the workspace box | reported; names the contact it left from |
| `large_slip` | in-hand displacement between lift-off and placement onset exceeds the declared bound | reported |

Flags follow the existing rule for penetration-assisted insertion: they are
reported next to the outcome and do not replace it, except `invalid`.

### 3.4 Diagnostics with a real reference (not gating)

Only robot-side signals have a per-frame real counterpart: stalled gripper
width, payload force after lift, and the time of first fixture contact. They
are reported for the nominal cell of each episode and are never part of the
outcome or the selection rule.

## 4. Matrix, verdict, and claims

### 4.1 Matrix

Selection uses **selection-only episodes** at the nominal completion.
Held-out evaluation uses **different episodes**; each is run on:

- the nominal completion at three `dt` values;
- low and high levels of every completed quantity that has a range (one at a
  time), including explicit object-pose and fixture-pose offsets at the edge
  of their ranges, and every declared shape variant.

Each cell has R=20 small perturbations. The perturbation is numerical (well
inside the measurement range); the range edges are separate cells. Episodes
supply the setup-transfer axis: another trajectory, operator, and scene.

### 4.2 Verdict per engine and episode

A cell **passes** when at least 18 of 20 rollouts are valid and match the real
label, **fails** at 2 or fewer, and is **mixed** otherwise.

| Verdict | Rule |
|---|---|
| `reproduces` | every cell passes |
| `fails` | every cell fails |
| `sensitive` | anything else; the report names the axis and level that change the result |

### 4.3 What may be claimed

| Result on an episode | Permitted statement |
|---|---|
| one engine `reproduces`, the other `fails` | "On this task A completes it and B does not", with the failing stage and flags. This is the headline result of the regime. |
| both `reproduce` | "Neither engine is contradicted by this episode." Not a fidelity claim. |
| both `fail` | **Completion suspect.** No engine conclusion; the completion is reviewed as a new revision. |
| either is `sensitive` | No ranking on this episode. Report the axis. If the same axis flips both engines the episode is completion-limited; if it flips only one, report that engine's sensitivity to that axis. |

A result is called *reproducing reality* only under the first two rows, that
is, only when it is invariant over every completed range. Everything else is
an engine-versus-engine statement about the stated scene.

When all admitted episodes have the same real label, agreement is
`not discriminative` for the missing class and the report says so.

## 5. Task registry

| Task | Source | Outcome classes | State |
|---|---|---|---|
| [Dish on rack](../../tasks/robot-playback/rh20t-dish-on-rack/task-metrics.md) | RH20T `task_0050` | success / task failure | pilot manifest drafted; data audit not started |

## Implementation status (2026-11-17)

The dish-on-rack pilot has been implemented and run as a pilot, with the
amendments recorded in the task document's "Pilot amendments" section. Code is
in `tasks/robot-playback/rh20t-dish-on-rack/real_playback/` (`rp_*.py`, `rh20t_*.py`, `range_tar.py`); the
report is in `exp-report/rh20t-dish-rack-playback-pilot.md`.
