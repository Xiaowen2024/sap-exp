# Real-task playback runner SOP

**Status: draft for review (2026-11-16).**

Execution addendum for tasks under
[real-playback-v1](real-playback-v1/README.md). The budget, candidate, and
R=20 rules are those of [UNIFIED-B5-SOP.md](UNIFIED-B5-SOP.md); the frozen
matrix and artifact contract are those of
[task_runner_sop.md](task_runner_sop.md). This document adds the phases that
come before a manifest can be declared, and the order in which data may be
looked at.

The governing rule: **a phase may use only what earlier phases produced, and
held-out episodes are not simulated with objects until P5.**

## Phases

| Phase | Does | Uses | Produces | Gate to continue |
|---|---|---|---|---|
| **P0 Data audit** | open the official files for the task; verify fields, frames, units, time base, calibration, labels; choose the robot configuration; list candidate episodes | real data only | audit report; episode list with ratings and video-derived failure stage | every field the drive needs is present and its frame is confirmed by a geometric check; calibration residual measured; at least one selection and three held-out success episodes |
| **P1 Scene completion** | fill the completion table for every candidate episode; build assets | real data only | completion table per episode; asset parameters and hashes | every quantity has value, range, class, source; no E3 or outcome-derived goal fixture |
| **P2 Admission and declaration** | run the margin test and the predicate check on reality; assign selection / held-out; fill and freeze the manifest | P0–P1 outputs | declared manifest with hash; admitted and not-admitted lists with *m* and *u* | predicate returns the real label on every admitted episode; manifest validated; owner approval |
| **P3a Drive test** | replay every admitted trajectory with **no objects** | manifest | tracking-error report per engine | base follows the recorded pose within tolerance in both engines; engine preflight passes |
| **P3b Object smoke** | selection episodes only, nominal completion, B=1, both engines | manifest | one short report with videos | runs are finite and the stage tests and flags evaluate as intended |
| **P4 B=1 / B=5 selection** | declared candidates on the selection episodes, R=20 | manifest | candidate table; frozen winner per engine | a candidate passes the gate, or the result is recorded as `no candidate passes` |
| **P5 Frozen matrix** | frozen winner on every held-out episode × cell, R=20 | manifest | raw per-rollout rows | all planned cells are present or marked `not run` with a reason |
| **P6 Report** | verdicts and permitted claims | P4–P5 artifacts | task card | independent pass on any penetration-based statement |

## Rules that are specific to this regime

1. **What P3 may change.** P3a may change drive parameters, because no object
   is present. P3b may change stage tests, flag thresholds, and settle times
   if they are shown not to measure what they were meant to; each change is a
   manifest revision with a note, as was done for the round-socket hold
   tolerance. P3b may **not** change any completed quantity or its range.
2. **Completion is not a tuning knob.** After P2, a completed value changes
   only through a new completion revision made from the real data, never in
   response to a simulated outcome. The `both fail → completion suspect` case
   (README §4.3) triggers exactly such a review and is reported as such.
3. **Engine budget stays native.** The five candidates per engine differ only
   in native contact and solver parameters. Friction, mass, closing force,
   coupling, geometry, and `dt` are fixed during selection.
4. **Selection rule.** Valid and correct rollouts over the selection cells,
   then fewest anomaly-flagged rollouts, then wall time. Diagnostics with a
   real reference are never used for selection.
5. **Videos are part of the artifact.** For every episode and engine save one
   rendered rollout of the nominal cell beside the real video, plus one for
   each failing or flagged cell. This regime is judged by eye first.
6. **Untrusted data.** Dataset files are read with restricted loaders in a
   directory of their own (no `allow_pickle`), as in the FMB audit.
7. **Where things run.** Raw data and all simulation stay on the GPU server;
   the workstation holds manifests, audit tables, and summary artifacts.

## P0 audit checklist

- task description text and the set of episodes per robot configuration, with
  rating counts;
- TCP pose: frame, quaternion order (checked geometrically), tool offset to
  the pad centre, native sample times;
- whether a higher-rate TCP stream exists and is used;
- gripper: width units and zero, whether a command or force is recorded,
  model and fingertip parts;
- wrist force/torque: frame, bias, whether tool weight is compensated;
- calibration: camera-to-base transforms, which views are marked bad, and the
  measured residual from projecting the gripper at known TCP poses;
- depth availability and alignment to colour;
- for each candidate episode: grasp type, whether the rack moves, goal type
  (upright in slot or lying on rack), real failure stage for rating-1
  episodes, anything occluding the plate at the first or last frame.

**Configuration choice (declared):** among configurations with validated
calibration and depth, prefer one with a wrist force sensor and at least five
success and one task-failure episodes; break ties by the simplest gripper
kinematics (linear parallel jaw before linkage grippers).

## Artifact layout

```
exp-results/real-playback-v1/rh20t-dish-rack/
  audit/        P0 report, episode list, calibration residuals
  completion/   per-episode completion tables, asset parameters, hashes
  admission/    margin test and predicate-on-reality results
  drive/        P3a tracking reports
  {sap,mj400}/  selection and held-out rollout rows, preflight JSON
  viz/          side-by-side videos
```

Per-rollout rows carry the fields required by task_runner_sop.md §4, plus:
episode ID, completion revision hash, cell ID and the axis it varies, real
label, stage reached, every anomaly flag, and per-interface peak and final
overlap.

## Reporting

Lead with one table: episode × engine, showing the verdict, the nominal-cell
count (for example `20/20`), the failing stage, and flags. Then the cells that
changed a verdict. Then coverage: episodes admitted and not admitted with the
reason, axes and levels run, and cells not run. The diagnostics with a real
reference come last.
