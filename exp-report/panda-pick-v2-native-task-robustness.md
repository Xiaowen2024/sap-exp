# Panda pick-and-transfer v2 — MuJoCo vs SAP Warp

> **Superseded on 2026-09-16.** The MuJoCo runner used `fr3_hand`, a fixed
> URDF link that MuJoCo's importer merges into `fr3_link7`.  The unresolved
> name became Python index `-1`, so its purported hand-relative metrics were
> measured in the cube's own frame.  Every MuJoCo number and conclusion below
> is retained only as provenance and must not be used.  The corrected runner
> uses `fr3_link7`; new frame-corrected results are under
> `exp-results/panda-pick-v2/mujoco/framefixed-*` and will be reported
> separately.  SAP uses an actual `fr3/fr3_hand` body label and was unaffected.

## Scope

Corrected task-level benchmark: approach, close, lift, horizontal transfer, and a final closed-gripper hold. This is an engine-native task-robustness comparison, not a per-timestep solver-equivalence test. The Panda/cube, joint targets, timing, initial pose, closed-finger target, and URDF effort limits are shared; controllers remain native.

## Critical metric correction

Older v2 output compared cube **world-frame** orientation before and after transfer. That is invalid because the Panda hand rotates as part of the planned trajectory. The final metric expresses cube pose in the hand frame:

\[
T_{H\to C}(t)=T_W^H(t)^{-1}T_W^C(t).
\]

Slip is the change in hand-relative position. Orientation is the change in hand-relative rotation, minimized over the 24 proper rotational symmetries of an unmarked cube. Only data under `relative-pose-calibration`, `force-cap-search`, and `force15-frozen-evaluation` use this definition; older world-frame results are superseded.

## Pass gate

A run must be finite; have no SAP contact-buffer truncation; maintain bilateral finger duty >=90%; never drop; remain below 5 mm hand-relative slip, 15 degrees symmetry-aware relative rotation, and 5 mm geometric overlap.

## Frozen formal configuration

| Item | MuJoCo | SAP Warp |
|---|---:|---:|
| dt | 1 ms | 5 ms |
| Contact | elliptic cone, Newton, `implicitfast` | SAP `drake` preset |
| Contact tuning | `solref=(3 ms, 0.75)` | pair \(k_e=5\times10^4\) N/m, pair \(\tau=1\) ms |
| Arm drive | inverse-dynamics PD, \(K_p/K_d=150/30\) | native target drive, \(K_p/K_d=5000/200\) |
| Closed-finger target | 16 mm | 16 mm |
| Finger effort cap | 15 N/side | 15 N/side |
| Measured nominal \(F_n\) | 14.5 / 14.9 N (L/R) | 14.7 / 14.1 N (L/R) |

## Why 15 N/side

Position target alone was discontinuous: over a broad target range it produced 40--60 N/side, then abruptly lost contact. The common finger effort cap therefore controls preload.

| Cap per finger | MuJoCo | SAP Warp |
|---:|---|---|
| 2 N | no bilateral contact | 10.43 mm static slip |
| 3 N | 1.92 mm transfer slip | 9.43 mm static slip |
| 5 N | pass; \(F_n\approx4.8\) N/side | 8.33 mm slip; 15.80° drift |
| 10 N | pass; \(F_n\approx9.5\) N/side | 6.96 mm static slip |
| 15 N | pass; \(F_n\approx14.5\) N/side | pass; 4.59 mm slip, 14.56° drift |
| 20 N | pass; \(F_n\approx19.5\) N/side | pass; 2.41 mm slip |

Thus 15 N is the lowest tested preload at which both pass nominal. It is close to SAP's quality boundary, making the external sweep informative. At \(\mu=0.2\), it gives roughly 6 N total nominal Coulomb capacity: enough for 100 g, but only moderately above the 0.5 kg load of 4.905 N.

## Frozen external evaluation

Parameters above were frozen before this 12-condition evaluation. Entries are `pass/failure — max slip mm / angle deg / penetration mm; minimum duty`.

| Condition | MuJoCo | SAP Warp |
|---|---|---|
| nominal | pass — 0.35 / 0.04 / 0.38; 1.00 | pass — 4.59 / 14.56 / 0.30; 1.00 |
| \(\mu=0.2\) | pass — 0.47 / 0.04 / 0.23; 1.00 | pass — 2.16 / 0.50 / 0.30; 1.00 |
| \(\mu=0.6\) | pass — 0.57 / 0.05 / 0.30; 1.00 | pass — 4.20 / 13.69 / 0.30; 1.00 |
| \(\mu=1.0\) | fail normal-contact instability; duty 0 | pass — 4.63 / 14.62 / 0.30; 1.00 |
| 50 g | pass — 0.41 / 0.04 / 0.31; 1.00 | pass — 3.82 / 5.55 / 0.54; 1.00 |
| 200 g | pass — 0.59 / 0.14 / 0.28; 1.00 | pass — 4.95 / 13.76 / 0.29; 1.00 |
| 500 g | fail normal-contact instability; duty 0 | fail static slip — 5.15 / 4.11 / 0.29; 1.00 |
| +5 mm x | pass — 0.57 / 0.12 / 0.34; 1.00 | fail rotation — 5.49 / 21.98 / 0.31; 1.00 |
| +5 mm y | pass — 0.37 / 0.04 / 0.28; 1.00 | pass — 4.65 / 13.82 / 0.31; 1.00 |
| +5 mm z | pass — 0.35 / 0.04 / 0.38; 1.00 | pass — 4.59 / 14.56 / 0.30; 1.00 |
| slow transfer | pass — 0.22 / 0.02 / 0.38; 1.00 | pass — 3.20 / 14.16 / 0.30; 1.00 |
| fast transfer | pass — 0.62 / 0.07 / 0.38; 1.00 | pass — 4.56 / 14.93 / 0.30; 1.00 |

Both engines pass **10/12** conditions, but with different failure modes.

- MuJoCo has very small relative drift whenever bilateral contact remains established. Its two failures, at \(\mu=1\) and 500 g, are discontinuous bilateral-contact loss. They indicate contact/trajectory parameter sensitivity, not that high physical friction is intrinsically harmful.
- SAP keeps bilateral contact in all 12 cases and has no capacity truncation. Its 500 g failure is genuine relative static slip; its +5 mm x failure is rotation and translation drift despite continuous contact. Its successful cases sit much closer to the 5 mm/15 degree quality limits.

## Interpretation and limits

For this fixed Panda/cube task, MuJoCo gives a larger hand-relative retention margin once contact establishes; SAP preserves bilateral contact more consistently but needs more preload to satisfy the same retention gate. This is a task-level finding, not a universal engine ranking: native contact formulations, timesteps, and controllers differ by design. One deterministic run per condition is a sensitivity map, not a statistical success probability.

The previous 100 N-cap evaluation is preserved as a high-preload baseline in `exp-results/panda-pick-v2/*/frozen-evaluation`, but it is not the primary friction conclusion because its 20--60 N/side normal force leaves excessive friction margin.

## Reproduction

- MuJoCo runner: `mujoco-sim/learn_mujoco/panda_pick_experiments/run_panda_pick_v2.py`
- SAP runner: `sap-sim/scripts/run_panda_pick_v2_sap.py`
- Formal MuJoCo data: `exp-results/panda-pick-v2/mujoco/force15-frozen-evaluation/evaluation/`
- Formal SAP data: `exp-results/panda-pick-v2/sap/force15-frozen-evaluation/evaluation/`

Each case contains `metrics.json` and `trace.csv`; SAP cases also retain the generated `scene.yaml`. Traces include hand-relative pose, left/right \(F_n\), \(F_t\), \(\rho=F_t/(\mu F_n)\), contact points, penetration, tracking, and saturation telemetry.
