# Panda pick-and-transfer v3 — corrected robustness sweeps

## Status and correction

This report supersedes the MuJoCo results in `panda-pick-v2-native-task-robustness.md`.
The old runner resolved the absent imported body name `fr3_hand` to Python index
`-1`, which is the cube body; its supposed hand-relative slip was therefore
invalid. The corrected MuJoCo runner uses the actual imported wrist body
`fr3_link7`. SAP Warp uses its valid `fr3/fr3_hand` label and was unaffected.

All results below use the same task intent: 100 g cube unless mass is swept,
shared cube/pad friction, 16 mm closed finger target, 15 N per-finger cap, a
closed-gripper final hold, and the pass gate: bilateral duty >= 0.9, no drop,
slip <= 5 mm, cube-symmetry-aware relative rotation <= 15 degrees, and overlap
<= 5 mm. MuJoCo is native `implicitfast`/Newton/elliptic at 1 ms with
`solref=(3 ms, 0.75)`; SAP Warp is native `drake` at 5 ms with pair
`ke=5e4 N/m`, `tau=1 ms`. This is a **native task robustness** comparison, not
controller- or timestep-identical solver equivalence.

## Main result

The corrected runs reverse the old MuJoCo interpretation. Over the shared
46-point fine mass/friction set, SAP passes **32/46** and MuJoCo passes
**4/46**. This must not be read as a universal engine ranking: the Panda drives
are deliberately native, and several MuJoCo points have contact-mode-sensitive,
non-monotonic outcomes. It does show that at this frozen 15 N grasp setting SAP
has a much larger observed task-feasible region.

## Fine mass/friction robustness map

`Y` is pass and `N` is failure under the common gate. Each row lists only the
friction values actually sampled.

| Mass | Friction samples | MuJoCo | SAP Warp |
|---:|---|---|---|
| 0.10 kg | .30, .35, .40, .45, .50, .60, .70, .80 | N Y N N N N Y Y | Y Y Y Y Y Y Y Y |
| 0.15 kg | .30, .35, .40, .45, .50, .60, .70, .80 | Y N N N N N N N | Y Y Y Y Y N Y Y |
| 0.20 kg | .30, .40, .50, .60, .70, .80 | N N N N N N | Y Y Y N Y Y |
| 0.30 kg | .10, .11, .12, .13, .14, .15 | N N N N N N | N N Y Y Y Y |
| 0.40 kg | .15, .16, .17, .18, .19, .20 | N N N N N N | N Y Y Y Y Y |
| 0.50 kg | .15, .16, .17, .18, .19, .20 | N N N N N N | N N N N N Y |
| 0.60 kg | .20, .21, .22, .23, .24, .25 | N N N N N N | N N N N Y Y |

SAP's nearly monotonic heavy-load slices identify observed transitions around
`(0.3 kg, mu=.12)`, `(0.4 kg, .16)`, `(0.5 kg, .20)`, and `(0.6 kg, .24)`.
These are task thresholds, not direct material-identification measurements.

MuJoCo's low-mass rows are non-monotonic (e.g. 0.10 kg passes at .35, fails at
.40–.60, then passes at .70/.80), so a single fitted Coulomb envelope would be
misleading. Its failures should be inspected as contact-mode/controller coupled
rollouts, with the recorded failure mode and trace, rather than smoothed into a
physical friction boundary.

## Pose robustness

The 19 condition pose suite uses x/y/z offsets of +/-2.5 and +/-5 mm, yaw
errors of +/-10, +/-20, +/-30 degrees, plus an x=5 mm/y=5 mm/yaw=20 degree
combined perturbation.

- MuJoCo: **14/19** pass. Failures: `x=+5 mm` static slip; yaw `+10` and
  `+20` static slip; yaw `+30` normal-contact loss; combined perturbation
  rotation/torque failure.
- SAP Warp: **10/19** pass. Failures include `x=+2.5/+5 mm`, `y=-5 mm`, yaw
  `-20/+10/+20`, and combined perturbation through the rotation gate; yaw
  `-30/+30` fails through static slip.

The asymmetry is meaningful for this particular grasp geometry, but it is not
evidence of an isotropic property of either contact solver.

## Transfer rate and direction

Rate is the original replayed transfer time scaled from 0.5x to 2.5x.

- MuJoCo: **7/7** pass.
- SAP Warp: **6/7** pass; 2.5x crosses the rotation threshold (15.09 degrees).

For actual direction rather than merely replay speed, three new 50 mm,
full-pose-IK targets were generated from the same lifted wrist pose: `+x`,
`+y`, and `-y`. They preserve wrist orientation and share the native 1.2 s
transfer duration.

| Direction | MuJoCo | SAP Warp |
|---|---|---|
| native replay | pass; 1.66 mm / 1.82 deg | pass; 4.59 mm / 14.56 deg |
| +x 50 mm | pass; 1.61 mm / 1.46 deg | pass; 2.65 mm / 10.55 deg |
| +y 50 mm | pass; 1.62 mm / 1.51 deg | pass; 2.68 mm / 10.62 deg |
| -y 50 mm | pass; 1.65 mm / 1.48 deg | pass; 2.65 mm / 10.58 deg |

## Long final hold: creep

Slip and rotation are measured in the corrected hand frame from the beginning
of final hold. MuJoCo's final-hold slip grows approximately linearly at
0.45–0.47 mm/s. SAP's tail drift is much smaller in the longer holds, although
its transient max excursion stays closer to the pass gate.

| Engine | hold | slip at hold start -> end | tail slip speed | outcome |
|---|---:|---:|---:|---|
| MuJoCo | 1.5 s | 1.00 -> 1.65 mm | 448 um/s | pass |
| MuJoCo | 5 s | 1.00 -> 3.30 mm | 473 um/s | pass |
| MuJoCo | 10 s | 1.00 -> 5.65 mm | 465 um/s | fail: static slip |
| SAP Warp | 1.5 s | 2.44 -> 3.16 mm | 290 um/s | pass |
| SAP Warp | 5 s | 2.44 -> 3.13 mm | 33 um/s | pass |
| SAP Warp | 10 s | 2.44 -> 3.33 mm | 43 um/s | pass |

Thus a 1.5 s pass alone was not enough to establish long-term retention for
the corrected MuJoCo configuration. SAP settles more strongly in this test,
but its max native-transfer rotation is already close to 15 degrees.

## Reproduction outputs

- MuJoCo corrected runs: `exp-results/panda-pick-v2/mujoco/framefixed-*`.
- SAP runs: `exp-results/panda-pick-v2/sap/{dense-mass-friction,fine-mass-friction,suite-pose,suite-speed,suite-hold,suite-direction}`.
- Runners: `mujoco-sim/learn_mujoco/panda_pick_experiments/run_panda_pick_v2.py`
  and `sap-sim/scripts/run_panda_pick_v2_sap.py`.

Every rollout has `metrics.json` and a sampled `trace.csv`, including cube and
joint states, contact forces, contact points/duty, penetration, slip and
orientation drift.
