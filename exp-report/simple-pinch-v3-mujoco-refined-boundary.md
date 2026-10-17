# MuJoCo simple-pinch v3: refined grasp-margin boundary

## Result

The earlier v3 grid first observed MuJoCo passing at `r=1.176` (N4) and
`r=1.182` (N6). Those were **coarse-grid pass samples, not physical or numerical
thresholds**. A new mass-only sweep, with all original MuJoCo v3 settings and
success criteria frozen, narrows the observed pass/fail transitions to:

| Preload series | Last failed mass (kg) | Last failed `r` | Lift follow (mm) | First passed mass (kg) | First passed `r` | Lift follow (mm) |
|---|---:|---:|---:|---:|---:|---:|
| N4 | 0.307706524 | 1.040878 | 56.9732 | 0.307698926 | 1.040904 | 57.0133 |
| N6 | 0.460339966 | 1.048455 | 56.9924 | 0.460334277 | 1.048468 | 57.0028 |

Thus the **task-gate transition** is near `r=1.041` and `r=1.048` for the two
tested preload levels. The six-digit values delimit tested deterministic
rollouts; they are not estimates of a universal friction coefficient or a
physical transition with micrometre-level certainty.

## What remained unchanged

The new sweep calls the original
[`run_simple_pinch_v2.py`](../mujoco-sim/learn_mujoco/pinch_grasp_experiments/run_simple_pinch_v2.py)
`one()` function. The v3 model still has a 40-mm cube and opposed pads,
free cube, `mu=0.4`, gravity off, known downward cube load `m*9.81 N`,
`dt=0.5 ms`, MuJoCo `implicitfast`/Newton/elliptic, explicit pad–cube pairs,
`solref=(4 ms,1)`, and `solimp=(.95,.99,.001,.5,2)`. Pad compression is fixed
at `0.20 mm` per side for N4 and `0.30 mm` for N6. The 1.5-s preload gate,
60-mm/1-s commanded lift, 0.25-s hold, and pass criterion of stable preload
plus `lift_follow >= 57 mm` are unchanged. **Only cube mass varies.**

The grasp margin uses the measured gate preload for each run:

```math
r = \frac{2\mu N_{\rm measured}}{mg}.
```

Changing mass also changes object inertia and can slightly change the measured
gate force. Therefore the horizontal axis is the recorded `r`, not an assumed
constant-preload estimate. The sweep tests this task's mass/load boundary; it
does not isolate a pure force-margin law independent of mass.

## Search and validity checks

- Each original coarse fail/pass mass interval was sampled with 20 subintervals.
  Each grid had one pass-to-fail transition, then binary refinement stopped
  when its mass bracket was below `0.00001 kg`.
- N4 used 29 unique cases; N6 used 30. All 59 passed the normal gate, had
  exactly four mean contacts on each side during that gate, retained finite
  lift-follow telemetry, and used at most eight simultaneous contacts.
- The four reused coarse endpoints reproduced the original v3 metrics exactly
  after excluding wall-clock time. Both sides of each refined boundary were
  independently rerun twice: all eight outcomes matched, with zero recorded
  lift-follow difference in this deterministic setup.
- The N4 bracket is only `0.000007598 kg` / `0.00002535` in measured `r`;
  the N6 bracket is `0.000005689 kg` / `0.00001284` in `r`. The N4 follow value
  changes by about `0.040 mm` across its bracket, so reporting many digits as
  a smooth physical critical value would be misleading.

## Revised SAP/Warp comparison

SAP was **not rerun**. Its existing coarse v3 results have a last failure /
first pass of `r=0.959 / 1.010` (N4), and `r=1.000 / 1.050` (N6). MuJoCo's
refined boundary is therefore much closer to the ideal static `r=1` than the
old `1.18` wording implied. For N4, SAP's first observed pass at `1.010` is
below MuJoCo's refined `1.041` transition. For N6, the SAP transition remains
somewhere in `(1.000,1.050]`, which **overlaps** MuJoCo's `1.04846` boundary;
these data do not resolve which is closer to `r=1` at that preload.

The `r=1` Coulomb condition is necessary for static support in the ideal
symmetric model, not a guarantee of passing a finite-speed, 95%-follow lift
gate. Neither engine's sampled task boundary should be called a universal
accuracy ranking.

## Reproduction and source data

- Refined runner:
  [`refine_simple_pinch_v3_mujoco_boundary.py`](../mujoco-sim/learn_mujoco/pinch_grasp_experiments/refine_simple_pinch_v3_mujoco_boundary.py)
- Run command: `mujoco-sim/.venv/bin/python mujoco-sim/learn_mujoco/pinch_grasp_experiments/refine_simple_pinch_v3_mujoco_boundary.py`
- Data: `exp-results/simple-pinch-v3/mujoco/refined-boundary-20261017/results.csv`,
  `summary.json`, `validation.csv`, `run_config.json`, and per-case `model.xml`,
  `trace.csv`, `metrics.json`.
- Original cross-engine v3 data:
  [`simple-pinch-v3-force-matched-sap-vs-mujoco.md`](simple-pinch-v3-force-matched-sap-vs-mujoco.md)

No website result has been changed as part of this rerun.
