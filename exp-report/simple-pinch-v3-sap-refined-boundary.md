# SAP/Warp simple-pinch v3: refined grasp-margin boundary

## Result

The old SAP v3 sweep first observed passes at `r=1.010` (N4) and `r=1.050`
(N6). Those were coarse-grid samples, **not the success thresholds**. Keeping
the original SAP model and gate fixed, this CPU mass-only refinement finds:

| Per-side preload series | Last failed mass (kg) | Last failed `r` | Follow (mm) | First passed mass (kg) | First passed `r` | Follow (mm) |
|---|---:|---:|---:|---:|---:|---:|
| N4, about 4 N | 0.332817969 | 0.989910 | 56.99985 | 0.332809570 | 0.989935 | 57.00153 |
| N6, about 6 N | 0.488736840 | 1.001137 | 56.99890 | 0.488731152 | 1.001149 | 57.00039 |

Thus the tested **57-mm task-gate** transitions are near `r=0.990` and
`r=1.001`. The printed precision describes the numerical brackets only. It
does not claim a physical friction threshold accurate to six digits.

For comparison, the same-setup MuJoCo mass refinement found task-gate
transitions near `r=1.041` (N4) and `r=1.048` (N6). Under these frozen native
setups, SAP passes this short lift at a lower *measured-gate* grasp margin in
both preload series. This is a **task outcome**, not an engine-wide friction
accuracy ranking or proof that `r<1` is a sustainable static grasp.

## What was held fixed

The experiment directly calls `run_case()` in
[`run_simple_pinch_v2_sap.py`](../sap-sim/scripts/run_simple_pinch_v2_sap.py),
the same source used by the original v3 SAP data. It keeps the 40-mm cube,
two opposed pads, free cube, `mu=0.4`, gravity off, downward cube load `m g`
during the lift, `dt=0.5 ms`, `drake` preset, `rigid_gap=0`, pair
`ke=1e4 N/m`, pair `tau=3 ms`, and contact capacity 128. The N4 and N6 pad
compressions remain `0.001 mm` and `0.050 mm` per side. Both retain the 1.5-s
normal gate, 60-mm lift over 1.0 s, 0.25-s hold, and success requirement of
normal-gate pass plus at least 57 mm of cube following. **Only mass varies.**

The plotted margin is computed from the measured force *before the lift*:

```math
r_{\rm gate}=\frac{2\mu N_{\rm gate}}{mg}.
```

The new run uses local ARM CPU because one case took about 18 seconds, versus
about 20 seconds in the old RTX 3090 data. It does not require a GPU. The four
old coarse endpoints reproduced on CPU with the same pass/fail outcomes and
follow differences of at most `0.00021 mm` from the prior GPU values. The CPU
and GPU traces are not claimed to be bitwise identical.

## Validity and interpretation

- Eight grid intervals per original pass/fail mass bracket established one
  sampled pass-to-fail transition in each series. Bisection narrowed N4 to a
  `0.000008399 kg` bracket and N6 to `0.000005688 kg`.
- All 35 unique cases passed the normal-stability gate, had four mean contact
  points per pad during that gate, and recorded zero truncated contacts. Four
  independent boundary repeats reproduced the corresponding outcomes and
  recorded follow values exactly on this CPU setup.
- N4 passes at `r_gate<1` because this is a finite-time, 95%-follow task
  criterion, not an indefinite static-support test. At its first passing
  sample, the gate preload was `4.040 N/side` but mean force during the lift
  was about `4.127 N/side`; the contact load is not perfectly frozen. The
  object also accumulates about `1.95 mm` relative slip and has about
  `-16.4 mm/s` mean tail vertical velocity. It should **not** be described as
  a zero-slip equilibrium at sub-Coulomb force.
- N6's first passing sample likewise has gate `N=6.000 N/side`, lift mean
  `N≈6.085 N/side`, about `1.91 mm` relative slip, and a tail vertical
  velocity around `-17.0 mm/s`. For both preload series, a longer hold or an
  explicit steady-slip criterion would answer a different question.
- The SAP contact candidate count reaches the configured capacity 128 in
  these runs, but the solver reports `max_truncated_contacts=0`. We use that
  direct truncation diagnostic to validate the recorded force results.

This result updates the coarse-grid reading in
[`simple-pinch-v3-force-matched-sap-vs-mujoco.md`](simple-pinch-v3-force-matched-sap-vs-mujoco.md).
The corresponding MuJoCo refinement is
[`simple-pinch-v3-mujoco-refined-boundary.md`](simple-pinch-v3-mujoco-refined-boundary.md).
Neither engine was retuned for this refinement.

## Data and reproduction

- Script: [`refine_simple_pinch_v3_sap_boundary.py`](../sap-sim/scripts/refine_simple_pinch_v3_sap_boundary.py)
- Command: `sap-sim/.sap-venv/bin/python -u sap-sim/scripts/refine_simple_pinch_v3_sap_boundary.py`
- Output: `exp-results/simple-pinch-v3/sap/refined-boundary-20261017-cpu/`
  with `results.csv` (35 cases), `validation.csv` (4 repeats), `summary.json`,
  `run_config.json`, and per-case `scene.yaml`, `trace.csv`, `metrics.json`.
- Measured simulation wall time: about `644 s` for unique cases plus `73 s`
  for repeats. This excludes compilation/setup overhead.

The website has not been updated as part of this local rerun.
