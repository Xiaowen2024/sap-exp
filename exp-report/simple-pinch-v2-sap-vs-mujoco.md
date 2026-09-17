# Controlled simple-pinch v2 — SAP Warp vs MuJoCo

> **Archive; do not use for the final engine comparison.** Read the unified
> report first: [simple-pinch-sap-vs-mujoco-final.md](simple-pinch-sap-vs-mujoco-final.md).
> The final force-matched v3 data remain available in
> [simple-pinch-v3-force-matched-sap-vs-mujoco.md](simple-pinch-v3-force-matched-sap-vs-mujoco.md).
> This document is retained as the original geometric-compression sweep and
> pipeline-validity record.

## Status and scope

**Valid controlled rerun completed on 2026-09-16.**  This replaces the old
simple-pinch SAP result for force/penetration conclusions.  The old scene hit
`max_contact_count=512`; every v2 SAP rollout uses a capacity of 128 and has
`max_truncated_contacts=0`.  It is therefore valid for the contact-force
telemetry below.

This is a deliberately minimal two-pad grasp, not the Panda task: it isolates
normal preload and Coulomb support before adding arm-controller differences.

## Protocol

- 40 mm cube, two 20 mm-thick pad faces, each pad constrained to vertical
  motion; no floor and no cube rotation support.
- `gravity=0`.  The 0.5 s **normal gate** holds the pads at fixed preload with
  no vertical cube load.  This cleanly measures per-pad normal force `N`.
- A 60 mm, 1 s pad lift is then commanded, while a known downward force
  `m g` is applied to the cube for 1.25 s (lift plus hold).
- Physical support criterion: `2 μ N >= m g`.  Results use the measured,
  gate-tail mean `N`, and normalize it as `r_N = 2 μ N / (m g)`.
- Gate pass requires both sides: contact duty >= 99.9%, `CV(Fn) <= 1%` and a
  stable four-point contact manifold.

### Native configurations

| Engine | dt | Contact configuration | Preload levels |
|---|---:|---|---|
| MuJoCo | 0.5 ms | Newton, elliptic cone, `implicitfast`; pair `solref=(4 ms,1)`, `solimp=(.95,.99,.001,.5,2)` | geometric compression 0.05 / 0.20 mm; measured `N≈0.95 / 3.82–3.91 N` |
| SAP Warp | 0.5 ms | `drake` SAP preset, `rigid_gap=0`; pair `ke=1e4 N/m`, `tau=3 ms` (shape values `2e4 N/m`, `1.5 ms`) | geometric compression 0.05 / 0.20 mm; measured `N≈6.00 / 12.00 N` |

SAP ran on an RTX 3090 using `cuda:0`.  Its first case took 140 s due to Warp
kernel compilation; post-warm-up cases took 16.1–16.6 s.  MuJoCo ran locally
on CPU at 0.045–0.103 s per case.

## Data-validity checks

- All 24 rollouts passed the normal-stability gate.
- Both engines report four force-bearing pad–cube points per side during the
  gate; SAP retained this manifold and had zero contact truncation throughout.
- `success` means gate pass and final cube lift at least 57 mm (95% of command).
  `partial / slip` is intentionally not called a success even when the cube
  rises some distance.

## Results

### MuJoCo

| compression (mm) | μ | mass (kg) | measured N / side (N) | r_N | lift (mm) | outcome |
|---:|---:|---:|---:|---:|---:|---|
| 0.05 | .1 | .10 | .954 | .195 | -7331 | slip |
| 0.05 | .1 | .25 | .979 | .080 | -7534 | slip |
| 0.05 | .2 | .10 | .954 | .389 | -6886 | slip |
| 0.05 | .2 | .25 | .979 | .160 | -7340 | slip |
| 0.05 | .4 | .10 | .954 | .778 | -5268 | slip |
| 0.05 | .4 | .25 | .979 | .319 | -6954 | slip |
| 0.20 | .1 | .10 | 3.818 | .778 | -5556 | slip |
| 0.20 | .1 | .25 | 3.915 | .319 | -7177 | slip |
| 0.20 | .2 | .10 | 3.818 | 1.557 | 59.73 | success |
| 0.20 | .2 | .25 | 3.915 | .639 | -6330 | slip |
| 0.20 | .4 | .10 | 3.818 | 3.113 | 59.80 | success |
| 0.20 | .4 | .25 | 3.915 | 1.277 | 59.33 | success |

### SAP Warp (`drake`, CUDA)

| compression (mm) | μ | mass (kg) | measured N / side (N) | r_N | lift (mm) | outcome |
|---:|---:|---:|---:|---:|---:|---|
| 0.05 | .1 | .10 | 6.000 | 1.223 | 59.01 | success |
| 0.05 | .1 | .25 | 6.000 | .489 | -5844 | slip |
| 0.05 | .2 | .10 | 6.000 | 2.446 | 59.01 | success |
| 0.05 | .2 | .25 | 6.000 | .979 | 51.07 | partial / slip |
| 0.05 | .4 | .10 | 6.000 | 4.893 | 59.01 | success |
| 0.05 | .4 | .25 | 6.000 | 1.957 | 59.01 | success |
| 0.20 | .1 | .10 | 12.000 | 2.446 | 59.01 | success |
| 0.20 | .1 | .25 | 12.000 | .979 | 32.67 | partial / slip |
| 0.20 | .2 | .10 | 12.000 | 4.893 | 59.01 | success |
| 0.20 | .2 | .25 | 12.000 | 1.957 | 59.01 | success |
| 0.20 | .4 | .10 | 12.000 | 9.786 | 59.01 | success |
| 0.20 | .4 | .25 | 12.000 | 3.914 | 58.97 | success |

Large negative lift values are expected free fall after contact loss under the
applied downward load; they are a failure signal, not a penetration metric.

## Findings

1. **The normal-contact confound is removed.**  Both engines were stable at
   preload, so a later slip cannot be attributed to a lost or chattering
   initial contact.  SAP's former capacity failure is absent.
2. **Both reproduce the correct Coulomb threshold qualitatively.**  Every
   clear success has `r_N>1`; every clear free-slip failure has `r_N<1`.
   SAP exposes the expected transition around `r_N≈1`: both `r_N=0.979`
   cases retain contact but creep/slip enough to lift only 51.1 and 32.7 mm.
3. **Do not rank native engines from raw compression.**  The same 0.05 mm
   geometry produces `N≈0.95 N` in MuJoCo but `N≈6.00 N` in SAP.  Thus the
   comparison is meaningful in normalized load margin `r_N`, not as a claim
   that one engine has a higher friction coefficient at a fixed geometric
   penetration.
4. **There is still a high-margin dynamic difference worth following.**  In
   SAP successful cases, mean relative vertical speed is about
   `0.0016 mm/s`, but the final cube vertical velocity remains about
   `-12.1 mm/s`; MuJoCo's matched successful cases end near `-0.14 to
   -0.16 mm/s`.  This is evidence to investigate SAP's lift/hold transient
   and contact damping, not yet evidence that one engine is generally better.

## Reproducibility

- MuJoCo runner: `mujoco-sim/learn_mujoco/pinch_grasp_experiments/run_simple_pinch_v2.py`
- SAP runner: `sap-sim/scripts/run_simple_pinch_v2_sap.py`
- Raw outputs: `exp-results/simple-pinch-v2/{mujoco,sap}/`

Each case directory contains `scene`/`model`, `trace.csv`, and `metrics.json`;
each engine root contains a compact `results.csv` and complete `results.json`.

## Next controlled comparison

For a direct engine-best comparison, calibrate a MuJoCo compression that gives
`N≈6 N` (and optionally SAP to `N≈3.8 N`), then replay the same `(N, μ, m)`
points.  The present sweep establishes the validity gate and the appropriate
dimensionless comparison variable before that calibration.
