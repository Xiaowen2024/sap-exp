# Simple pinch v3 — measured-force-matched SAP Warp vs MuJoCo

> **Detailed-data archive.** For the integrated conclusion and a Chinese
> explanation, read [simple-pinch-sap-vs-mujoco-final.md](simple-pinch-sap-vs-mujoco-final.md)
> first. This file retains every calibration and per-case v3 row.

## Status

**Completed 2026-09-16.** This is the detailed data archive for the
controlled simple-pinch task. The integrated conclusion is in
`simple-pinch-sap-vs-mujoco-final.md`. V3 supersedes the *cross-engine
conclusion* in `simple-pinch-v2-sap-vs-mujoco.md`: v2 varied geometric
compression, which produced very different normal preloads in the two
engines. V2 remains useful as a pipeline / sanity-check record, but it is not
a fair grasp-robustness comparison.

All raw results are retained in
`exp-results/simple-pinch-v3/{mujoco,sap}/`. Every case directory contains the
scene/model, a time trace, and `metrics.json`; each sweep directory has its
own `results.csv` and `results.json`.

## Question and physical reference

Can a two-pad pinch support a known vertical load once the **measured normal
preload**, rather than the geometric compression, is matched between engines?

The cube has mass `m`, each pad applies measured normal force `N`, and the
friction coefficient is `mu = 0.4`. With gravity disabled, the cube receives
only a known downward external force `m g` during the lift. The ideal static
Coulomb condition is

```math
2 mu N >= m g,   r = 2 mu N / (m g) >= 1.
```

Thus the meaningful independent variable is the dimensionless measured grasp
margin `r`, not pad compression.

## Common task and success criterion

- 40 mm cube; two opposed 20 mm pad faces; cube rotation constrained.
- Four pad--cube contacts per side in the stable preload configuration.
- `dt = 0.5 ms`; pad lift = 60 mm over 1.0 s; then a 0.25 s hold.
- Preload gate: 1.5 s fixed pad position, no vertical cube load. Each side
  must have contact duty >= 99.9%, `CV(Fn) <= 1%`, and four mean contact
  points.
- Success: normal gate passes and the cube follows at least 57 mm (95% of the
  60 mm command). A near-threshold case can therefore be physically close but
  recorded as a failure if it slips more than 3 mm.
- Telemetry: gate `Fn`, `CV(Fn)`, contact duty / point count; lift following,
  relative slip, mean tangential relative speed `|vt|`, friction utilization
  `rho = |Ft|/(mu Fn)`, and contact duty. SAP also records its constraint gap
  `phi0` and geometric signed overlap at the same time.

## Native configurations and calibration

| engine | configuration | per-side compression used for common sweep | measured preload used |
|---|---|---:|---:|
| MuJoCo (CPU) | Newton, elliptic cone, `implicitfast`; pair `solref=(4 ms, 1)`, `solimp=(.95,.99,.001,.5,2)` | 0.20 / 0.30 mm | nominal N4 / N6 |
| SAP Warp (RTX 3090) | `drake` preset; `rigid_gap=0`; pair `ke=1e4 N/m`, `tau=3 ms` (shape `ke=2e4`, `tau=1.5 ms`) | 0.001 / 0.050 mm | nominal N4 / N6 |

The different compression values are intentional: they are the calibration
knobs required to obtain comparable physical `N`.

### MuJoCo preload calibration

| compression / side (mm) | measured `N` / side (N) | gate duty | `CV(Fn)` | points |
|---:|---:|---:|---:|---:|
| 0.10 | 1.9088 | 1.000 | < 1e-13 | 4 |
| 0.20 | 3.8176 | 1.000 | < 1e-13 | 4 |
| 0.30 | 5.7265 | 1.000 | < 1e-13 | 4 |

This is approximately linear at 19.1 kN/m per pad for this geometry/contact
configuration. In the complete sweeps the actual N changes slightly with
cube mass, so every reported `r` below uses the measured value, not this
nominal calibration.

### SAP preload calibration and the 0.1 mm offset

| compression / side (mm) | measured `N` / side (N) | gate result |
|---:|---:|---|
| <= -0.01 | 0 | no contact |
| 0.000 | 1.434 | unstable: duty 0.500, `CV(Fn)=1.07`, 2 points |
| 0.001 | 4.040 | stable: duty 1.000, 4 points |
| 0.010 | 4.400 | stable |
| 0.020 | 4.800 | stable |
| 0.030 | 5.200 | stable |
| 0.040 | 5.600 | stable |
| 0.050 | 6.000 | stable |
| 0.060 | 6.400 | stable |

For every stable SAP case, including both force-matched sweeps,

```math
(-phi0) - delta_geometric = 0.100000 mm (+/- 0.000002 mm).
```

Here `delta_geometric` is the independently measured signed pad--cube
geometric overlap. This reproduces the earlier layered-stack 0.1 mm effect in
a different geometry: it is systematic contact representation / constraint
offset in this SAP configuration, not random numerical error.

**Consequence for N=2 N/side.** Under the fixed native SAP configuration,
there is no stable 2 N preload: the gate changes from no stable contact to
4.040 N at +1 micrometre compression. It would be misleading to claim an
N=2 force-matched cross-engine curve. Changing `rigid_gap` or other native
SAP settings to engineer a 2 N plateau is a valid future study, but not part
of this fixed-configuration comparison.

## Force-matched outcome curves

All 28 cases passed the initial normal-stability gate. `rho` and lift duty
are during the lift; values near one in a failing case mean the friction cone
was saturated before contact loss. Negative follow values mean the cube fell
after loss of support, not a penetration measurement.

### Nominal N = 4 N per side

| engine | mass (kg) | actual N (N) | r | follow (mm) | slip (mm) | mean abs vt (mm/s) | duty | rho | outcome |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| MuJoCo | .6524 | 3.956 | .495 | -6758.20 | 6815.32 | 5452 | .012 | 1.000 | fail |
| MuJoCo | .4077 | 3.941 | .788 | -5581.20 | 5638.59 | 4510 | .028 | 1.000 | fail |
| MuJoCo | .3434 | 3.933 | .934 | -3579.08 | 3636.99 | 2910 | .078 | 1.000 | fail |
| MuJoCo | .3262 | 3.931 | .983 | 2.59 | 57.33 | 45.8 | .511 | .998 | fail |
| MuJoCo | .3107 | 3.928 | 1.031 | 56.38 | 3.54 | 2.84 | .966 | .968 | fail |
| MuJoCo | .2718 | 3.920 | 1.176 | 59.00 | .93 | .744 | .992 | .850 | pass |
| MuJoCo | .1631 | 3.880 | 1.940 | 59.76 | .19 | .156 | 1.000 | .516 | pass |
| SAP Warp | .6524 | 4.040 | .505 | -912.30 | 970.33 | 780 | .664 | 1.000 | fail |
| SAP Warp | .4077 | 4.040 | .808 | 37.40 | 21.53 | 17.2 | 1.000 | 1.000 | fail |
| SAP Warp | .3434 | 4.040 | .959 | 54.64 | 4.30 | 3.45 | 1.000 | .995 | fail |
| SAP Warp | .3262 | 4.040 | 1.010 | 57.96 | .99 | .793 | 1.000 | .980 | pass |
| SAP Warp | .3107 | 4.040 | 1.060 | 58.77 | .18 | .146 | 1.000 | .942 | pass |
| SAP Warp | .2718 | 4.040 | 1.212 | 58.96 | .00 | .0016 | 1.000 | .826 | pass |
| SAP Warp | .1631 | 4.040 | 2.020 | 58.99 | .00 | .0016 | 1.000 | .496 | pass |

### Nominal N = 6 N per side

| engine | mass (kg) | actual N (N) | r | follow (mm) | slip (mm) | mean abs vt (mm/s) | duty | rho | outcome |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| MuJoCo | .9786 | 5.948 | .496 | -6763.49 | 6820.62 | 5460 | .015 | 1.000 | fail |
| MuJoCo | .6116 | 5.932 | .791 | -5627.37 | 5684.75 | 4550 | .033 | 1.000 | fail |
| MuJoCo | .5151 | 5.924 | .938 | -3636.75 | 3694.64 | 2960 | .091 | 1.000 | fail |
| MuJoCo | .4893 | 5.922 | .987 | .07 | 59.81 | 47.8 | .555 | .999 | fail |
| MuJoCo | .4660 | 5.919 | 1.036 | 56.27 | 3.61 | 2.89 | .970 | .964 | fail |
| MuJoCo | .40775 | 5.911 | 1.182 | 58.96 | .94 | .750 | .994 | .845 | pass |
| MuJoCo | .24465 | 5.870 | 1.957 | 59.73 | .21 | .167 | 1.000 | .512 | pass |
| SAP Warp | .9786 | 6.000 | .500 | -2452.07 | 2509.40 | 2010 | .448 | 1.000 | fail |
| SAP Warp | .6116 | 6.000 | .800 | 25.55 | 33.32 | 26.7 | 1.000 | 1.000 | fail |
| SAP Warp | .5151 | 6.000 | .950 | 51.33 | 7.57 | 6.07 | 1.000 | .997 | fail |
| SAP Warp | .4893 | 6.000 | 1.000 | 56.92 | 1.99 | 1.60 | 1.000 | .986 | fail (0.08 mm below gate) |
| SAP Warp | .4660 | 6.000 | 1.050 | 58.50 | .41 | .327 | 1.000 | .950 | pass |
| SAP Warp | .40775 | 6.000 | 1.200 | 58.93 | .00 | .0016 | 1.000 | .834 | pass |
| SAP Warp | .24465 | 6.000 | 2.000 | 58.97 | .00 | .0016 | 1.000 | .501 | pass |

## What the results do and do not show

1. **The expected static threshold is recovered.** Both engines fail clearly
   below `r=1` and pass at sufficient margin. This validates the measured-force
   normalization and confirms that the initial normal contact is not the
   reason for the observed transition.
2. **Threshold intervals differ in this task/configuration.** SAP changes
   from failure to pass between `r=.959` and `1.010` at N4, and between `1.000`
   and `1.050` at N6. MuJoCo changes between `1.031` and `1.176` (N4), and
   between `1.036` and `1.182` (N6). SAP is closer to the ideal `r=1` boundary
   under this particular native setup and 95%-follow metric.
3. **This is not a universal engine ranking.** The engines use different
   contact formulations and native solver settings. The result is a
   force-matched, task-level comparison of these configurations; it does not
   prove that SAP always models grasp friction more accurately.
4. **The near-threshold difference is accumulated slip.** Around `r=1`, both
   engines keep contact for much of the lift but have `rho` close to one and
   enough tangential relative motion to miss the 57 mm gate. A longer force
   hold is needed before making a separate claim about steady creep.
5. **No SAP contact-capacity invalidation remains.** All v3 SAP cases use
   capacity 128 and record `max_truncated_contacts=0`; unlike the abandoned
   old SAP pinch run, force and gap telemetry are valid.

## Relation to earlier simple-pinch work

| dataset | status | reason |
|---|---|---|
| `exp-results/simple-pinch-v2/` | retained, but not force-fair | compared the same geometric compression, which generated different N |
| v3 MuJoCo `rN_N2`, `rN_N4`, `rN_N6` | retained exploratory data | used initial analytic compression guesses; actual measured N drifted, so they are not the final matched curves |
| v3 `matched-N4`, `matched-N6` | final result | gate-validated and compared by actual measured N and r |
| N2 matched curve | intentionally absent | no stable 2 N SAP preload exists with the fixed SAP configuration |

## Appendix — preliminary MuJoCo analytic-compression trials

Before the final empirical N matching, MuJoCo was run at three compression
values derived from an initial scalar calibration. They are included here for
completeness, but are **not** used in the final cross-engine conclusion:
actual preloads are about 2.03, 4.12, and 6.20 N rather than exactly the
nominal 2, 4, and 6 N. All passed the normal gate.

### Initial nominal N2 sweep

| mass (kg) | compression (mm) | actual N (N) | r | follow (mm) | slip (mm) | outcome |
|---:|---:|---:|---:|---:|---:|---|
| .3262 | .10478 | 2.0592 | .5148 | -6623.80 | 6680.96 | fail |
| .2039 | .10478 | 2.0432 | .8172 | -5209.22 | 5266.70 | fail |
| .1717 | .10478 | 2.0353 | .9667 | -225.05 | 284.52 | fail |
| .1631 | .10478 | 2.0326 | 1.0163 | 56.60 | 3.36 | fail |
| .1553 | .10478 | 2.0300 | 1.0660 | 58.28 | 1.68 | pass |
| .1359 | .10478 | 2.0222 | 1.2134 | 59.30 | .67 | pass |
| .08155 | .10478 | 1.9815 | 1.9814 | 59.80 | .18 | pass |

### Initial nominal N4 sweep

| mass (kg) | compression (mm) | actual N (N) | r | follow (mm) | slip (mm) | outcome |
|---:|---:|---:|---:|---:|---:|---|
| .6524 | .20956 | 4.1456 | .5182 | -6657.12 | 6714.27 | fail |
| .4077 | .20956 | 4.1293 | .8259 | -5295.65 | 5353.11 | fail |
| .3434 | .20956 | 4.1212 | .9787 | -720.73 | 779.79 | fail |
| .3262 | .20956 | 4.1185 | 1.0296 | 56.25 | 3.67 | fail |
| .3107 | .20956 | 4.1158 | 1.0803 | 58.12 | 1.80 | pass |
| .2718 | .20956 | 4.1077 | 1.2325 | 59.21 | .72 | pass |
| .1631 | .20956 | 4.0653 | 2.0326 | 59.76 | .19 | pass |

### Initial nominal N6 sweep

| mass (kg) | compression (mm) | actual N (N) | r | follow (mm) | slip (mm) | outcome |
|---:|---:|---:|---:|---:|---:|---|
| .9786 | .31434 | 6.2321 | .5193 | -6693.35 | 6750.50 | fail |
| .6116 | .31434 | 6.2157 | .8288 | -5333.25 | 5390.70 | fail |
| .5151 | .31434 | 6.2075 | .9828 | -869.55 | 928.53 | fail |
| .4893 | .31434 | 6.2048 | 1.0341 | 56.06 | 3.82 | fail |
| .4660 | .31434 | 6.2020 | 1.0854 | 58.05 | 1.83 | pass |
| .40775 | .31434 | 6.1939 | 1.2388 | 59.17 | .72 | pass |
| .24465 | .31434 | 6.1509 | 2.0503 | 59.73 | .20 | pass |

## Reproducibility and remaining work

- MuJoCo runner: `mujoco-sim/learn_mujoco/pinch_grasp_experiments/run_simple_pinch_v2.py`
- SAP runner: `sap-sim/scripts/run_simple_pinch_v2_sap.py`
- This report is based on `exp-results/simple-pinch-v3/`.

The next clean extension is a 5--10 s sub-threshold constant-force hold with
the same measured preloads, recording tail `vt`, accumulated slip, `rho`,
and contact duty. A smooth preload ramp would additionally remove the current
instantaneous-position-preload transient. Neither extension is silently
included in the results above.
