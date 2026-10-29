# Task runner SOP: aligned frozen-setting generalizability

This SOP is the required execution addendum for every task reported under the
shared B=1 / B=5 comparison protocol. It prevents a selection result from
being presented as parameter or setup robustness. It applies to SAP Warp and
MuJoCo Warp alike: the engines may use different native solver/contact
parameters, but their *physical evaluation coverage* must be aligned.

The budget, literal-default definition, candidate selection procedure, and
R=20 repeat rule remain those in [UNIFIED-B5-SOP.md](UNIFIED-B5-SOP.md). This
document specifies the required frozen-setting sweep after B=5 is selected.

## 1. Required phases for every engine and task

1. **B=1 audit.** Run the literal compiled/native default on the preregistered
   selection condition. Save actual compiled values, health telemetry, and the
   task outcome.
2. **B=5 selection.** Evaluate at most five declared native-parameter
   candidates (including B=1) on the same selection-only conditions. Select
   one candidate with the preregistered task gate and freeze its complete
   configuration. Do not tune it again during the sweep below.
3. **Frozen-setting generalizability sweep.** Run the frozen B=5 winner on a
   preregistered matrix of the task's important physical and numerical axes.
   These cases are evaluation cases, not extra B candidates. Each condition
   has R=20 independent small initial-state perturbations, with matched seed
   IDs across engines where possible.
4. **Report coverage before result.** State exactly which axes and values were
   varied, the total condition and rollout denominators, then the outcome,
   primary metric, mechanism metric, and failed cases. A partial grid is a
   partial-coverage result; it is never evidence for untested axes.

## 2. Design the aligned frozen matrix before running

For every task, write a matrix manifest before the B=5 sweep. It must contain
the selection cases, disjoint frozen cases, values/units for each axis,
expected qualitative/binary outcome where applicable, R, seed list, duration,
and the exact pass/failure gate.

### 2.1 Mandatory axes

Every frozen matrix must include:

- **Timestep (`dt`)**: at least three values spanning nominal, including one
  smaller and one larger stable/plausible value.
- **Initial-state / pose disturbance**: R=20 independent perturbations at
  every cell; where pose is a physical factor, include an explicit held-out
  pose/offset level as an axis as well.
- **Task-driving physical axes**: sweep each parameter that directly governs
  the task mechanism. Use at least three preregistered plausible levels per
  axis unless the task-specific protocol sets a stronger requirement.
- **One setup-transfer axis**: a meaningful setup variation not used in
  selection (for example geometry/clearance, thickness, object shape, surface,
  trajectory, impact sample, or controller/input profile).

The task owner may add axes; they may not omit an applicable axis because one
engine has not yet been run. If a combination is infeasible, record it as
`not run` with the reason. Do not silently replace it with a different grid.

### 2.2 Mechanism-to-axis checklist

| Task mechanism | Required physical axes in addition to `dt` and R=20 perturbations | Representative frozen-setting metric |
| --- | --- | --- |
| Static friction / pushing / holding | mass or normal load; friction coefficient; applied force or force profile; contact geometry/footprint or push height | retain/slip accuracy; tail creep; contact duty / normal-force stability |
| Pinch / grasp | object mass; commanded compression or grip force; friction coefficient; object/contact geometry or surface | retain/release accuracy; measured preload; slip/height error |
| Sustained contact / stacking | thickness or scale; mass/load; contact geometry; `dt`; pose/placement offset | success gate; maximum normalized overlap; maximum normal-load error; settling |
| Dynamic friction / sliding / pushing | mass; friction coefficient; slope/normal load or force profile; geometry/surface; trajectory/input | outcome accuracy; displacement/velocity or trajectory error; contact/force validity |
| Impact / bounce / toss | impact speed or drop height; angle/orientation; mass or material; `dt`; held-out sample/trajectory | event/outcome accuracy; apex/trajectory/energy error; finite/truncation status |
| Insertion / peg-in-hole | clearance; lateral and angular offset; insertion speed/force; geometry or surface transfer; `dt` | insertion/jam accuracy; radial error; peak/final penetration; contact mode |
| Articulated/contact-rich control | payload/mass; pose/state; contact surface/geometry; input trajectory; `dt` | task success; pose/trajectory error; penetration/contact/health metrics |

If a listed axis is fixed by the physical definition, document why and use the
closest meaningful substitute. The matrix must test the mechanism, not merely
repeat the nominal scene.

## 3. Cross-engine alignment rule

The same frozen matrix manifest is the default for SAP Warp and MuJoCo Warp:
same physical axes and values, geometry asset/representation, control/input
trace, expected labels, runtime, R, and seed IDs. Native B=5 settings are
allowed to differ; the evaluated world must not.

An exception is allowed only when a case cannot be represented in one engine
or an existing historical artifact has narrower coverage. In that case:

1. report each engine's actual matrix separately;
2. label the conclusion **coverage-asymmetric**;
3. state the untested axes and values for each engine;
4. do not make an engine-versus-engine robustness ranking; and
5. create a follow-up run using the common manifest before claiming a fair
   robustness comparison.

For example, a SAP simple-pinch sweep over five masses at fixed `mu`, `dt`,
and 0.2-mm compression supports only a mass-sweep conclusion. It cannot be
called friction, preload, or timestep robust merely because MuJoCo has a
17-condition sweep over those axes. To compare them fairly, freeze both
winners and run the same mass, friction, compression/preload, and `dt` cells.

## 4. Minimum artifact and reporting contract

For every `engine × frozen condition × repeat`, save:

- task ID, artifact/geometry hash, engine and version/commit, device;
- frozen B=5 configuration and candidate index;
- all physical-axis values, `dt`, duration, controller/input trace ID, seed,
  perturbation ID, and whether the condition was selection or held-out;
- expected label and observed task outcome/pass-gate components;
- primary accuracy/loss metric plus task mechanism metrics;
- contact/force validity, finite-state/overflow/truncation flags, and timing.

Aggregate by condition before pooling. For every engine, publish the matrix it
actually ran; per-condition numerator/denominator; pooled accuracy or loss;
range/worst condition; and principal failed gate. When the outcome has only
one class, state that accuracy is **not discriminative** and report mechanism
metrics and gate counts instead.

Use the section title **“Generalizability”** on the website. Its first sentence
must say whether coverage is matched across engines, asymmetric, or pending.
“Generalizable” is warranted only for axes and values actually covered by the
frozen matrix; it never follows from B=5 selection alone.

## 5. Pre-run gate

Do not start the full sweep until the task owner has checked and saved:

- the candidate table and B=5 selection gate;
- the common frozen-matrix manifest, including axis values and held-out IDs;
- selection/held-out disjointness;
- expected labels or reference traces and all metric thresholds;
- the R=20 seed/perturbation schedule; and
- the report template with an explicit `not run` field for every planned cell.

Any later expansion of the matrix is a new declared evaluation revision. Keep
the original matrix and results; do not overwrite its coverage claim.

## 6. Current website-card alignment audit (2026-10-29)

This is a run queue, not a result table. A row remains `needs common rerun`
until both engines have completed the declared matrix from a runner that
reproduces its frozen B=5 selection point.

| Website task | Existing SAP coverage | Existing MuJoCo Warp coverage | Required next common frozen matrix | State |
| --- | --- | --- | --- | --- |
| Three-box push | 14 force/target cells × R=20 | 22 force, friction, mass and `dt` cells × R=20 | Force/target, friction, mass, `dt`, and one geometry/push-height axis; same cells and seeds | needs common rerun |
| Simple pinch | Five masses at fixed `mu=0.4`, 0.2-mm compression and fixed `dt`, R=20 | 17 mass/preload/friction/`dt` cells × R=20 | The declared 17-cell MuJoCo matrix for both engines, plus a preregistered geometry/surface-transfer axis | blocked at reproducibility preflight |
| Three-plate stack | Same eight held-out thickness/`dt` cells as the paired artifact | Same eight held-out thickness/`dt` cells | Add mass/load, placement-offset, and geometry/setup-transfer axes; retain the matched thickness/`dt` cells | needs expanded common rerun |
| GAUGE ball bounce | Same 11 held-out trajectory IDs × R=20 | Same 11 held-out trajectory IDs × R=20 | Add drop height/impact speed, angle, mass/material, and three `dt` values, while retaining the held-out trajectory split | needs expanded common rerun |
| ContactNets cube toss | Same 32 held-out state/trajectory IDs × R=20 | Same 32 held-out state/trajectory IDs × R=20 | Add impact speed, orientation, mass/material, and three `dt` values, while retaining the held-out state split | needs expanded common rerun |
| Round socket | Nine clearance/offset cells × R=20 at SAP `dt=.25 ms` | Nine clearance/offset cells × R=20 at MuJoCo `dt=.5 ms` | Same clearance/offset, tilt, insertion speed/force, and three common `dt` values; add a geometry/surface transfer cell | needs common rerun |

### 6.1 Reproducibility preflight for an old frozen artifact

Before running any row above, recover the exact runner revision and parameter
mapping, then rerun the excluded selection condition for R=20. It passes the
preflight only when all of the following agree with the published artifact:

- the frozen configuration has the same complete parameter values and units;
- the selection outcome count matches exactly;
- the reported mechanism metric is within the task's preregistered tolerance;
- all validity gates (finite state, contact capacity, overflow/truncation) are
  the same; and
- the artifact records the runner commit/version, engine version, device, and
  selection trace hash.

For the currently published SAP simple-pinch artifact, this preflight fails:
the available runner with the recorded labels (`ke=1e5 N/m`, `tau=3 ms`,
0.2-mm compression) measures roughly 120 N/side, while the published B=5
selection artifact reports 3.636 N/side. Do not start its common 17-cell
sweep or publish substitute results until this parameter/runner mismatch is
resolved. The historical five-mass SAP result remains reportable only as its
own explicitly asymmetric coverage.
