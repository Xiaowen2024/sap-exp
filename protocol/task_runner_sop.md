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

- a passing **engine preflight** for the exact simulator source, device, and
  Warp version used by the sweep. For SAP this is
  `SAP-WARP-INTERNAL/local_experiments/diagnostics/sap_preflight.py --compare-devices --json <out>`
  (static load = m g at three `dt`, free fall of every body in a multi-root
  scene, batched-vs-alone contact-force readout, CPU/GPU agreement). Store the
  JSON with the artifact. A failed check blocks the sweep, or else restricts
  it to metrics shown to be unaffected and records that restriction. As of
  `db8d236`: CPU fails free fall (bug A), CPU and GPU fail batched readout
  (bug B); with the `fix-gamma-readout` branch GPU passes all checks;

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

The full cross-regime execution inventory, including tasks that are blocked by
missing released inputs rather than an unrun simulation, is maintained in
[generalizability_rerun_register.md](generalizability_rerun_register.md).

| Website task | Existing SAP coverage | Existing MuJoCo Warp coverage | Required next common frozen matrix | State |
| --- | --- | --- | --- | --- |
| Three-box push | Historical 12-cell R=20 sweep exists, but no pre-run common manifest or raw per-repeat contract was saved. | Same limitation. | Declare a shared selection screen and a new common matrix: breakaway ratio, mass, friction, three `dt` levels, and pushed-body setup. | **rerun required**; historical sweep remains descriptive only. |
| Simple pinch | A recovered GPU 17-cell sweep exists, but labels are computed from SAP's own measured preload and there is no surface/geometry transfer cell. | A separate 17-cell sweep exists with engine-specific preload labels. | Use a shared external task-success label, common mass / commanded grip / friction / three-`dt` / surface-or-geometry matrix, and common seeds. | **rerun required**; historical accuracy values are not cross-engine comparable. |
| Three-plate stack | Historical 11-cell R=20 sweep exists, but no pre-run common manifest/raw-repeat contract was saved. | Same limitation. | Declare a common calibration screen and common thickness, mass, radius, placement-offset, and three-`dt` held-out matrix. | **rerun required**; historical sweep remains descriptive only. |
| GAUGE ball bounce | A 12-cell R=20 artifact is marked `partial`; its pre-run manifest and fixed-frame `dt` protocol are absent. | Same limitation. | Declare common trajectory IDs, release height, lateral offset, mass, three `dt` values, sampler, and continuous apex/trajectory loss. | **rerun required**. |
| ContactNets cube toss | 12 non-`dt` plus 3 `dt` R=20 artifacts exist, but the non-`dt` artifact is marked `partial` and no pre-run manifest/raw-repeat contract was saved. | Same limitation. | Declare common selection samples and common speed, yaw, mass, friction, held-out state, and three-`dt` matrix. | **rerun required**. |
| Round socket | The recovered historical selection script reproduces 18/20 credible insertions, but its B=5 screen changes `dt` from the B=1 physical task and therefore is not a compliant B=5 selection under this SOP. | An expanded 12-cell R=20 sweep exists, but it was not paired with a compliant SAP selection/matrix. | Re-select B=5 at fixed nominal `dt`, then run the common clearance, lateral/angle offset, insertion input, setup-transfer, and three-`dt` matrix. | **rerun required**; historical result remains descriptive only. |

### 6.1 Reproducibility preflight for an old frozen artifact

Before running any row above, recover the exact runner revision and parameter
mapping, then rerun the excluded selection condition for R=20. It passes the
preflight only when all of the following agree with the published artifact:

- the frozen configuration has the same complete parameter values and units;
- the selection outcome count matches exactly;
- at least one historical held-out/boundary anchor from every reported outcome
  class also matches exactly (selection-point agreement alone is insufficient);
- the reported mechanism metric is within the task's preregistered tolerance;
- all validity gates (finite state, contact capacity, overflow/truncation) are
  the same; and
- the artifact records the runner commit/version, engine version, device, and
  selection trace hash.

For the currently published SAP simple-pinch artifact, the recovered original
runner on CPU passes the selection point (3.636335 N/side versus the published
3.636346 N/side, 20/20 retain, no truncation), but fails this stronger
preflight at the 0.40-kg boundary anchor: the historical artifact has 0/20
retain while the CPU rerun has 20/20 retain. Its completed 17-cell CPU matrix
is therefore a device-specific replication, not a replacement for the
historical SAP result or a fair SAP–MuJoCo robustness comparison. Repeat the
common matrix on the historical SAP GPU/runtime before publishing an
engine-level robustness conclusion.

**Update 2026-11-03 ([audit](../exp-report/sap-force-readout-audit.md)).**
The 0.40-kg disagreement has been traced to a SAP CPU solver bug, not to
runtime nondeterminism. On CPU, gravity and `body_f` reach only the first body
of each world, so the pinch cube (body 4) is weightless. The CPU matrix is
invalid and the aligned runner now refuses `--device cpu`. The GPU 17-cell
matrix agrees 340/340 once labels use the true PD-spring preload. Its
gamma-based labels were corrupted because `contact_gamma` reads zero for
early-converged worlds in a batch. Every preflight must now also check:
(a) force readouts divide by the step's actual `dt`; (b) preload/load is
cross-checked against a force-independent quantity (spring deflection,
statics); and (c) no SAP CPU run uses a scene with more than one root body.
