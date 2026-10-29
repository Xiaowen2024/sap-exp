# Result-summary SOP: mechanism-aware B=1 / B=5 reporting

## Purpose and scope

This document governs the result summary shown to reviewers and the public
website. It does not replace the per-task metric specification or
[`UNIFIED-B5-SOP.md`](UNIFIED-B5-SOP.md). Its purpose is to ensure that every
task is reported in the same order and that a high aggregate score never hides
an invalid contact regime, an uninformative class balance, or a brittle B=5
choice.

Every report must cover **MuJoCo CPU, MuJoCo Warp, and SAP Warp separately**.
`Not run`, `not comparable`, `pending audit`, and `invalid` are valid status
values; they must never be rendered as zero, a failure, or silently omitted.

## 1. Required per-task result block

For every task, publish one compact result block in this exact order.

### A. Task identity and validity

- protocol family, task name, task version, and reference type
  (`analytic`, `simulation-consistency`, or `real-data fidelity`);
- physical condition grid, selection-only cases, frozen held-out cases, and
  local-repeat count `R`;
- per-engine version/device and geometry/asset identity;
- result status for each engine: `complete`, `complete-negative`,
  `pending-audit`, `not-run`, `invalid`, or `not-comparable`;
- validity/health gates relevant to the task: finite state, contact duty,
  contact capacity/truncation, force-readout validity, support/normal-load
  closure, and any task-specific state/asset audit.

### B. B=1 and B=5 scorecard — mandatory for every available engine

Use one row per `engine × budget`.

| Engine | Budget | Exact selected configuration | Evaluation denominator | Task success rate | Outcome accuracy / agreement | Balanced accuracy | FPR / FNR | 95% interval | Validity status |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| MuJoCo CPU | B=1 | literal compiled default | ... | ... | ... | ... | ... | ... | ... |
| MuJoCo CPU | B=5 | frozen selected candidate | ... | ... | ... | ... | ... | ... | ... |
| MuJoCo Warp | B=1 | literal compiled default | ... | ... | ... | ... | ... | ... | ... |
| MuJoCo Warp | B=5 | frozen selected candidate | ... | ... | ... | ... | ... | ... | ... |
| SAP Warp | B=1 | literal compiled default | ... | ... | ... | ... | ... | ... | ... |
| SAP Warp | B=5 | frozen selected candidate | ... | ... | ... | ... | ... | ... | ... |

Definitions:

- **Task success rate** is the fraction of rollouts satisfying that task's
  preregistered pass criterion (for example retain, settled, credible insertion,
  or trajectory gate). It is mandatory even when a classification metric exists.
- **Outcome accuracy/agreement** compares predicted and expected discrete task
  outcomes. Report its denominator and confusion matrix `TP/FP/FN/TN` whenever
  an expected outcome label exists.
- **Balanced accuracy, FPR, FNR** are mandatory only if both outcome classes
  exist. If one class has denominator zero, write `not identifiable: no
  expected-retain` or `no expected-release`; do not present 100% accuracy as a
  discriminative result.
- Use Wilson 95% intervals for binomial success/accuracy metrics. For continuous
  metrics report mean, SD (or median/IQR when predeclared), sample count, and
  units.

### C. Discriminative mechanism metrics — mandatory

In addition to the scorecard, report **one to three task-specific metrics**
that expose the dominant contact mechanism. Choose them from the task metric
file before looking at cross-engine results; do not choose them after seeing
which engine looks better.

| Task family | Required representative metrics (in addition to success/accuracy) |
| --- | --- |
| Static friction / grasp | measured preload or normal force, friction/support margin, hold creep or lift-follow error, contact duty |
| Static friction / stacked push | tail creep, breakaway/force threshold, normal-load/support health, contact capacity |
| Dynamic friction / slope | travel or trajectory nRMSE, terminal velocity/displacement, contact and normal-support duty |
| Dynamic friction / push | final translation/orientation error, wrench and twist error, pusher/table duty, normal-load error |
| Dynamic impact / bounce | restitution/apex or impulse/closure residual, contact duration, finite/capacity status |
| Dynamic impact / toss | held-out position/orientation error, angular/linear momentum or impact-loss diagnostic, finite/capacity status |
| Sustained contact / stack | force/load error, settling/tail motion, contact duty, penetration/overlap where defined |
| Sustained contact / resting tail | tail pose/velocity, support-force closure, force coefficient of variation, contact duty |
| Energy | maximum energy drift/envelope error, retained energy, finite state, contact/constraint health |
| Composite insertion | credible-insertion/pass rate, depth/alignment/penetration metrics, force/torque and capacity health |

If a task's specification has another more diagnostic metric, report that
metric instead of forcing an inapplicable one. Each value must carry units and
the condition over which it was computed.

### D. B=5 mechanism-aware usability and robustness — mandatory

This is a separate section, not an optional tuning footnote. It answers two
different questions:

1. **Candidate usability:** across the five predeclared configurations
   (including literal B=1), which candidates pass the selection health/task
   gate under `R=20`? Report all five candidates, not only the winner.
2. **Frozen-setting robustness:** after selection, does the single frozen B=5
   winner retain validity and task performance on the held-out physical grid
   (mass, friction, geometry, dt, pose, trajectory, or material variants as
   applicable)?

Required table:

| Engine | Candidate index/configuration | Selection pass fraction | Selection primary loss | Health-gate result | Selected? | Held-out condition coverage | Held-out pass fraction/range | Worst representative metric | Robustness status |
| --- | --- | ---: | ---: | --- | --- | --- | --- | --- | --- |

Rules:

- Candidate indices must preserve the declared B=1...B=5 order. B=1 is part of
  the candidate table.
- `Held-out condition coverage` is `evaluated valid conditions / preregistered
  held-out conditions`, followed by the reason for every missing condition.
- Report the range and worst held-out value, not only the mean. Also report the
  fraction of held-out conditions satisfying the task gate.
- If candidate-level `R=20` data or the held-out grid is absent, label
  `robustness: unavailable` or `pending audit`; never infer robustness from a
  single winning rollout.
- A completed negative robustness result is reportable. For example, a winner
  may pass the nominal case but fail dt variants; this must be displayed as
  `complete-negative`, not hidden.

### E. Cross-engine deltas — only when comparable

After the six-row scorecard and robustness table, list at most three
cross-engine observations that meet all of the following requirements:

- same physical condition and reference type;
- both values pass their required validity gates;
- metric definition and denominator are the same;
- the magnitude, unit, and direction are stated.

Example format: `At 4 N/side, SAP B=5 tail creep = ... µm/s versus MuJoCo CPU
B=5 = ... µm/s (Δ=... µm/s).` A difference that is large but has failed force
health or mismatched geometry belongs in the validity note, not in a ranking.

## 2. Whole-benchmark summary required before task-by-task presentation

Before presenting individual task cards, publish an inventory table covering
every registered task:

| Family | Task | CPU status | Warp status | SAP status | B=1/B=5 scorecard complete? | Robustness evidence | Main blocker or audit note |
| --- | --- | --- | --- | --- | --- | --- | --- |

The table must include completed, negative, pending, blocked, and planned tasks.
Only after this table may the report drill into one task at a time. Do not use a
single task's result to imply benchmark-wide completion.

## 3. Publishing and audit gate

1. Extract the result block from numerical artifacts, not prose summaries.
2. Verify each displayed value against the raw artifact and record the artifact
   path/commit.
3. Check scorecard denominators and class balance before computing accuracy or
   balanced accuracy.
4. Check the B=5 candidate table and held-out coverage before claiming
   mechanism-aware usability.
5. Have a second pass audit contact/force telemetry for any task whose
   conclusion depends on force balance, friction margin, or penetration.
6. Mark the task card publishable only when its result block, robustness row,
   and artifact links are complete. Otherwise report only its inventory status.

## 4. Current implementation note

Existing artifacts do not yet expose candidate/held-out robustness in a fully
uniform schema. The following evidence exists and must be carried into the
whole-benchmark inventory rather than omitted:

- three-plate stack: complete SAP B=1/B=5 robustness matrix; MuJoCo Warp has a
  completed negative robustness result across thickness/dt variants;
- round socket: CPU and SAP candidate lists plus frozen clearance/offset
  matrices exist; MuJoCo Warp was not run;
- pendulum/Newton cradle: strict B=1/B=5 selection robustness exists, but
  frozen held-out amplitude/dt transfer is pending;
- impact tasks: candidate lists and held-out artifacts exist, but legacy
  artifacts that lack `R=20` per candidate cannot be relabeled as compliant;
- GAUGE slope, MIT Push, cube-toss sustained tail: numerical B=5 runs exist,
  but force/contact-health audit is still required before a mechanism/force
  conclusion is publishable.

