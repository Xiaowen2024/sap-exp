# Website result-summary protocol (v2)

## Purpose

This is the publishing protocol for the mechanism-aware result library. It does not replace a task's metric specification or `UNIFIED-B5-SOP.md`. Its job is to prevent a task page from becoming a short conclusion without the physical setup, the actual B=1/B=5 configurations, or the evidence for transfer.

The public comparison is **SAP Warp versus MuJoCo Warp**. `pending`, `not comparable`, `not discriminative`, and `complete-negative` are legitimate outcomes and must never be silently rendered as a zero or a success.

## Mandatory structure: one task card, three sections

Every published task must appear once and must use the following order. A task without enough evidence for all three sections belongs in the inventory, not among the result cards.

### 1. Task setup — explain and show the physical experiment

Start with a concise description of:

- the bodies, contacts, command/initial state, and the expected physical behaviour or reference;
- the condition grid, selection versus frozen held-out split, repeat count `R`, and geometry/reference asset identity;
- the task's primary outcome definition and its health gates.

Every card must include a **clear setup schematic** (photo/render/SVG is acceptable) with labelled bodies, direction of motion/load, and the measured mechanism. It must also include one result visual that makes the B=1 to B=5 change visible. A decorative image is not a substitute for either figure.

### 2. B=1 → B=5 core evidence — exact settings, result, and failure mode

Use one table with these four rows whenever both engines are available:

| Engine | B=1: literal compiled/default setting and primary result | B=5: frozen selected setting, what changed, and primary result | One or two mechanism metrics and main failure case |
| --- | --- | --- | --- |
| SAP Warp | ... | ... | ... |
| MuJoCo Warp | ... | ... | ... |

Requirements:

1. Spell out the actual B=1 and B=5 parameters, including native options such as `NoSlip`, solver/friction settings, SAP `ke`, `tau`, gap, or the task-specific control/geometry parameter. “Tuned” is not a setting.
2. State **what changed from B=1 to B=5** and the corresponding observed change. Do not imply causal certainty beyond this controlled candidate comparison.
3. Report accuracy/agreement and denominator when the task has discrete, preregistered expected outcomes. For example: `260/280 = 92.9%`. If a class is absent, write `not discriminative: no expected-retain/release`; never present a 100% one-class agreement as a meaningful accuracy.
4. If the task is continuous rather than a classification task, explicitly write `no binary accuracy is defined` and report its actual primary fidelity measure (e.g. valid-apex rate and apex loss, trajectory loss, position RMSE). Do not invent a success percentage.
5. Report one to three predeclared, mechanism-specific metrics with units:
   - friction/grasp: preload or normal force, friction margin, tail creep, lift-follow error, contact duty;
   - sustained contact: overlap/penetration, normal-load error, settling, contact duty;
   - impact: valid rebound/apex, loss, contact duration, finite/capacity status;
   - trajectory/toss: held-out position/orientation/velocity error and contact/finite status;
   - insertion: credible-insertion rate, depth/alignment/penetration and force/capacity health.
6. Name the dominant failure explicitly (for example false slip at a threshold, no rebound apex, lost contact, excessive overlap, or a failing mass/dt condition). A mean must not hide it.

The website may show only B=1, the frozen B=5 winner, and at most one representative intermediate candidate. The raw artifact must retain the full B=1…B=5 candidate ordering.

### 3. Generalizability and mechanism-aware usability

This is a separate subsection on every result card. It answers two questions and must give numbers for both when they exist:

1. **Candidate usability.** Which B=1…B=5 configurations pass the selection task/health gate under the declared repeats? The public card can summarize this as `B=1 fails; B=3 and B=5 pass; B=5 selected`, but must identify the selected configuration exactly.
2. **Frozen-setting robustness.** With B=5 frozen, what happens on the preregistered held-out mass/friction/dt/geometry/pose/trajectory grid? Give coverage, denominator, range or worst case, representative mechanism metric, and the failed condition if any.

Use a compact table or a precise sentence in this form:

| Engine | Candidate-usability evidence | Frozen held-out coverage | Held-out result/range | Robustness status |
| --- | --- | --- | --- | --- |
| SAP Warp | ... | ... | ... | robust / complete-negative / pending |
| MuJoCo Warp | ... | ... | ... | robust / complete-negative / pending |

Never infer robustness from a single nominal rollout. If candidate-level `R=20` evidence, a frozen grid, or a necessary force/contact audit is absent, say `unavailable` or `pending audit`. A nominal winner that fails dt or geometry transfer is a reportable **complete-negative** result, not a reason to omit the subsection.

## Metric and comparability rules

- Use the task metric specification chosen before cross-engine inspection; every displayed number carries a denominator or sample count and units.
- Balanced accuracy, FPR/FNR, and confusion matrices are diagnostic details, not default website fields. Add them only when they clarify a material difference; link the raw artifact for the full matrix.
- Cross-engine deltas may be stated only if reference, physical condition, validity gates, and metric definition match. Never turn a B=5 selected configuration into a universal simulator ranking.
- A force-, friction-margin-, or penetration-based conclusion requires the relevant telemetry/health audit. Before it is audited, the task remains in the inventory as pending even if numerical candidates exist.

## Inventory and publication gate

The page begins with a small inventory of all registered tasks, including completed, complete-negative, pending, blocked, and not-comparable entries. Only a task with both SAP Warp and MuJoCo Warp evidence can make the main comparison card. If one backend is still running (for example round socket Warp), retain it in the inventory rather than publishing a partial comparison.

Before publishing a card:

1. extract all displayed numbers from the raw artifact, not prose;
2. verify B=1 is literal default and B=5 is the frozen selected candidate;
3. verify denominators, class balance, units, and validity/health flags;
4. confirm that both figures and all three required sections are present;
5. link the source artifact and record the repository commit;
6. have an independent pass audit any conclusion that depends on contact force, force balance, friction margin, or penetration.
