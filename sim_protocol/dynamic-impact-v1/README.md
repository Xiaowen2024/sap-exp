# Dynamic-impact v1 — shared evaluation protocol

New comparison runs follow the [Unified B=5 SOP](../UNIFIED-B5-SOP.md). Existing B=100 artifacts are historical upper bounds, not the primary budget.

This folder defines the reusable **v1 comparison protocol** for dynamic-impact
and collision tasks. Future tasks can use different geometries (for example, a
ball drop, a tossed cube, or a falling foot) while reporting the same three
metric layers below.

## Scope and coordinate convention

The target regime is the impulsive normal-contact transition:

\[
\Delta t_c \ll T_{\rm macro}, \qquad F_n \gg mg, \qquad v_n^- \to v_n^+.
\]

Every displacement, velocity, and trajectory is expressed in the support
frame (for example, ground or tabletop). Friction \(\mu\) must be obtained
from an independent friction calibration and frozen before impact evaluation.

## 1. Real-world accuracy

Use this layer only when synchronized physical reference data exist. Record
sampling rate, support-frame registration, and the evaluation window.

| Metric | Definition |
|---|---|
| Trajectory RMSE | Framewise RMSE between simulated and measured support-relative pose/position trajectory. |
| DTW | Dynamic-time-warping distance between simulated and measured support-relative trajectories. Always report it alongside time-aligned RMSE. |

## 2. Simulation-level consistency

These quantities are available even without a real-world reference. State the
evaluation window and the touch/liftoff detection rule.

| Metric | Definition | Ideal impact interpretation |
|---|---|---|
| Effective restitution \(e\) | \(e=-v_n^+/v_n^-\), measured immediately before contact and after separation. | Matches the reference over impact speed without energy growth \((e\leq1)\). |
| Contact duration \(t_c\) | \(t_c=t_{\rm liftoff}-t_{\rm touch}\), for continuous normal-force support. | Matches calibrated dynamic softness; avoids excessive compliance or instability. |
| Impulse closure | \(J=\sum_t F_n\,dt\), checked against normal momentum change when a unique support-normal wrench is defined. | Force integration and state change are physically consistent. For coupled multi-point/closed-chain contact, report it only after defining the full resultant wrench and momentum balance. |

## 3. Mechanism-aware usability

These are experiment-design metrics, not quantities inferred retroactively
from one successful trace.

| Metric | Definition |
|---|---|
| Bounded tuning budget | Within native contact/solver controls, report `Pass@B` and \(B_{\rm first\ pass}\) for a preregistered impact criterion; compare with a same-budget brute-force baseline where available. |
| Plausible-range controllability | Freeze tuning, then vary mass, normal load, impact velocity, drop height/geometry, and independently calibrated \(\mu\). Check whether the expected physical trend is recovered. |
| Numerical local robustness | With physics and tuning frozen, perturb timestep, initial state, and seed. For every `sample × frozen configuration`, run at least `R=20` draws from a predeclared perturbation distribution; report per-sample mean/SD and pooled within-sample SD. |
| Setup-variant robustness | Repeat on a different meaningful environment/loading setup and report whether the accuracy/consistency conclusion transfers. |

### Calibration-budget stages

A budget unit is one complete native parameter-configuration evaluation over
all training rollouts, not one physics step or individual trial.

| Stage | Budget | Rule | What it measures |
|---|---:|---|---|
| Default | \(B=1\) | Documented/default engine contact and solver settings; no trajectory fit. | Out-of-the-box behavior. |
| First-principles calibration | \(B\leq5\) | Up to five predeclared, mechanism-motivated candidates selected only on training data. | Expert calibration under a realistic budget. |
| Brute-force upper bound | \(B\approx100\) | Fixed disclosed native grid selected only on training data. | Grid-limited oracle ceiling, not a practical usability claim. |

Freeze the selected configuration before held-out scoring. All real-data
metrics report mean, SD, and 95% CI across held-out physical samples; this is
**between-sample** variation, not repeat variance. Exact replay with identical
state, control, timestep, and seed is a reproducibility check only: a
deterministic engine should give zero variance. Report its maximum state
difference once per engine/version, but do not count duplicated traces as
robustness rollouts.

The current ball-bounce, cube-toss, and Cassie files are **pilot /
across-sample only** until their `R=20` local-robustness rollouts are complete.

## Task-level metric registry

Each task reports the three shared layers first, then only its additional
physical outcomes.

| Task | Additional task-level metrics | Extension |
|---|---|---|
| Bouncing ball | Apex-height decay error over the primary \(K\) bounces. | [ball-bounce.md](task-metrics/ball-bounce.md) |
| Cube toss | Final pose error (position, orientation, resting face) and post-impact linear/angular speed-decay errors. | [cube-toss.md](task-metrics/cube-toss.md) |
| Cassie landing | Landing success, post-impact stability/chatter, and weighted 50-ms state error. | [cassie-landing.md](task-metrics/cassie-landing.md) |

## Reporting requirements

For every experiment, save geometry, mass/inertia, ground pose, initial
height/velocity/angle, timestep, duration, pass criterion, frozen engine
settings, calibration source, comparison type, contact aggregation rule, all
perturbation/seed identifiers, and whether a synchronized real reference
exists. Without one, label results simulation-level consistency—not real-world
accuracy.

