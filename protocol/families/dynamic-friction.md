# Dynamic-friction v2 — shared evaluation protocol

New comparison runs follow the [Unified B=5 SOP](../UNIFIED-B5-SOP.md).

This protocol defines reusable metrics for tasks with sustained kinetic sliding. Each concrete task uses these shared quantities first, then adds its own task-level endpoint and expected-outcome rule. All trajectories are expressed in the support frame and evaluated over a declared sliding window.

## Scope

For ideal Coulomb kinetic sliding,

\[
\lVert F_t\rVert=\mu_k F_n,
\qquad
\rho=\frac{\lVert\sum F_t\rVert}{\mu_k\sum F_n}\approx1.
\]

This protocol targets atomic tasks with approximately constant kinetic friction and normal support. It intentionally does not make energy loss, terminal velocity, or sliding-velocity RMSE primary metrics: in this regime they are largely redundant with acceleration and terminal displacement. They can still be logged as diagnostics when contact is intermittent, restitution is present, or the normal load is time-varying.

## 1. Real-world accuracy

Use this layer only when a synchronized physical reference trajectory is available. Record the support-frame registration, sampling rate, and fitting window.

| Metric | Definition |
|---|---|
| Trajectory RMSE | Framewise error between simulated and measured support-frame trajectories. |
| DTW | Dynamic-time-warping distance between the support-frame trajectories, allowing limited phase offset. Always report it alongside time-aligned RMSE. |
| Tangential acceleration/deceleration error | Fit \(\hat a_{\rm sim}\) and \(\hat a_{\rm real}\) from the full sliding displacement trace \(s(t)\), then report \(\left|\hat a_{\rm sim}-\hat a_{\rm real}\right|\) and optionally its normalized form. Do **not** estimate acceleration through framewise second differences, which amplify motion-capture noise. |
| Terminal displacement error | At fixed time \(T\), \(\left|s_{\rm sim}(T)-s_{\rm real}(T)\right|\). Report the physical unit and, when useful, normalize by the measured travel distance. |

For the acceleration fit, use a stated regression model over the entire valid sliding interval—e.g., \(s(t)=s_0+v_0t+\tfrac12 at^2\)—and report its fitting window. Exclude launch/transient frames only by a predeclared rule.

## 2. Simulation-level consistency

These metrics need no physical dataset. They test the simulated trajectory against the task's analytical Coulomb reference.

| Metric | Definition |
|---|---|
| Tangential acceleration error | Compare \(\hat a_{\rm sim}\) with analytical \(a^*\). For an incline, \(a^*=g(\sin\theta-\mu_k\cos\theta)\); for horizontal free deceleration, \(a^*=-\mu_k g\). Report absolute and/or relative error, with the convention stated when \(a^*\) is near zero. |
| Terminal displacement error | \(\left|s_{\rm sim}(T)-s^*(T)\right|\), where \(s^*(T)\) comes from the same analytical acceleration and initial state. |
| Friction utilization | \(\rho=\lVert\sum F_t\rVert/(\mu_k\sum F_n)\), aggregated over the support interface. During continuous kinetic sliding it should be near 1. |
| Contact duty | \(D=\operatorname{mean}\,\mathbf1[C(t)>0]\), where \(C(t)\) is the required support-interface contact count. Under intended continuous sliding it should be near 1. |
| Normal-support / lift-off error | Report maximum normal gap, maximum height deviation from the support geometry, and/or fraction of time without support contact. This prevents a tangential acceleration that is coincidentally correct while the body is partly airborne. |

The normal-support diagnostic is deliberately retained. In its oblique-sliding evaluation, CRISP visualizes block height and identifies lift-off/gliding as a contact artifact rather than interpreting tangential trajectory alone ([CRISP, Sec. VI-D](https://arxiv.org/html/2609.21761v1#S6.D)).

## 3. Mechanism-aware usability

Apply the same study-level rules as [Static-friction v2](static-friction.md):

| Metric | Definition |
|---|---|
| Bounded tuning budget | Within each engine's native controls, report `Pass@B` and \(B_{\rm first\ pass}\) for a preregistered criterion; compare with a same-budget brute-force baseline when available. |
| Plausible-range controllability | Freeze tuning, then vary \(\mu_k\), mass, slope/external load, initial speed, and geometry over a stated physical range. Check whether the correct response trend is produced. |
| Numerical local robustness | With physics and tuning frozen, perturb \(dt\), initial state, and seed; report the stability of the metrics and task outcome. |
| Setup-variant robustness | Evaluate a second meaningful geometry or loading variant and report whether the conclusion transfers. |

## 4. Future task-level outcomes and repeated rollouts

Every future dynamic-friction task must define its own binary expected outcome before running: for example, reaches the stop line versus overshoots, remains supported versus lifts off, or traverses an expected distance band versus does not. The shared protocol deliberately does **not** prescribe a universal error cutoff, because an acceptable terminal-displacement error depends on task scale, reference noise, and downstream use.

Run every condition for \(M\) rollouts using a declared perturbation/seed distribution. The task extension must then report its condition-level fraction, pooled `TP/FP/FN/TN`, false-positive rate, false-negative rate, balanced accuracy, denominators, and 95% Wilson or bootstrap confidence intervals. Also report macro averages over physical-condition groups so an oversampled boundary region does not dominate the aggregate statistic.

## 5. Task-level metric registry

| Task | Additional task-level metrics | Extension |
|---|---|---|
| Slope Slider | None. It is the real-world atomic validation of the shared sliding-law and support metrics. | [slope-slider.md](../../tasks/real-data-simple/slope-slider/task-metrics.md) |
| MIT Push | Final planar object-pose error and windowed wrench--twist response errors. | [mit-push.md](../../tasks/real-data-simple/mit-push/task-metrics.md) |

Future dynamic-friction tasks should add only the endpoint metrics that are not already represented by the shared protocol. Their binary outcome labels and thresholds must be defined at task level before repeated rollouts are run.

## Required record for every run

Save geometry; \(m,\mu_k,\theta\); gravity; initial state; support frame; timestep; duration; expected analytical reference; frozen engine settings; contact aggregation rule; and all perturbation/seed identifiers. When physical data are used, also save registration and synchronization details.

