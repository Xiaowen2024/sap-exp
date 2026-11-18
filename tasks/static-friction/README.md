# Static-friction v2 — shared evaluation protocol

New comparison runs follow the [Unified B=5 SOP](../UNIFIED-B5-SOP.md).

This folder defines the reusable **v2 comparison protocol** for static-friction tasks. It is not a new result set and does not replace the historical three-box `v3` run. The latter is retained as the first implementation of this protocol family. Future tasks can use a different geometry (for example, a single block, an inclined support, or a two-finger grasp) while reporting the same categories below.

## Scope and coordinate convention

The target regime is

\[
\lVert F_t\rVert < \mu F_n .
\]

Over the evaluation window, the body should remain stationary **relative to its support**. Every displacement, velocity, and trajectory below is therefore computed in the support frame. The protocol separates four questions: agreement with reality, internal contact consistency, calibration effort, and robustness.

## 1. Real-world accuracy

Use this section only when a synchronized physical reference trajectory exists. Report the sampling rate, support-frame registration method, and evaluation window.

| Metric | Definition |
|---|---|
| Trajectory RMSE | Framewise RMSE between simulated and measured support-relative pose/position trajectory. |
| DTW | Dynamic-time-warping distance between the simulated and measured support-relative trajectories; use it only alongside RMSE, not as a replacement for time-aligned error. |
| Relative-displacement error | Error between \(x_{\rm rel}^{\rm sim}(t)\) and \(x_{\rm rel}^{\rm real}(t)\). |
| Relative tangential-velocity error | Error between \(v_t^{\rm sim}(t)\) and \(v_t^{\rm real}(t)\). |

## 2. Simulation-level consistency

These quantities are available even without a real-world reference. State the evaluation window and how an interface/contact set is aggregated.

| Metric | Definition | Ideal sticking interpretation |
|---|---|---|
| Tail relative displacement | Accumulated \(x_{\rm rel}\) in the final hold window. | 0 |
| Tail tangential velocity / creep rate | Report mean and RMS \(v_t\), or the OLS slope of \(x_{\rm rel}(t)\), over the final hold window. | 0 |
| Friction utilization | \(\rho=\lVert\sum F_t\rVert/(\mu\sum F_n)\), aggregated over the active support interface. | \(0<\rho<1\) under loaded static friction; \(\rho\approx1\) indicates saturation. It is **not** 0 merely because the body is stationary. |
| Contact duty | \(D=\operatorname{mean}\mathbf{1}[C(t)>0]\), where \(C(t)\) is the active-contact count for the required support interface. | 1 for continuous support |

Also retain task-specific normal-state diagnostics before interpreting tangential motion: mean \(F_n\) versus expected load, \(\mathrm{CV}(F_n)\), gap/overlap fluctuation, vertical RMS speed, and contact-capacity truncation. A correct mean force alone does not rule out intermittent impulsive support.

## 3. Mechanism-aware usability

These are experiment-design metrics. They are not inferred retroactively from a single successful trace.

| Metric | Definition |
|---|---|
| Bounded tuning budget | For each engine's native contact/solver controls, report whether it passes a preregistered sticking criterion within budget \(B\): `Pass@B` and \(B_{\rm first\ pass}\). Compare against the same-budget brute-force baseline when available. |
| Plausible-range controllability | Freeze the chosen engine tuning, then vary physical conditions over a stated plausible range—\(\mu\), mass, normal load, tangential-load ratio, and geometry. Check whether the model predicts the expected stick/non-stick transition. |
| Numerical local robustness | With physics and tuning frozen, perturb timestep, initial pose, and seed. Report whether the sticking classification and main metrics remain stable. |
| Setup-variant robustness | Repeat on a different but meaningful environment setup. Report whether the real-world accuracy or simulation-level consistency conclusion transfers. |

## 4. Task-level metric registry

Every task in this family reports the shared metrics above first, then adds only the metrics needed for its physical objective. This avoids redefining a complete scorecard for every new geometry.

| Task | Additional task-level metrics | Extension |
|---|---|---|
| Two-finger simple pinch | Minimum measured per-finger hold force \(F_{\rm grip,sim}^{*}\), grip-force error \(E_{\rm grip}\), and lift-height error \(E_z(T)\). | [simple-pinch.md](task-metrics/simple-pinch.md) |
| Three-box push | Expected-outcome agreement for the middle box, plus repeated-condition `TP/FP/FN/TN`, FPR, FNR, and balanced accuracy. | [three-box-push.md](task-metrics/three-box-push.md) |

Future static-friction tasks should add a row and a short extension file here. For example, a block-on-slope task may add critical inclination; a push task may add breakaway load. Neither replaces the common support-relative motion, \(\rho\), duty, and normal-state checks.

## 5. Common outcome-statistics rule

Whenever a task has expected retain/non-retain outcomes, run repeated trials rather than treating one trajectory as conclusive. The task extension must define its positive class and its fixed state gate; report condition-level fractions, `TP/FP/FN/TN`, FPR, FNR, balanced accuracy, denominators, and 95% confidence intervals. Keep both pooled and macro-averaged statistics: pooled results describe all trials, while macro averages prevent oversampled condition groups from dominating the conclusion.

## Reporting requirements

For every static-friction experiment, record:

1. geometry, mass, \(\mu\), gravity, support frame, force/loading profile, timestep, duration, and pass/fail criterion;
2. the frozen engine settings and the calibration data used to choose them;
3. whether results are a matched-physics comparison, an engine-native comparison, or a tuning ablation;
4. contact aggregation and any missing telemetry; and
5. whether a real-world reference exists. If it does not, label results as simulation-level consistency rather than real-world accuracy.

## Existing implementation: three-box sub-Coulomb task

The existing three-box experiment is a **simulation-level** use of this protocol. Its current B=1/B=5 artifact records repeated (`R=20`) expected-outcome statistics, a calibration-only selection condition, and a frozen held-out grid covering force sweeps plus the declared ranges \(\mu\in\{0.3,0.5,0.7\}\), mass \(\in\{0.5,1.0\}\) kg, and `dt` \(\in\{0.25,0.5,1.0\}\) ms. It also measures contact duty, normal-force stability, relative slip, \(\rho\), macroscopic slip onset, and tail creep. This is sufficient for a scoped parameter-robustness conclusion; it is not a full setup-variant study because geometry (for example footprint or push height) was not varied, and it remains unrelated to a synchronized real trajectory. See [the B=5 artifact](../../exp-results/protocol-v2-native-default/b5-static-friction-v2/mujoco/static_friction_v2_summary.json) and [the historical ramp report](../mujoco-vs-sap-frictional-three-box-ramp.md).

