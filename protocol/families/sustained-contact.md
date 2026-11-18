# Sustained-contact v1 — shared evaluation protocol

New comparison runs follow the [Unified B=5 SOP](../UNIFIED-B5-SOP.md).

This folder defines the reusable **v1 comparison protocol** for sustained normal contact, multi-contact support, and static equilibrium. 

Unlike impulsive impact or macroscopic sliding friction, this protocol evaluates the **post-settling steady-state horizon** ($t \ge t_{\rm settle}$), where bodies are expected to remain in unyielding, unchattering contact under gravity and normal preloads.

---

## Scope and coordinate convention

The target regime is the **sustained normal equilibrium phase**:

\[
\lVert\mathbf{v}\rVert \le \epsilon_v, \qquad t \in [t_{\rm settle}, T_{\rm end}], \qquad
\sum_i \mathbf F_i\!\cdot\!\mathbf n_i \approx -\bigl(m\mathbf g+\mathbf F_{\rm ext}\bigr)\!\cdot\!\mathbf n .
\]

Every displacement, contact force, and penetration depth below is evaluated in the **support frame** (e.g., table or ground plane).

### Time-Window Slicing Rule
For trajectories originating from a drop, throw, or jump (such as Cube Toss or Cassie Landing), **all transient impact and collision phases ($t < t_{\rm settle}$) are strictly excluded** from this protocol (they belong to `dynamic-impact-v1`). Evaluation begins at the first instant at which both $\lVert\mathbf{v}\rVert < 0.01\,\text{m/s}$ and $\lVert\boldsymbol{\omega}\rVert < 0.05\,\text{rad/s}$ hold continuously for 0.2 s. The sustained window must then contain at least 1.0 s of data. If no such window exists before $T_{\rm end}$, record **not settled** as a task failure rather than silently excluding the run.

---

## 1. Real-world accuracy

Use this section only when synchronized real-world static ground truth (e.g., optical mocap, laser displacement sensors, or calibrated multi-axis load cells) is available.

| Metric | Definition |
|---|---|
| Steady-state position/height error | Absolute error in final resting vertical height and planar position: \(\lvert z_{\rm sim}^{\rm steady} - z_{\rm real}^{\rm steady}\rvert\) and \(\lVert\mathbf{p}_{xy,\rm sim}^{\rm steady} - \mathbf{p}_{xy,\rm real}^{\rm steady}\rVert\). |
| Static normal force error | \(\lvert\sum F_{n,\rm sim}^{\rm steady} - \sum F_{n,\rm real}^{\rm steady}\rvert\), comparing aggregated support reactions against physical force-plate/load-cell readings. |

---

## 2. Simulation-level consistency

These quantities evaluate whether the numerical contact formulation enforces rigid non-penetration, static force equilibrium, and discrete contact persistence without requiring external mocap.

| Metric | Definition | Ideal sustained interpretation |
|---|---|---|
| **Equilibrium overlap (\(\delta_{\rm eq}\))** | For each declared support interface, use its signed normal geometric separation \(g_i(t)\), and report mean plus maximum overlap: <br><br> \(\delta_{\rm eq}=\max_t\max_i\max(0,-g_i(t))\). | Diagnoses solver softness. `-z` is valid only for a horizontal plane; for arbitrary geometry, define and save the same interface/witness convention for both engines. |
| **Normal Load Balance Error (\(\Delta F_n\))** | Relative normal-force closure error: <br><br> \(\Delta F_n=\frac{\left|\sum_i\overline{\mathbf F_i\!\cdot\!\mathbf n_i}+\overline{(m\mathbf g+\mathbf F_{\rm ext})\!\cdot\!\mathbf n}\right|}{\left|\overline{(m\mathbf g+\mathbf F_{\rm ext})\!\cdot\!\mathbf n}\right|}\). | 0. Handles gravity, prescribed normal preload, inclined support, and multi-layer stacks. For a three-plate stack, report each interface separately: expected loads are \(mg\), \(2mg\), and \(3mg\), top to bottom. |
| **Contact Duty (\(D\)) & Normal Force CV** | 1. Contact persistence duty: \(D = \operatorname{mean}\mathbf{1}[C(t) > 0]\). <br> 2. Normal force coefficient of variation: \(\mathrm{CV}(F_n) = \frac{\sigma(F_n)}{\mu(F_n)}\) over the steady window. | \(D = 1.0\), \(\mathrm{CV}(F_n) \to 0\). Detects discrete contact chattering, micro-bouncing, and numerical jitter under static rest. |

---

## 3. Mechanism-aware usability

These are experiment-design metrics evaluating calibration efficiency and numerical health.

| Metric | Definition |
|---|---|
| Bounded tuning budget | For each engine's native contact/solver parameters (stiffness, constraint regularization, solver tolerance), report whether it satisfies a **task-declared**, scale-normalized overlap threshold plus \(\Delta F_n < 1\%\) within budget \(B\): `Pass@B` and \(B_{\rm first\ pass}\). Use a calibration condition only to choose a setting; do not include that exact condition in the final evaluation score. |
| Plausible-range controllability | Freeze chosen engine settings, then vary stack layers, mass ratios (e.g., 10:1 to 100:1), and external normal preloads. Verify whether the engine maintains non-penetration without exploding. |
| Numerical local robustness | With physics and tuning frozen, run \(R=20\) independent initial-state perturbations and a predeclared \(\Delta t\) grid. Integrator type and solver iteration limits may be declared ablations. Report distributions—not just one trajectory—of \(\delta_{\rm eq}\), \(D\), force CV, and task outcome. |
| Setup-variant robustness | Repeat across different geometric support primitives (e.g., flat plate vs. multi-point sphere support). Report whether non-penetration and equilibrium conclusions transfer. |

---

## Task-level extensions

Tasks in this family report the shared metrics above first, then add only task-specific requirements (such as tangential drift or multi-support internal forces).

| Task | Additional task-level metrics | Extension |
|---|---|---|
| Cube toss (sustained phase) | Residual creep / drift rate across the final resting window. | [cube-toss.md](../../tasks/real-data-simple/cube-toss-tail/task-metrics.md) |
| Three-plate stacking | Stack order preservation, center-of-mass height deflection, and thickness sweep critical breakdown. | [three-plate-stack.md](../../tasks/simulation-only-atomic/three-plate-stack/task-metrics.md) |

---

## Reporting requirements

For every sustained-contact experiment, record:

1. geometry, mass, gravity, contact configuration (actual support interface/manifold), declared sustained window $[t_{\rm settle}, T_{\rm end}]$, timestep, duration, and the explicit **not-settled** rule;
2. the frozen engine contact parameters and the solver convergence tolerance ($\epsilon_{\rm abs}$, max iterations);
3. the signed-gap/witness definition used for each contact interface, including whether it is computed from analytic primitives, collision pieces, or mesh vertices;
4. whether a physical force sensor/mocap reference exists; if not, report as simulation-level consistency; and
5. explicit verification that transient impact frames ($t < t_{\rm settle}$) were excluded before computing sustained metrics; and
6. the calibration condition, held-out evaluation matrix, independent-rollout seed/perturbation distribution, and timestep grid.

