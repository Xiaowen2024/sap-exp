# Cube toss (sustained phase) — task-level metrics

This extension evaluates the **steady-state resting phase** of the Cube Toss task after all tumbling, rolling, and impact bounces have fully dissipated:

\[
t \ge t_{\rm settle}, \qquad \lVert\mathbf{v}\rVert < 0.01\,\text{m/s}, \qquad \lVert\boldsymbol{\omega}\rVert < 0.05\,\text{rad/s}.
\]

The shared metrics—**Equilibrium overlap (\(\delta_{\rm eq}\))**, **Normal Load Balance Error (\(\Delta F_n\))**, and **Contact Duty (\(D\))**—are aggregated over the actual support contact manifold. A tossed cube can end face-down, edge-down, or corner-down; the benchmark must not assume four supporting face corners unless face-down rest is explicitly imposed as part of the task setup.

This task adds a single task-level metric to evaluate tangential numerical creep and drift under continuous static gravity.

---

## Task-level metric

| Metric | Definition | Interpretation |
|---|---|---|
| **Residual Creep / Drift Rate** | Over the declared steady-state window \([t_{\rm settle}, T_{\rm end}]\), compute: <br><br> 1. Planar translational drift rate: \(v_{\rm drift} = \frac{\lVert\mathbf{p}_{xy}(T_{\rm end}) - \mathbf{p}_{xy}(t_{\rm settle})\rVert}{T_{\rm end} - t_{\rm settle}}\) <br> 2. Angular yaw drift rate: \(\omega_{\rm drift} = \frac{\lvert\theta_{\rm yaw}(T_{\rm end}) - \theta_{\rm yaw}(t_{\rm settle})\rvert}{T_{\rm end} - t_{\rm settle}}\) | Evaluates whether the contact solver induces artificial micro-sliding or rotational drift under pure vertical gravity without external tangential excitation. Ideal physics = 0. |

---

## Evaluation rule

1. **Window Identification**: The sustained window begins only when both linear velocity remains below \(0.01\,\text{m/s}\) and angular velocity below \(0.05\,\text{rad/s}\) for at least \(0.2\,\text{s}\) continuously. It must contain at least 1.0 s; otherwise record **not settled**.
2. **Contact-Manifold Aggregation**: Record the active support collision pieces/contact points, their signed gaps, and their normal forces. Aggregate force balance over that actual manifold; do not infer it from an assumed four-corner geometry.
3. **Scope**: The toss and collision trajectory itself belongs to `dynamic-impact-v1`. This extension begins only after the registered settling time and measures residual static creep/jitter.
