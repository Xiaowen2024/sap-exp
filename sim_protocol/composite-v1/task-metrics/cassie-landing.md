# Cassie landing — composite task metrics

Cassie landing evaluates whether contact dynamics remain predictive when impact, articulated closed linkages, compliant leg elements, floating-base motion, and robot actuation are coupled.

Cassie is a **composite robot-level task**, not a pure impact ranking. Its impact window isolates the first-contact transient; its post-impact window evaluates whether the same simulator continues to predict the observed recovery.

## Conditions

### A. Real-data replay

Initialize each simulator from the mapped measured state immediately before the first touchdown and replay the measured motor-torque trace. State registration, ground-plane alignment, generalized-coordinate mapping, and fixed evaluation windows must be predeclared and identical across engines.

### B. Closed-loop stress test

If run, use the same controller architecture, command interface, update rate, and torque limits in every engine. This measures simulated task robustness and must not be reported as real-world fidelity without paired physical trials.

## 1. Real-world fidelity

Let \(\tilde x_t=x_t^{\rm sim}-x_t^{\rm real}\). Use the Acosta et al. weighted state error

\[
\bar e_{\rm Cassie}(\mathcal W)
=\frac{1}{|\mathcal W|}
\sum_{t\in\mathcal W}
\tilde x_t^\top W\tilde x_t .
\]

Use the same state weights as Acosta et al.: all position states have weight 10; floating-base linear and angular velocity have weights 100 and 5; hip-roll, knee-spring, and toe velocities have weight 0.01; other measured joint velocities have weight 1. Unmeasured ankle-spring velocity is excluded.

| Metric | Window | Interpretation |
|---|---|---|
| **Impact weighted state error** | \(\mathcal W_{\rm impact}\): fixed 50 ms window around first touchdown | Predictive fidelity of the first impact transient. |
| **Recovery weighted state error** | \(\mathcal W_{\rm recovery}\): touchdown + 50 ms to a preregistered recovery horizon | Whether impact error grows during observed landing recovery. |
| **State-error breakdown** | The two windows above | Diagnostic decomposition into (i) pelvis pose, (ii) pelvis twist, and (iii) aggregate load-bearing leg state; not an additional ranking score. |

Do not use DTW as a primary Cassie score: it can obscure an incorrect touchdown phase. Foot-event timing may be reported only if a common, reliable event label is available from the physical record.

## 2. Simulation consistency

These are internal physical-validity diagnostics. They do not claim direct real-world accuracy without independent measurements.

| Metric | Definition | Use |
|---|---|---|
| **Maximum loop-closure residual** \(r_{\rm loop}\) | \(\max_{\ell\in\{\rm Achilles,Plant\},\,t\in\mathcal W_{\rm impact}}\|p_{\ell,A}(t)-p_{\ell,B}(t)\|_2/L_\ell\), where \(L_\ell\) is the loop characteristic length. | Detects impact-time loss of a modeled four-bar closure. |
| **Maximum normalized foot penetration** \(\hat\delta_{\max}\) | \(\delta_{\max}/\delta_{\rm allowed}\), where \(\delta_{\rm allowed}\) is a preregistered foot-pad compression or collision-geometry tolerance. | Physical-validity gate; \(\hat\delta_{\max}>1\) is labelled penetration-assisted, not automatically less accurate. |
| **Tail internal-shear ratio** \(\eta_{\rm internal}\) | For each foot resultant \(F_i\), let \(F_{i,\parallel}=F_i-(F_i\cdot n)n\). Then \(F_{\rm internal}=\frac12(\sum_i\|F_{i,\parallel}\|-\|\sum_iF_{i,\parallel}\|)\), and \(\eta_{\rm internal}=\overline{F_{\rm internal}}/(mg)\) over a steady double-support tail window. | Detects excess cancelling tangential ground reactions / solver-induced self-stress. Use only with both feet in contact, no horizontal external load, and near-zero COM horizontal acceleration. |

Raw peak contact force and \(dF/dt\) are saved as diagnostic traces only; they are timestep- and regularization-dependent and are not v1 cross-engine scores.

## 3. Mechanism-aware usability

The native settings selected on calibration landings are frozen before all held-out evaluation and sweeps.

| Check | Definition |
|---|---|
| **Bounded tuning budget** | Compare default \(B=1\), up to \(B\leq5\) mechanism-motivated native candidates, and—if run—a disclosed \(B\approx100\) blind-grid ceiling. Report \(\mathrm{Pass}@B\) and \(B_{\rm first\ pass}\) using the preregistered impact/recovery criterion. |
| **Plausible-range controllability** | With native settings frozen, vary physically plausible landing height/vertical speed, body mass, and initial orientation. Check for the expected qualitative transition from stable recovery to failure, without penetration-assisted support or excessive loop residual. |
| **Numerical local robustness** | At each frozen condition, perturb timestep, initial state, and seed under a preregistered distribution. Report mean/SD across \(R\geq20\) rollouts. |
| **Setup-variant robustness** | With native settings frozen, repeat on held-out landing profiles or meaningful environment variants, then report whether the real-data and consistency conclusions transfer. |

## Notes on interpretation

- A low state error does not imply a low loop residual or internal shear.
- A zero closure residual can arise from reduced-coordinate enforcement; it measures model-plus-constraint treatment, not necessarily real structural deformation.
- Cassie has actuation, compliant components, and model mismatch. Do not use restitution, contact duration, impulse closure, or energy conservation as primary Cassie metrics.
