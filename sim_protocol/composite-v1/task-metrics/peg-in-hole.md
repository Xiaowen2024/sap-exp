# Peg-in-hole — composite task metrics

Peg-in-hole evaluates narrow-clearance insertion where contact geometry, manifold generation, friction, compliance, and actuation are coupled. It is a simulation-only composite task in v1.

## Conditions

Use a simple analytic cavity as a control and the intended concave/socket representation as the geometry stress case. Freeze clearance, peg and hole geometry, mass/inertia, friction, insertion trajectory, force limits, timestep, and collision representation within each declared comparison.

## 1. Simulation consistency

For rollout \(i\), define **task success** \(s_i\in\{0,1\}\) from the preregistered completion condition: required insertion depth and final radial/axial pose, followed by stable retention in the tail window. It is a per-rollout outcome.

| Metric | Definition | Use |
|---|---|---|
| **Task success** | Per rollout, \(s_i=1\) if the registered completion condition is met; otherwise \(s_i=0\). Across \(N\) rollouts report task-success rate \(\frac{1}{N}\sum_i s_i\), together with the numerator and denominator. | Primary task outcome. |
| **Peak geometric penetration** | \(\delta_{\max}=\max_t\delta(t)\), reported in physical units. | Shows the absolute collision overlap. |
| **Penetration-to-clearance ratio** | \(\delta_{\max}/c\), where \(c\) is radial clearance. | Puts the same overlap in context: a fixed \(\delta_{\max}\) can be negligible for a loose fit and invalid for a tight fit. |
| **Final contact mode** | Per rollout, label **inserted**, **self-centered but unseated**, **jammed**, or **escaped** using preregistered geometric rules. | Explains failure modes rather than collapsing every non-success into one label. |

Report task success and both penetration measures side by side. A successful-looking final pose with excessive penetration is labelled **penetration-assisted**; this is a validity warning, not a replacement success criterion.

### Contact / force diagnostics

Save the peg contact-resultant and, if available, wrist wrench traces. Inspect unexpected contact loss/switching, discontinuous spikes, and sign/direction inconsistent with the prescribed insertion phase. Peak force, force variation, or a force trace may be reported as **within-task diagnostics**, but are not v1 cross-engine accuracy scores: without matched physical wrench data, their magnitude depends on each engine's contact regularization, aggregation, and timestep.

## 2. Mechanism-aware usability

The native settings selected on calibration cases are frozen before all sweeps.

| Check | Definition |
|---|---|
| **Bounded tuning budget** | Compare default \(B=1\), up to \(B\leq5\) mechanism-motivated native candidates, and—if run—a disclosed \(B\approx100\) blind-grid ceiling. Report \(\mathrm{Pass}@B\) and \(B_{\rm first\ pass}\) for the preregistered task-success condition. |
| **Plausible-range controllability** | With native settings frozen, sweep clearance, initial lateral/angle offset, insertion speed, and commanded axial force through preregistered plausible ranges. Check whether expected insertion, self-centering, jam, and escape transitions occur. |
| **Numerical local robustness** | At each frozen case, perturb timestep, initial pose, and seed under a preregistered distribution. Report mean/SD across \(R\geq20\) rollouts. |
| **Setup-variant robustness** | With native settings frozen, transfer to a held-out geometry or clearance regime and report whether the task-success, penetration, and contact-mode conclusion transfers. |
