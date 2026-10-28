# Peg-in-hole — composite task metrics

Peg-in-hole evaluates narrow-clearance insertion where contact geometry, manifold generation, friction, compliance, and actuation are coupled. It is a simulation-only composite task in v1.

## Conditions

Use a simple analytic cavity as a control and the intended concave/socket representation as the geometry stress case. Freeze clearance, peg and hole geometry, mass/inertia, friction, insertion trajectory, force limits, timestep, and collision representation within each declared comparison.

## 1. Simulation consistency

| Metric | Definition | Use |
|---|---|---|
| **Physically valid insertion rate** | Fraction of rollouts that enter the mouth, reach the required insertion depth and final radial/axial pose, remain retained, and do not require invalid penetration. | Primary task outcome. |
| **Maximum penetration-to-clearance ratio** | \(\max_t \delta(t)/c\), where \(c\) is radial clearance. | Physical-validity gate; a visual insertion that crosses an impossible barrier through overlap is not a valid success. |
| **Jam / escape outcome** | For preregistered aligned, offset, and overconstrained cases, record insertion, self-centering, jam, or escape. | Tests whether contact geometry produces the expected qualitative mode. |

## 2. Mechanism-aware usability

The native settings selected on calibration cases are frozen before all sweeps.

| Check | Definition |
|---|---|
| **Bounded tuning budget** | Compare default \(B=1\), up to \(B\leq5\) mechanism-motivated native candidates, and—if run—a disclosed \(B\approx100\) blind-grid ceiling. Report \(\mathrm{Pass}@B\) and \(B_{\rm first\ pass}\) for physically valid insertion. |
| **Plausible-range controllability** | With native settings frozen, sweep clearance, initial lateral/angle offset, insertion speed, and commanded axial force through preregistered plausible ranges. Check whether expected insertion, self-centering, jam, and escape transitions occur. |
| **Numerical local robustness** | At each frozen case, perturb timestep, initial pose, and seed under a preregistered distribution. Report mean/SD across \(R\geq20\) rollouts. |
| **Setup-variant robustness** | With native settings frozen, transfer to a held-out geometry or clearance regime and report whether the insertion and validity conclusion transfers. |

## Notes

Report success and geometry validity together. Neither final penetration alone nor a visually successful final pose is sufficient evidence of physically valid insertion.
