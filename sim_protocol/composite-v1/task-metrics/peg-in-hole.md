# Peg-in-hole — composite task metrics

Peg-in-hole evaluates narrow-clearance insertion where contact geometry, manifold generation, friction, compliance, and actuation are coupled. It is a simulation-only composite task in v1.

## Conditions

Use a simple analytic cavity as a control and the intended concave/socket representation as the geometry stress case. Freeze clearance, peg and hole geometry, mass/inertia, friction, insertion trajectory, force limits, timestep, and collision representation within each declared comparison.

## 1. Simulation consistency

| Metric | Definition | Use |
|---|---|---|
| **Physically valid insertion rate** | Fraction of rollouts that enter the mouth, reach the required insertion depth and final radial/axial pose, remain retained, and do not require invalid penetration. | Primary task outcome. |
| **Maximum penetration-to-clearance ratio** | \(\max_t \delta(t)/c\), where \(c\) is the radial clearance. | Physical-validity gate; a visual insertion that crosses an impossible barrier through overlap is not a valid success. |
| **Jam / escape outcome** | For preregistered aligned, offset, and overconstrained cases, record whether the result is insertion, self-centering, jam, or escape. | Tests whether contact geometry produces the expected qualitative mode. |

## 2. Mechanism-aware usability

Use the shared calibration-budget stages in the composite README. Evaluate transfer across clearance, insertion speed, offset, contact stiffness/compliance regime, and timestep only after engine settings are frozen.

## Notes

Report success and geometry validity together. Neither final penetration alone nor a visually successful final pose is sufficient evidence of physically valid insertion.
