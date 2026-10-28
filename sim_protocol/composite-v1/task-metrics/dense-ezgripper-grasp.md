# DENSE EZGripper grasp — composite task metrics

This task compares matched SAP/Warp and MuJoCo reproductions of the SAKE Robotics EZGripper: the same finger geometry, tendon-driven underactuation, object, gravity, and command profile. It evaluates whether the simulator predicts **retention versus slip** and the resulting grasped-object motion.

Use synchronized DENSE physical trials when available. Register the object geometry/mass, surface/material condition, initial pose, gripper command, external load or motion profile, sampling rate, and support/world frame before tuning. Split physical trials into calibration and held-out sets; do not fit a native engine configuration to each rollout.

## 1. Physical fidelity

| Metric | Definition | Interpretation |
|---|---|---|
| **Slip / no-slip prediction** | Compare the simulated and measured binary outcome under the same trial condition. Report the confusion matrix and balanced accuracy across held-out trials. A slip event follows the preregistered relative-motion/contact-loss rule. | Does the simulator predict whether the grasp retains the object? |
| **Object pose drift** | Support-frame object-pose error over the held interval, reported as translational drift and orientation drift; also report final-pose error. | For a retained grasp, does it predict the accumulated motion rather than only the final binary outcome? |

Do not use simulator-internal contact forces as a real-world fidelity metric unless matched force measurements are available.

## 2. Simulation consistency

Report these diagnostics for every trial. They explain a result; they are not interchangeable accuracy scores.

| Metric | Definition | Interpretation |
|---|---|---|
| **Friction utilization / grasp margin** | Aggregate tangential and normal contact resultants over the two fingers: \(\rho=\|\sum F_t\|/(\mu\sum F_n)\). Define margin \(m=1-\rho\) only while normal contact exists. | \(\rho\approx1\) indicates a saturated frictional hold/sliding boundary; \(m>0\) indicates reserve. |
| **Force and torque balance residual** | During a preregistered quasi-static hold window, compute the norm of the residual external wrench after including gravity, contact resultants, and measured/applied load; normalize by \(mg\) and \(mgL\). | Detects whether the reported contact forces support the object consistently. This is a diagnostic, not ground truth. |
| **Peak geometric penetration** | Maximum signed overlap between the object and each finger collision geometry, normalized by a preregistered geometric tolerance. | Physical-validity gate. Flag penetration-assisted retention; do not rank engines by raw overlap alone. |
| **Contact duty / switching** | Per finger, report the fraction of samples with normal contact and the number/times of make–break transitions within the hold window. | Detects intermittent support or contact chatter hidden by an average force. |

Use a common contact aggregation rule per engine (per-finger resultant, not raw contact-point count).

## 3. Mechanism-aware usability

Apply the shared \(B=1\), \(B\leq5\), and \(B\approx100\) calibration stages in the [composite protocol](../README.md). The native settings selected on calibration trials are frozen for all held-out trials.

| Check | Definition |
|---|---|
| **Setup-variant robustness** | With frozen native settings, repeat over held-out object mass/shape, surface/material condition, initial object pose, and applied-load or motion profiles. Report slip/no-slip prediction and pose drift by variant. |
| **Grasp-stability boundary** | Map the retained/slip boundary over physical task controls: commanded tendon/grip force versus object weight and external load. Compare the simulated boundary with held-out physical trials when available; otherwise report it only as simulation-level controllability. |
| **Bounded mechanism-aware tuning** | State which native controls were varied, the physical mechanism motivating each candidate, \(\mathrm{Pass}@B\), and \(B_{\rm first\ pass}\). Contrast with same-budget blind search if run. |

The stability boundary changes **environment/task controls**, not the frozen material parameters or native solver settings. It asks whether one calibrated simulator can generate the expected transition between stable and unstable grasps.
