# Simple pinch — task-level metrics

This extension sits on top of the [Static-friction v2 shared protocol](../../../protocol/families/static-friction.md). A simple pinch is not evaluated only by whether the cube happens to remain between the pads: it must establish the physical grip-force threshold and whether the cube follows the prescribed lift without relative slip.

## Definitions

Let \(N_L(t)\) and \(N_R(t)\) be the **measured** inward normal forces at the left and right pad--cube contacts. Use the preload window to report

\[
N=\frac{\overline{N_L}+\overline{N_R}}{2},
\qquad
r=\frac{2\mu N}{mg}.
\]

For a symmetric two-finger pinch under ideal static Coulomb friction, the per-finger threshold is

\[
F_{\rm grip,ideal}^{*}=\frac{mg}{2\mu},
\qquad r=1.
\]

Here, `grip force` means **measured per-finger normal contact force**, not commanded pad displacement or nominal geometric compression. Those commanded settings must still be reported because they are the calibration controls that produce the measured force.

## Task-level metrics

| Metric | Definition | Interpretation |
|---|---|---|
| Minimum hold grip force \(F_{\rm grip,sim}^{*}\) | The smallest measured per-finger preload \(N\) for which the cube meets the preregistered hold/lift criterion throughout the evaluation window. Determine it by a bracketed preload sweep, not a single trial. | Compare the simulated static-grasp threshold with the ideal \(mg/(2\mu)\), or with a measured physical threshold if a real experiment exists. |
| Grip-force error \(E_{\rm grip}\) | \(\displaystyle E_{\rm grip}=\frac{\left|F_{\rm grip,sim}^{*}-F_{\rm grip,ideal}^{*}\right|}{F_{\rm grip,ideal}^{*}}\). If a physical threshold is available, additionally report the same expression with \(F_{\rm grip,real}^{*}\) in place of the ideal value. | 0 means the simulated threshold matches the stated reference. This is a threshold metric, not a score for one arbitrarily selected preload. |
| Lift-height error \(E_z(T)\) | \(\displaystyle E_z(T)=\left|z_{\rm cube}^{\rm sim}(T)-z_{\rm cube}^{\rm ideal}(T)\right|\), where \(z_{\rm cube}^{\rm ideal}\) follows the commanded gripper lift with no pad--cube slip. Report in mm; optionally normalize by commanded lift height \(H\): \(E_z(T)/H\). | Separates an apparently retained cube from a cube that slipped or lagged during lift. |

## Shared metrics to retain

At each pad--cube interface, also log the static-friction v2 common metrics:

- support-frame relative displacement and tangential velocity;
- tail creep rate during a hold;
- friction utilisation \(\rho=\lVert F_t\rVert/(\mu F_n)\);
- contact duty, normal-force stability, gap/overlap fluctuation, and any contact-capacity truncation.

The two sides must be reported separately before any mean is used: asymmetric \(N_L,N_R\), force sharing, or contact loss can make a mean preload misleading.

## Expected-outcome agreement and overall statistics

Each condition must have a preregistered binary physical label:

- **retain**: the cube should remain in the grasp through the specified lift/hold gate;
- **release/slip**: the selected mass, friction, or preload condition is intended to exceed the stated static-grasp capacity and should fail that gate.

For every repeat, use the fixed task gate to classify the simulated outcome. Treat **release/slip** as the positive class. Report a per-condition pass fraction \(\hat p\) with \(M\) repeats, pooled `TP/FP/FN/TN`, and:

\[
\mathrm{FPR}=\frac{FP}{FP+TN},\qquad
\mathrm{FNR}=\frac{FN}{FN+TP},\qquad
\mathrm{balanced\ accuracy}=1-\frac{\mathrm{FPR}+\mathrm{FNR}}{2}.
\]

Here FPR means a cube releases when the condition was expected to retain it; FNR means a cube remains retained when the condition was expected to release. Report 95% Wilson or bootstrap confidence intervals and macro-average across mass/friction/preload groups, rather than allowing a densely sampled boundary region to dominate one pooled score.

## Required protocol fields

For each run, save: cube mass, \(\mu\), pad geometry/material, commanded compression or force-control target, measured \(N_L,N_R\), lift trajectory and duration, evaluation window, expected binary label, pass/fail gate, perturbation/seed, and frozen engine settings. A finite-horizon lift pass must not be described as proof of indefinite static equilibrium.
