# Three-plate stacking — task-level metrics

Three-plate stacking is an atomic multi-body static contact benchmark. Three identical circular plates are initially placed in a perfectly aligned, concentric vertical stack at rest ($t=0, \mathbf{v}=0$), **without any dropping or impact phase**. The nominal reference includes each declared contact-activation/rigid gap \(\gamma_j\); it is not always a zero-gap rigid stack:

\[
z_{1,\rm ref} = \frac{h}{2}+\gamma_0, \quad
z_{2,\rm ref} = \frac{3h}{2}+\gamma_0+\gamma_1, \quad
z_{3,\rm ref} = \frac{5h}{2}+\gamma_0+\gamma_1+\gamma_2,
\qquad \mathbf{v}_i(0) = \mathbf{0}.
\]

This task directly evaluates whether the contact solver maintains non-penetration, static load transmission, and geometric stability as plate thickness $h$ is scaled down into the ultra-thin regime.

---

## Task-level metrics

| Metric | Definition | Physical Interpretation |
|---|---|---|
| **Stack Order Preservation** | Binary check evaluated throughout the steady window: <br><br> \[ \mathbf{1}[z_{\rm top}(t) > z_{\rm mid}(t) > z_{\rm bottom}(t) > 0] \] | **Tunneling and inversion check**. Detects whether a thin plate slips through or tunnels into adjacent plates under gravity. Failure = complete geometric breakdown. |
| **Center-of-Mass Height Bias (\(\Delta z_{\rm com}\))** | For each plate report signed and absolute height bias against the gap-aware reference: <br><br> \[b_i=z_i^{\rm steady}-z_{i,\rm ref},\qquad \Delta z_i=\lvert b_i\rvert.\] <br> Report maximum \(\Delta z_i\) and the signed stack-height bias \(b_{\rm stack}=(z_{\rm top}^{\rm steady}+h/2)-[3h+\gamma_0+\gamma_1+\gamma_2]\). | Separates intentional activation-gap bias from actual geometric overlap. Pair with the shared signed-gap metric; height alone is not penetration. |
| **Stacking Task Success Rate (`Pass@Threshold`)** | Binary success condition: <br><br> 1. stack order is preserved throughout the sustained window; <br> 2. every interface satisfies a predeclared overlap tolerance, e.g. \(\delta_{\rm eq,i}\le0.05h\); <br> 3. the run reaches the registered sustained window. | Task-level pass/fail outcome. This does not treat a positive activation gap as failure. |

---

## Thickness Sweep Protocol

To evaluate solver robustness against thin-geometry breakdown, run a parameter sweep over plate thickness while keeping plate radius $R = 0.1\,\text{m}$ and total mass fixed:

\[
h \in [20\,\text{mm}, \ 10\,\text{mm}, \ 5\,\text{mm}, \ 2\,\text{mm}, \ 1\,\text{mm}, \ 0.5\,\text{mm}].
\]

This is explicitly a **numerical thin-geometry stress test**: fixed mass means density changes with thickness. A material-fidelity variant with constant density is a separate experiment, not pooled with this result.

Record the **critical failure thickness** ($h_{\rm critical}$) at which the stack order is violated, the overlap threshold is exceeded, or the run fails to settle. Select native parameters only on one declared calibration thickness/load; freeze them, then evaluate the remaining thicknesses with a predeclared timestep grid and \(R=20\) independent perturbation rollouts per condition.
