# MIT Push — task-level metrics

MIT Push is not an atomic free-sliding task. The planar object simultaneously receives a pusher wrench and table-contact wrench, and an off-centre push can rotate it:

\[
m\mathbf a=\mathbf F_{\rm pusher}+\mathbf F_{\rm table},
\qquad
I_z\alpha=\mathbf r_{\rm contact}\times\mathbf F_{\rm pusher}+\tau_{\rm table}.
\]

Therefore the pusher/wrist force must **not** be compared directly with a scalar object deceleration through \(F=ma\). The shared dynamic-friction metrics remain useful, but the following two additions evaluate the coupled force--motion response.

## Task-level metrics

| Metric | Definition | Interpretation |
|---|---|---|
| Final object-pose error | Final simulation--real error in planar pose \((x,y,\theta)\). Report translation (mm) and orientation (degrees/radians) separately, rather than hiding unlike units in one unweighted scalar. | Did the pusher place the object at the correct final position and orientation? |
| Force--motion response error | In each preregistered stable-contact window, report both \(E_{\rm wrench}=\left\|\overline{\mathbf w}_{\rm sim}-\overline{\mathbf w}_{\rm real}\right\|\) and \(E_{\rm twist}=\left\|\overline{\boldsymbol\xi}_{\rm sim}-\overline{\boldsymbol\xi}_{\rm real}\right\|\), where \(\mathbf w=(F_x,F_y,\tau_z)\) and \(\boldsymbol\xi=(v_x,v_y,\omega)\). Do not collapse them into one opaque scalar. | Separates contact-wrench fidelity from resulting object motion. |

Use 3--5 preregistered windows of 50--100 ms, such as: first sustained contact, early steady push, mid-push, late push, and optionally post-release. Average within each window rather than comparing individual samples.

Interpret them jointly:

- wrench agreement but twist disagreement suggests table friction, limit-surface, or contact-mode modeling error;
- twist agreement but wrench disagreement can indicate a coincidentally correct trajectory with untrustworthy contact forces;
- disagreement in both points to pusher--object contact or overall dynamics mismatch.

## Dataset synchronization

The MIT dataset stores timestamped pusher pose, object pose, and force/torque measurements, but explicitly notes that those streams are not synchronized and recommends resampling before comparison. Its documented force/torque stream is the wrench **on the pusher**; simulation values must be transformed to the same coordinate frame and sign convention before computing an error. See the [dataset documentation](https://web.mit.edu/mcube//push-dataset/).

## Repeated-rollout rule

For each selected physical push, run \(M\) rollouts under a declared initial-pose/contact/seed perturbation distribution. A later task protocol may define a binary expected outcome (for example, reaches a target pose region versus not), but its pose tolerance must be preregistered. Until then, report the continuous task metrics above and their across-rollout mean, spread, and confidence interval—do not invent a universal pass threshold.
