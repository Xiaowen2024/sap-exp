# Cube toss — task-level metrics

Cube toss evaluates 3D multi-point frictional impact with tumbling dynamics, where corner and edge collisions induce continuous transitions in contact modes and angular momentum:

\[
m\ddot{\mathbf{p}} = -mg\mathbf{k} + \sum_i \mathbf{F}_{i}, \qquad \mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega}\times(\mathbf{I}\boldsymbol{\omega}) = \sum_i (\mathbf{r}_i \times \mathbf{F}_i).
\]

In addition to the shared evaluation protocol, this task defines the following task-specific metrics to evaluate 3D rotation, energy dissipation, and final resting state.

---

## Task-level metrics

| Metric | Definition | Physical Interpretation |
|---|---|---|
| **Final Pose Error** | Error in the final resting pose after motion has fully settled: <br> 1. Position error: \(\lVert\mathbf{p}_{\rm sim}(T) - \mathbf{p}_{\rm real}(T)\rVert\) (optionally normalized by cube width \(l\)). <br> 2. Orientation error: \(\Delta\theta_{\rm rest} = \operatorname{Angle}(\mathbf{R}_{\rm sim}(T), \mathbf{R}_{\rm real}(T))\). <br> 3. Resting face agreement: whether the upward face ID matches ground truth. | Captures the macro outcome of 3D impact and tumbling. Avoids time-averaging dilution from the airborne flight phase. |
| **Speed Decay After Major Bounces** | Evaluates the kinetic energy and velocity drop immediately after the first \(K\) major corner/edge impacts: <br> 1. Translational speed drop: \(\Delta v^+_k = \lvert \lVert\mathbf{v}^+_{\rm sim,k}\rVert - \lVert\mathbf{v}^+_{\rm real,k}\rVert\rvert\) <br> 2. Angular speed drop: \(\Delta \omega^+_k = \lvert \lVert\boldsymbol{\omega}^+_{\rm sim,k}\rVert - \lVert\boldsymbol{\omega}^+_{\rm real,k}\rVert\rvert\) | Isolates normal restitution and moment arm impulse transfer during edge/corner impacts, preventing errors where the simulated cube glides or spins unphysically. |

---

## Evaluation rule

1. **Airborne vs. Contact Separation**: Trajectory evaluation begins at the first detected contact (\(t_0 = t_{\rm touch}\)) to avoid weighting the unconstrained parabolic flight.
2. **Resting Classification**: The cube is declared at rest when \(\lVert\mathbf{v}\rVert < 0.01\,\text{m/s}\) and \(\lVert\boldsymbol{\omega}\rVert < 0.05\,\text{rad/s}\) for at least 0.2 s.
