# Shared protocol: static friction

## Scope

Each rollout is configured so that the intended contact regime is static:
\[
\|F_t\| < \mu F_n.
\]
The object should remain at rest relative to its support over the evaluation
window. All relative quantities use the support frame.

Mass, geometry, external load, and friction coefficient are **environment
parameters** fixed per rollout and shared across engines. Engine tuning may
only change native solver/contact parameters.

## Common metrics

### Real-world accuracy

| Metric | Definition |
| --- | --- |
| Trajectory RMSE | Frame-aligned error between simulated and real support-relative trajectory. |
| DTW | Time-warped support-relative trajectory error. |
| Relative-displacement error | Error between \(x_{\rm rel}^{sim}(t)\) and \(x_{\rm rel}^{real}(t)\). |
| Relative tangential-velocity error | Error between \(v_t^{sim}(t)\) and \(v_t^{real}(t)\). |

### Simulation-level consistency

| Metric | Definition |
| --- | --- |
| Tail relative displacement | Accumulated \(x_{\rm rel}\) in the final hold window. |
| Tail tangential velocity / creep rate | Tail mean or RMS \(v_t\), or the final-window slope of \(x_{\rm rel}\). |
| Friction utilisation | \(\rho=\|\sum F_t\|/(\mu\sum F_n)\). Static support requires \(0<\rho<1\). |
| Contact duty | \(D=\operatorname{mean}\mathbf1[C(t)>0]\). Continuous support requires \(D\) near one. |

### Mechanism-aware usability

| Evaluation | Definition |
| --- | --- |
| Bounded tuning budget | Report \(\mathrm{Pass@}B\), \(B_{\rm first\ pass}\), and final loss under the same small budget as brute-force search. |
| Plausible-range controllability | Under physically plausible environment variations, the engine should produce the expected stick or non-stick outcome without pathological native settings. |
| Numerical local robustness | Hold the physical setup fixed and perturb timestep, initial pose, or seed. The sticking conclusion should remain stable. |
| Setup-variant robustness | Evaluate meaningful changes in mass, load ratio, geometry, and support motion. The common metrics should remain predictive across the task family. |
