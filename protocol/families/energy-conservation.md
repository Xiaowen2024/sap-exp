# Energy-conservation v1 — shared evaluation protocol

New comparison runs follow the [Unified B=5 SOP](../UNIFIED-B5-SOP.md).

This folder defines the reusable **v1 comparison protocol** for mechanical energy conservation, dissipation fidelity, and momentum preservation across conservative and near-conservative systems.

It evaluates both **continuous Hamiltonian dynamics** (e.g., long-horizon unforced pendulums) and **discrete multi-body collision chains** (e.g., Newton's cradle).

---

## Scope and coordinate convention

The target regime is the **mechanical energy and momentum evolution horizon**:

\[
E(t) = E_{\rm kin}(t) + E_{\rm pot}(t), \qquad \dot{E} \le 0 \quad (\text{under passive dissipative physics}) .
\]

Every generalized coordinate, velocity, and trajectory is evaluated in the **inertial world frame**. 

In ideal conservative setups, energy drift must theoretically be zero ($E(t) \equiv E_0$). In damped setups, energy dissipation must monotonically match physical dissipation laws, strictly prohibiting numerical energy spurts ($\Delta E > 0$ with unphysical acceleration).

---

## 1. Real-world accuracy

Use this section only when synchronized physical reference data (e.g., optical mocap tracking) is available. Report sampling rate, frame registration, and evaluation duration.

| Metric | Definition |
|---|---|
| Trajectory RMSE | Framewise RMSE between simulated and measured position/angle trajectory: \(\sqrt{\frac{1}{T}\sum_{t=1}^T \lVert \mathbf{x}_{\rm sim}(t) - \mathbf{x}_{\rm real}(t)\rVert^2}\). |
| DTW | Dynamic-time-warping distance between simulated and measured trajectories; used alongside RMSE to evaluate waveform similarity while tolerating cumulative phase lag. |

---

## 2. Simulation-level consistency

These quantities evaluate whether the numerical integrator and contact solver preserve fundamental conservation laws without requiring external mocap data.

| Metric | Definition | Ideal conservative interpretation |
|---|---|---|
| **Mechanical Energy Drift / Loss Ratio (\(\Delta E_{\rm loss}\))** | Relative deviation between simulated total mechanical energy and theoretical energy profile: <br><br> \[\Delta E_{\rm loss}(t) = \frac{\lvert E(t) - E_{\rm theoretical}(t)\rvert}{E(0)}\] | 0 in conservative benchmarks. Detects artificial numerical dissipation (solver damping) or energy blow-up (non-symplectic divergence). |

---

## 3. Mechanism-aware usability

These are experiment-design metrics evaluating calibration efficiency and numerical health.

| Metric | Definition |
|---|---|
| Bounded tuning budget | For each engine's native integrator controls and solver tolerances, report whether it satisfies \(\Delta E_{\rm loss} \le \epsilon_{\rm tol}\) across \(N\) cycles within budget \(B\): `Pass@B` and \(B_{\rm first\ pass}\). |
| Plausible-range controllability | Freeze engine tuning, then vary physical conditions over a plausible range—initial energy, amplitude/angle, pendulum length, and sphere mass. Check whether conservation and dynamics scale as predicted. |
| Numerical local robustness | With physics and tuning frozen, sweep integration timestep \(\Delta t \in [0.1\,\text{ms}, 10\,\text{ms}]\). Plot the **Energy Drift vs. \(\Delta t\)** Pareto curve to identify the critical timestep where numerical instability begins. |
| Setup-variant robustness | Repeat across different system scales (e.g., single pendulum vs. double pendulum vs. multi-ball cradle). Report whether conservation conclusions transfer. |

---

## 4. Task-level metric registry

Tasks in this family report the shared metrics above first, then add only task-specific requirements (such as oscillation period or impulse chain metrics).

| Task | Additional task-level metrics | Extension |
|---|---|---|
| Pendulum | Period Duration (PD) evaluated via Fourier spectral analysis. | [pendulum.md](../../tasks/simulation-only-atomic/pendulum-ideal/SOP.md) |
| Newton's cradle | Momentum Transfer Efficiency (MTE) and Longest Stationary Duration (LSD). | Not in this repository yet |

---

## Reporting requirements

For every energy conservation experiment, record:

1. system parameters (mass, length/radius, inertia, gravity $g$, initial state $q_0, \dot{q}_0$, and total simulation horizon $T$);
2. numerical integrator type (Explicit Euler, Semi-implicit Euler, Verlet, Runge-Kutta 4, Implicit);
3. integration timestep $\Delta t$, substeps, and contact solver tolerances;
4. whether friction/damping was enabled; if disabled, declare the experiment an ideal conservative benchmark; and
5. whether a real-world mocap reference exists; if not, evaluate against closed-form analytical Hamiltonians.

