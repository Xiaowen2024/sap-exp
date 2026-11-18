# Pendulum — task-level metrics

The pendulum benchmark evaluates continuous oscillation dynamics, frequency stability, and integration timing following the GAUGE (Wang et al., 2026) benchmark protocol.

In addition to shared mechanical energy drift, this task adopts the exact observable defined in GAUGE to assess oscillation timing and non-linear period fidelity.

---

## Task-level metric (GAUGE Official)

| Metric | Definition | Physical Interpretation |
|---|---|---|
| **Period Duration (PD)** | Evaluated via **Fourier Spectral Analysis**: <br><br> Apply Fast Fourier Transform (FFT) to the simulated trajectory (position or angular displacement time-series) to extract the dominant frequency peak \(f_{\rm dominant}\). The estimated period duration is: <br><br> \[ T_{\rm sim} = \frac{1}{f_{\rm dominant}} \] <br> Report the absolute period error \(\Delta T = \lvert T_{\rm sim} - T_{\rm ref}\rvert\) or normalized ratio \(\frac{T_{\rm sim}}{T_{\rm ref}}\), where \(T_{\rm ref}\) is the measured real-world period or theoretical non-linear period. | Measures whether the simulated integration scheme correctly captures oscillation frequency, timing stability, and non-linear Hamiltonian period under continuous motion without relying on small-angle linear approximations. |

---

## Experimental Setup (GAUGE Reference)

* **Initial Condition**: Release from a demanding large-amplitude state (e.g., horizontal release \(\theta_0 = 90^\circ\)) rather than near-zero small angles, to fully expose non-linear inertial acceleration and integrator timing errors.
* **Evaluation Horizon**: Trajectory evaluated over multiple full back-and-forth oscillation cycles with spectral frequency estimation.
