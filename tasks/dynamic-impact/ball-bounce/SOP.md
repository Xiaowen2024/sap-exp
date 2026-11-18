# Bouncing ball — task-level metrics

Bouncing ball is the atomic single-point normal impact benchmark for the dynamic-impact v1 protocol. It directly tests impulsive normal contact and continuous energy dissipation under gravity:

\[
m\ddot{z} = -mg + F_n(t), \qquad v_n^+ = -e v_n^-.
\]

In addition to the shared evaluation metrics (Trajectory RMSE, DTW, effective restitution \(e\), and contact duration \(t_c\)), this task adds a single phase-shift invariant metric to rigorously assess the discrete energy decay sequence.

---

## Task-level metric

| Metric | Definition | Interpretation |
|---|---|---|
| **Apex Height Decay Error** | For the \(k\)-th bounce, let \(h_k = \max_{t \in [t_k, t_{k+1}]} z(t)\). Report both the per-bounce absolute error \(\Delta h_k = \lvert h_{k,\rm sim} - h_{k,\rm real}\rvert\) and the normalized decay error: <br><br> \[ E_{\rm apex} = \frac{1}{K}\sum_{k=1}^K \frac{\lvert h_{k,\rm sim} - h_{k,\rm real}\rvert}{h_{k,\rm real}} \] | Directly measures the energy dissipation sequence without suffering from time-alignment phase shifts. Because \(E_k = mgh_k\), this directly isolates the normal impulse restitution across consecutive cycles. |

---

## Peak extraction and evaluation rule

1. **Window Selection**: Report the apex heights for the first \(K = 3 \sim 5\) primary bounces. Do not extend into sub-millimeter chattering regime where sensor noise dominates.
2. **Noise-Free Peak Detection**: Identify \(h_k\) via local maxima detection on raw position data \(z(t)\). Do **not** infer apex points from zero-crossings of numerical velocity derivatives, which amplify discretization noise.
3. **Analytical Verification Mode**: When physical mocap data is unavailable, use the closed-form restitution solution as the ground truth:
   \[
   h_k^{\rm ref} = e^{2k} h_0.
   \]
   This verifies whether the engine's integration scheme and contact regularizer preserve theoretical energy conservation.

