# Slope Slider — no additional task-level metric

Slope Slider is the atomic real-world validation task for the dynamic-friction v2 protocol. It introduces **no additional task-specific scalar metric**.

Use all shared layers directly:

- real-world trajectory RMSE, DTW, fitted tangential acceleration/deceleration error, and terminal displacement error;
- analytical acceleration/displacement consistency, \(\rho\), contact duty, and normal-support/lift-off diagnostics;
- bounded calibration budget, physical-range controllability, numerical local robustness, and setup-variant robustness.

Adding another score here would duplicate either the sliding law itself (acceleration/displacement) or its support validity (duty/lift-off). Any binary expected-outcome label belongs to a future task-specific protocol only if a concrete stop-line, distance band, or lift-off criterion is preregistered.
