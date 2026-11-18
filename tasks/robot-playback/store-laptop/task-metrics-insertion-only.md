# Closed laptop insertion — SAP vs MuJoCo protocol v1

Authorization: user requests at most five adaptive native numerical configurations per engine, one rollout per configuration, stopping early when adequate. This explicitly replaces Unified B=5 SOP R=20 perturbations and predeclared-grid requirements for this pilot. Do not call R=1 outcomes a success-rate estimate or a held-out benchmark.

## Frozen task

Use pinned RoboDojo store_laptop_and_headphones episode 0, arx_x5. Replay only the already-closed laptop insertion after handoff. Exclude headphone handling and lid closing. Recover the shared initial object poses from source images and camera calibration before observing either engine result. Store fit inputs, residuals, uncertainty/limitations and hashes. Fix the closed lid at the same angle in both engines; preserve mass and geometry, disclose rigidification. The rack is static as in the official Geometry task configuration. Both engines use identical convex geometry, Coulomb friction, mass/inertia, initial state, dt and detached articulated X5 gripper drive. Never change slot clearance or initial pose separately to help one engine.

Retain the robot trajectory as input. Record its tracking error as an implementation check. If a segment starts with a moving or loaded grasp, explicitly record initial twist and missing contact/preload history; do not hide initialization transients by deleting metric samples. Prefer a stationary grasp interval when visible.

## Iterations

Candidate 1 uses literal native contact/solver defaults, recording compiled values. Frozen physical settings and actuator settings are not native tuning variables. Every subsequent candidate changes only native contact/solver parameters; write the reason and chosen values before launching. At most five full object rollouts per engine, exactly one per candidate; no hidden repeats. Short adapter/debug checks, if required, are separately labeled and never treated as independent outcomes. A failed numerical implementation is invalid, not an engine failure.

Stop early if both are finite, capacity-valid, insert and release without gross wall crossing, and maintain support in the fixed observation window. Smaller residual motion alone does not establish physical fidelity. If they differ, use the remaining budget to diagnose contact behavior rather than tune to one lucky binary success. Record all candidates, including failed ones, and distinguish exploratory selection from independent evaluation.

## Telemetry and metrics

Save time-aligned object and gripper poses/velocities at 1 ms when feasible; state whether a metric uses coarser data. Save native per-physical-body-pair candidate counts, minimum contact gaps and positive normal-force sums; do not combine all static geoms into the table label. Candidate contact is not load-bearing contact. Report pair contact duty and longest uninterrupted loss in predeclared grasp and rack-support phases, normal force thresholds 0.01 N and sensitivity at 0.001/0.1 N. Normal contact switching is not automatically an error.

Use one independent geometry observer on both traces for laptop/rack, laptop/fingers and laptop/table. Report sampled depth and timestamps, sampling interval, coverage, relation to local wall thickness and representation error. Vertex interior samples give a lower bound, not exact global penetration. Preserve native gaps separately; do not rank engines using incomparable native gap definitions. Check full wall crossings, including cases with small final overlap.

In a common post-release window report COM linear-speed and angular-speed RMS, net drift, peak speed and orientation change. Verify jaws open and withdraw before attributing stability to the rack. Record insertion depth, orientation and outcome separately from geometric validity. If reference release is absent, do not invent it: label any appended release as a separate diagnostic. Appended hold/settling has no source pose ground truth.

Reference trajectory/final-pose errors require reliable estimated visual poses with coverage and uncertainty. Missing estimates are null; engine-to-engine differences are not errors to reference. Official policy reward is not the reproduction criterion. A single recorded episode permits descriptive comparison, not a general engine ranking.

## Deliverables

Per-run scene/targets/parameter/source hashes, health.json, trace.npz, contact_pairs.npz, native settings, stdout log and timing boundaries; iterative journal; reference and replay videos plus close-up; metrics JSON and Markdown report. No Canvas, no IPC. Publication only uses public setup/numerical artifacts; proprietary SAP source stays on private runtime paths.
