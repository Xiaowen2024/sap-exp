# Closed laptop pickup → handoff → insertion: frozen-parameter transfer

User requests one complete laptop sequence starting at pickup, using the best existing parameters, to assess discrimination. Preserve the original insertion-only experiment. Do not retune this transfer test.

Source: pinned RoboDojo store_laptop_and_headphones episode 0, arx_x5. Start at frame 225 (9.00 s), after closing and before left-hand pickup, through frame 549 (21.96 s); append 1.5 s hold. No headphones or articulated closing stage. Closed lid remains rigidified exactly as in the short-fragment experiment. Total 14.46 s at dt=1 ms, 14460 steps. One rollout per engine, no repeats or intermediate resets.

Reuse all object/gripper geometry, masses/inertias, friction and controllers from insertion-only. Add the official initial laptop stand: common source-only reconstructed pose; deck, left/right leg envelopes and phone bracket are separate convex regions, preserving the open underside. Side-leg openings are approximated; inspect contacts involving the stand before attributing a failure solely to the simulator. Initial laptop is supported, with a 0.1 mm nominal gap; no initial grasp or preload is inserted. Pose uses image corners and an explicitly declared supported-deck-height assumption. This is an inferred initial scene, not released state ground truth.

Frozen parameters: SAP combined pair ke=100000 N/m, tau=0.02 s (shape ke=200000, tau=0.01), from insertion candidate 2. MuJoCo Warp geom solref=(0.002,1), remaining native contact/solver defaults, from insertion candidate 3. Record exact parameter equality, input/source/geometry hashes and initial no-overlap LP audit before launch. Solver changes are forbidden in this transfer test.

Keep 1-ms state and physical body-pair candidate/normal-force/gap telemetry. Phases in source-relative time: left-grasp [3.2,7.6] s, right-grasp [8.3,11.6], rack-support [12.3,14.46], final source [12.5,12.96], final appended hold [13.96,14.46]. Report pickup height, grasp loss/slip, handoff retention, insertion and release separately; identify the first divergence stage. Contact-window duty is descriptive, not a task verdict by itself.

Use the validated convex penetration bounds from the insertion study, sampled at 10 ms; retain lower/upper bounds and sampling coverage. Reject the earlier unvalidated FCL EPA depths. Force thresholds 0.001/0.01/0.1 N. No reference trajectory error without admitted object reference and uncertainty. Record missing fields as null, not zero.

An engine that fails pickup may never contact the rack; zero rack penetration then is not superior fidelity. A common failure can expose reconstruction/controller issues and does not establish an engine ranking. A difference in this one frozen case is evidence of parameter transfer behavior, not a success-rate estimate. Deliver comparison videos, native data, JSON metrics and Markdown only. No Canvas, IPC or further parameter search.
