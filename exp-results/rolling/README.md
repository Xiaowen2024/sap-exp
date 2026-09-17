# SimBenchmark rolling: MuJoCo vs. SAP Warp

This directory contains a source-level reproduction of the rolling scenario
from `leggedrobotics/SimBenchmark`: a 10 kg box on a plane, 25 one-kilogram
balls in a 5 by 5 grid, 150 N at 60 degrees in the horizontal plane, and
`dt=0.001 s` for 4 s.  The metric is the original mean squared velocity error
of the box plus the first corner ball against the benchmark's analytic model.

All reproducibility drivers are in `scripts/`; engine code remains in
`mujoco-sim/` and `sap-sim/`.

Run MuJoCo:

```sh
mujoco-sim/.venv/bin/python exp-results/rolling/scripts/run_mujoco_rolling_benchmark.py
```

Run SAP Warp (on CUDA for the full four-second test):

```sh
sap-sim/.sap-venv/bin/python exp-results/rolling/scripts/run_sap_rolling_benchmark.py
```

SAP mixes shape friction harmonically, unlike the original MuJoCo model's
`max` rule.  Its shape coefficients are therefore transformed so that its
effective box--ground and ball--box coefficients remain 0.4 and 0.8.

The local macOS SAP build is CPU-only.  Its raw collision pipeline reports 29
contacts in the initial configuration, so a capacity of 64 is sufficient for
the smoke test.  `SolverSAP.last_contact_count` currently reports its
preallocated matrix capacity rather than this active contact count; the runner
therefore records the collision buffer counter instead.  MuJoCo's result is
recorded in `mujoco/metrics.json`.
