# Genesis + IPC runner SOP (a3)

Status: **revision a3**, dated 2026-11-16. The user reviewed the preliminary run of 2026-11-15, and all results from
that run are superseded. This SOP adds IPC-specific rules to
[genesis_rigid_runner_sop.md](genesis_rigid_runner_sop.md). B=1, B=5, R=20, held-out matrices and reporting are
unchanged from [UNIFIED-B5-SOP.md](../../../protocol/UNIFIED-B5-SOP.md).

## 1. Scope

The tasks are those of the RigidSolver track, minus the two pendulums, which have no contact. Under a3,
**ball bounce v2 is not run**, and **ball bounce v3 is run and reported but not ranked**. The reason is that IPC
has no restitution or damping parameter: the energy lost at impact is the numerical damping of implicit Euler, and the
collision surface is a 162-vertex hull. The registry is
[`genesis_ipc_rerun.json`](../../../protocol/manifests/genesis_ipc_rerun.json). The runners are `tasks/<family>/<task>/genesis/run_*_genesis_ipc.py`.
The shared adapter is [`genesis_ipc_common.py`](../genesis_ipc_common.py).

## 2. Routes (who integrates what)

| Body | `coup_type` | Notes |
| --- | --- | --- |
| Ground plane, fixed board | `ipc_only` | IPC ground / fixed ABD |
| Free bodies (plates, boxes, cube, ball, slider, MIT object) | `ipc_only` | IPC integrates them; Genesis only mirrors the pose |
| Pinch pads | `two_way_soft_constraint` | Genesis keeps the x/z slide joints and the manifest control law; IPC follows |
| Slope board (task-3), MIT pusher | `two_way_soft_constraint` | Genesis pose and velocity are written every step; IPC follows |

- `coup_type` is always explicit. The rigid collider drops a pair only when both sides carry an explicit IPC type.
  Every run records `max_rigid_solver_contacts`, which must be 0.
- `external_articulation` was not used. A pad has two slide joints on one body, which gives no parent/child link
  pair, and that mode has no force readout.

## 3. Matching the manifest

| Item | Rule | Exact? |
| --- | --- | --- |
| Mass | ρ = m / V of the IPC surface mesh | yes; checked against F/m in the preflight |
| Inertia | ABD uses the uniform-density inertia | no for cube toss/tail (ratio recorded) and the MIT object |
| Friction | pair μ = √(μa μb); every body carries the pair μ; MIT pusher 0.25²/0.22 | yes; preflight sliding deceleration 2.949 vs 2.943 |
| dt, gravity | SimOptions dt (`substeps=1`) and gravity, which IPC uses directly | yes |
| Initial pose / velocity | Written into the IPC `transform` / `velocity` attributes before world init | yes; preflight |
| External force | `AffineBodyExternalBodyForce`, set every step through an Animator | yes; preflight |
| Ball mesh (a2) | IPC uses the convex hull of 162 farthest-point-sampled vertices of the original 8,066-vertex mesh; the original costs about 20 s per step | no (vertex radius 29.6–30.4 mm vs 30.0 mm) |
| Start gap | Resting contacts start g0 = d_hat apart. Slope: lift along the board normal, only as far as needed. MIT: move the object away from the tip if it is closer than g0. Every shift is recorded. | declared change |

## 4. Readouts

- **Pose and velocity:** taken from IPC's own ABD accessor. The Genesis-side velocity of an IPC-only body is zeroed
  by the bridge.
- **Contact force (direct, used by every load/preload gate since 2026-11-16):** libuipc's `ContactSystemFeature`
  returns the gradient of each contact potential per vertex, for barrier (`…+N`) and friction (`…+F`) primitives. The
  force on vertex i is f_i = −∂E/∂x_i / dt². Vertices map to bodies through each ABD geometry's `global_vertex_offset`.
  An interface load is the sum over the vertices on one face of the receiving body, using the body-frame sign of the
  rest vertex position. Applied as:
  - three-plate: the lower face of each upper plate;
  - three-box: the lower face of the pushed box;
  - pinch: the cube's −x and +x faces;
  - cube tail: the whole cube.
  
  The adapter calls are `Built.vertex_forces` and `Built.direct_force`.
- **Momentum balance (diagnostic only):** m(Δv/dt − g) − F_ext equals whatever net force produces the observed motion,
  so it matches the weight whenever a body does not accelerate. Gates must not use it. Each run records `readout_check`:
  the direct readout minus the momentum readout, which is the Newton residual of the accepted step.
- **Validity of the direct readout:** the feature returns the gradient assembled at the last Newton iterate. It agrees
  with momentum balance only when Newton has converged.
  - Checks in `genesis_ipc_cforce_probe.py`:
    - rest, at dt 2 ms and 1 ms: equals m·g to 1e-13;
    - three-box stack: interface loads match to 5e-5, Newton's third law to 1e-16;
    - drop: differs by 0.016% at newton_tol 1e-4, by 1.8% at the default.
  - At the default newton_tol of 0.05, the friction part reads 0 while a box creeps, because Newton stops after one
    iteration at the step-start state.
  - The residual for creep at r = 0.7 shrinks with the tolerance: 0.70 μN at 0.05, 0.116 at 1e-4, 0.0075 at 1e-5,
    4e-5 at 1e-6.
- **Contact flag:** the face carries barrier force, i.e. at least one vertex has a non-zero barrier gradient.
- **Overlap gates:** IPC never interpenetrates, so overlap gates cannot fail. They are reported as not applicable, and
  the rest gap is reported instead.
- **MIT wrench:** the bridge's coupling force on the pusher. This has not yet been moved to the direct readout.

## 5. Candidates

- **B=1:** `IPCCouplerOptions()` defaults: d_hat 10 mm, eps_velocity 0.01 m/s, resistance 1e9, Newton tolerance
  0.05, bridge strength 100.
- **Candidates 2–5 (a3):** `d_hat=1mm`; `d_hat=1mm,eps_v=1e-3`; `d_hat=0.3mm,eps_v=1e-3`; `d_hat=0.1mm,eps_v=1e-3`.
  Each one also carries **`newton_tol=1e-4`**.
  - The default tolerance of 0.05 m/s stops after a single linearisation whenever motion is slower than 5 cm/s.
    Friction then acts as viscous drag (v = r·eps_v/2), and a body never slides above the friction limit.
  - With 1e-4, the remaining coupling between rollouts solved in one scene changes a creep speed by 1.7%
    (`diagnostics/batch_probe.json`).
- **Standoff gate (a3):** IPC rests separated, never interpenetrated, so a task's overlap gate cannot fail. For IPC,
  the mirror of that gate applies, with the same threshold, over the task's rest window:
  - three-plate: rest gap ≤ 5% of the plate thickness over the last 1 s;
  - cube tail: lowest-corner standoff ≤ 1 mm over the settled tail.
  
  Tasks without an overlap gate get no new gate; their standoff is reported. In code this is
  `plate_summary(..., standoff=True)` and `tail_v2_common.execute(..., standoff=True)`. Both are off by default, so
  other engines are unaffected.
- **Pinch (amendment a1):** candidates 2–5 add `strength=1`, because the default strength makes the pads diverge
  to NaN. Under a3, `newton_tol=1e-4` is added as well.
- **Three-box ±10% sensitivity:** scales `contact_resistance`.

## 6. Crash rule

libuipc aborts the whole process when a PCG/Newton solve diverges. Every simulation call therefore runs in a child
process. A crashed call counts as invalid: all its rollouts become non-finite. For impact and tail this happens per
64-rollout chunk. The reason is recorded under `failures` / `chunk_failures`.

The child's stderr is kept. A death caused by **CUDA out of memory** happens when the GPU is shared with other jobs. It
is an environment failure, not a solver result, so it is never scored. The call waits 120 s and is retried, up to
30 times. Every result file records `isolation_log.oom_retries` and `isolation_log.failures`.

## 7. Running

```bash
cd /root/SIM-EXP
python-genesis engines/genesis/genesis_ipc_preflight.py --out <dir>/preflight.json          # must print ALL PASS
setsid nohup bash engines/genesis/run_genesis_ipc_queue_a3.sh <dir> < /dev/null > <dir>/../queue.out 2>&1 &
```

The GPU holds at most about four IPC processes (2–2.7 GB each on 12 GB). When other jobs share the GPU, run three
lanes; `run_genesis_ipc_queue_a3.sh` does this.
