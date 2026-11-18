# Genesis RigidSolver runner SOP

Status: in force from 2026-11-14 for the Genesis 1.4.6 RigidSolver track (no coupler). Genesis + IPC gets its own
registry and SOP later. This SOP adds Genesis-specific rules to [UNIFIED-B5-SOP.md](../UNIFIED-B5-SOP.md) and
[task_runner_sop.md](../task_runner_sop.md); where they overlap, those two documents still govern B=1, B=5, R=20,
held-out matrices and reporting.

## 1. Scope

| Task | Manifest | Runner (`tasks/<family>/<task>/genesis/`) |
| --- | --- | --- |
| Three-plate v3.1 | `three_plate_stack_strict_v3p1` | `run_threeplate_genesis_rigid.py` |
| Three-box v4 | `three_box_push_strict_v4` | `run_threebox_genesis_rigid.py` |
| Cube toss v2 | `contactnets_cube_toss_dynamic_impact_v2` | `run_impact_genesis_rigid.py --task cube` |
| Ball bounce v2 / v3 | `gauge_ball_bounce_dynamic_impact_v2` / `_v3` | `run_impact_genesis_rigid.py --task ball` |
| Cube tail v3 | `contactnets_cube_tail_sustained_v3` | `run_tail_genesis_rigid.py` (inherits the Genesis cube-toss winner) |
| Simple pinch v3 | `simple_pinch_force_controlled_v3` | `run_pinch_genesis_rigid.py` |
| GAUGE slope v3 | `gauge_slope_slider_dynamic_friction_v3` | `run_slope_genesis_rigid.py` |
| MIT Push v3 | `mit_push_dynamic_friction_v3` | `run_mitpush_genesis_rigid.py` |
| Pendulum v2 (idealised) | `pendulum_energy_v2` | `run_pendulum_v2_genesis_rigid.py` |
| Pendulum GAUGE real v3.1 | `pendulum_gauge_real_v1` | `run_gauge_pendulum_real_genesis_rigid.py` |

Not run: Newton's cradle (idealised v3 and real v4.1), Cassie, round socket, GLIDE, DENSE.

The task manifests are unchanged and are checked by SHA-256. The Genesis candidates live in
[`genesis_rigid_rerun.json`](../registries/genesis_rigid_rerun.json), written before any task metric was computed.

## 2. How a runner is built

1. **Physics from the manifest, numerics from Genesis.** Each runner writes small physical MJCF snippets: geometry,
   mass and inertia, joints, and friction. It does not import the MuJoCo runners' MJCF. Genesis drops `<pair>`,
   margin and gap without warning, and it copies any `solref`/`solimp` it finds, which would make B=1 a MuJoCo setting.
   Gravity, `dt` (with `substeps=1`), inputs and perturbations come from the manifest and from the shared `*_common`
   module.
2. **One entity per physical body.** Genesis disables collision between links of the same entity that touch in the
   initial pose (`enable_neutral_collision=False`). Stacked boxes, plates and pads touching the cube would lose contact
   without any warning. Separate entities avoid this and keep `RigidOptions` at its defaults.
3. **Shared logic is reused unchanged.** Trial selection, perturbation arrays, labels, gates, losses, the selection
   rule and the held-out loop are all the existing `strict_v3_common`, `impact_v2_common`, `tail_v2_common`,
   `pinch_v3_common`, `slope_v3_common`, `mitpush_v3_common`, `pendulum_v2_common` and `gauge_pendulum_v3_common`
   modules. Each runner supplies only the engine callback.
4. **Common adapter.** [`genesis_rigid_common.py`](../../common/genesis_rigid_common.py) holds the version pin,
   CUDA initialisation, candidate parsing, scene building, contact readout and provenance.

## 3. Alignment rules (Genesis-specific)

| Item | Rule | Verified by |
| --- | --- | --- |
| Armature | `default_armature=None`. Without it Genesis adds 0.1 to every revolute or prismatic joint. | preflight: free and slide dofs read back 0 |
| Mass / inertia | Explicit values from the manifest; mesh bodies use MuJoCo-compiled mesh inertia, as the MuJoCo driver does. | preflight readback (0.37 kg, I = 8.1e-4) |
| Friction | Genesis takes pair μ = max of the two geoms (lower bound 0.01). Every geom carries the task μ. MIT Push uses table 0.22, object 0.22 and pusher 0.25, which gives exactly 0.22 and 0.25. condim 3 → torsional and rolling 0. | provenance `geoms_friction` |
| Meshes | Convex hull, `decimate=False`, no convex decomposition. This matches MuJoCo's mesh collision. Plates are Genesis cylinders, which are 64-vertex hulls; this is recorded. | provenance `geoms` |
| dt | Actual step, `substeps=1`. The manifest `dt` is used, with the exact frame divisor where the common module uses one. | provenance `dt_s` |
| Initial state | qpos = [pos, quat wxyz]; dofs velocity = [world linear, **body** angular], the same as MuJoCo qvel. | preflight: rotation axis and translation |
| External force | `apply_links_external_wrench(ref=link_COM)` every step. | preflight: F/m acceleration |
| Joint drives | `control_dofs_force` with the manifest control law (pinch). Kinematic inputs (slope board, MIT pusher) are written every step as pose plus velocity, as in the MuJoCo driver; the written-pose error is recorded. | runner `tracking` |
| Contact force | `collider.get_contacts` force acts on `geom_b` in the world frame; normal load = \|f·n\| summed over the pair. Read contacts **before** any state write, because a write clears them. | preflight: m g; 3/2/1 m g stack |
| Capacity | Genesis stops with an error when contacts overflow, so a finished run has no truncation. `max_contacts_seen` is recorded. | runner provenance |

## 4. B=1 and candidates

- **B=1:** `RigidOptions()` defaults and the default geom `sol_params` (resolved timeconst 0.01 s, dampratio 1,
  solimp 0.9/0.95/0.001/0.5/2, approximate_implicitfast, Newton, 25 iterations, pyramidal cone). Every run records
  the resolved values.
- **Candidates 2–5** are the registry entries. Genesis has no direct (negative) solref, so a MuJoCo direct (k, b)
  candidate becomes the exact equivalent tc = 2/b, dr = b/(2√k). Where MuJoCo used b = 0, Genesis uses b = 0.02 s⁻¹,
  because dampratio = 0 is not allowed. Genesis raises each contact's mixed timeconst to at least 2·dt; when that
  happens, the stored `geoms_sol_params` and `min_contact_timeconst_s` show it.
- **Three-box:** Genesis averages the two geoms' parameters, so the box geoms carry the box-box target and the ground
  geom carries 2 × (ground target) − (box value). This reproduces both MuJoCo pair targets exactly.
- **Cube tail:** the only candidate is the Genesis cube-toss frozen winner. Its artifact path and SHA-256 are recorded.
- **Pendulums:** the candidates are Genesis' three integrators. Genesis has no implicit or RK4 integrator, so these
  tasks have three candidates and none are added as padding.

## 5. Running

```bash
# server, Genesis runtime wrapper; GPU must be idle or shared politely
cd /root/SIM-EXP
python-genesis tools/genesis/genesis_rigid_preflight.py --out <dir>/preflight.json     # must print ALL PASS
setsid nohup bash tools/genesis/run_genesis_rigid_queue.sh <dir> < /dev/null > <dir>/queue.out 2>&1 &
```

The queue runs the preflight first and stops if it fails. It then runs three lanes in parallel; the tail task waits
for cube toss. Each task writes `genesis_rigid_<task>.json`, saved after every cell, plus a `.log` file. A run counts
as finished only when `complete: true`.

## 6. Reporting

- Report each task next to SAP and MuJoCo Warp with the same metrics and denominators. Never mix in a better track.
- Name the engine-specific representation differences: Genesis cylinders are 64-vertex hulls; the slope, MIT Push and
  impact candidates may be floored to 2·dt; the first MIT Push sample has no contact force because Genesis has no
  forward-only call.
- Wall time comes from three parallel lanes on one RTX 3080 Ti. It is the last selection tie-breaker only, and it
  cannot be compared with runs on other machines.
