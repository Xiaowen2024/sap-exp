# Genesis native rigid and Genesis + IPC: environment and task setup

Declared 2026-11-14. This document covers environment preparation and task
routing. It does not certify any Genesis B=1/B=5 or generalizability result.
The machine-readable inventory is
[genesis_ipc_task_setup.json](genesis_ipc_task_setup.json).

## Version and environment contract

- Genesis World **1.4.6**, the latest published release checked against
  [GitHub](https://github.com/Genesis-Embodied-AI/genesis-world/releases/tag/v1.4.6)
  and PyPI.
- libuipc / pyuipc **0.0.29**, likewise checked against
  [the release](https://github.com/spiriMirror/libuipc/releases/tag/v0.0.29)
  and PyPI.
- Quadrants **1.4.0**, required by this Genesis release.
- Python **3.12.15**; the actual Genesis 1.4.6 wheel requires Python >=3.12.
- PyTorch **2.8.0+cu128**, a compatible CUDA-12.8 build; Genesis and IPC use
  the NVIDIA GPU. Torch is a pinned runtime dependency, not the simulator
  version being compared.

The evaluation server is Ubuntu 20.04, with system glibc 2.31. The newest IPC
wheel targets manylinux 2.34. An isolated glibc 2.35 loader is used to run the
new Python environment; the OS libraries are not upgraded. Always use the
wrapper `/root/autodl-tmp/genesis-runtime/bin/python-genesis` on this server.
Directly invoking the venv Python bypasses the required loader.

The venv is `/root/autodl-tmp/venvs/genesis146-ipc029-py312`. CUDA libraries
and some Python dependencies use additional package directories because storage is
split between the server's two filesystems. The previous MuJoCo 3.17/4.0
environments and experiment outputs are retained. The unused 3.14 environment
created before the old-version correction was removed to recover space.

Environment acceptance requires real CUDA stepping, finite states and a
falling body that reaches the floor and remains supported, for **each** of
the native-rigid and IPC routes. Version imports and `pip check` alone are
insufficient. See [environment_probe.py](../scripts/genesis/environment_probe.py).
Verification is complete. [Environment record](../exp-results/genesis-ipc-setup-20261114/environment.json),
[package lock](../exp-results/genesis-ipc-setup-20261114/requirements-lock.txt),
[server wrapper](../exp-results/genesis-ipc-setup-20261114/python-genesis.sh),
and individual probe results are retained under
`exp-results/genesis-ipc-setup-20261114/`. Executable/shared libraries are on
persistent filesystems; shared memory holds only temporary files and caches.

Six positive probes passed: floor support, initial linear velocity and a
constant centre-of-mass force for each track, with two parallel environments.
Native rigid inputs used Genesis APIs; IPC inputs required a direct IPC
adapter. Two additional negative controls show that the tested Genesis
velocity/force setters do **not** correctly transfer those inputs to
`ipc_only` bodies. These are documented unsupported input routes, not passing
physics probes. Angular initial velocity, prescribed motion, articulated
actuation and contact-force telemetry still need task-specific validation.
No full benchmark task has yet been run in either Genesis track.

The literal IPC drop probe settled with an approximately **8.62-mm surface
gap**, compared with near-zero gap for native rigid. The captured libuipc
scene default has `contact.d_hat=0.01 m`. This is a barrier activation distance,
not a penetration depth or a guaranteed resting gap. Record the actual
resolved scene configuration for every task; this probe does not establish
superior contact accuracy. Thin plates and narrow socket clearances require
particular care without silently changing B=1 defaults.

## What the two tracks mean

**Genesis native rigid** uses its own RigidSolver contact resolution. B=1
retains the release's native numerical defaults, including the resolved
integrator and constraint solver. Physical setup values still come from the
common task manifest.

**Genesis + IPC** uses the IPCCoupler and libuipc's CUDA contact backend.
IPC represents rigid bodies with affine body dynamics (ABD). Both are named
tracks, each with its own declared B<=5 budget and frozen winner.

For passive free bodies, the initial route is `ipc_only`. A driven rigid body
may need a direct IPC force/kinematic adapter or validated
`two_way_soft_constraint` coupling. A fixed-base mechanism may use
`external_articulation`. These routes have different dynamics ownership and
must be recorded; they are not interchangeable labels.

The v1.4.6 bridge explicitly clears Genesis-side velocities after returning
an IPC-only pose. That velocity getter is not a physical settling witness.
Likewise, a native Genesis contact-force getter must not be assumed to report
IPC contact forces. Input routing and telemetry need separate probes.

Native rigid `friction` and IPC `coup_friction` are separate material fields.
The pinned IPC bridge combines the latter by geometric mean; set them from
the intended physical pair coefficients and verify the resolved contact pair.
For mass/geometry generalizability cells, rebuild the IPC scene rather than
relying on native rigid-info randomization. The bridge rejects batched rigid
info; R=20 copies of one physical cell can share a common topology.

Sources: [Genesis IPC documentation](https://genesis-world.readthedocs.io/en/latest/user_guide/theory/coupling/ipc_coupler.html),
[pinned bridge implementation](https://github.com/Genesis-Embodied-AI/genesis-world/blob/v1.4.6/genesis/engine/couplers/ipc_coupler/coupler.py).

## Routing for all registered task families

### Both tracks: passive contact and impact tasks

- **Three-plate stack:** three dynamic plates plus fixed ground. Audit the
  actual circular collision surface, initial touching and 0.5-mm thin cases;
  read IPC interface loads for the existing load-error gate.
- **GAUGE ball bounce:** released ball mesh plus fixed ground. Release-state
  velocity perturbations must be written into the IPC state.
- **ContactNets cube toss:** dynamic cube plus fixed ground. Match both
  initial linear/angular velocity and the explicit inertia; retain the
  symmetry-aware orientation metric and real-data split.
- **Cube-toss resting tail:** preserve each released initial state and
  measure actual IPC velocity and support force. Do not reset the cube into
  a conveniently settled, non-intersecting state.
- **Round socket / peg-in-hole:** passive peg plus fixed socket. Preserve
  the audited bore rather than replacing it with one convex hull. Check
  contact thickness against clearance. Use the declared task revision;
  the current v6 draft is not automatically an approved run specification.

These are recommendations for two tracks, subject to task-specific initial
geometry and telemetry checks; the environment drop probe does not by itself
validate these five task implementations.

### Both tracks: validate inputs and controllers first

- **Three-box push:** the prescribed horizontal force must actually drive
  the pushed body in IPC. Verify free-space acceleration against F/m before
  interpreting retention, slip or tip.
- **Simple pinch:** preserve force-controlled x slides, z-PD lift, zero
  scene gravity, and the external cube-weight load. The existing preload
  gate needs an independently checked IPC force readout. The touching pads
  and pitch perturbations must be checked for initial intersection.
- **GAUGE slope slider:** distinguish static from prescribed moving-board
  cells. A Genesis pose/velocity setter is not assumed to move the IPC
  board; validate its trajectory and tracking error.
- **MIT Push:** preserve the pusher trajectory and its separate friction
  pair with the object. Validate tracking and the reported pusher wrench.
- **GLIDE-inspired transfer:** keep the declared rigid fork/plate/three-ball
  surrogate and validate moving-support inputs. It remains a simulation-only
  surrogate; adding IPC does not supply missing real object geometry or turn
  it into a deformable-tomato experiment.

### Both after articulation validation

- **Newton's cradle:** retain five fixed-anchor hinges and their ball
  lengths. The published MJCF has a massless, collision-free anchor, whereas
  the IPC external-articulation bridge expects geometry slots for both sides
  of a joint. Validate an equivalent anchor representation before claiming
  the task is runnable. Do not replace each pendulum ball by a free ABD prop.

### Native rigid only for the main comparison

- **Pendulum:** the declared task has no contact. Use native rigid joint
  dynamics. IPC is optional as a separately labelled joint-coupling control,
  rather than a required contact-algorithm comparison. Preserve the real
  setup and clearly identify the chosen energy-reference objective.

### Native rigid first; IPC remains deferred

- **Cassie:** first validate the floating-base model, springs/damping,
  coordinate/state and torque mapping. The pinned external-articulation
  route rejects non-fixed bases, so an IPC extension needs a separately
  validated bridge, potentially compliant coupling at foot links. Keep the
  real-data claim within the released 50-ms window.
- **DENSE/EZGripper:** first preserve tendon-driven underactuation and
  establish registered object/command/reference inputs. IPC on independently
  driven finger props would not preserve the declared task.

## Shared setup requirements before scoring

1. Audit actual mass/inertia, world coordinates, friction pairing, collision
   surfaces, gravity, input traces and integration substeps. Equal option
   numbers or equal asset filenames are not sufficient.
2. Run finite-state, capacity, initial-velocity, force and kinematic tracking
   checks as applicable. Keep engine telemetry unavailable when its readout
   has not been validated; never replace it by zero.
3. Require an admissible IPC initial state. If the original manifest starts
   in an intersection or an incompatible thickness configuration, record that
   cell as not runnable under that setup. A lifted/projected/separated start
   is a new physical revision that must be shared across engines.
4. Record resolved IPC barrier distance, friction regularization, contact
   resistance and coupling strengths. Do not tune them before calling a run
   literal-default B=1. Validate that each pair has one contact owner.
5. Preserve the approved success/accuracy and representative mechanism
   metrics. Each track selects on the same selection cases, freezes its
   winner and uses the same held-out matrix and R=20 perturbations.

## Server intermediate cleanup

The 2026-11-14 cleanup preserved experiment results (including raw rollouts),
authored scripts, manifests/audit snapshots, input datasets/model weights and
installed runtimes. Only known regenerable compiler/download/bytecode caches,
temporary compilation workspaces and already-installed package archives were
removed. The running socket job's driver and Warp caches were deferred until
no CUDA process remained; that second pass has now completed without skips.

Across both passes, recorded disk free space increased by **2.36 GB**
(decimal; about 2.19 GiB), and shared-memory free space by **4.70 GB**.
Final recorded free space: system disk **2.08 GB**, data disk **0.57 GB**.
Shared memory is separate from these disk totals. See
[first-pass record](../exp-results/cleanup-20261114/server_cleanup.json),
[idle-pass record](../exp-results/cleanup-20261114/deferred_cleanup.json), and
[allowlisted cleanup script](../scripts/maintenance/clean_sim_intermediates.py).
