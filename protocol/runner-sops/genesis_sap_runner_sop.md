# Genesis SAP coupler runner SOP (2026-11-17, a1)

Uses Genesis 1.4.6 (SAP requires precision=64), the same runtime as the Genesis Rigid and IPC comparisons. Applies UNIFIED-B5-SOP.md: literal-default B1, five predeclared candidates, unchanged task selection and held-out conditions, R=20, original gates and seeds. Seven primary contact tasks match the IPC a3 main comparison. Ball bounce remains excluded as in that comparison. Socket is a separately reported draft task and cannot be pooled with the primary results.

Registry: `protocol/registries/genesis_sap_rerun.json`. Adapter: `common/genesis_sap_common.py`. Runners: `tasks/<family>/<task>/genesis/run_*_genesis_sap.py`.

## Physical and numerical alignment

Each body remains a separate Genesis MJCF entity with manifest geometry, explicit mass/inertia, no armature, original joints, controls and external forces, dt and gravity. Surface meshes retain the Rigid track's convex-hull representation. SAP internally tetrahedralizes the surfaces and constructs its hydroelastic pressure field; its modulus is not the point-contact stiffness used in SAP Warp. This representation difference must accompany all comparisons.

B1 uses `SAPCouplerOptions()` without numerical overrides (hydroelastic stiffness 1e8 Pa, dissipation time .1 s, 5 Newton iterations). B5 adds four settings declared before any task metric: modulus 1e7/1e8/1e9 Pa with .01 s dissipation, then 1e8 Pa with .001 s dissipation; each uses 20 SAP iterations. All other SAP controls remain default. No geometry refinement is selected on held-out results.

Floor friction uses the moving body's `coup_friction`; rigid pairs use sqrt(mu_a mu_b). All ordinary bodies carry the task coefficient. MIT pusher uses .25^2/.22, object .22, giving actual pusher/object .25. Floor coup_friction stays the task value. No initial position shift or IPC standoff gate applies to SAP.

Direct force comes from each SAP handler's `sap_info.gamma / dt`, transformed by its stored contact tangent/normal basis. Rigid-pair positive force acts on link_idx0, negative on link_idx1. Floor gamma is world-frame and acts on link_idx. Validate 3/2/1 mg stack and signs before scoring. Momentum-balance forces cannot replace this readout. The RigidSolver SAP branch bypasses its own constraint solve (upstream rigid_solver.py lines 1336–1350); no double contact solve occurs.

Cube tail inherits the frozen cube-toss setting and artifact SHA. Three-box +/-10% sensitivity scales hydroelastic modulus. Shared scoring modules are unchanged. Each output records complete=true only after its full matrix finishes. Environment failures must be repaired and rerun, never scored as a solver result. Results on a shared GPU do not support inter-engine timing rankings.

## Running

Use the existing `python-genesis` runtime wrapper on the GPU server. Run `genesis_sap_preflight.py --out <dir>/preflight.json` first. The queue requires all preflight checks to pass and records task exit codes. Preserve logs and copy all raw outputs before reporting.

Upstream implementation: https://github.com/Genesis-Embodied-AI/genesis-world/blob/v1.4.6/genesis/engine/couplers/sap_coupler.py

For a long run on the shared GPU, `manage_genesis_sap.py --out <dir> --lanes 3` resumes completed tasks, runs at most three independent tasks, and admits a new task only when at least 2.2 GiB GPU memory is free. A running three-box process may be adopted by PID, avoiding duplicate selection work. The inherited cube tail waits for its own cube-toss artifact. Task failures are recorded while independent tasks continue; they require repair before the run can be reported complete. This schedules processes only; per-task batches, seeds, candidates and scoring are unchanged.

## Runtime amendment after the initial shared-GPU launch

The first launch's three-plate, impact, slope and MIT builds ran out of GPU memory and are voided. They are environment failures. Three-box encountered an illegal CUDA address after its first two native selection cells; rerun it in a fresh process per scene before interpreting that error as a numerical result. The long-running pinch process may finish without interruption; its physical model and candidate set are identical.

New scenes execute in fresh spawn children, reusing the existing IPC transport without invoking IPC physics. Successful calls are checkpointed in `/dev/shm/genesis_sap_call_cache` using a hash of module source and full physical inputs. OOM retries wait 60 seconds and are never scored. A numerical/kernel crash in an otherwise valid environment is recorded and its rollouts are non-finite, as in IPC. Any exhausted environment retry stops the task rather than scoring an invalid environment.

Three-plate, slope and MIT use at most 5 rollouts per scene. The complete R=20 perturbation array is generated first and sliced without changing a seed or perturbation; raw trajectories are concatenated before the original scoring function runs. Impact/tail use at most 64 rollouts per scene. This changes execution batching only; SAP norms and convergence flags are per environment. Every chunk and numerical failure is retained in provenance. No numerical candidate, mesh, inertia, metric or gate changes. Admit at most two task parents on the shared GPU and at least 3 GiB free before a new one starts; each child separately retries environment failures.
