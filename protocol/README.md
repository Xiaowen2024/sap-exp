# Protocol

Rules that apply across tasks, from the general protocol down to the family a task
belongs to. Each task's own SOP and manifests are in its folder under
[`tasks/`](../tasks/README.md).

- [`UNIFIED-B5-SOP.md`](UNIFIED-B5-SOP.md) — the B=1/B=5 tuning and frozen-generalization protocol.
- [`task_runner_sop.md`](task_runner_sop.md) — how a task runner must behave.
- [`result_summary_protocol.md`](result_summary_protocol.md) — how results are summarized and reported.
- [`generalizability_rerun_register.md`](generalizability_rerun_register.md) — register of reruns.
- [`families/`](families/) — family protocols (static friction, dynamic friction, dynamic impact,
  energy conservation, sustained contact, composite, real playback): the metrics shared by
  the tasks of each family.
- [`runner-sops/`](runner-sops/) — how the Genesis Rigid, SAP coupler and IPC runners align a
  task to its manifest, and how those reruns were set up and run.
- [`registries/`](registries/) — cross-task rerun registries (Genesis Rigid / SAP / IPC,
  MuJoCo Warp 4.0). They pin each task's manifest by name and SHA-256; do not edit them in place.

Some older manifests and registries still name runners by their pre-reorganization path
(`scripts/...`); runners are found by file name, and [`../MOVED_PATHS.md`](../MOVED_PATHS.md)
maps old paths to new ones.
