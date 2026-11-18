# Protocol

Cross-task rules and every manifest.

- [`UNIFIED-B5-SOP.md`](UNIFIED-B5-SOP.md) — the B=1/B=5 tuning and frozen-generalization protocol.
- [`task_runner_sop.md`](task_runner_sop.md) — how a task runner must behave.
- [`result_summary_protocol.md`](result_summary_protocol.md) — how results are summarized and reported.
- [`generalizability_rerun_register.md`](generalizability_rerun_register.md) — register of reruns.
- [`manifests/`](manifests/) — all task manifests and rerun registries. Runners look them up by
  name and record their SHA-256 in every result, so do not edit a manifest in place: declare a
  new revision instead. Some older manifests still name runners by their pre-reorganization
  path (`scripts/...`); see [`../MOVED_PATHS.md`](../MOVED_PATHS.md).

Task SOPs are next to their code under [`tasks/`](../tasks/).
