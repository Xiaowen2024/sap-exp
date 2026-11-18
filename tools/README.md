# Tools

Utilities that work across tasks. Task-specific code is in [`tasks/`](../tasks/README.md);
libraries and runners shared by several tasks are in [`common/`](../common/).

- [`genesis/`](genesis/) — Genesis reruns over all tasks: queue scripts (run from the repository
  root on the server, `SIMEXP_ROOT`, default `/root/SIM-EXP`), preflights, the SAP-coupler task
  manager and reverse worker, cross-engine comparisons, and probes. See
  [`protocol/runner-sops/`](../protocol/runner-sops/).
- `run_mujoco4_rerun.py` — the seven-task MuJoCo Warp 4.0 rerun
  ([registry](../protocol/registries/mujoco_warp_4p0_rerun.json)).
- `run_sap_frozen_b5.py` — SAP frozen-B5 rerun of a task's held-out matrix.
- `check_layout.py` — source check for the folder layout (run after moving or adding scripts).
- `generate_protocol_v2_reports.py`, `verify_result_artifacts.py`,
  `validate_generalizability_manifest.py` — report generation and validation.
- `render_contact_correction.py`, `validate_contact_correction.py` — the 2026-11-09
  three-box / three-plate contact correction.
- `clean_math_syntax.py`, `finish_math_and_setup.py` — report website math cleanup.
- `maintenance/clean_sim_intermediates.py` — allow-listed GPU-server cache cleanup.
