# Engines

Code shared by every task for one simulator. Task-specific runners live under
`tasks/<family>/<task>/<engine>/`.

- [`genesis/`](genesis/) — Genesis Rigid, SAP and IPC adapters (`genesis_*_common.py`),
  preflights, probes, the queue scripts that run all tasks, the cross-engine
  comparison scripts, and the runner SOPs in [`genesis/sop/`](genesis/sop/).
  Queue scripts are run from the repository root on the server
  (`SIMEXP_ROOT`, default `/root/SIM-EXP`).
- [`mujoco/`](mujoco/) — the MuJoCo Warp 4.0 seven-task rerun and the contact-capacity audit.
- [`sap/`](sap/) — the SAP frozen-B5 rerun (`run_sap_frozen_b5.py`).

Registries for these reruns are in [`protocol/manifests/`](../protocol/manifests/)
(`genesis_*_rerun.json`, `mujoco_warp_4p0_rerun.json`).
