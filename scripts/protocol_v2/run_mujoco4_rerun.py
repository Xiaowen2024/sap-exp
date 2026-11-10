#!/usr/bin/env python3
"""Prepare or execute the seven-task MuJoCo Warp 4.0 rerun, without replacing old results.

Preparation needs no simulator. Execution must use an isolated 4.0 CUDA environment.
This checks coverage/provenance; it does not replace numerical/result audits.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / "sim_protocol/manifests/mujoco_warp_4p0_rerun.json"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2) + "\n")
    temp.replace(path)


def inputs(registry):
    from validate_generalizability_manifest import validate

    paths = list((ROOT / "scripts/protocol_v2").glob("*.py"))
    paths += [ROOT / "scripts/cube_orientation_metrics.py", REGISTRY]
    paths += [ROOT / "sim_protocol" / n for n in
              ("UNIFIED-B5-SOP.md", "task_runner_sop.md", "result_summary_protocol.md")]
    for task in registry["tasks"]:
        p = ROOT / "sim_protocol/manifests" / task["manifest"]
        if sha(p) != task["sha256"]:
            raise ValueError(f"Manifest changed: {p.name}; declare a new revision first")
        errors = validate(json.loads(p.read_text()))
        if errors:
            raise ValueError(f"{p.name}: {errors}")
        paths.append(p)
    return {str(p.relative_to(ROOT)): sha(p) for p in paths}


def check_output(path, task):
    data = json.loads(path.read_text())
    man = json.loads((ROOT / "sim_protocol/manifests" / task["manifest"]).read_text())
    if data.get("mujoco_version") != "4.0.0" or data.get("engine") != "MuJoCo Warp":
        raise ValueError(f"Wrong engine/version in {path}")
    if data.get("manifest_sha256") != task["sha256"]:
        raise ValueError(f"Wrong manifest in {path}")
    if [r["cell"] for r in data["heldout"]] != man["held_out_conditions"]:
        raise ValueError(f"Incomplete or changed held-out matrix in {path}")
    if len(data["selection_candidates"]) != len(man["candidate_rule"]["mujoco_warp_candidates"]):
        raise ValueError(f"Incomplete candidate screen in {path}")
    for cell in data["heldout"]:
        if cell.get("R") != man["R"] or cell["candidate"] != data["frozen_candidate"]:
            raise ValueError(f"Changed repetitions/frozen candidate in {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--execute", action="store_true", help="otherwise only prepare the plan")
    ap.add_argument("--correction-root", type=Path,
                    default=ROOT / "exp-results/contact-correction-20261109")
    a = ap.parse_args()
    registry = json.loads(REGISTRY.read_text())
    fingerprints = inputs(registry)
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    plan_path = out / "run_plan.json"
    if plan_path.exists():
        plan = json.loads(plan_path.read_text())
        if plan["input_sha256"] != fingerprints:
            raise ValueError("Inputs changed since preparation; use a new output directory/revision")
    else:
        plan = {"prepared_at_utc": datetime.now(timezone.utc).isoformat(),
                "registry": registry, "input_sha256": fingerprints,
                "state": "prepared_not_run",
                "tasks": [{"id": t["id"], "state": "not_run"} for t in registry["tasks"]]}
        save(plan_path, plan)
        for rel in fingerprints:
            dst = out / "inputs" / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes((ROOT / rel).read_bytes())
    if not a.execute:
        print(f"Prepared {len(registry['tasks'])} tasks; no simulations started: {plan_path}")
        return

    # Respect the requested order: finish and validate the corrected plate pair first.
    from validate_contact_correction import validate as validate_plate
    manifest = ROOT / "sim_protocol/manifests/three_plate_stack_strict_v3p1.json"
    for engine, prefix in (("sap", "sap"), ("mujoco", "mjwarp")):
        validate_plate(a.correction_root / f"three-plate/{engine}/{prefix}_three_plate_strict_v3.json", manifest)

    versions = {n: importlib.metadata.version(n) for n in registry["required_packages"]}
    if versions != registry["required_packages"]:
        raise ValueError(f"Use an isolated 4.0 environment; found {versions}")
    import warp as wp
    if not wp.is_cuda_available():
        raise RuntimeError("Evaluation requires CUDA; CPU preflight is not an evaluation result")
    wp.set_device("cuda:0")
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(ROOT / "scripts"), str(ROOT / "scripts/protocol_v2"),
                                        env.get("PYTHONPATH", "")])
    save(out / "environment.json", {"packages": versions,
         "warp": importlib.metadata.version("warp-lang"), "python": sys.version,
         "device": str(wp.get_device()), "pip_freeze": subprocess.check_output(
             [sys.executable, "-m", "pip", "freeze"], text=True),
         "gpu": subprocess.check_output(["nvidia-smi", "--query-gpu=name,uuid,driver_version",
                                          "--format=csv,noheader"], text=True)})
    with (out / "preflight.log").open("w") as log:
        subprocess.run([sys.executable, "-m", "unittest", "test_box_v4_validity",
                        "test_mujoco_capacity_audit"], cwd=ROOT / "scripts/protocol_v2",
                       env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    plan["state"] = "running"
    save(plan_path, plan)
    for task, status in zip(registry["tasks"], plan["tasks"]):
        if fingerprints != inputs(registry):
            raise ValueError("Source or protocol changed while running; stop before next task")
        task_out = out / task["id"]
        artifact = task_out / task["artifact"]
        if artifact.exists():
            check_output(artifact, task)
            status.update(state="coverage_checked_pending_numerical_audit", artifact_sha256=sha(artifact))
            save(plan_path, plan)
            continue
        task_out.mkdir(exist_ok=True)
        command = [sys.executable, str(ROOT / "scripts/protocol_v2" / task["runner"]),
                   "--manifest", str(out / "inputs/sim_protocol/manifests" / task["manifest"]),
                   "--out", str(task_out), *task.get("args", [])]
        status.update(state="running", command=command)
        save(plan_path, plan)
        try:
            with (task_out / "run.log").open("w") as log:
                subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
            check_output(artifact, task)
        except Exception:
            status["state"] = "failed_requires_inspection"
            plan["state"] = "stopped_requires_inspection"
            save(plan_path, plan)
            raise
        status.update(state="coverage_checked_pending_numerical_audit", artifact_sha256=sha(artifact))
        save(plan_path, plan)
    plan["state"] = "all_outputs_present_pending_numerical_audit"
    save(plan_path, plan)


if __name__ == "__main__":
    main()
