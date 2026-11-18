#!/usr/bin/env python3
"""SAP frozen-B5 rerun (no selection): run a task's held-out matrix with the B5 candidate frozen by an earlier SAP artifact.

Used for the 2026-11-18 rerun of SAP with the Newton-synced convex collision path (branch fix/convex-collision-newton-sync,
commit 2325082, env SAP_WARP_CONVEX_BACKEND=newton).  Physics and scoring are unchanged: every cell is simulated by the task's
own SAP runner functions (run_cell / make_run_cell / make_simulate / simulate) and scored by the shared SIM-EXP modules, with
the same seeds and R as the full runner.  Only the selection screen is skipped; the frozen candidate is read from the prior
artifact and checked against the manifest's declared candidate list.  B1, parameter sensitivity and robustness are not run.

Writes <out>/<result file> after every cell; on restart, cells already present (same id) are kept and skipped.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import hashlib
import importlib.util
import json
import os
import sys
import time
from pathlib import Path


def load_runner(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec); sys.modules[path.stem] = mod; spec.loader.exec_module(mod); return mod


def check_preflight(pf: dict, device: str = "cuda:0"):
    dev = pf.get("devices", {}).get(device)
    if not pf.get("all_pass") or not dev or not all(v["pass"] for v in dev.values()) or (pf.get("cpu_gpu") or {}).get("pass") is not True:
        raise SystemExit(f"SAP preflight must pass on {device} with CPU/GPU agreement")


RUNNERS = {"three-box": "run_three_box_strict_v4.py", "three-plate": "run_three_plate_strict_v3.py",
           "simple-pinch": "run_pinch_v3_sap.py", "cradle": "run_cradle_v2_sap.py", "socket": "run_socket_v8_sap.py"}
OUTFILE = {"three-box": "sap_three_box_strict_v4.json", "three-plate": "sap_three_plate_strict_v3.json",
           "simple-pinch": "sap_simple_pinch_v3.json", "cradle": "sap_cradle_v3.json", "socket": "result.json"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", required=True, choices=sorted(RUNNERS))
    ap.add_argument("--runner-dir", type=Path, required=True, help="SAP local_experiments/strict_v3")
    ap.add_argument("--simexp-scripts", type=Path, required=True)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--prior", type=Path, required=True, help="earlier SAP artifact that froze the B5 candidate")
    ap.add_argument("--preflight-json", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--check-only", action="store_true", help="verify inputs and frozen candidate, no simulation")
    a = ap.parse_args()
    pf = json.loads(a.preflight_json.read_text()); check_preflight(pf)
    sys.path.insert(0, str(a.simexp_scripts))
    import warp as wp
    T = load_runner(a.runner_dir / RUNNERS[a.task])
    prior = json.loads(a.prior.read_text()); fidx = prior["frozen_index"]; frozen = prior["frozen_candidate"]
    a.out.mkdir(parents=True, exist_ok=True); path = a.out / OUTFILE[a.task]
    old = json.loads(path.read_text()) if path.exists() else {}
    wp.init()
    import sap_warp
    src = Path(sap_warp.__file__).parent
    commit = next((p / "COMMIT").read_text().strip() for p in src.parents if (p / "COMMIT").exists()) if any((p / "COMMIT").exists() for p in src.parents) else None
    log = lambda s: print(s, flush=True)

    if a.task == "socket":
        import socket_v8_common as sc
        T.sc = sc
        man, sha = sc.load_manifest(a.manifest)
        decl = man["candidates"]["sap_warp"]["declared_order"][fidx - 1]
        assert abs(decl["ke"] - frozen["ke"]) < 1e-6 * frozen["ke"] and decl["tau_s"] == frozen["tau"] and decl["gap_m"] == frozen["gap"], (decl, frozen)
        cand = frozen
        conds = man["conditions"]; idx = {c["id"]: j for j, c in enumerate(conds)}
        todo = [c for c in conds if c["role"] == "frozen"]
        rows = {r["id"]: r for r in old.get("frozen_matrix", {}).get("B5", {}).get("conditions", [])}
    else:
        if a.task in ("three-box", "three-plate"):
            import strict_v3_common as common; man, sha = common.load_manifest(a.manifest)
        elif a.task == "simple-pinch":
            import pinch_v3_common as common; man, sha = common.load_manifest(a.manifest)
        else:
            import cradle_v2_common as common; man, sha = common.load_manifest(a.manifest)
        decl = T.parse_candidate(man["candidate_rule"]["sap_candidates"][fidx - 1])
        assert decl == frozen, (decl, frozen)
        cand = frozen; todo = list(enumerate(man["held_out_conditions"]))
        rows = {r["cell"]["id"]: r for r in old.get("heldout", [])}
    assert prior.get("manifest_sha256") in (None, sha) or a.task == "socket", "prior artifact was made with a different manifest"

    header = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
              "engine": "SAP Warp internal", "sap_source": str(src), "sap_commit": commit,
              "sap_convex_backend": os.environ.get("SAP_WARP_CONVEX_BACKEND", "newton (branch default)"),
              "warp_version": wp.config.version, "device": "cuda:0", "preflight": pf,
              "rerun_kind": "frozen-B5 only; selection, B1, sensitivity and robustness NOT rerun",
              "frozen_from": {"artifact": str(a.prior), "sha256": hashlib.sha256(a.prior.read_bytes()).hexdigest(),
                              "frozen_index": fidx, "frozen_candidate": frozen},
              "frozen_index": fidx, "frozen_candidate": frozen, "complete": False}

    def save(body):
        path.write_text(json.dumps((sc if a.task == "socket" else common).json_safe({**header, **body}), indent=1) + "\n")

    if a.check_only:
        print(json.dumps({"task": a.task, "manifest_sha256": sha[:12], "frozen_index": fidx, "frozen": frozen, "n_cells": len(todo),
                          "already_done": len(rows), "sap_source": str(src), "sap_commit": commit,
                          "convex_backend": header["sap_convex_backend"]}), flush=True)
        return
    t_all = time.time()
    if a.task == "socket":
        out_rows = []
        for c in todo:
            if c["id"] in rows:
                out_rows.append(rows[c["id"]]); continue
            r = sc.run_condition(cand, c, man["seed0"] + idx[c["id"]], man, T.simulate, a.out / "traj" / "frozen_B5", "")
            r["phase"] = "frozen_B5"; out_rows.append(r); log(f"[frozen_B5] {cand['id']} {sc.fmt(r)}")
            save({"interpretation": sc.INTERPRETATION, "frozen_matrix": {"B5": {"candidate_index": fidx, "conditions": out_rows}}})
        body = {"interpretation": sc.INTERPRETATION, "frozen_matrix": {"B5": {"candidate_index": fidx, "conditions": out_rows, "tally": sc.tally(out_rows)}}}
        log(f"frozen B5 tally {body['frozen_matrix']['B5']['tally']}")
    else:
        R, seed0 = man["R"], man["seed0"]; held = []
        if a.task == "simple-pinch":
            run_pinch = T.make_run_cell(common)
        if a.task == "cradle":
            simulate = T.make_simulate(common)
        for j, cell in todo:
            if cell["id"] in rows:
                held.append(rows[cell["id"]]); continue
            t0 = time.perf_counter()
            if a.task == "three-box":
                r = T.run_cell(cand, cell, seed0 + 1000 + j, R, a.out, "cuda:0", common); log(f"held {cell['id']} {r['expected']} {T.fmt(r)}")
            elif a.task == "three-plate":
                r = T.run_cell(cand, cell, common.cell_seed(seed0, "heldout", j), R, man["duration_s"], a.out, "cuda:0", common)
                log(f"held {cell['id']} {r['pass_count']} {r['failed_gate_counts']} {r['max_overlap_mm_mean']:.4f}mm {r['load_error_max_mean']:.4%}")
            elif a.task == "simple-pinch":
                r = run_pinch(cand, cell, seed0 + 1000 + j, R); r["wall_s"] = time.perf_counter() - t0; r.update({"cell": cell, "candidate": cand})
                log(f"held {cell['id']} {r['expected']} {r['agree_and_valid_count']}/{R} valid={r['valid_count']} preload_err<={r['preload_rel_error_max']:.1%}")
            else:
                r = common.run_cell(cand, cell, seed0 + 1000 + j, R, simulate); log(f"held {cell['id']} gate={r['gate']} MTE={r['mte']} E_ratio={r['energy_ratio']}")
            held.append(r); save({"heldout": held})
        body = {"heldout": held}
        if a.task in ("three-box", "simple-pinch"):
            body["heldout_accuracy_total"] = [sum(r["agree_and_valid_count"] for r in held), R * len(held)]
        elif a.task == "three-plate":
            body["heldout_pass_total"] = [sum(r["pass_count"] for r in held), R * len(held)]
        log(f"total {body.get('heldout_accuracy_total') or body.get('heldout_pass_total') or len(held)}")
    body["wall_s_this_process"] = time.time() - t_all
    header["complete"] = True; save(body)


if __name__ == "__main__":
    main()
