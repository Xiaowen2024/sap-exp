#!/usr/bin/env python3
"""Selection-screen comparison (same cells, same seeds, same scoring) across five engines.

Genesis SAP was run selection-only (user decision 2026-11-18), so the comparison uses each engine's own selection rows:
B1 = native default (candidate 1) and B5 = the engine's frozen candidate, both scored on the task's selection cells.
Held-out results of the other engines are not used here.  Prints a markdown table and writes JSON next to --out.
"""
import argparse
import json
import math
from pathlib import Path

E = Path(__file__).resolve().parents[2] / "exp-results"
GS = E / "genesis-sap-20261117/selection-only-20261118"
FILES = {
    "three-box": {"SAP Warp": "aligned-generalizability-strict-v3/v4/sap_three_box_strict_v4.json",
                  "MuJoCo 4.0": "mujoco-warp-4p0/three-box/mjwarp_three_box_strict_v4.json",
                  "Genesis Rigid": "genesis-rigid-20261114/three-box/genesis_rigid_three_box_strict_v4.json",
                  "Genesis IPC": "genesis-ipc-a3-20261116/three-box/genesis_ipc_three_box_strict_v4.json",
                  "Genesis SAP": GS / "sel/three-box/genesis_sap_three_box_strict_v4.json"},
    "simple-pinch": {"SAP Warp": "simple-pinch-v3/sap_simple_pinch_v3_inertia_fixed.json",
                     "MuJoCo 4.0": "mujoco-warp-4p0/simple-pinch/mjwarp_simple_pinch_v3.json",
                     "Genesis Rigid": "genesis-rigid-20261114/simple-pinch/genesis_rigid_simple_pinch_v3.json",
                     "Genesis IPC": "genesis-ipc-a3-20261116/simple-pinch/genesis_ipc_simple_pinch_v3.json",
                     "Genesis SAP": GS / "sel/simple-pinch/genesis_sap_simple_pinch_v3.json"},
    "three-plate": {"SAP Warp": "contact-correction-20261109/three-plate/sap/sap_three_plate_strict_v3.json",
                    "MuJoCo 4.0": "mujoco-warp-4p0/three-plate/mjwarp_three_plate_strict_v3.json",
                    "Genesis Rigid": "genesis-rigid-20261114/three-plate/genesis_rigid_three_plate_strict_v3.json",
                    "Genesis IPC": "genesis-ipc-a3-20261116/three-plate/genesis_ipc_three_plate_strict_v3.json",
                    "Genesis SAP": GS / "sel/three-plate/genesis_sap_three_plate_strict_v3.json"},
    "cube-toss": {"SAP Warp": "dynamic-impact-v2/sap_cube_dynamic_impact_v2.json",
                  "MuJoCo 4.0": "mujoco-warp-4p0/cube-toss/mjwarp_cube_dynamic_impact_v2.json",
                  "Genesis Rigid": "genesis-rigid-20261114/cube-toss/genesis_rigid_cube_dynamic_impact_v2.json",
                  "Genesis IPC": "genesis-ipc-a3-20261116/cube-toss/genesis_ipc_cube_dynamic_impact_v2.json",
                  "Genesis SAP": GS / "a2/cube-toss/genesis_sap_cube_dynamic_impact_v2.json"},
    "cube-tail": {"SAP Warp": "cube-tail-v3/sap_cube_tail_v3.json",
                  "MuJoCo 4.0": "mujoco-warp-4p0/cube-tail-v3/mjwarp_cube_tail_v3.json",
                  "Genesis Rigid": "genesis-rigid-20261114/cube-tail/genesis_rigid_cube_tail_v3.json",
                  "Genesis IPC": "genesis-ipc-a3-20261116/cube-tail/genesis_ipc_cube_tail_v3.json",
                  "Genesis SAP": GS / "sel/cube-tail/genesis_sap_cube_tail_v3.json"},
    "gauge-slope": {"SAP Warp": "gauge-slope-v3/sap_gauge_slope_v3.json",
                    "MuJoCo 4.0": "mujoco-warp-4p0/gauge-slope/mjwarp_gauge_slope_v3.json",
                    "Genesis Rigid": "genesis-rigid-20261114/gauge-slope/genesis_rigid_gauge_slope_v3.json",
                    "Genesis IPC": "genesis-ipc-a3-20261116/gauge-slope/genesis_ipc_gauge_slope_v3.json",
                    "Genesis SAP": GS / "sel/gauge-slope/genesis_sap_gauge_slope_v3.json"},
    "mit-push": {"SAP Warp": "mit-push-v3/sap_mit_push_v3.json",
                 "MuJoCo 4.0": "mujoco-warp-4p0/mit-push/mjwarp_mit_push_v3.json",
                 "Genesis Rigid": "genesis-rigid-20261114/mit-push/genesis_rigid_mit_push_v3.json",
                 "Genesis IPC": "genesis-ipc-a3-20261116/mit-push/genesis_ipc_mit_push_v3.json",
                 "Genesis SAP": GS / "sel/mit-push/genesis_sap_mit_push_v3.json"},
}


def f(x, p=3):
    return "—" if x is None or (isinstance(x, float) and not math.isfinite(x)) else (f"{x:.{p}g}" if isinstance(x, float) else str(x))


def summarize(task, row):
    """One selection-candidate row -> (headline string, numeric dict)."""
    if task in ("three-box", "simple-pinch"):
        k = sum(r["agree_and_valid_count"] for r in row); n = sum(r["R"] for r in row)
        per = "+".join(str(r["agree_and_valid_count"]) for r in row)
        extra = ""
        if task == "simple-pinch":
            pe = [r["preload_rel_error_max"] for r in row if r.get("preload_rel_error_max") is not None and math.isfinite(r["preload_rel_error_max"])]
            extra = f", preload err ≤{max(pe):.1%}" if pe else ", preload err —"
        else:
            extra = f", valid {sum(r['valid_count'] for r in row)}/{n}"
        return f"{k}/{n} ({per}){extra}", {"correct": k, "n": n}
    if task == "three-plate":
        fin = (row.get("failed_gate_counts") or {}).get("finite", 0)
        return f"{row['pass_count']}/{row['R']}" + (f", non-finite {fin}" if fin else "") + \
               (f", load err {row['load_error_max_mean']:.1%}" if row.get("load_error_max_mean") is not None and math.isfinite(row["load_error_max_mean"]) else ""), \
               {"correct": row["pass_count"], "n": row["R"]}
    if task == "cube-tail":
        return f"{row['pass_count']}/{row['n']} pass, settled {row['settled_count']}", {"correct": row["pass_count"], "n": row["n"]}
    if task == "cube-toss":
        return f"loss {f(row['loss'], 4)}", {"loss": row["loss"]}
    if task == "gauge-slope":
        gates = all(c.get("gate") for c in row["cells"])
        return f"loss {f(row['loss'], 4)}" + ("" if gates else ", gate FAIL"), {"loss": row["loss"]}
    if task == "mit-push":
        return f"loss {f(row['loss'], 4)}, wrench err {f(row.get('wrench_error_mean_N'), 3)} N" + ("" if row.get("gate") else ", gate FAIL"), {"loss": row["loss"]}
    raise KeyError(task)


def cand_id(c):
    c = c.get("candidate", c) if isinstance(c, dict) else c
    if isinstance(c, list):
        c = c[0].get("candidate", {})
    return c.get("id") if isinstance(c, dict) else str(c)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, default=GS / "selection_comparison.md"); a = ap.parse_args()
    lines, js = [], {}
    for task, eng in FILES.items():
        lines += [f"### {task}", "", "| Engine | B1 native (selection cells) | B5 frozen candidate | B5 on selection cells |", "|---|---|---|---|"]
        js[task] = {}
        for name, rel in eng.items():
            p = E / rel if not isinstance(rel, Path) else rel
            if not p.exists():
                lines.append(f"| {name} | missing | | |"); continue
            d = json.loads(p.read_text()); sel = d.get("selection_candidates") or []
            fi = d.get("frozen_index")
            fi = int(fi) if fi not in (None, "") else None
            b1 = summarize(task, sel[0]) if sel else ("—", {})
            b5 = summarize(task, sel[fi - 1]) if fi and len(sel) >= fi else ("not frozen", {})
            fc = d.get("frozen_candidate"); fcid = fc.get("id") if isinstance(fc, dict) else (fc or "—")
            if task == "cube-tail":
                fcid = (d.get("inherited_from") or {}).get("candidate") or fcid
            lines.append(f"| {name} | {b1[0]} | {fcid} | {b5[0]} |")
            js[task][name] = {"B1": b1[1], "B5": b5[1], "frozen": fcid, "file": str(p)}
        lines.append("")
    txt = "\n".join(lines)
    a.out.write_text(txt + "\n"); a.out.with_suffix(".json").write_text(json.dumps(js, indent=1) + "\n")
    print(txt)


if __name__ == "__main__":
    main()
