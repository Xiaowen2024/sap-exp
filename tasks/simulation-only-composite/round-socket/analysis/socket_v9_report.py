#!/usr/bin/env python3
"""Round socket strict-v9-draft: main-result tables (reports.2_frozen_result) from frozen-matrix result files.

Per block (E, W, R, I, C) and group (same clearance = split dev, unseen clearance = split heldout):
  labelled conditions correct / unstable / wrong, correct rollouts / rollouts, inadmissible-rest conditions (all conditions),
  and for each deviation column of rules.deviation_columns the median over conditions (of the per-condition rollout mean) and the
  worst condition (largest |deviation| over its rollouts).  No cross-block total is computed (v9 rule).

Usage: socket_v9_report.py --manifest M --config LABEL=result.json:B5 [--config ...] --out report.md [--json out.json]
A result.json written by the native-matrix process holds frozen_matrix.B1; the main process holds frozen_matrix.B5.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

HOLD = ("held_in_wedge", "held_on_rim")
COLS_HOLD = [("max_displacement_mm", "max displacement, mm", 0.0), ("max_tilt_change_deg", "max tilt change, deg", 0.0),
             ("creep_rate_mm_s", "creep rate (last 0.5 s), mm/s", 0.0), ("tilt_rate_deg_s", "tilt rate (last 0.5 s), deg/s", 0.0)]
COLS_INS = [("final_z_mm", "final centre height, mm (ref 35.0)", 35.0), ("final_tilt_dev_deg", "final tilt off nearest admissible, deg", 0.0),
            ("radial_excess_um", "radial beyond clearance, um", 0.0), ("creep_rate_mm_s", "creep rate (last 0.5 s), mm/s", 0.0)]
COL_CTRL = ("first_base_contact_ms", "first base contact, ms (ref 80.77)", 80.77)


def load(path, key):
    d = json.loads(Path(path).read_text()); fm = d["frozen_matrix"][key]
    return {c["id"]: c for c in fm["conditions"]}, d


def roll_vals(cond, cell, key):
    out = []
    for r in cond["rollouts"]:
        if key == "final_tilt_dev_deg":
            if r.get("final_tilt_deg") is None:
                continue
            ts = (cell.get("final_state_set") or {}).get("tilt_deg", [0.0])
            out.append(min(abs(r["final_tilt_deg"] - t) for t in ts))
        elif key == "radial_excess_um":
            v = r.get(key)
            out.append(v if v is not None else np.nan)
        else:
            v = r.get(key)
            if v is not None:
                out.append(v)
    return np.array([v for v in out if v is not None and np.isfinite(v)], float)


def col_summary(conds, cells, key, ref):
    per = []
    for cid, c in conds.items():
        v = roll_vals(c, cells[cid], key)
        if len(v):
            per.append((cid, float(np.mean(v)), float(v[np.argmax(np.abs(v - ref))])))
    if not per:
        return None
    med = float(np.median([p[1] for p in per])); worst = max(per, key=lambda p: abs(p[2] - ref))
    return {"median": med, "worst_value": worst[2], "worst_condition": worst[0], "n_conditions": len(per)}


def block_table(conds, cells, blk, split):
    sub = {cid: c for cid, c in conds.items() if c["block"] == blk and c["split"] == split}
    lab = {cid: c for cid, c in sub.items() if c["expected"] is not None}
    out = {v: sum(c["verdict"] == v for c in lab.values()) for v in ("correct", "unstable", "wrong")}
    out.update(labelled=len(lab), correct_rollouts=sum(c["correct_rollouts"] for c in lab.values()), rollouts=sum(c["R"] for c in lab.values()),
               inadmissible_rest_conditions=sum(c["inadmissible_rests"] > 0 for c in sub.values()), conditions=len(sub))
    hold = {cid: c for cid, c in lab.items() if c["expected"] in HOLD}
    ins = {cid: c for cid, c in lab.items() if c["expected"] == "inserted"}
    out["hold"] = {k: col_summary(hold, cells, k, ref) for k, _, ref in COLS_HOLD} if hold else None
    if hold:
        sh = [(cid, c["strict_hold_count"]) for cid, c in hold.items()]
        out["hold"]["strict_hold_count"] = {"median": float(np.median([s for _, s in sh])), "worst_value": min(s for _, s in sh),
                                            "worst_condition": min(sh, key=lambda x: x[1])[0], "n_conditions": len(sh)}
    out["inserted"] = {k: col_summary(ins, cells, k, ref) for k, _, ref in COLS_INS} if ins else None
    if ins and blk == "C":
        out["inserted"][COL_CTRL[0]] = col_summary(ins, cells, COL_CTRL[0], COL_CTRL[2])
    return out


def fmt_col(s, f=".3g"):
    return "–" if s is None else f'{format(s["median"], f)} / {format(s["worst_value"], f)} ({s["worst_condition"]})'


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", required=True); ap.add_argument("--config", action="append", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--json")
    a = ap.parse_args(); man = json.loads(Path(a.manifest).read_text()); cells = {c["id"]: c for c in man["conditions"]}
    cfgs = []
    for s in a.config:
        lab, rest = s.split("=", 1); path, key = rest.rsplit(":", 1); conds, _ = load(path, key); cfgs.append((lab, conds))
    res = {}; L = []
    for split, gname in (("dev", "same clearance (0.3 mm)"), ("heldout", "unseen clearance (0.1 and 1.0 mm)")):
        L.append(f"\n### {gname}\n")
        L.append("| Block | " + " | ".join(f"{lab}: c / u / w (rollouts)" for lab, _ in cfgs) + " |")
        L.append("|---|" + "---|" * len(cfgs))
        for blk in ("E", "W", "R", "I", "C"):
            row = []
            for lab, conds in cfgs:
                t = block_table(conds, cells, blk, split); res.setdefault(lab, {}).setdefault(split, {})[blk] = t
                row.append(f'{t["correct"]} / {t["unstable"]} / {t["wrong"]} ({t["correct_rollouts"]}/{t["rollouts"]})'
                           + (f', inadm {t["inadmissible_rest_conditions"]}' if t["inadmissible_rest_conditions"] else ""))
            L.append(f"| {blk} | " + " | ".join(row) + " |")
    L.append("\n### Deviation columns (report only): median over conditions / worst value (worst condition)\n")
    for split in ("dev", "heldout"):
        for blk in ("E", "W", "R", "I", "C"):
            lines = []
            for kind, cols in (("hold", COLS_HOLD + [("strict_hold_count", "strict-hold rollouts (of 8)", None)]), ("inserted", COLS_INS + [COL_CTRL])):
                for key, name, _ in cols:
                    vals = [(res[lab][split][blk][kind] or {}).get(key) for lab, _ in cfgs]
                    if all(v is None for v in vals):
                        continue
                    lines.append(f"| {kind}: {name} | " + " | ".join(fmt_col(v) for v in vals) + " |")
            if lines:
                L.append(f"\n**{blk}, {'same' if split == 'dev' else 'unseen'} clearance**\n")
                L.append("| Column | " + " | ".join(lab for lab, _ in cfgs) + " |"); L.append("|---|" + "---|" * len(cfgs)); L.extend(lines)
    Path(a.out).write_text("\n".join(L) + "\n")
    if a.json:
        Path(a.json).write_text(json.dumps(res, indent=1) + "\n")


if __name__ == "__main__":
    main()
