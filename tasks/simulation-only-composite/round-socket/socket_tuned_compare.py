#!/usr/bin/env python3
"""Round socket v9: compare engines/versions on the 46 labelled frozen conditions (R=8 each, same seeds and scoring).

Inputs are result.json files that hold frozen_matrix.B5.conditions.  Tuned results (socket_tune.py, 2026-11-18) were tuned
on these same conditions (no held-out), so they show the best each engine reached, not generalisation.
"""
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import json, sys
from pathlib import Path

ROOT = REPO_ROOT; E = ROOT / "exp-results"
MAN = json.loads((ROOT / "tasks/simulation-only-composite/round-socket/manifests/round_socket_strict_v9_draft.json").read_text())
LAB = [c for c in MAN["conditions"] if c["role"] == "frozen" and c.get("expected") is not None]
RUNS = [("SAP old code, v8-pilot B5 (ke=57255,tau=1.5ms,gap=.1mm)", E / "round-socket-v8-draft/sap/result.json"),
        ("SAP Newton-convex, same old B5", E / "sap-newton-convex-b5-20261118/socket/result.json"),
        ("SAP Newton-convex, tuned (ke=1e7,tau=5ms,gap=0)", E / "socket-tune-20261118/sap/iterfinal_R8/result.json"),
        ("MuJoCo 4.0, v9 B5 (tc=2.021ms)", E / "round-socket-v9-draft/mj400/result.json"),
        ("MuJoCo 4.0, tuned (tc=1ms,impratio=100)", E / "socket-tune-20261118/mj/iterfinal_R8/result.json"),
        ("Genesis IPC, v9 B5 (d_hat=0.01mm)", E / "round-socket-v9-draft/ipc/result.json")]


def score(rows):
    by = {r["id"]: r for r in rows}; out = {"n": 0, "c": 0, "u": 0, "w": 0, "k": 0, "R": 0, "inadm": 0, "blocks": {}}
    for cell in LAB:
        r = by.get(cell["id"])
        if r is None:
            continue
        out["n"] += 1; v = r["verdict"]; key = {"correct": "c", "unstable": "u"}.get(v, "w"); out[key] += 1
        out["k"] += r["correct_rollouts"]; out["R"] += r["R"]; out["inadm"] += bool(r.get("inadmissible_rests"))
        b = ("same" if cell["clearance_mm"] == 0.3 else "unseen") + "_" + cell["block"]
        bb = out["blocks"].setdefault(b, [0, 0, 0]); bb["cuw".index(key)] += 1
    return out


def main():
    lines = ["| Run | labelled frozen conditions | correct / unstable / wrong | correct rollouts | conditions with inadmissible rest |", "|---|---|---|---|---|"]
    res = {}
    for name, p in RUNS:
        if not p.exists():
            lines.append(f"| {name} | missing | | | |"); continue
        d = json.loads(p.read_text()); s = score(d["frozen_matrix"]["B5"]["conditions"]); res[name] = s
        lines.append(f"| {name} | {s['n']} | {s['c']} / {s['u']} / {s['w']} | {s['k']}/{s['R']} ({s['k'] / max(1, s['R']):.1%}) | {s['inadm']} |")
    blocks = sorted({b for s in res.values() for b in s["blocks"]})
    lines += ["", "| Block (c/u/w) | " + " | ".join(n.split(",")[0] + ("" if "tuned" not in n else " tuned") for n in res) + " |", "|---|" + "---|" * len(res)]
    for b in blocks:
        lines.append(f"| {b} | " + " | ".join("/".join(map(str, s["blocks"].get(b, ["-"] * 3))) for s in res.values()) + " |")
    txt = "\n".join(lines); print(txt)
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text(txt + "\n")


if __name__ == "__main__":
    main()
