"""Generate comprehensive Markdown reports and CSV tables for static-friction-v2 and dynamic-friction-v2."""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
RES_ROOT = ROOT / "exp-results" / "protocol-v2"
REP_ROOT = ROOT / "exp-report"


def _fmt_ci(ci: list[float] | tuple[float, float], scale: float = 1.0, digits: int = 3) -> str:
    return f"[{ci[0]*scale:.{digits}f}, {ci[1]*scale:.{digits}f}]"


def export_static_friction_v2() -> None:
    path = RES_ROOT / "static-friction-v2" / "static_friction_v2_all.json"
    if not path.exists():
        print(f"Skipping static-friction-v2 export: {path} not found")
        return
    data = json.loads(path.read_text())
    backends = data["backends"]

    # Export CSV of B=1,2,30 budget candidates for both tasks & backends
    csv_path = RES_ROOT / "static-friction-v2" / "static_friction_v2_budget_sweep.csv"
    rows = []
    for bname, bdata in backends.items():
        for tname in ("three_box_push", "simple_pinch"):
            for cand in bdata[tname]["budget_candidates"]:
                rows.append({"backend": bname, "task": tname, **cand})
    if rows:
        fieldnames = sorted({k for r in rows for k in r.keys()})
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    print(f"Wrote {csv_path}")


def export_dynamic_friction_v2() -> None:
    path = RES_ROOT / "dynamic-friction-v2" / "dynamic_friction_v2_all.json"
    if not path.exists():
        print(f"Skipping dynamic-friction-v2 export: {path} not found")
        return
    data = json.loads(path.read_text())
    backends = data["backends"]

    csv_path = RES_ROOT / "dynamic-friction-v2" / "dynamic_friction_v2_budget_sweep.csv"
    rows = []
    for bname, bdata in backends.items():
        for tname in ("slope_slider", "mit_push"):
            for cand in bdata[tname]["budget_candidates"]:
                rows.append({"backend": bname, "task": tname, **cand})
    if rows:
        fieldnames = sorted({k for r in rows for k in r.keys()})
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    export_static_friction_v2()
    export_dynamic_friction_v2()
