"""Equal-budget MuJoCo tuning for the shared-CoACD round peg/socket task.

This is the MuJoCo half of a pre-registered-style best-achieved protocol.  It
tunes only the nominal 2-mm-clearance / 1-mm-offset / 1-degree-tilt case;
the selected setting is later frozen and evaluated on the full matrix.
"""
from __future__ import annotations

import csv
import argparse
import json
import re
import time
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT / "mujoco-sim" / "learn_mujoco" / "peg_in_hole_experiments"
XML = HERE / "peg_convex_decomposition.xml"
OUT = ROOT / "exp-results" / "peg-in-hole" / "mujoco-round-socket-equal-budget"
ASSETS = {"blind_socket.obj": (HERE / "assets" / "blind_socket.obj").read_bytes()}
ASSETS.update({p.name: p.read_bytes() for p in (HERE / "assets" / "collision").glob("socket_piece_*.obj")})

DURATION_S = 2.0
CLEARANCE_MM = 2.0
OFFSET_MM = 1.0
TILT_DEG = 1.0


def configs() -> list[dict[str, object]]:
    """48 frozen candidates: 24 contact, 12 damping, 12 solve variants."""
    rows: list[dict[str, object]] = []
    # Main contact grid: dt x normal time scale x near-contact impedance.
    for dt_ms in (.25, .5, 1.0):
        for timeconst_ms in (.75, 1.0, 2.0, 4.0):
            for d0 in (.70, .95):
                rows.append(dict(family="contact", dt_ms=dt_ms, timeconst_ms=timeconst_ms,
                                 dampratio=1.0, d0=d0, dwidth=.995,
                                 solver="Newton", integrator="implicitfast"))
    # Damping ablation near the expected feasible time-scale.
    for timeconst_ms in (.75, 1.0, 2.0):
        for d0 in (.70, .95):
            for dampratio in (.5, 2.0):
                rows.append(dict(family="damping", dt_ms=.5, timeconst_ms=timeconst_ms,
                                 dampratio=dampratio, d0=d0, dwidth=.995,
                                 solver="Newton", integrator="implicitfast"))
    # Solver/integrator ablation at two contact scales.
    for timeconst_ms in (1.0, 2.0):
        for d0 in (.70, .95):
            for solver, integrator in (("Newton", "implicitfast"),
                                       ("CG", "implicitfast"),
                                       ("Newton", "Euler")):
                rows.append(dict(family="solve", dt_ms=.5, timeconst_ms=timeconst_ms,
                                 dampratio=1.0, d0=d0, dwidth=.995,
                                 solver=solver, integrator=integrator))
    assert len(rows) == 48
    return rows


def source_for(cfg: dict[str, object], radius_mm: float, offset_mm: float) -> str:
    source = XML.read_text()
    option = (f'<option timestep="{float(cfg["dt_ms"]) / 1000:g}" gravity="0 0 -9.81" '
              f'integrator="{cfg["integrator"]}" cone="elliptic" solver="{cfg["solver"]}"/>')
    source = re.sub(r'<option\s+[^>]*/>', option, source, count=1)
    solref = f'{float(cfg["timeconst_ms"]) / 1000:g} {float(cfg["dampratio"]):g}'
    solimp = f'{float(cfg["d0"]):g} {float(cfg["dwidth"]):g} .001 .5 2'
    source = source.replace('solref="0.004 1"', f'solref="{solref}"')
    source = source.replace('solimp="0.95 0.99 0.001 0.5 2"', f'solimp="{solimp}"')
    source = source.replace('size="0.010 0.025"', f'size="{radius_mm / 1000:g} 0.025"')
    source = source.replace('pos="0.001 0 0.120"', f'pos="{offset_mm / 1000:g} 0 0.120"')
    return source


def run(cfg: dict[str, object], run_id: int, clearance_mm: float = CLEARANCE_MM,
        offset_mm: float = OFFSET_MM) -> dict[str, object]:
    radius_mm = 12.0 - clearance_mm
    model = mujoco.MjModel.from_xml_string(source_for(cfg, radius_mm, offset_mm), assets=ASSETS)
    data = mujoco.MjData(model)
    peg = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom")
    table = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table")
    peak_pen_mm = 0.0
    max_contacts = 0
    start = time.perf_counter()
    for _ in range(round(DURATION_S / model.opt.timestep)):
        mujoco.mj_step(model, data)
        max_contacts = max(max_contacts, int(data.ncon))
        for i in range(data.ncon):
            contact = data.contact[i]
            if peg in (contact.geom1, contact.geom2):
                other = contact.geom2 if contact.geom1 == peg else contact.geom1
                if other != table:
                    peak_pen_mm = max(peak_pen_mm, float(-contact.dist * 1000.0))
    wall_s = time.perf_counter() - start
    final_pen_mm = 0.0
    for i in range(data.ncon):
        contact = data.contact[i]
        if peg in (contact.geom1, contact.geom2):
            other = contact.geom2 if contact.geom1 == peg else contact.geom1
            if other != table:
                final_pen_mm = max(final_pen_mm, float(-contact.dist * 1000.0))
    x_mm, y_mm, z_mm = (float(v) * 1000.0 for v in data.qpos[:3])
    radial_mm = (x_mm * x_mm + y_mm * y_mm) ** .5
    entered = bool(z_mm <= 45.0 and radial_mm <= 3.0)
    escaped = bool(radial_mm > 50.0 or z_mm < -10.0 or z_mm > 160.0)
    # MuJoCo grows its contact buffer; a CONTACTFULL warning is the applicable
    # no-truncation check rather than a fixed candidate-capacity counter.
    contact_full = int(data.warning[mujoco.mjtWarning.mjWARN_CONTACTFULL].number)
    credible = bool(entered and radial_mm <= clearance_mm and not escaped and contact_full == 0
                    and peak_pen_mm <= .1 * clearance_mm and final_pen_mm <= .1 * clearance_mm)
    row = dict(run_id=run_id, **cfg, clearance_mm=clearance_mm, peg_radius_mm=radius_mm, initial_offset_mm=offset_mm,
               initial_tilt_deg=TILT_DEG, duration_s=DURATION_S,
               final_x_mm=x_mm, final_y_mm=y_mm, final_z_mm=z_mm, final_radial_mm=radial_mm,
               entered=entered, escaped=escaped, contactfull_warning=contact_full,
               max_contacts=max_contacts, pmax_mm=peak_pen_mm, pfinal_mm=final_pen_mm,
               pmax_over_c=peak_pen_mm / clearance_mm, credible=credible, stepping_wall_s=wall_s)
    print(json.dumps(row), flush=True)
    return row


def ranking(row: dict[str, object]) -> tuple[float, float, float, float, float]:
    # Lower tuple is better.  Credible cases come first; then physical screen
    # diagnostics, final centering, and cost.
    return (0.0 if row["credible"] else 1.0, float(row["pmax_over_c"]),
            float(row["final_radial_mm"]), float(row["stepping_wall_s"]), float(row["run_id"]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("tune", "evaluate", "all"), default="all")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if args.phase in ("tune", "all"):
        rows = [run(cfg, i + 1) for i, cfg in enumerate(configs())]
        rows.sort(key=ranking)
        fields = list(rows[0])
        with (OUT / "tuning_results.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader(); writer.writerows(rows)
        payload = {"protocol": {"representation": "shared_32_piece_CoACD", "selection_case":
                   "2-mm clearance, 1-mm offset, 1-degree tilt, gravity drop, 2 s",
                   "ranking": "credible -> pmax/c -> final radial -> stepping wall time",
                   "candidate_budget": 48}, "results_ranked": rows}
        (OUT / "tuning_results.json").write_text(json.dumps(payload, indent=2) + "\n")
    else:
        rows = json.loads((OUT / "tuning_results.json").read_text())["results_ranked"]
    best = rows[0]
    print("BEST", json.dumps(best, indent=2))
    if args.phase in ("evaluate", "all"):
        frozen = {key: best[key] for key in ("family", "dt_ms", "timeconst_ms", "dampratio", "d0", "dwidth", "solver", "integrator")}
        matrix = [run(frozen, i + 1, clearance, offset)
                  for i, (clearance, offset) in enumerate((c, o) for c in (2.0, 1.5, 1.0, .5, .25) for o in (0.0, 1.0))]
        with (OUT / "frozen_matrix.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(matrix[0]))
            writer.writeheader(); writer.writerows(matrix)
        (OUT / "frozen_matrix.json").write_text(json.dumps({"frozen_configuration": frozen, "cases": matrix}, indent=2) + "\n")


if __name__ == "__main__":
    main()
