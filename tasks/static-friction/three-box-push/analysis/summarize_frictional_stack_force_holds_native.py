"""Summarize the MuJoCo native friction-control sweep.

The primary metric is an OLS estimate of steady middle--top creep over the
last two seconds of an eight-second constant-force hold.  It is intentionally
the same convention used by ``summarize_frictional_stack_force_holds_v3.py``
so native MuJoCo and the earlier matched-compliance/SAP runs are comparable.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "exp-results" / "frictional-stack-v3" / "force-holds"
NATIVE = BASE / "mujoco-native"
FORCES = (1.0, 2.0, 3.0, 4.0)
TAIL_START = 8.5
HOLD_START = 2.5
TOP = "middle_top"


def read_trace(path: Path) -> dict[str, np.ndarray]:
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def regress(time: np.ndarray, values: np.ndarray) -> tuple[float, float]:
    slope, intercept = np.polyfit(time, values, 1)
    residual = np.sum((values - (slope * time + intercept)) ** 2)
    total = np.sum((values - np.mean(values)) ** 2)
    return float(slope), float(1.0 - residual / total) if total else 1.0


def analyze(tag: str, config: dict, force: float) -> dict:
    path = NATIVE / tag / f"trace_F_{force:g}N.csv"
    data = read_trace(path)
    time = data["time_s"]
    tail = time >= TAIL_START
    hold = time >= HOLD_START
    slip = data[f"{TOP}_slip_m"]
    velocity = data[f"{TOP}_vt_m_s"]
    fn = data[f"{TOP}_Fn_N"]
    ft = data[f"{TOP}_Ft_N"]
    rho = data[f"{TOP}_rho"]
    gap = data[f"{TOP}_gap_m"]
    contacts = data[f"{TOP}_contacts"]
    slope, r2 = regress(time[tail], slip[tail])
    hold_start = np.flatnonzero(hold)[0]
    finite_rho = rho[tail][np.isfinite(rho[tail])]
    body_vz = np.vstack([data[f"box_{i}_vz_m_s"][tail] for i in range(3)])
    return {
        "configuration": config["name"],
        "tag": tag,
        "force_target_N": force,
        "trace": str(path.relative_to(ROOT)),
        "tail_slope_um_s": slope * 1e6,
        "tail_slope_R2": r2,
        "tail_vt_mean_um_s": float(np.mean(velocity[tail]) * 1e6),
        "tail_vt_std_um_s": float(np.std(velocity[tail]) * 1e6),
        "tail_vt_rms_um_s": float(np.sqrt(np.mean(velocity[tail] ** 2)) * 1e6),
        "hold_displacement_um": float((slip[-1] - slip[hold_start]) * 1e6),
        "tail_Fn_mean_N": float(np.mean(fn[tail])),
        "tail_Fn_cv": float(np.std(fn[tail]) / np.mean(fn[tail])),
        "tail_Ft_mean_N": float(np.mean(ft[tail])),
        "tail_rho_mean": float(np.mean(finite_rho)),
        "tail_contact_duty": float(np.mean(contacts[tail] > 0)),
        "tail_contacts_mean": float(np.mean(contacts[tail])),
        "tail_gap_mean_mm": float(np.mean(gap[tail]) * 1e3),
        "tail_gap_std_um": float(np.std(gap[tail]) * 1e6),
        "tail_max_body_vz_rms_mm_s": float(np.max(np.sqrt(np.mean(body_vz**2, axis=1)) * 1e3)),
        "tail_solver_iterations_mean": float(np.mean(data["solver_iterations"][tail])),
        "tail_solver_iterations_max": int(np.max(data["solver_iterations"][tail])),
    }


def sap_reference_4n() -> dict:
    summary = json.loads((BASE / "summary.json").read_text())
    for case in summary["cases"]:
        if case["engine"] == "sap" and case["force_target_N"] == 4.0:
            metric = case["interfaces"][TOP]
            return {
                "tail_slope_um_s": metric["tail_slope_mm_s"] * 1e3,
                "tail_vt_mean_um_s": metric["tail_vt_mean_mm_s"] * 1e3,
                "tail_vt_std_um_s": metric["tail_vt_std_mm_s"] * 1e3,
                "hold_displacement_um": metric["hold_displacement_mm"] * 1e3,
                "tail_rho_mean": metric["tail_rho_mean"],
            }
    raise RuntimeError("SAP 4 N reference not found")


def render(rows: list[dict]) -> Path:
    order = ["native_2ms", "native_1ms", "impratio_10", "impratio_100", "noslip_1", "noslip_3"]
    labels = {
        "native_2ms": "tc=2 ms, ir=1",
        "native_1ms": "tc=1 ms, ir=1",
        "impratio_10": "tc=1 ms, ir=10",
        "impratio_100": "tc=1 ms, ir=100",
        "noslip_1": "ir=10 + NoSlip 1",
        "noslip_3": "ir=10 + NoSlip 3",
    }
    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.25), constrained_layout=True)
    for name in order:
        subset = [row for row in rows if row["configuration"] == name]
        axes[0].plot(
            [row["force_target_N"] for row in subset],
            [abs(row["tail_slope_um_s"]) for row in subset],
            marker="o", linewidth=2, label=labels[name],
        )
        axes[1].plot(
            [row["force_target_N"] for row in subset],
            [abs(row["hold_displacement_um"]) for row in subset],
            marker="o", linewidth=2, label=labels[name],
        )
    axes[0].axhline(2.516, color="black", linestyle="--", linewidth=1.3, label="SAP 4 N: 2.516 µm/s")
    axes[0].set_yscale("log")
    axes[0].set_xlabel("Held horizontal force (N)")
    axes[0].set_ylabel("|tail creep slope| (µm/s; log)")
    axes[0].set_title("MuJoCo native anti-slip sweep")
    axes[0].grid(alpha=0.25)
    axes[0].legend(fontsize=8)
    axes[1].set_yscale("log")
    axes[1].set_xlabel("Held horizontal force (N)")
    axes[1].set_ylabel("|relative displacement over 8 s| (µm; log)")
    axes[1].set_title("Accumulated middle--top motion")
    axes[1].grid(alpha=0.25)
    axes[1].legend(fontsize=8)
    path = NATIVE / "native-friction-hold-sweep.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def main() -> None:
    metadata = json.loads((NATIVE / "metadata.json").read_text())
    configurations: dict[str, dict] = {}
    for item in metadata:
        configurations[item["configuration_tag"]] = item["configuration"]
    rows = [analyze(tag, config, force) for tag, config in configurations.items() for force in FORCES]
    rows.sort(key=lambda row: (row["configuration"], row["force_target_N"]))
    payload = {
        "method": {
            "benchmark": "same three-box, mu=0.5 constant-force hold as v3",
            "dt_ms": 0.5,
            "tail_window_s": [8.5, 10.5],
            "creep_estimator": "OLS slope of middle-top relative x displacement over final 2 s",
            "contact_model": "native MuJoCo default geom contact; margin=gap=0, elliptic cone, Newton, implicitfast",
            "note": "These cases intentionally do not match SAP normal compliance.",
        },
        "sap_4N_reference": sap_reference_4n(),
        "cases": rows,
    }
    (NATIVE / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")
    fields = list(rows[0])
    with (NATIVE / "top-interface-summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    plot = render(rows)
    print(json.dumps({"summary": str(NATIVE / "summary.json"), "plot": str(plot)}, indent=2))


if __name__ == "__main__":
    main()
