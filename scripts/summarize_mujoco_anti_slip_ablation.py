"""Summarize the fixed-normal-contact MuJoCo 4 N anti-slip ablation."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "exp-results" / "frictional-stack-v3" / "mujoco-anti-slip-ablation-v3"
TOP = "middle_top"
TAIL_START, HOLD_START = 8.5, 2.5
EXPECTED_FN, MU = 9.81, 0.5


def read_trace(path: Path) -> dict[str, np.ndarray]:
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def regress(time: np.ndarray, values: np.ndarray) -> tuple[float, float]:
    slope, intercept = np.polyfit(time, values, 1)
    residual = np.sum((values - (slope * time + intercept)) ** 2)
    total = np.sum((values - np.mean(values)) ** 2)
    return float(slope), float(1.0 - residual / total) if total else 1.0


def tfr_label(value: float | None) -> str:
    return "inherit (2 ms)" if value is None else f"{value * 1e3:g} ms"


def analyze(item: dict) -> dict:
    trace = BASE / item["trace"]
    data = read_trace(trace)
    time = data["time_s"]
    tail, hold = time >= TAIL_START, time >= HOLD_START
    slip = data[f"{TOP}_slip_m"]
    velocity = data[f"{TOP}_vt_m_s"]
    fn, ft = data[f"{TOP}_Fn_N"], data[f"{TOP}_Ft_N"]
    rho, contacts, gap = data[f"{TOP}_rho"], data[f"{TOP}_contacts"], data[f"{TOP}_gap_m"]
    slope, r2 = regress(time[tail], slip[tail])
    hold_start_index = int(np.flatnonzero(hold)[0])
    finite_rho = rho[tail][np.isfinite(rho[tail])]
    contact_duty = float(np.mean(contacts[tail] > 0))
    fn_mean = float(np.mean(fn[tail]))
    vz = np.vstack([data[f"box_{i}_vz_m_s"][tail] for i in range(3)])
    config = item["configuration"]
    return {
        "tag": item["tag"],
        "dt_ms": config["dt_s"] * 1e3,
        "friction_timeconst_ms": None if config["friction_timeconst_s"] is None else config["friction_timeconst_s"] * 1e3,
        "friction_solreffriction": item["friction_solreffriction"],
        "impratio": config["impratio"],
        "noslip_iterations": config["noslip_iterations"],
        "trace": item["trace"],
        "wall_s": item["wall_s"],
        "tail_slope_um_s": slope * 1e6,
        "tail_slope_R2": r2,
        "tail_vt_mean_um_s": float(np.mean(velocity[tail]) * 1e6),
        "tail_vt_std_um_s": float(np.std(velocity[tail]) * 1e6),
        "tail_vt_rms_um_s": float(np.sqrt(np.mean(velocity[tail] ** 2)) * 1e6),
        "hold_displacement_um": float((slip[-1] - slip[hold_start_index]) * 1e6),
        "tail_Fn_mean_N": fn_mean,
        "tail_Fn_cv": float(np.std(fn[tail]) / fn_mean) if fn_mean else None,
        "tail_Fn_balance_error_pct": 100.0 * (fn_mean - EXPECTED_FN) / EXPECTED_FN if fn_mean else None,
        "tail_Ft_mean_N": float(np.mean(ft[tail])),
        "tail_rho_mean": float(np.mean(finite_rho)) if len(finite_rho) else None,
        "tail_contact_duty": contact_duty,
        "tail_contacts_mean": float(np.mean(contacts[tail])),
        "tail_gap_std_um": float(np.std(gap[tail]) * 1e6),
        "tail_max_body_vz_rms_mm_s": float(np.max(np.sqrt(np.mean(vz ** 2, axis=1)) * 1e3)),
        "tail_solver_iterations_mean": float(np.mean(data["solver_iterations"][tail])),
        "valid_steady_contact": bool(contact_duty >= 0.99),
    }


def plot_no_noslip(rows: list[dict]) -> Path:
    fig, axes = plt.subplots(1, 3, figsize=(14, 3.8), sharey=True, constrained_layout=True)
    colors = {1.0: "C0", 10.0: "C1", 100.0: "C2"}
    tfr = [None, 4.0, 8.0]
    labels = ["inherit (2 ms)", "4 ms", "8 ms"]
    for axis, dt in zip(axes, (0.25, 0.5, 1.0)):
        for impratio in (1.0, 10.0, 100.0):
            values = []
            for timeconst in tfr:
                row = next(row for row in rows if row["dt_ms"] == dt and row["impratio"] == impratio and row["noslip_iterations"] == 0 and row["friction_timeconst_ms"] == timeconst)
                values.append(abs(row["tail_slope_um_s"]) if row["valid_steady_contact"] else np.nan)
            axis.plot(labels, values, marker="o", linewidth=2, color=colors[impratio], label=f"impratio={impratio:g}")
        axis.set_yscale("log")
        axis.grid(alpha=0.25)
        axis.set_title(f"dt = {dt:g} ms; NoSlip=0")
        axis.tick_params(axis="x", rotation=18)
    axes[0].set_ylabel("|tail creep slope| (µm/s; log)")
    axes[1].set_xlabel("Friction solreffriction time constant")
    axes[-1].legend(fontsize=8)
    path = BASE / "creep-vs-solreffriction-impratio.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_matrix(rows: list[dict]) -> Path:
    fig, axes = plt.subplots(3, 3, figsize=(13.5, 10), sharex=True, sharey=True, constrained_layout=True)
    tfr = [None, 4.0, 8.0]
    labels = ["inherit (2ms)", "4ms", "8ms"]
    tones = ["C0", "C1", "C2"]
    for i, dt in enumerate((0.25, 0.5, 1.0)):
        for j, impratio in enumerate((1.0, 10.0, 100.0)):
            axis = axes[i, j]
            for timeconst, label, color in zip(tfr, labels, tones):
                values = []
                for ns in (0, 1, 3):
                    row = next(row for row in rows if row["dt_ms"] == dt and row["impratio"] == impratio and row["noslip_iterations"] == ns and row["friction_timeconst_ms"] == timeconst)
                    values.append(abs(row["tail_slope_um_s"]) if row["valid_steady_contact"] else np.nan)
                axis.plot([0, 1, 3], values, marker="o", linewidth=1.8, color=color, label=label)
            axis.set_yscale("log")
            axis.grid(alpha=0.25)
            axis.set_title(f"dt={dt:g} ms, ir={impratio:g}", fontsize=10)
            if i == 2:
                axis.set_xlabel("NoSlip iterations")
            if j == 0:
                axis.set_ylabel("|creep| (µm/s)")
    axes[0, 2].legend(fontsize=7)
    path = BASE / "creep-ablation-matrix.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def main() -> None:
    metadata = json.loads((BASE / "metadata.json").read_text())
    rows = [analyze(item) for item in metadata]
    rows.sort(key=lambda row: (row["dt_ms"], -1 if row["friction_timeconst_ms"] is None else row["friction_timeconst_ms"], row["impratio"], row["noslip_iterations"]))
    payload = {
        "definition": {
            "force_N": 4.0,
            "mu": MU,
            "coulomb_limit_N": MU * EXPECTED_FN,
            "normal_solref": [0.002, 1.0],
            "normal_contact_controls_fixed": True,
            "tail_window_s": [TAIL_START, 10.5],
            "creep_estimator": "OLS slope of middle-top relative x position over final 2 seconds",
            "valid_contact_gate": "tail middle-top contact duty >= 99%",
            "solreffriction_note": "Elliptic cone only; positive format's dampratio is ignored because friction residual r=0.",
        },
        "cases": rows,
    }
    (BASE / "summary.json").write_text(json.dumps(payload, indent=2) + "\n")
    with (BASE / "top-interface-summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    first = plot_no_noslip(rows)
    second = plot_matrix(rows)
    print(json.dumps({"summary": str(BASE / "summary.json"), "plots": [str(first), str(second)]}, indent=2))


if __name__ == "__main__":
    main()
