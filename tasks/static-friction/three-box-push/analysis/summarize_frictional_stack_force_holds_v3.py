"""Summarize constant-force hold traces and render the creep comparison plot."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = REPO_ROOT
BASE = ROOT / "exp-results" / "frictional-stack-v3" / "force-holds"
FORCES = (1.0, 2.0, 3.0, 4.0)
INTERFACES = ("ground_bottom", "bottom_middle", "middle_top")
HOLD_START, END, TAIL_START = 2.5, 10.5, 8.5


def read_trace(path: Path) -> dict[str, np.ndarray]:
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def continuous_loss_time(time: np.ndarray, contacts: np.ndarray, start: float, duration: float = 0.05) -> float | None:
    period = float(np.median(np.diff(time)))
    samples = max(1, round(duration / period))
    lost = contacts <= 0
    run = np.convolve(lost.astype(np.int32), np.ones(samples, dtype=np.int32), mode="valid")
    indices = np.flatnonzero((run == samples) & (time[: len(run)] >= start))
    return float(time[indices[0]]) if len(indices) else None


def regress(time: np.ndarray, position: np.ndarray) -> tuple[float, float]:
    slope, intercept = np.polyfit(time, position, 1)
    predicted = slope * time + intercept
    residual = float(np.sum((position - predicted) ** 2))
    total = float(np.sum((position - np.mean(position)) ** 2))
    return float(slope), 1.0 - residual / total if total > 0 else 1.0


def analyze_case(engine: str, force: float) -> dict:
    path = BASE / engine / f"trace_F_{force:g}N.csv"
    data = read_trace(path)
    time = data["time_s"]
    tail = time >= TAIL_START
    hold = time >= HOLD_START
    body_vz_rms = [
        float(np.sqrt(np.mean(data[f"box_{body}_vz_m_s"][tail] ** 2)) * 1e3)
        for body in range(3)
    ]
    result = {
        "engine": engine,
        "force_target_N": force,
        "trace": str(path.relative_to(ROOT)),
        "tail_body_vz_rms_mm_s": body_vz_rms,
        "tail_max_body_vz_rms_mm_s": max(body_vz_rms),
        "interfaces": {},
    }
    for name in INTERFACES:
        slip = data[f"{name}_slip_m"]
        velocity = data[f"{name}_vt_m_s"]
        fn = data[f"{name}_Fn_N"]
        ft = data[f"{name}_Ft_N"]
        rho = data[f"{name}_rho"]
        contacts = data[f"{name}_contacts"]
        slope, r2 = regress(time[tail], slip[tail])
        first_tail = (time >= TAIL_START) & (time < TAIL_START + 1.0)
        last_tail = time >= END - 1.0
        hold_start_index = int(np.flatnonzero(hold)[0])
        finite_rho = rho[tail][np.isfinite(rho[tail])]
        metric = {
            "tail_slope_mm_s": slope * 1e3,
            "tail_slope_R2": r2,
            "tail_vt_mean_mm_s": float(np.mean(velocity[tail]) * 1e3),
            "tail_vt_abs_mean_mm_s": float(np.mean(np.abs(velocity[tail])) * 1e3),
            "tail_vt_std_mm_s": float(np.std(velocity[tail]) * 1e3),
            "tail_vt_rms_mm_s": float(np.sqrt(np.mean(velocity[tail] ** 2)) * 1e3),
            "tail_vt_first_1s_mean_mm_s": float(np.mean(velocity[first_tail]) * 1e3),
            "tail_vt_last_1s_mean_mm_s": float(np.mean(velocity[last_tail]) * 1e3),
            "hold_displacement_mm": float((slip[-1] - slip[hold_start_index]) * 1e3),
            "final_slip_mm": float(slip[-1] * 1e3),
            "tail_Fn_mean_N": float(np.mean(fn[tail])),
            "tail_Fn_std_N": float(np.std(fn[tail])),
            "tail_Fn_cv": float(np.std(fn[tail]) / np.mean(fn[tail])) if np.mean(fn[tail]) > 0 else None,
            "tail_Ft_mean_N": float(np.mean(ft[tail])),
            "tail_Ft_std_N": float(np.std(ft[tail])),
            "tail_rho_mean": float(np.mean(finite_rho)) if len(finite_rho) else None,
            "tail_contact_duty": float(np.mean(contacts[tail] > 0)),
            "tail_contacts_mean": float(np.mean(contacts[tail])),
            "tail_gap_mean_mm": float(np.mean(data[f"{name}_gap_m"][tail]) * 1e3),
            "tail_gap_std_um": float(np.std(data[f"{name}_gap_m"][tail]) * 1e6),
            "contact_loss_time_s": continuous_loss_time(time, contacts, HOLD_START),
        }
        metric["valid_steady_contact"] = bool(metric["tail_contact_duty"] >= 0.99)
        metric["practically_significant_creep"] = bool(
            metric["valid_steady_contact"]
            and abs(metric["tail_slope_mm_s"]) >= 0.01
            and metric["tail_slope_R2"] >= 0.99
        )
        result["interfaces"][name] = metric
    return result


def render_plot(results: list[dict]) -> Path:
    plot_path = BASE / "steady-creep-comparison.png"
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), constrained_layout=True)
    for engine, label, marker in (("mujoco", "MuJoCo", "o"), ("sap", "SAP Warp", "s")):
        selected = [case for case in results if case["engine"] == engine]
        forces = [case["force_target_N"] for case in selected]
        values, invalid = [], []
        for case in selected:
            metric = case["interfaces"]["middle_top"]
            values.append(abs(metric["tail_slope_mm_s"]) * 1e3 if metric["valid_steady_contact"] else np.nan)
            invalid.append(not metric["valid_steady_contact"])
        axes[0].plot(forces, values, marker=marker, linewidth=2, label=label)
        invalid_label_used = False
        for force, is_invalid in zip(forces, invalid):
            if is_invalid:
                axes[0].scatter(
                    [force], [0.2], marker="x", s=70, color="C0",
                    label=f"{label}: contact lost" if not invalid_label_used else None,
                )
                invalid_label_used = True
    axes[0].set_yscale("log")
    axes[0].set_xlabel("Held horizontal force (N)")
    axes[0].set_ylabel("Absolute tail x-rel slope (µm/s, log scale)")
    axes[0].set_title("Steady-creep estimate (last 2 s)")
    axes[0].grid(alpha=0.25)
    axes[0].legend()

    for engine, label, marker in (("mujoco", "MuJoCo", "o"), ("sap", "SAP Warp", "s")):
        selected = [case for case in results if case["engine"] == engine]
        forces = [case["force_target_N"] for case in selected]
        displacement = [abs(case["interfaces"]["middle_top"]["hold_displacement_mm"]) for case in selected]
        axes[1].plot(forces, displacement, marker=marker, linewidth=2, label=label)
    axes[1].set_yscale("log")
    axes[1].set_xlabel("Held horizontal force (N)")
    axes[1].set_ylabel("Top-middle displacement over hold (mm)")
    axes[1].set_title("Accumulated motion over hold (includes failures)")
    axes[1].grid(alpha=0.25)
    axes[1].legend()
    fig.savefig(plot_path, dpi=180)
    plt.close(fig)
    return plot_path


def main() -> None:
    results = [analyze_case(engine, force) for engine in ("mujoco", "sap") for force in FORCES]
    output = {
        "definition": {
            "settle_s": 1.5,
            "force_ramp_s": 1.0,
            "force_hold_s": 8.0,
            "tail_window_s": 2.0,
            "sample_period_ms": 5.0,
            "primary_creep_estimator": "OLS slope of relative tangential displacement over final 2 s",
            "valid_contact_gate": "tail contact duty >= 99%",
            "practically_significant_creep_flag": "valid contact, |slope| >= 0.01 mm/s, and slope R2 >= 0.99",
        },
        "cases": results,
    }
    (BASE / "summary.json").write_text(json.dumps(output, indent=2) + "\n")
    compact_rows = []
    for case in results:
        metric = case["interfaces"]["middle_top"]
        compact_rows.append({
            "engine": case["engine"],
            "force_target_N": case["force_target_N"],
            "tail_slope_um_s": metric["tail_slope_mm_s"] * 1e3,
            "tail_slope_R2": metric["tail_slope_R2"],
            "tail_vt_mean_um_s": metric["tail_vt_mean_mm_s"] * 1e3,
            "tail_vt_std_um_s": metric["tail_vt_std_mm_s"] * 1e3,
            "hold_displacement_um": metric["hold_displacement_mm"] * 1e3,
            "tail_Fn_mean_N": metric["tail_Fn_mean_N"],
            "tail_Fn_cv": metric["tail_Fn_cv"],
            "tail_Ft_mean_N": metric["tail_Ft_mean_N"],
            "tail_rho_mean": metric["tail_rho_mean"],
            "tail_contact_duty": metric["tail_contact_duty"],
            "tail_contacts_mean": metric["tail_contacts_mean"],
            "tail_max_body_vz_rms_mm_s": case["tail_max_body_vz_rms_mm_s"],
            "contact_loss_time_s": metric["contact_loss_time_s"],
            "valid_steady_contact": metric["valid_steady_contact"],
        })
    with (BASE / "top-interface-summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=compact_rows[0])
        writer.writeheader()
        writer.writerows(compact_rows)
    plot = render_plot(results)
    print(json.dumps({"summary": str(BASE / "summary.json"), "plot": str(plot)}, indent=2))


if __name__ == "__main__":
    main()
