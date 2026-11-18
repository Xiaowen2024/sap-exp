"""Summarize the gated and normal-compliance-matched three-box friction study."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "exp-results" / "frictional-stack-v3"
INTERFACES = ("ground_bottom", "bottom_middle", "middle_top")
FORCE_LEVELS = (0.5, 1.0, 2.0, 3.0, 4.0, 4.5, 4.9)
DT = 0.0005
SETTLE = 1.5


def load_trace(path: Path) -> dict[str, np.ndarray]:
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}


def first_continuous(mask: np.ndarray, samples: int, eligible: np.ndarray) -> int | None:
    run = np.convolve(mask.astype(np.int32), np.ones(samples, dtype=np.int32), mode="valid")
    indices = np.flatnonzero((run == samples) & eligible[: len(run)])
    return int(indices[0]) if len(indices) else None


def analyze_trace(engine: str, path: Path) -> dict:
    data = load_trace(path)
    t, command = data["time_s"], data["force_command_N"]
    pre = (t >= 1.0) & (t <= SETTLE)
    result = {"engine": engine, "path": str(path.relative_to(ROOT)), "normal_preforce": {}, "interfaces": {}}
    for body in range(3):
        key = f"box_{body}_vz_m_s"
        if key in data:
            result.setdefault("body_vz_rms_mm_s", []).append(float(np.sqrt(np.mean(data[key][pre] ** 2)) * 1e3))
    for name in INTERFACES:
        fn = data[f"{name}_Fn_N"]
        count = data[f"{name}_contacts"]
        slip = np.abs(data[f"{name}_slip_m"])
        velocity = np.abs(data[f"{name}_vt_m_s"])
        rho = data[f"{name}_rho"]
        normal = {
            "contacts_mean": float(np.mean(count[pre])),
            "contact_duty": float(np.mean(count[pre] > 0)),
            "Fn_mean_N": float(np.mean(fn[pre])),
            "Fn_std_N": float(np.std(fn[pre])),
            "Fn_cv": float(np.std(fn[pre]) / np.mean(fn[pre])),
        }
        gap_key = f"{name}_gap_m"
        if gap_key in data:
            normal.update({
                "gap_mean_mm": float(np.mean(data[gap_key][pre]) * 1e3),
                "gap_std_um": float(np.std(data[gap_key][pre]) * 1e6),
            })
        result["normal_preforce"][name] = normal

        snapshots = []
        for force in FORCE_LEVELS:
            window = (t >= SETTLE) & (np.abs(command - force) <= 0.01)
            snapshots.append({
                "force_N": force,
                "slip_mm": float(np.mean(slip[window]) * 1e3),
                "vt_abs_mean_mm_s": float(np.mean(velocity[window]) * 1e3),
                "rho_mean": float(np.nanmean(rho[window])) if np.any(np.isfinite(rho[window])) else None,
                "contact_duty": float(np.mean(count[window] > 0)),
            })

        thresholds = {}
        for threshold_mm in (0.1, 1.0, 5.0):
            indices = np.flatnonzero((t >= SETTLE) & (slip >= threshold_mm * 1e-3))
            if len(indices):
                index = int(indices[0])
                thresholds[str(threshold_mm)] = {
                    "force_N": float(command[index]),
                    "vt_abs_mm_s": float(velocity[index] * 1e3),
                    "rho": float(rho[index]) if np.isfinite(rho[index]) else None,
                    "contacts": int(count[index]),
                }
            else:
                thresholds[str(threshold_mm)] = None

        velocity_onsets = {}
        for threshold_mm_s in (1.0, 10.0):
            samples = round(0.05 / DT)
            index = first_continuous(
                (velocity >= threshold_mm_s * 1e-3) & (count > 0), samples, t >= SETTLE
            )
            velocity_onsets[str(threshold_mm_s)] = None if index is None else {
                "force_N": float(command[index]),
                "slip_mm": float(slip[index] * 1e3),
                "rho": float(rho[index]) if np.isfinite(rho[index]) else None,
                "contacts": int(count[index]),
                "window_contact_duty": float(np.mean(count[index : index + samples] > 0)),
            }

        result["interfaces"][name] = {
            "force_snapshots": snapshots,
            "displacement_thresholds": thresholds,
            "velocity_onsets_50ms": velocity_onsets,
        }
    return result


def main() -> None:
    gate = json.loads((BASE / "sap-normal-gate" / "summary.json").read_text())
    sap = analyze_trace("sap_warp", BASE / "sap-friction-kpair-1e4" / "trace.csv")
    mujoco = analyze_trace("mujoco", BASE / "mujoco-matched-kpair-1e4" / "trace.csv")
    summary = {
        "normal_gate_definition": {
            "tail_s": 1.0,
            "contacts_mean_min": 3.95,
            "contact_duty_min": 0.999,
            "Fn_relative_error_max": 0.01,
            "Fn_cv_max": 0.01,
            "body_vz_rms_mm_s_max": 0.1,
            "gap_std_um_max": 1.0,
            "truncated_contacts_required": 0,
        },
        "sap_normal_gate": gate,
        "friction_metric_definition": {
            "force_snapshot_half_width_N": 0.01,
            "low_velocity_diagnostic": "|vt| >= 1 mm/s continuously for 50 ms; not treated as breakaway",
            "macroscopic_sustained_slip_proxy": "|vt| >= 10 mm/s continuously for 50 ms while interface contact exists",
        },
        "friction_ramps": [mujoco, sap],
    }
    (BASE / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
