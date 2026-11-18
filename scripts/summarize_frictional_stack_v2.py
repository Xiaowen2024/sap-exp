"""Summarize synchronized MuJoCo/SAP three-box friction-ramp traces."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "exp-results" / "frictional-stack-v2"
INTERFACES = ("ground_bottom", "bottom_middle", "middle_top")
EXPECTED_FN = {"ground_bottom": 29.43, "bottom_middle": 19.62, "middle_top": 9.81}
SETTLE = 1.5


def first_row(frame: pd.DataFrame, mask: pd.Series, columns: list[str]) -> dict | None:
    hit = frame[mask]
    if hit.empty:
        return None
    row = hit.iloc[0]
    return {column: float(row[column]) for column in columns}


def finite_stats(values: pd.Series) -> tuple[float, float]:
    array = values.to_numpy(dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return float("nan"), float("nan")
    return float(np.mean(array)), float(np.max(array))


def json_clean(value):
    if isinstance(value, dict):
        return {key: json_clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_clean(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def summarize(engine: str) -> dict:
    trace = pd.read_csv(BASE / engine / "trace.csv")
    metadata = json.loads((BASE / engine / "metadata.json").read_text())
    settle = trace[(trace.time_s >= 1.0) & (trace.time_s < SETTLE)]
    result: dict = {"engine": engine, "metadata": metadata, "settle_interfaces": {}, "displacement_thresholds": {}, "rho_thresholds": {}}

    for name in INTERFACES:
        fn = settle[f"{name}_Fn_N"]
        contacts = settle[f"{name}_contacts"]
        result["settle_interfaces"][name] = {
            "Fn_expected_N": EXPECTED_FN[name],
            "Fn_mean_N": float(fn.mean()),
            "Fn_std_N": float(fn.std(ddof=0)),
            "Fn_mean_relative_error": float(abs(fn.mean() - EXPECTED_FN[name]) / EXPECTED_FN[name]),
            "contacts_mean": float(contacts.mean()),
            "contact_duty": float((contacts > 0).mean()),
        }

    for threshold_mm in (0.1, 1.0, 5.0):
        threshold_m = threshold_mm / 1000
        hit = first_row(
            trace,
            (trace.time_s >= SETTLE) & (trace.middle_top_slip_m.abs() >= threshold_m),
            ["time_s", "force_command_N", "middle_top_Fn_N", "middle_top_Ft_N", "middle_top_rho", "middle_top_point_rho_max", "middle_top_contacts"],
        )
        result["displacement_thresholds"][f"{threshold_mm:g}_mm"] = hit

    creep = result["displacement_thresholds"]["0.1_mm"]
    creep_time = creep["time_s"] if creep else trace.time_s.max()
    pre = trace[(trace.time_s >= SETTLE) & (trace.time_s < creep_time)]
    pre_tail = pre[pre.time_s >= max(SETTLE, creep_time - 0.1)]
    pre_mean, pre_max = finite_stats(pre.middle_top_rho)
    pre_tail_mean, pre_tail_max = finite_stats(pre_tail.middle_top_rho)
    result["pre_creep_rho"] = {
        "mean": pre_mean,
        "max": pre_max,
        "last_100ms_mean": pre_tail_mean,
        "last_100ms_max": pre_tail_max,
    }

    for name in INTERFACES:
        aggregate = first_row(
            trace,
            (trace.time_s >= SETTLE) & (trace[f"{name}_rho"] >= 0.99),
            ["time_s", "force_command_N", f"{name}_rho", f"{name}_slip_m", f"{name}_contacts"],
        )
        point = first_row(
            trace,
            (trace.time_s >= SETTLE) & (trace[f"{name}_point_rho_max"] >= 0.99),
            ["time_s", "force_command_N", f"{name}_point_rho_max", f"{name}_slip_m", f"{name}_contacts"],
        )
        result["rho_thresholds"][name] = {"aggregate_0.99": aggregate, "point_0.99": point}

    last = trace.iloc[-1]
    result["final_slip_mm"] = {name: float(last[f"{name}_slip_m"] * 1000) for name in INTERFACES}
    result["max_truncated_contacts"] = int(trace.truncated_contacts.max()) if "truncated_contacts" in trace else 0
    return result


def main() -> None:
    summary = json_clean({engine: summarize(engine) for engine in ("mujoco", "sap")})
    (BASE / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
