"""Render the GAUGE bounce bridge report from frozen configuration rollouts."""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "exp-results" / "gauge-rigid-reproduction" / "bounce_bridge"
OUT = ROOT / "exp-results" / "gauge-rigid-reproduction" / "bounce_bridge"
REPORT = ROOT / "exp-report" / "gauge-bouncing-ball-impact-bridge.md"


def load(engine: str, config: str, dt: str):
    stem = f"{engine}_{config}_dt{dt}ms"
    return json.loads((BASE / f"{stem}.json").read_text()), np.load(BASE / f"{stem}.npz")


def main() -> None:
    cases = [
        ("MuJoCo default", "mujoco", "gauge_default", "5p55556"),
        ("SAP Warp default", "sap", "gauge_default", "5p55556"),
        ("MuJoCo impact-transfer", "mujoco", "impact_time_matched", "5p55556"),
        ("SAP Warp impact-transfer", "sap", "impact_time_matched", "5p55556"),
        ("MuJoCo repeated-transfer", "mujoco", "repeated_elastic", "5p55556"),
        ("SAP Warp repeated-transfer", "sap", "repeated_elastic", "5p55556"),
    ]
    loaded = [(label, *load(engine, config, dt)) for label, engine, config, dt in cases]
    real = loaded[0][2]["real_trials"]
    mean, std = real.mean(axis=0), real.std(axis=0, ddof=1)
    t = np.arange(len(mean)) / 30.0
    color = {"MuJoCo": "#d97706", "SAP Warp": "#2563eb"}
    style = {"default": ":", "impact": "-", "repeated": "--"}
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
    for ax, component, ylabel in ((axes[0], 2, "z position (m)"), (axes[1], 0, "x position (m)")):
        ax.fill_between(t, mean[:, component] - std[:, component], mean[:, component] + std[:, component], color="#9ca3af", alpha=.25, label="real mean ± 1σ" if component == 2 else None)
        ax.plot(t, mean[:, component], color="#111827", lw=2.5, label="real mean" if component == 2 else None)
        for label, result, trace in loaded:
            engine = "SAP Warp" if label.startswith("SAP") else "MuJoCo"
            flavor = "default" if "default" in label else ("impact" if "impact" in label else "repeated")
            ax.plot(t, trace["simulated"][:, component], color=color[engine], ls=style[flavor], lw=1.8, label=label if component == 2 else None)
        ax.set_xlabel("time (s)"); ax.set_ylabel(ylabel); ax.grid(alpha=.25)
    axes[0].set_title("Vertical bounce trajectory")
    axes[1].set_title("Lateral-motion diagnostic")
    axes[0].legend(fontsize=8, ncol=2)
    fig.suptitle("GAUGE bouncing ball: default vs frozen controlled-impact configurations", fontsize=14)
    figure = OUT / "gauge-bounce-bridge-trajectories.png"
    fig.savefig(figure, dpi=180); plt.close(fig)

    rows = []
    for label, result, _ in loaded:
        m = result["metrics"]
        rows.append((label, m["sim_rmse_mm"], m["sim_nrmse"], m["sim_dtw_mm"], m["sim_ndtw"], m["sim_first_apex_z_m"], m["sim_apex_e_proxy"]))
    native = [
        ("MuJoCo impact-transfer @ 0.025 ms", *[load("mujoco", "impact_time_matched", "0p025")[0]["metrics"][k] for k in ("sim_rmse_mm", "sim_nrmse", "sim_first_apex_z_m", "sim_apex_e_proxy")]),
        ("SAP Warp impact-transfer @ 0.025 ms", *[load("sap", "impact_time_matched", "0p025")[0]["metrics"][k] for k in ("sim_rmse_mm", "sim_nrmse", "sim_first_apex_z_m", "sim_apex_e_proxy")]),
        ("MuJoCo repeated-transfer @ 0.5 ms", *[load("mujoco", "repeated_elastic", "0p5")[0]["metrics"][k] for k in ("sim_rmse_mm", "sim_nrmse", "sim_first_apex_z_m", "sim_apex_e_proxy")]),
        ("SAP Warp repeated-transfer @ 0.5 ms", *[load("sap", "repeated_elastic", "0p5")[0]["metrics"][k] for k in ("sim_rmse_mm", "sim_nrmse", "sim_first_apex_z_m", "sim_apex_e_proxy")]),
    ]
    baseline = loaded[0][1]["metrics"]
    text = f"""# GAUGE bouncing ball — bridge to controlled impact benchmarks

## Question

Does a parameter set that behaves well in our controlled dynamic-impact or repeated-elastic-impact tests transfer to GAUGE's real bouncing-ball trajectory?

**Short answer: no configuration transfers cleanly without calibration to this real setup.** This is informative rather than surprising: the controlled tests isolate numerical contact behavior, while GAUGE evaluates sim-to-real agreement of a particular rubber ball and physical board.

## Protocol

This follows the publicly documented GAUGE engine-track definition as closely as the released materials permit: one simulation begins from the **mean released frame-0 position with zero velocity** and is compared to the mean 3-D trajectory of all 22 released trials at 30 Hz. Physics is stepped at GAUGE's stated 180 Hz (`dt=5.556 ms`). RMSE is frame-aligned and DTW is normalized by the optimal path length. The paper states 20 trials and reports an RMSE baseline of 5.29 mm / DTW baseline of 4.23 mm; the public directory contains 22 files but no published selection list/scorer. Our directly computed 22-trial baselines are **{baseline['real_baseline_rmse_mm_mean']:.2f} ± {baseline['real_baseline_rmse_mm_std']:.2f} mm RMSE** and **{baseline['real_baseline_dtw_mm_mean']:.2f} ± {baseline['real_baseline_dtw_mm_std']:.2f} mm DTW**, so results below are a public-data bridge, not a claim of exact Table-3 reproduction.

The default track has no contact tuning: MuJoCo uses its native default Newton/Euler contact; SAP Warp uses the literal configuration used in our GAUGE reconstruction (`approx32`, shape `ke=2500 N/m`, shape `tau=10 ms`, `rigid_gap=0`).

The two transfer tracks are frozen **before** looking at GAUGE trajectories:

- **Dynamic impact transfer:** parameters from our 1-kg, frictionless, one-DOF sphere--plane benchmark calibrated to `e=0.8`, `t_c=30 ms`: MuJoCo direct `solref=(-11027.6,-15.6294)`, constant `solimp=0.9`, `implicitfast` + Newton; SAP Warp `drake`, shape `ke=20000 N/m`, shape `tau=0.7305 ms` (pair `ke=10000 N/m`, pair `tau=1.4609 ms`).
- **Repeated elastic transfer:** parameters from the 49-ball SimBenchmark replay: MuJoCo direct `solref=(-10000,0)`, `solimp=.999`; SAP Warp `approx32`, shape `ke=10000 N/m`, `tau=0`.

![Mean trajectory and frozen transfer tracks](../exp-results/gauge-rigid-reproduction/bounce_bridge/gauge-bounce-bridge-trajectories.png)

## GAUGE-protocol result (`dt=5.556 ms`)

| Configuration | RMSE (mm) | nRMSE vs 22-trial baseline | DTW (mm) | nDTW | sampled apex z (m) | apex-e proxy |
|---|---:|---:|---:|---:|---:|---:|
"""
    for label, rmse, nrmse, dtw, ndtw, apex, e in rows:
        text += f"| {label} | {rmse:.2f} | {nrmse:.2f} | {dtw:.2f} | {ndtw:.2f} | {apex:.3f} | {e:.3f} |\n"
    text += f"""

The real mean has first sampled apex `z={baseline['real_mean_first_apex_z_m']:.3f} m` and apex-derived `e≈{baseline['real_apex_e_proxy']:.3f}`. Under the paper's 180-Hz protocol:

- **Defaults reproduce GAUGE's qualitative finding.** Both under-bounce severely: MuJoCo RMSE 117.6 mm and SAP Warp 119.0 mm; both are about 20× the released-data RMSE baseline.
- **The controlled time-matched impact configuration helps MuJoCo** (RMSE 67.1 mm, `e≈0.793`), but not SAP Warp (115.3 mm, `e≈0.296`). Its synthetic target was `e=0.8`, which is already higher than the real bounce proxy `≈0.724`.
- **The repeated-elastic configuration helps SAP Warp most** (72.2 mm), but its target is nearly elastic and its `e≈0.580` still under-bounces the real trial. It makes MuJoCo over-bounce (`e≈0.953`).
- None approaches the real experiment's repeatability. This is expected because no configuration was fitted to the GAUGE ball--board pair.

## Native-resolution check

This check is not GAUGE protocol: it retains the frozen parameters but restores each source benchmark's timestep to separate time-resolution effects from constitutive/geometry mismatch.

| Frozen source configuration | RMSE (mm) | nRMSE | sampled apex z (m) | apex-e proxy |
|---|---:|---:|---:|---:|
"""
    for label, rmse, nrmse, apex, e in native:
        text += f"| {label} | {rmse:.2f} | {nrmse:.2f} | {apex:.3f} | {e:.3f} |\n"
    text += """

Restoring the native timestep does **not** make either family match GAUGE: the repeated-elastic settings over-bounce in both engines, while the time-matched SAP Warp setting still under-bounces. Thus the mismatch is not reducible to GAUGE's 180-Hz timestep.

## Why the controlled and real-data conclusions differ

| Controlled benchmark | What it establishes | Missing from GAUGE transfer |
|---|---|---|
| Dynamic impact / restitution | After matching `e` and contact duration, how a solver transfers restitution across timestep and velocity in a single frictionless normal DOF. | GAUGE has a 121-g rubber **mesh** ball, gravity, free 6-DOF motion, a physical board simplified to a plane, and a measured ball--board restitution that is not necessarily the synthetic target. |
| Repeated elastic impact | Long-horizon energy retention under deliberately near-elastic repeated contacts. | GAUGE's rubber impact is not an elastic-energy-conservation target; correct real behavior requires the correct dissipative contact law, not maximal rebound retention. |
| GAUGE sim-to-real bounce | Whether a reconstructed scene matches measured 3-D trajectory. | It mixes contact constitutive error, collision geometry/board compliance, release-time and initial-velocity alignment, 180-Hz discretization, and sampling/trajectory error. |

The GAUGE metadata restitution is `0.576733`, obtained with a collision-based air-bearing measurement. The release-ball trajectory's apex proxy is about `0.724`; these need not coincide because they describe different contact pairs and measurement conditions. This makes “set one scalar restitution and expect trajectory match” invalid.

## Defensible conclusion

GAUGE and the synthetic benchmarks are complementary:

1. The synthetic tests identify **numerical/contact-model transfer** under controlled conditions; they show that parameters are not universally transferable across timestep and impact regime.
2. GAUGE adds the necessary external test: even a synthetic configuration with the desired restitution or energy behavior can fail sim-to-real because it does not identify the real ball--board contact and release protocol.
3. The fair next experiment is a two-stage real-data procedure: estimate incoming/outgoing velocity and contact-time proxies from a **calibration subset** of GAUGE trials, fit each engine only on that subset, freeze settings, then report Eq.-(2) RMSE/DTW and apex metrics on held-out trials. This would test parameter transfer to real repetitions without grading a fit on its own calibration data.

## Reproducibility

- bridge runner: [`run_bounce_bridge.py`](../exp-results/gauge-rigid-reproduction/run_bounce_bridge.py)
- output JSON/trace data: [`bounce_bridge`](../exp-results/gauge-rigid-reproduction/bounce_bridge)
- controlled impact source: [`dynamic-impact-restitution-sap-vs-mujoco.md`](dynamic-impact-restitution-sap-vs-mujoco.md)
- GAUGE reconstruction context: [`gauge-seven-rigid-body-mujoco-vs-sap.md`](gauge-seven-rigid-body-mujoco-vs-sap.md)
"""
    REPORT.write_text(text)
    print(REPORT)


if __name__ == "__main__":
    main()
