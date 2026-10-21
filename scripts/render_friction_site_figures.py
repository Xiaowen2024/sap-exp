"""Render two deliberately narrow, static figures for the public friction page.

The goal is to separate (1) the original matched-compliance baseline from
(2) MuJoCo's optional native anti-slip controls.  The latter is not presented
as a fair compliant-contact-model comparison.
"""
from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
HOLD = ROOT / "exp-results/frictional-stack-v3/force-holds"
OUT = ROOT / "docs/assets"
TOP = "middle_top"


def baseline() -> None:
    payload = json.loads((HOLD / "summary.json").read_text())
    cases = {(row["engine"], row["force_target_N"]): row for row in payload["cases"]}
    forces = np.array([1.0, 2.0, 3.0, 4.0])
    sap = np.array([
        cases[("sap", force)]["interfaces"][TOP]["tail_slope_mm_s"] * 1e3
        for force in forces
    ])
    mj = [cases[("mujoco", force)]["interfaces"][TOP] for force in forces]
    mj_valid = np.array([entry["valid_steady_contact"] for entry in mj])
    mj_creep = np.array([entry["tail_slope_mm_s"] * 1e3 for entry in mj])
    mj_loss = [entry["contact_loss_time_s"] for entry in mj]

    # Manual margins keep both the title and the explanatory line visible in
    # raster embeds; constrained_layout clips the title with this log plot.
    fig, axis = plt.subplots(figsize=(8.4, 5.2))
    fig.subplots_adjust(left=0.105, right=0.985, top=0.88, bottom=0.23)
    axis.plot(forces, sap, marker="s", lw=2.8, color="#e07a1f", label="SAP/Warp baseline")
    axis.plot(forces[mj_valid], mj_creep[mj_valid], marker="o", lw=2.8,
              color="#2878b5", label="MuJoCo matched-soft baseline")
    # A failed contact has no valid steady-creep metric.  It is shown as an
    # annotation, rather than assigning it an artificial creep value.
    for force, loss in zip(forces[~mj_valid], np.asarray(mj_loss, dtype=object)[~mj_valid]):
        axis.scatter(force, 550, marker="x", s=54, linewidths=1.8, color="#2878b5", zorder=4)
        axis.annotate(
            f"contact lost\n{loss:.3f} s",
            xy=(force, 550), xytext=(force, 180), ha="center", va="bottom",
            color="#2878b5", fontsize=9,
            arrowprops={"arrowstyle": "-", "color": "#2878b5", "lw": 1.2},
        )
    axis.set_yscale("log")
    axis.set_ylim(0.25, 1800)
    axis.set_xticks(forces)
    axis.set_xlabel("Held horizontal force (N)")
    axis.set_ylabel("Tail middle–top creep (µm/s; log scale)")
    axis.set_title("Matched-compliance baseline: SAP retains contact; MuJoCo soft baseline does not")
    axis.grid(alpha=0.25, which="both")
    axis.legend(loc="lower left")
    axis.text(
        0.5, -0.25,
        "A cross means no steady-creep value: the middle–top contact separated before the tail window.",
        transform=axis.transAxes, ha="center", fontsize=9, color="#475763",
    )
    fig.savefig(OUT / "friction-baseline-vs-sap.png", dpi=180)
    plt.close(fig)


def anti_slip() -> None:
    payload = json.loads((HOLD / "mujoco-native/summary.json").read_text())
    cases = {(row["configuration"], row["force_target_N"]): row for row in payload["cases"]}
    sap = payload["sap_4N_reference"]["tail_slope_um_s"]
    names = ["SAP/Warp\nbaseline", "MuJoCo native\nir=1", "MuJoCo native\nir=10", "MuJoCo native\nir=100", "MuJoCo native\nir=10 + NoSlip=3"]
    values = [
        sap,
        abs(cases[("native_1ms", 4.0)]["tail_slope_um_s"]),
        abs(cases[("impratio_10", 4.0)]["tail_slope_um_s"]),
        abs(cases[("impratio_100", 4.0)]["tail_slope_um_s"]),
        abs(cases[("noslip_3", 4.0)]["tail_slope_um_s"]),
    ]
    colors = ["#e07a1f", "#76a5d4", "#4f92c8", "#2878b5", "#275d38"]
    fig, axis = plt.subplots(figsize=(8.4, 4.8), constrained_layout=True)
    bars = axis.bar(range(len(values)), values, color=colors, width=0.68)
    axis.set_yscale("log")
    axis.set_ylim(1e-3, 300)
    axis.set_xticks(range(len(values)), names)
    axis.set_ylabel("Tail middle–top creep at 4 N (µm/s; log scale)")
    axis.set_title("4-N hold: MuJoCo anti-slip controls versus SAP baseline")
    axis.grid(axis="y", alpha=0.25, which="both")
    for bar, value in zip(bars, values):
        axis.text(bar.get_x() + bar.get_width() / 2, value * 1.35, f"{value:.4g}",
                  ha="center", va="bottom", fontsize=9)
    axis.text(
        0.5, -0.30,
        "All MuJoCo native bars retain normal contact. NoSlip is a friction-only post-process, not a matched contact-model setting.",
        transform=axis.transAxes, ha="center", fontsize=9, color="#475763",
    )
    fig.savefig(OUT / "friction-anti-slip-vs-sap.png", dpi=180)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    baseline()
    anti_slip()


if __name__ == "__main__":
    main()
