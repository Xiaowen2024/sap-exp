"""Render the completed MuJoCo--SAP bouncing speed--accuracy PNG."""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "plots" / "bouncing-speed-accuracy.png"


def load(path: Path) -> list[dict[str, float]]:
    with path.open(newline="") as f:
        return [{key: (float(value) if key not in {"engine", "contact", "preset"} else value)
                 for key, value in row.items()} for row in csv.DictReader(f)]


def draw(rows: list[dict[str, float]], *, label: str, color: str, marker: str, ax_error, ax_energy) -> None:
    x = [r["simulated_seconds_per_wall_second"] for r in rows]
    mse = [r["energy_mse_J2"] for r in rows]
    energy = [r["final_energy_ratio"] for r in rows]
    labels = [f"{r['dt_s'] * 1e3:g} ms" for r in rows]
    ax_error.scatter(x, mse, color=color, marker=marker, s=72, label=label, zorder=3)
    ax_energy.scatter(x, energy, color=color, marker=marker, s=72, label=label, zorder=3)
    for xi, yi, text in zip(x, mse, labels): ax_error.annotate(text, (xi, yi), xytext=(5, 5), textcoords="offset points", fontsize=9)
    for xi, yi, text in zip(x, energy, labels): ax_energy.annotate(text, (xi, yi), xytext=(5, 5), textcoords="offset points", fontsize=9)


def main() -> None:
    mujoco = load(ROOT / "mujoco-speed-accuracy" / "results.csv")
    sap = load(ROOT / "sap-speed-accuracy" / "results.csv")
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13, 4.8), constrained_layout=True)
    draw(mujoco, label="MuJoCo: direct solref=[-1e4, 0]", color="#2563eb", marker="o", ax_error=ax0, ax_energy=ax1)
    draw(sap, label="SAP: approx32, ke=1e4, tau=0", color="#dc2626", marker="s", ax_error=ax0, ax_energy=ax1)
    for ax in (ax0, ax1):
        ax.set_xscale("log"); ax.grid(True, which="both", alpha=.25); ax.set_xlabel("Pure stepping throughput (simulated s / wall s)")
        ax.legend(fontsize=9, loc="best")
    ax0.set_yscale("log"); ax0.set_ylabel("Mechanical-energy MSE (J²)"); ax0.set_title("Speed–accuracy: original energy metric")
    ax1.axhline(1.0, color="black", linewidth=1, linestyle="--", label="analytic E(T)/E₀ = 1")
    ax1.set_ylabel("Final mechanical-energy ratio E(20 s) / E₀"); ax1.set_ylim(0, 1.08); ax1.set_title("Long-horizon energy preservation")
    fig.suptitle("49-ball elastic bouncing benchmark — fixed native contact material", fontsize=14)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(OUT)


if __name__ == "__main__": main()
