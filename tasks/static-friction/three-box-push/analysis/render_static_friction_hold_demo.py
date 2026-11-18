"""Render a trace-driven 4-N sub-Coulomb friction-hold comparison.

The animation is deliberately a telemetry replay, not a synthetic engine
render: displacement, velocity, normal force and friction utilization are
read directly from the recorded three-box traces.
"""
from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import animation, patches
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MJC = ROOT / "exp-results/frictional-stack-v3/force-holds/mujoco-native/tc1ms_ir10_ns3/trace_F_4N.csv"
SAP = ROOT / "exp-results/frictional-stack-v3/force-holds/sap/trace_F_4N.csv"
OUT = ROOT / "exp-results/site-video-demos/04_static_friction_hold_trace_replay.mp4"


def load(path: Path) -> dict[str, np.ndarray]:
    with path.open() as file:
        rows = list(csv.DictReader(file))
    return {
        key: np.asarray([float(row[key]) if row[key] else np.nan for row in rows])
        for key in rows[0]
    }


def interp(data: dict[str, np.ndarray], key: str, t: np.ndarray) -> np.ndarray:
    return np.interp(t, data["time_s"], data[key])


def main() -> None:
    traces = {"MuJoCo (NoSlip=3 control)": load(MJC), "SAP/Warp": load(SAP)}
    colors = {"MuJoCo (NoSlip=3 control)": "#f28e2b", "SAP/Warp": "#4e79a7"}
    # Force reaches 4 N at 2.5 s.  Show the full final 5.5 s hold window.
    t = np.linspace(2.5, 8.0, 330)
    fig = plt.figure(figsize=(13.5, 7.6), facecolor="#101820")
    grid = fig.add_gridspec(2, 2, height_ratios=(1.15, 1), hspace=0.35, wspace=0.24)
    scenes = [fig.add_subplot(grid[0, i]) for i in range(2)]
    ax_slip = fig.add_subplot(grid[1, 0])
    ax_rho = fig.add_subplot(grid[1, 1])
    axes = [*scenes, ax_slip, ax_rho]
    for ax in axes:
        ax.set_facecolor("#162434")
        ax.tick_params(colors="#dce6f2")
        for spine in ax.spines.values():
            spine.set_color("#55718d")
    fig.suptitle("Sub-Coulomb static-friction hold — trace-driven comparison", color="white", fontsize=18, fontweight="bold")
    fig.text(0.5, 0.94, "Three 1-kg boxes · μ=0.5 · 4-N horizontal hold (< 4.905-N top-interface Coulomb limit)", color="#c7d8ea", ha="center", fontsize=10)

    blocks = {}
    for ax, name in zip(scenes, traces):
        ax.set_xlim(-0.015, 0.035); ax.set_ylim(-0.02, 0.16)
        ax.plot([-0.02, 0.04], [0, 0], color="#c4d2df", lw=3)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_title(name, color="white", fontsize=13)
        rects = []
        for height in (0.01, 0.055, 0.10):
            rect = patches.Rectangle((0, height), 0.025, 0.035, facecolor=colors[name], edgecolor="white", lw=1.1)
            ax.add_patch(rect); rects.append(rect)
        ax.arrow(-0.012, 0.118, 0.010, 0, color="#e15759", width=0.0012, head_width=0.008, length_includes_head=True)
        ax.text(-0.014, 0.135, "4 N", color="#e15759", fontsize=10)
        blocks[name] = rects

    ax_slip.set_title("Middle–top relative slip", color="white", fontsize=13)
    ax_slip.set_xlabel("hold time (s)", color="#dce6f2"); ax_slip.set_ylabel("x_rel (mm)", color="#dce6f2")
    ax_rho.set_title("Middle–top friction utilization", color="white", fontsize=13)
    ax_rho.set_xlabel("hold time (s)", color="#dce6f2"); ax_rho.set_ylabel("ρ = |Fₜ|/(μFₙ)", color="#dce6f2")
    for ax in (ax_slip, ax_rho): ax.set_xlim(0, t[-1] - t[0])
    ax_rho.set_ylim(0, 1.05); ax_rho.axhline(1.0, color="#c4d2df", ls="--", lw=1)

    slip_lines, rho_lines = {}, {}
    for name, data in traces.items():
        shift = interp(data, "middle_top_slip_m", t)
        rel = (shift - shift[0]) * 1000.0
        rho = interp(data, "middle_top_rho", t)
        x = interp(data, "ground_bottom_slip_m", t)
        data["_rel"], data["_rho"], data["_x"] = rel, rho, x
        slip_lines[name], = ax_slip.plot([], [], color=colors[name], lw=2.5, label=name)
        rho_lines[name], = ax_rho.plot([], [], color=colors[name], lw=2.5, label=name)
    ax_slip.set_ylim(-0.02, max(np.nanmax(v["_rel"]) for v in traces.values()) * 1.15 + 0.02)
    for ax in (ax_slip, ax_rho):
        leg = ax.legend(facecolor="#162434", edgecolor="#55718d", labelcolor="white", loc="upper left")
        for txt in leg.get_texts(): txt.set_color("white")
    clock = fig.text(0.5, 0.02, "", color="white", ha="center", fontsize=13)

    def update(frame: int):
        for name, data in traces.items():
            # Relative top-interface slip moves the upper box most; distribute
            # a small visual displacement across the stack for readability.
            dx = data["_rel"][frame] / 1000.0
            for idx, rect in enumerate(blocks[name]):
                rect.set_x(data["_x"][frame] + idx * dx / 2.0)
            slip_lines[name].set_data(t[:frame + 1] - t[0], data["_rel"][:frame + 1])
            rho_lines[name].set_data(t[:frame + 1] - t[0], data["_rho"][:frame + 1])
        clock.set_text(f"hold time: {t[frame] - t[0]:.2f} s")
        return [*sum(blocks.values(), []), *slip_lines.values(), *rho_lines.values(), clock]

    OUT.parent.mkdir(parents=True, exist_ok=True)
    animation.FuncAnimation(fig, update, frames=len(t), interval=1000 / 30, blit=False).save(OUT, writer="ffmpeg", fps=30, dpi=100)
    plt.close(fig)
    print(OUT)


if __name__ == "__main__":
    main()
