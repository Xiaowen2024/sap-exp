"""Render the force-matched simple-pinch near-boundary replay.

Each pane advances through its own recorded lift phase at the same playback
fraction.  This avoids pretending that the native controllers share a common
duration while retaining direct trace provenance for all plotted signals.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import csv
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import animation, patches
import numpy as np

ROOT = REPO_ROOT
MJC = ROOT / "exp-results/simple-pinch-v3/mujoco/matched-N4/c0p2mm_mu0p4_m0p3262/trace.csv"
SAP = ROOT / "exp-results/simple-pinch-v3/sap/matched-N4/c0p001mm_mu0p4_m0p3262/trace.csv"
OUT = ROOT / "exp-results/site-video-demos/05_simple_pinch_force_matched_boundary.mp4"


def load(path: Path) -> dict[str, np.ndarray]:
    with path.open() as file:
        rows = [row for row in csv.DictReader(file) if row["phase"] == "lift"]
    return {key: np.asarray([float(row[key]) if row[key] else np.nan for row in rows]) for key in rows[0] if key != "phase"}


def sample(data: dict[str, np.ndarray], key: str, u: np.ndarray) -> np.ndarray:
    return np.interp(u, np.linspace(0.0, 1.0, len(data[key])), data[key])


def main() -> None:
    traces = {"MuJoCo": load(MJC), "SAP/Warp": load(SAP)}
    colors = {"MuJoCo": "#f28e2b", "SAP/Warp": "#4e79a7"}
    u = np.linspace(0.0, 1.0, 330)
    fig = plt.figure(figsize=(13.5, 7.6), facecolor="#101820")
    grid = fig.add_gridspec(2, 2, height_ratios=(1.15, 1), hspace=.35, wspace=.24)
    scenes = [fig.add_subplot(grid[0, i]) for i in range(2)]
    ax_z, ax_rho = fig.add_subplot(grid[1, 0]), fig.add_subplot(grid[1, 1])
    for ax in [*scenes, ax_z, ax_rho]:
        ax.set_facecolor("#162434"); ax.tick_params(colors="#dce6f2")
        for spine in ax.spines.values(): spine.set_color("#55718d")
    fig.suptitle("Force-matched simple pinch — near-threshold lift boundary", color="white", fontsize=18, fontweight="bold")
    fig.text(0.5, .94, "m=0.3262 kg · μ=0.4 · measured preload ≈4 N/side · capacity ratio 2μN/(mg) ≈1", color="#c7d8ea", ha="center", fontsize=10)
    pads, cubes = {}, {}
    for ax, name in zip(scenes, traces):
        ax.set_xlim(-.07, .07); ax.set_ylim(-.01, .13); ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(name, color="white", fontsize=14)
        left = patches.Rectangle((-.055, .0), .022, .04, facecolor="#a5b7c8", edgecolor="white")
        right = patches.Rectangle((.033, .0), .022, .04, facecolor="#a5b7c8", edgecolor="white")
        cube = patches.Rectangle((-.02, .0), .04, .04, facecolor=colors[name], edgecolor="white", lw=1.2)
        for p in (left, right, cube): ax.add_patch(p)
        pads[name], cubes[name] = (left, right), cube
    ax_z.set_title("Cube vertical position", color="white", fontsize=13); ax_z.set_xlabel("native lift-phase progress", color="#dce6f2"); ax_z.set_ylabel("cube z (mm)", color="#dce6f2")
    ax_rho.set_title("Mean finger friction utilization", color="white", fontsize=13); ax_rho.set_xlabel("native lift-phase progress", color="#dce6f2"); ax_rho.set_ylabel("ρ", color="#dce6f2")
    ax_z.set_xlim(0,1); ax_z.set_ylim(-5,65); ax_rho.set_xlim(0,1); ax_rho.set_ylim(0,1.05); ax_rho.axhline(1,color="#c4d2df",ls="--",lw=1)
    lines_z, lines_rho = {}, {}
    for name, data in traces.items():
        data["_z"] = sample(data,"cube_z_m",u) * 1000
        data["_rho"] = .5 * (sample(data,"left_rho",u) + sample(data,"right_rho",u))
        data["_t"] = sample(data,"time_s",u)
        lines_z[name], = ax_z.plot([],[],color=colors[name],lw=2.5,label=name)
        lines_rho[name], = ax_rho.plot([],[],color=colors[name],lw=2.5,label=name)
    for ax in (ax_z,ax_rho):
        leg=ax.legend(facecolor="#162434",edgecolor="#55718d",labelcolor="white",loc="upper left")
        for txt in leg.get_texts():txt.set_color("white")
    clock=fig.text(.5,.02,"",color="white",ha="center",fontsize=12)
    def update(frame:int):
        for name,data in traces.items():
            z=data["_z"][frame]/1000
            cubes[name].set_y(z)
            # Pads follow the prescribed lift path visually; their relative
            # separation stays constant because preload is already calibrated.
            for pad in pads[name]: pad.set_y(z)
            lines_z[name].set_data(u[:frame+1],data["_z"][:frame+1])
            lines_rho[name].set_data(u[:frame+1],data["_rho"][:frame+1])
        clock.set_text("native lift-phase progress: %.0f%%  (MuJoCo %.2f s · SAP %.2f s)" % (100*u[frame], traces['MuJoCo']['_t'][frame], traces['SAP/Warp']['_t'][frame]))
        return [*cubes.values(), *sum(pads.values(),()), *lines_z.values(),*lines_rho.values(),clock]
    OUT.parent.mkdir(parents=True,exist_ok=True)
    animation.FuncAnimation(fig,update,frames=len(u),interval=1000/30,blit=False).save(OUT,writer="ffmpeg",fps=30,dpi=100)
    plt.close(fig); print(OUT)

if __name__=="__main__": main()
