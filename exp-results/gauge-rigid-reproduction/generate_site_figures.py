"""Generate figures for the GAUGE Real-to-Sim benchmark page (docs/real-to-sim.html)."""
from __future__ import annotations

import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT.parents[1] / "docs" / "assets"
sys.path.insert(0, str(ROOT))
import run_turntable_slope_slider_benchmark as bench

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
    "axes.edgecolor": "#c4cbd2",
    "axes.linewidth": 0.8,
})


def make_fig1():
    mj_t5 = json.loads((ROOT / "table5_full_mujoco_results.json").read_text())
    sap_t5 = json.loads((ROOT / "table5_full_sap_results.json").read_text())

    fig, axes = plt.subplots(2, 1, figsize=(11.2, 6.8), dpi=180)
    for idx, (fam, title) in enumerate([
        ("slope slider", "Slope Slider — 9 conditions (3 subtasks × 3 materials, nRMSE capped at 8.0)"),
        ("turntable", "Turntable — 9 conditions (3 subtasks × 3 materials, nRMSE capped at 8.0)"),
    ]):
        ax = axes[idx]
        mj_rows = [r for r in mj_t5 if r["family"] == fam]
        sap_rows = [r for r in sap_t5 if r["family"] == fam]
        labels = [f"{r['subtask']}\n({r['material']})" for r in mj_rows]
        x = np.arange(len(labels))
        w = 0.16

        isaac = [min(r["paper_isaac_nrmse"], 8.0) for r in mj_rows]
        genesis = [min(r["paper_genesis_nrmse"], 8.0) for r in mj_rows]
        newton = [min(r["paper_newton_nrmse"], 8.0) for r in mj_rows]
        mj_val = [min(r["mj_default_6dof"]["nrmse"], 8.0) for r in mj_rows]
        sap_val = [min(r["sap_drake_1e5_6dof"]["nrmse"], 8.0) for r in sap_rows]

        ax.bar(x - 2 * w, isaac, w, label="Paper: Isaac Sim", color="#8c9ba5", alpha=0.88)
        ax.bar(x - 1 * w, genesis, w, label="Paper: Genesis", color="#b0bec5", alpha=0.88)
        ax.bar(x + 0 * w, newton, w, label="Paper: Newton (v1.3.0)", color="#e57373", alpha=0.88)
        ax.bar(x + 1 * w, mj_val, w, label="Ours: MuJoCo Default", color="#1f608f")
        ax.bar(x + 2 * w, sap_val, w, label="Ours: SAP Warp (drake)", color="#d9822b")

        for i, r in enumerate(mj_rows):
            if r["paper_newton_nrmse"] > 8.0:
                ax.text(
                    x[i], 7.55, f"{r['paper_newton_nrmse']:.1f}×",
                    ha="center", va="bottom", fontsize=7.5, color="#b71c1c", fontweight="bold"
                )

        ax.axhline(1.0, color="#2e7d32", linestyle="--", linewidth=1.2, label="Human trial variance (nRMSE = 1.0)")
        ax.set_ylabel("Normalized RMSE (nRMSE)", fontsize=9.5, fontweight="bold", color="#252b32")
        ax.set_title(title, fontsize=10.5, fontweight="bold", color="#252b32", loc="left", pad=8)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=8.5, color="#252b32")
        ax.set_ylim(0, 8.4)
        ax.grid(axis="y", linestyle=":", alpha=0.55, color="#c4cbd2")
        if idx == 0:
            ax.legend(loc="upper right", ncol=3, fontsize=8, frameon=True, facecolor="#f8fafb", edgecolor="#d9dee3")

    fig.tight_layout(pad=1.5)
    fig.savefig(ASSETS / "gauge-table5-18cases-comparison.png")
    plt.close(fig)
    print("Saved gauge-table5-18cases-comparison.png")


def make_fig2():
    sap_trajs = json.loads((ROOT / "sap_3trajs.json").read_text())

    fig, axes = plt.subplots(1, 3, figsize=(11.8, 3.8), dpi=180)

    # Panel A: Slope Slider task-3 plastic (X displacement vs time)
    spec_a = bench.TABLE5_SPECS[7]
    info_a = bench.load_case_data(spec_a)
    t_a = np.arange(info_a["frames"]) / 30.0
    ax = axes[0]
    for tr in info_a["cubes_arr"]:
        ax.plot(t_a, (tr[:, 0] - tr[0, 0]) * 1000.0, color="#90a4ae", alpha=0.25, linewidth=0.9)
    mean_dx = (info_a["mean_cube"][:, 0] - info_a["mean_cube"][0, 0]) * 1000.0
    ax.plot(t_a, mean_dx, color="#252b32", linewidth=2.0, label="Real Mocap Mean (20 trials)")
    mj_traj = np.array(bench.sim_mujoco(info_a, solref=(0.02, 1.0))["pred_traj_mm"])
    sap_traj = np.array(sap_trajs["slope3_plastic"])
    ax.plot(t_a, mj_traj[:, 0] - mj_traj[0, 0], color="#1f608f", linewidth=1.8, linestyle="--", label="MuJoCo Default (nRMSE=0.61)")
    ax.plot(t_a, sap_traj[:, 0] - sap_traj[0, 0], color="#d9822b", linewidth=1.8, linestyle="-.", label="SAP Warp Default (nRMSE=0.52)")
    ax.set_title("A. Slope Slider (task-3, plastic)", fontsize=9.8, fontweight="bold", color="#252b32", loc="left")
    ax.set_xlabel("Time (s)", fontsize=8.5)
    ax.set_ylabel("Sliding displacement Δx (mm)", fontsize=8.5)
    ax.grid(True, linestyle=":", alpha=0.5, color="#c4cbd2")
    ax.legend(loc="upper left", fontsize=7.2, frameon=True, facecolor="#f8fafb")

    # Panel B: Turntable task-3 metal (XY planar trajectory)
    spec_b = bench.TABLE5_SPECS[17]
    info_b = bench.load_case_data(spec_b)
    ax = axes[1]
    for tr in info_b["cubes_arr"]:
        ax.plot(tr[:, 0] * 1000.0, tr[:, 1] * 1000.0, color="#90a4ae", alpha=0.25, linewidth=0.9)
    ax.plot(info_b["mean_cube"][:, 0] * 1000.0, info_b["mean_cube"][:, 1] * 1000.0, color="#252b32", linewidth=2.0, label="Real Mocap Mean")
    mj_tr_b = np.array(bench.sim_mujoco(info_b, solref=(0.02, 1.0))["pred_traj_mm"])
    sap_tr_b = np.array(sap_trajs["turntable3_metal"])
    ax.plot(mj_tr_b[:, 0], mj_tr_b[:, 1], color="#1f608f", linewidth=1.8, linestyle="--", label="MuJoCo Default (nRMSE=0.81)")
    ax.plot(sap_tr_b[:, 0], sap_tr_b[:, 1], color="#d9822b", linewidth=1.8, linestyle="-.", label="SAP Warp Default (nRMSE=0.84)")
    ax.set_title("B. Turntable (task-3, metal — XY plane)", fontsize=9.8, fontweight="bold", color="#252b32", loc="left")
    ax.set_xlabel("World X (mm)", fontsize=8.5)
    ax.set_ylabel("World Y (mm)", fontsize=8.5)
    ax.grid(True, linestyle=":", alpha=0.5, color="#c4cbd2")
    ax.legend(loc="lower left", fontsize=7.2, frameon=True, facecolor="#f8fafb")

    # Panel C: Turntable task-2 metal — Cone artifact (pyramidal vs elliptic vs SAP)
    spec_c = bench.TABLE5_SPECS[14]
    info_c = bench.load_case_data(spec_c)
    t_c = np.arange(info_c["frames"]) / 30.0
    ax = axes[2]
    for tr in info_c["cubes_arr"]:
        disp = np.linalg.norm((tr - tr[0]) * 1000.0, axis=-1)
        ax.plot(t_c, disp, color="#90a4ae", alpha=0.25, linewidth=0.9)
    mean_disp_c = np.linalg.norm((info_c["mean_cube"] - info_c["mean_cube"][0]) * 1000.0, axis=-1)
    ax.plot(t_c, mean_disp_c, color="#252b32", linewidth=2.0, label="Real Mocap Mean (μ_s > μ_k)")

    mj_pyr = np.array(bench.sim_mujoco(info_c, solref=(0.02, 1.0), cone="pyramidal")["pred_traj_mm"])
    mj_ell = np.array(bench.sim_mujoco(info_c, solref=(0.02, 1.0), cone="elliptic")["pred_traj_mm"])
    sap_c = np.array(sap_trajs["turntable2_metal"])

    ax.plot(t_c, np.linalg.norm(mj_pyr - mj_pyr[0], axis=-1), color="#1f608f", linewidth=1.8, linestyle="--",
            label="MuJoCo pyramidal (13.6 mm, √2 cone)")
    ax.plot(t_c, np.linalg.norm(mj_ell - mj_ell[0], axis=-1), color="#7b1fa2", linewidth=1.8, linestyle=":",
            label="MuJoCo elliptic (32.1 mm, true cone)")
    ax.plot(t_c, np.linalg.norm(sap_c - sap_c[0], axis=-1), color="#d9822b", linewidth=1.8, linestyle="-.",
            label="SAP Warp elliptic (32.3 mm, true cone)")
    ax.set_title("C. Turntable (task-2, metal — Cone effect)", fontsize=9.8, fontweight="bold", color="#252b32", loc="left")
    ax.set_xlabel("Time (s)", fontsize=8.5)
    ax.set_ylabel("Displacement ||Δp|| (mm)", fontsize=8.5)
    ax.grid(True, linestyle=":", alpha=0.5, color="#c4cbd2")
    ax.legend(loc="upper left", fontsize=7.0, frameon=True, facecolor="#f8fafb")

    fig.tight_layout(pad=1.3)
    fig.savefig(ASSETS / "gauge-trajectories-real-vs-sim.png")
    plt.close(fig)
    print("Saved gauge-trajectories-real-vs-sim.png")


def make_fig3():
    # Exact 7x6 matched (k_pair, tau_pair) grid from §4.2 of gauge-turntable-slope-slider-deep-dive.md
    k_grid = [35.0, 100.0, 250.0, 500.0, 2500.0, 10000.0, 50000.0]
    tau_grid = [0.005, 0.010, 0.020, 0.040, 0.060, 0.100]

    # Exact SAP (drake) numbers from §4.2 Table (Turntable metal task-3)
    sap_mat = np.array([
        [21.62, 25.72, 23.21, 31.26, 23.03, 22.50],
        [ 0.99,  1.00,  1.00,  0.99,  1.00,  1.00],
        [ 0.90,  0.89,  0.88,  0.87,  0.87,  0.87],
        [ 0.86,  0.87,  0.86,  0.85,  0.85,  0.85],
        [ 0.85,  0.84,  0.84,  0.83,  0.84,  0.83],
        [ 0.89,  0.88,  0.86,  0.83,  0.84,  0.83],
        [ 0.95,  0.91,  0.88,  0.85,  0.84,  0.83],
    ])

    # Standard mode (tref, dref) mapped to same (k_pair, tau_pair) or direct mode with decoupling
    info = bench.load_case_data(bench.TABLE5_SPECS[17])
    m_eff = 0.070 / 4.0
    d_0 = 0.95
    mj_def_mat = np.zeros((len(k_grid), len(tau_grid)))
    mj_dec_mat = np.zeros((len(k_grid), len(tau_grid)))

    for i, k_p in enumerate(k_grid):
        for j, tau in enumerate(tau_grid):
            k_mj = k_p / (m_eff * d_0)
            b_mj = 2.0 / (d_0 * tau)
            r_mj_def = bench.sim_mujoco(info, solref=(-k_mj, -b_mj), impratio=1.0, cone="pyramidal")
            mj_def_mat[i, j] = r_mj_def["nrmse"]

            fixed = info["fixed"]
            frms = info["frames"]
            mu = info["mu_pair"]
            assets_dir = str(bench.ROOT / "assets")
            pf_str = " ".join(map(str, info["p_f"]))
            qf_str = " ".join(map(str, info["q_f_wxyz"]))
            pm_str = " ".join(map(str, info["p_m"]))
            qm_str = " ".join(map(str, info["q_m_wxyz"]))
            xml = f"""
            <mujoco>
              <compiler meshdir="{assets_dir}"/>
              <option timestep="{bench.DT}" gravity="0 0 -9.81" integrator="Euler" solver="Newton" impratio="1.0" cone="elliptic"/>
              <default><geom solref="{-k_mj} {-b_mj}"/></default>
              <asset><mesh name="cube_mesh" file="cube.obj"/><mesh name="fixed_mesh" file="{fixed}.obj"/></asset>
              <worldbody>
                <geom name="ground" type="plane" pos="0 0 0" size="2 2 .1" friction="{mu} 0 0"/>
                <body name="{fixed}" pos="{pf_str}" quat="{qf_str}">
                  <joint name="driver_joint" type="hinge" axis="0 0 1"/>
                  <geom name="fixed_geom" type="mesh" mesh="fixed_mesh" mass="{info['mass_f']}" friction="{mu} 0 0"/>
                </body>
                <body name="cube" pos="{pm_str}" quat="{qm_str}">
                  <freejoint name="cube_free"/>
                  <geom name="cube_geom" type="mesh" mesh="cube_mesh" mass="{info['mass_c']}" friction="{mu} 0 0"/>
                </body>
              </worldbody>
              <contact>
                <pair geom1="fixed_geom" geom2="cube_geom" condim="3" friction="{mu} {mu} 0 0 0" solref="{-k_mj} {-b_mj}" solreffriction="0.004 1.0"/>
              </contact>
            </mujoco>"""
            m = mujoco.MjModel.from_xml_string(xml)
            d = mujoco.MjData(m)
            cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube")
            mujoco.mj_forward(m, d)
            pred = []
            for step in range((frms - 1) * 6 + 1):
                ang, omg = bench.interp_hinge(info["mean_f_ang"], step)
                d.qpos[0] = ang
                d.qvel[0] = omg
                mujoco.mj_forward(m, d)
                if step % 6 == 0:
                    pred.append(d.xpos[cid].copy())
                if step < (frms - 1) * 6:
                    mujoco.mj_step(m, d)
            mj_dec_mat[i, j] = bench.eval_pred(info, np.asarray(pred))["nrmse"]

    fig, axes = plt.subplots(1, 3, figsize=(12.0, 4.2), dpi=180)
    mats = [
        (sap_mat, "A. SAP Warp drake (36/42 pass ≤ 1.00)\nNative decoupled R_t = σ w_i"),
        (mj_def_mat, "B. MuJoCo Coupled (6/42 pass ≤ 1.20)\nDefault pyramidal solref=(-k,-b)"),
        (mj_dec_mat, "C. MuJoCo Decoupled (19/42 pass ≤ 1.20)\nelliptic + pair solreffriction=(4ms,1.0)"),
    ]
    for idx, (mat, title) in enumerate(mats):
        ax = axes[idx]
        disp_mat = np.clip(mat, 0.7, 5.0)
        im = ax.imshow(disp_mat, cmap="RdYlGn_r", vmin=0.75, vmax=3.5, aspect="auto")
        ax.set_title(title, fontsize=9.3, fontweight="bold", color="#252b32", pad=8)
        ax.set_xticks(range(len(tau_grid)))
        ax.set_xticklabels([f"{int(t*1000)}" for t in tau_grid], fontsize=8)
        ax.set_xlabel("Relaxation time τ_pair (ms)", fontsize=8.5)
        if idx == 0:
            ax.set_yticks(range(len(k_grid)))
            ax.set_yticklabels([f"{int(k)}" for k in k_grid], fontsize=8)
            ax.set_ylabel("Pair stiffness k_pair (N/m)", fontsize=8.5)
        else:
            ax.set_yticks(range(len(k_grid)))
            ax.set_yticklabels(["" for _ in k_grid])

        for i in range(len(k_grid)):
            for j in range(len(tau_grid)):
                val = mat[i, j]
                txt = f"{val:.2f}" if val < 10.0 else ">10"
                col = "#ffffff" if val > 2.2 else "#1c242b"
                ax.text(j, i, txt, ha="center", va="center", fontsize=7.2, color=col, fontweight="bold" if val <= 1.20 else "normal")

    fig.tight_layout(pad=1.3)
    fig.savefig(ASSETS / "gauge-matched-grid-heatmaps.png")
    plt.close(fig)
    print("Saved gauge-matched-grid-heatmaps.png")


if __name__ == "__main__":
    make_fig1()
    make_fig2()
    make_fig3()
