#!/usr/bin/env python3
"""Round-socket v6 rim-rest: full-trajectory replay of the different final outcomes (diagnostic visualisation).

Each panel replays one diagnostic npz (diag_socket_rim_contacts_*.py, 5-ms peg poses) in the same MuJoCo scene with the
exact-bore socket drawn semi-transparent, side view along the tilt plane normal plus a top view.  Writes an mp4 and a
filmstrip png (rows = cases, columns = selected times).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams["font.family"] = ["PingFang SC", "Arial Unicode MS", "DejaVu Sans"]
ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "assets/round_socket_exact"
FILES = ["socket_base.obj"] + [f"socket_wall_{k:02d}.obj" for k in range(32)]
W, H = 520, 400


def scene_xml(radius_m):
    assets = "".join(f'<mesh name="m{i}" file="{f}"/>' for i, f in enumerate(FILES))
    pieces = "".join(f'<geom type="mesh" mesh="m{i}" rgba=".45 .5 .68 .35" contype="0" conaffinity="0"/>' for i in range(len(FILES)))
    return (f'<mujoco><compiler meshdir="{ASSETS}"/><visual><global offwidth="{W}" offheight="{H}"/><headlight ambient=".5 .5 .5" diffuse=".55 .55 .55"/></visual>'
            f'<asset><texture type="skybox" builtin="flat" rgb1=".97 .97 .97" rgb2=".97 .97 .97" width="16" height="16"/>{assets}</asset>'
            f'<worldbody><light pos="0 -0.3 0.5" dir="0 0.5 -1"/>{pieces}'
            f'<body name="peg" mocap="true"><geom type="cylinder" size="{radius_m} 0.025" rgba=".95 .5 .1 1" contype="0" conaffinity="0"/>'
            f'<geom type="box" pos="0 0 0.0252" size="{radius_m * 0.9} 0.0012 0.0003" rgba=".15 .15 .15 1" contype="0" conaffinity="0"/></body>'
            f'</worldbody></mujoco>')


def tilt_deg(q):
    w, x, y, z = q
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


class Rend:
    def __init__(self, radius_m, az):
        self.m = mujoco.MjModel.from_xml_string(scene_xml(radius_m)); self.d = mujoco.MjData(self.m); self.r = mujoco.Renderer(self.m, H, W)
        self.side = mujoco.MjvCamera(); self.side.lookat[:] = [0, 0, 0.042]; self.side.distance = 0.22; self.side.azimuth = 90 + math.degrees(az); self.side.elevation = -12
        self.top = mujoco.MjvCamera(); self.top.lookat[:] = [0, 0, 0.04]; self.top.distance = 0.24; self.top.azimuth = 90 + math.degrees(az); self.top.elevation = -89

    def __call__(self, pos, quat):
        self.d.mocap_pos[0] = pos; self.d.mocap_quat[0] = quat; mujoco.mj_forward(self.m, self.d)
        out = []
        for cam in (self.side, self.top):
            self.r.update_scene(self.d, camera=cam); out.append(self.r.render().copy())
        return out


def state_text(p, q):
    z, tl, rad = p[2] * 1e3, tilt_deg(q), math.hypot(p[0], p[1]) * 1e3
    if z < -20:
        return "已从底座边缘掉下去（场景没有地面，一直往下落）"
    if tl < 5 and abs(z - 65) < 1:
        return "直立在孔口台面上"
    if tl > 80 and abs(z - 51.5) < 1.5:
        return "横躺在孔口上（销长 50 mm 比孔径 24 mm 长，掉不进去）"
    if 30 < tl < 70 and z > 40:
        return f"倾角 {tl:.0f}°：下端探进孔口，斜靠在孔边上卡住"
    return f"倾角 {tl:.0f}°，中心高 {z:.1f} mm"


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dir", type=Path, required=True); ap.add_argument("--cases", nargs="+", required=True, help="file::title")
    ap.add_argument("--out", required=True); ap.add_argument("--strip-times", type=float, nargs="+", default=[0, 0.1, 0.2, 0.25, 0.3, 0.5, 1.0, 3.0])
    a = ap.parse_args()
    cases = []
    for spec in a.cases:
        f, title = spec.split("::"); z = np.load(a.dir / f); meta = json.loads(str(z["meta"])); cell = meta["cell"]
        cases.append({"t": z["t"], "pos": z["pos"], "quat": z["quat"], "title": title, "cell": cell, "meta": meta,
                      "rend": Rend((12 - cell["clearance_mm"]) * 1e-3, math.radians(cell["azimuth_deg"]))})
    n = min(len(c["t"]) for c in cases); dt = cases[0]["t"][1] - cases[0]["t"][0]
    # ---- video
    fig, axs = plt.subplots(2, len(cases), figsize=(4.6 * len(cases), 8.4), gridspec_kw={"hspace": 0.12, "wspace": 0.04, "top": 0.86, "bottom": 0.06, "left": 0.01, "right": 0.99})
    ks = list(range(0, int(0.6 / dt) + 1, 2)) + list(range(int(0.6 / dt) + 10, n, 10)) + [n - 1]
    with imageio.get_writer(ROOT / a.dir / f"{a.out}.mp4" if not a.dir.is_absolute() else a.dir / f"{a.out}.mp4", fps=15, codec="libx264", quality=8, macro_block_size=8) as wr:
        for k in ks:
            for j, c in enumerate(cases):
                side, top = c["rend"](c["pos"][k], c["quat"][k])
                for i, img in enumerate((side, top)):
                    ax = axs[i][j]; ax.clear(); ax.imshow(img); ax.axis("off")
                axs[0][j].set_title(c["title"], fontsize=10.5)
                axs[1][j].text(0.5, -0.02, state_text(c["pos"][k], c["quat"][k]), transform=axs[1][j].transAxes, ha="center", va="top", fontsize=10,
                               color="#2b8a3e" if state_text(c["pos"][k], c["quat"][k]).startswith("直立") else "#c92a2a")
            slow = "慢放 0.15×" if k <= int(0.6 / dt) else "0.75×"
            fig.suptitle(f"rim 格完整轨迹（上：侧视；下：俯视）   t = {cases[0]['t'][k]:.2f} s   {slow}\n正确答案（解析）：销一直直立在原处不动", fontsize=12.5)
            fig.canvas.draw(); wr.append_data(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())
    plt.close(fig)
    # ---- filmstrip (side view only)
    ts = a.strip_times
    fig, axs = plt.subplots(len(cases), len(ts), figsize=(2.6 * len(ts), 2.35 * len(cases) + 0.6), gridspec_kw={"hspace": 0.28, "wspace": 0.02})
    for i, c in enumerate(cases):
        for j, t in enumerate(ts):
            k = min(n - 1, int(round(t / dt))); side, _ = c["rend"](c["pos"][k], c["quat"][k])
            ax = axs[i][j]; ax.imshow(side[40:-40, 40:-40]); ax.set_xticks([]); ax.set_yticks([])
            if i == 0:
                ax.set_title(f"t = {t:g} s", fontsize=11)
            if j == 0:
                ax.set_ylabel(c["title"].replace("：", "\n"), fontsize=9.5, rotation=0, ha="right", va="center", labelpad=8)
        axs[i][-1].text(1.03, 0.5, state_text(c["pos"][-1], c["quat"][-1]).replace("（", "\n（"), transform=axs[i][-1].transAxes, fontsize=9, va="center")
    fig.suptitle("rim 格完整轨迹关键帧（侧视，沿倾倒平面的法向看）；正确答案：销一直直立在原处", fontsize=12)
    fig.savefig(a.dir / f"{a.out}_filmstrip.png", dpi=110, bbox_inches="tight"); print("wrote", a.out)


if __name__ == "__main__":
    main()
