#!/usr/bin/env python3
"""Round-socket v6 rim-rest: SAP vs MuJoCo Warp support-polygon videos (diagnostic visualisation).

Input: the npz files of diag_socket_rim_contacts_{sap,mjwarp}.py (peg pose + every contact, 5 ms samples).
Per cell, one mp4 (+ a key-frame png) with two columns (SAP | MuJoCo) and three rows:
  1. 3D replay (semi-transparent exact-bore socket), contact points (red = carrying normal force, grey = in the contact
     set but zero force), analytic support edge (green bar on the rim top) and the CoM plumb line (black).
  2. Top view in the tilt-plane frame (x' = toward the peg's offset, y' perpendicular): rim top (grey), bore 32-gon (white),
     peg bottom face (orange), analytic contact region = crescent (green), analytic support polygon (green dashed; its
     inner edge = support edge), the engine's support polygon = convex hull of force-carrying rim contacts (red), CoM (black +).
     Static stability: the CoM projection must lie inside the support polygon.
  3. Tilt(t) of both engines and the support margin (CoM x' minus the polygon's inner edge at the CoM's y').
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

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
from matplotlib.patches import Circle, PathPatch, Polygon  # noqa: E402
from matplotlib.path import Path as MPath  # noqa: E402

plt.rcParams["font.family"] = ["PingFang SC", "Arial Unicode MS", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
ROOT = REPO_ROOT
ASSETS = ROOT / "assets/round_socket_exact"
FILES = ["socket_base.obj"] + [f"socket_wall_{k:02d}.obj" for k in range(32)]
R_VERT, N_SEG, Z_RIM, PEG_HALF, PEG_W = 12.058063, 32, 40.0, 25.0, 0.08 * 9.81   # mm; weight in N
W3, H3 = 640, 400
F_MIN = 0.01 * PEG_W                                   # "force-carrying" contact: normal force > 1 % of the peg weight
COL = {"sap": "#d9480f", "mj": "#1864ab"}


# ---------------------------------------------------------------- geometry
def quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q):
    w, x, y, z = q
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def bore_polygon(az):
    """Bore inner 32-gon in the tilt-plane frame (vertices at k*11.25 deg world azimuth)."""
    a = np.radians(np.arange(N_SEG) * 360.0 / N_SEG) - az
    return np.column_stack([R_VERT * np.cos(a), R_VERT * np.sin(a)])


def inside_poly(p, poly):
    path = MPath(poly); return path.contains_points(p)


def hull(pts):
    pts = sorted(map(tuple, pts))
    if len(pts) < 3:
        return np.array(pts)

    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0:
                h.pop()
            h.append(p)
        return h
    lo, up = half(pts), half(pts[::-1])
    return np.array(lo[:-1] + up[:-1])


def inner_edge_at(hp, y):
    """Smallest x' where the line y' = y crosses polygon hp (None if it misses)."""
    if hp is None or len(hp) < 2:
        return None
    xs = []
    for i in range(len(hp)):
        (x0, y0), (x1, y1) = hp[i], hp[(i + 1) % len(hp)]
        if (y0 - y) * (y1 - y) <= 0 and y0 != y1:
            xs.append(x0 + (y - y0) * (x1 - x0) / (y1 - y0))
        elif y0 == y1 == y:
            xs += [x0, x1]
    return min(xs) if xs else None


def analytic_support(ecc, r, az):
    """Crescent (peg bottom disk at offset ecc minus the bore 32-gon) and its convex hull, tilt-plane frame."""
    th = np.linspace(0, 2 * np.pi, 2000, endpoint=False)
    disk = np.column_stack([ecc + r * np.cos(th), r * np.sin(th)])
    bore = bore_polygon(az)
    rim = disk[~inside_poly(disk, bore)]
    bv = bore[np.hypot(bore[:, 0] - ecc, bore[:, 1]) <= r]            # bore vertices inside the peg disk also bound the crescent
    hp = hull(np.vstack([rim, bv]))
    return hp, inner_edge_at(hp, 0.0)


# ---------------------------------------------------------------- data
def load(path, az):
    z = np.load(path); meta = json.loads(str(z["meta"])); C, off = z["contacts"], z["offsets"]
    ca, sa = math.cos(az), math.sin(az); Rz = np.array([[ca, sa], [-sa, ca]])         # world xy -> tilt-plane x', y'
    out = {"meta": meta, "t": z["t"], "pos": z["pos"], "quat": z["quat"], "Rz": Rz, "frames": []}
    for k in range(len(z["t"])):
        c = C[off[k]:off[k + 1]]
        p2 = (c[:, 0:2] @ Rz.T) * 1e3 if len(c) else np.zeros((0, 2))
        fn = np.abs(c[:, 7]) if len(c) else np.zeros(0)
        vert = np.abs(c[:, 5]) > 0.5 if len(c) else np.zeros(0, bool)
        com = (z["pos"][k, 0:2] @ Rz.T) * 1e3
        act = (fn > F_MIN) & vert
        hp = hull(p2[act]) if act.sum() >= 3 else (p2[act] if act.any() else None)
        edge = inner_edge_at(hp, com[1]) if hp is not None and len(hp) >= 2 else None
        tl = tilt_deg(z["quat"][k])
        out["frames"].append({"c": c, "p2": p2, "fn": fn, "vert": vert, "act": act, "hull": hp, "com": com, "tilt": tl,
                              "margin": (com[0] - edge) if (edge is not None and tl < 10) else np.nan,
                              "edge": edge})
    out["tilt"] = np.array([f["tilt"] for f in out["frames"]]); out["margin"] = np.array([f["margin"] for f in out["frames"]])
    return out


# ---------------------------------------------------------------- 3D render
def scene_xml(radius_m):
    assets = "".join(f'<mesh name="m{i}" file="{f}"/>' for i, f in enumerate(FILES))
    pieces = "".join(f'<geom type="mesh" mesh="m{i}" rgba=".45 .5 .68 .30" contype="0" conaffinity="0"/>' for i in range(len(FILES)))
    return (f'<mujoco><compiler meshdir="{ASSETS}"/><visual><global offwidth="{W3}" offheight="{H3}"/><headlight ambient=".5 .5 .5" diffuse=".55 .55 .55"/></visual>'
            f'<asset><texture type="skybox" builtin="flat" rgb1=".97 .97 .97" rgb2=".97 .97 .97" width="16" height="16"/>{assets}</asset><worldbody><light pos="0 -0.3 0.5" dir="0 0.5 -1"/>{pieces}'
            f'<body name="peg" mocap="true"><geom type="cylinder" size="{radius_m} 0.025" rgba=".95 .55 .15 .45" contype="0" conaffinity="0"/></body>'
            f'</worldbody></mujoco>')


class Render3D:
    def __init__(self, radius_m, az):
        self.m = mujoco.MjModel.from_xml_string(scene_xml(radius_m)); self.d = mujoco.MjData(self.m)
        self.r = mujoco.Renderer(self.m, H3, W3); self.az = az
        self.cam = mujoco.MjvCamera(); self.cam.distance = 0.072; self.cam.azimuth = 90 + math.degrees(az); self.cam.elevation = -38

    def _add(self, gtype, size, pos, rgba, mat=np.eye(3)):
        s = self.r.scene
        if s.ngeom >= s.maxgeom:
            return
        mujoco.mjv_initGeom(s.geoms[s.ngeom], gtype, np.asarray(size, float), np.asarray(pos, float), np.asarray(mat, float).reshape(-1), np.asarray(rgba, np.float32)); s.ngeom += 1

    def _line(self, a, b, w, rgba):
        s = self.r.scene
        if s.ngeom >= s.maxgeom:
            return
        g = s.geoms[s.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).reshape(-1), np.asarray(rgba, np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, w, np.asarray(a, float), np.asarray(b, float)); s.ngeom += 1

    def frame(self, pos, quat, fr, chord_xy):
        self.d.mocap_pos[0] = pos; self.d.mocap_quat[0] = quat; mujoco.mj_forward(self.m, self.d)
        self.cam.lookat[:] = [0.006 * math.cos(self.az), 0.006 * math.sin(self.az), 0.043]
        self.r.update_scene(self.d, camera=self.cam)
        ca, sa = math.cos(self.az), math.sin(self.az); toW = lambda x, y: np.array([x * ca - y * sa, x * sa + y * ca])
        (x0, y0), (x1, y1) = chord_xy
        self._line([*(toW(x0, y0) * 1e-3), Z_RIM * 1e-3 + 2e-4], [*(toW(x1, y1) * 1e-3), Z_RIM * 1e-3 + 2e-4], 0.0007, [0.1, 0.75, 0.2, 1])
        self._line(pos, [pos[0], pos[1], Z_RIM * 1e-3], 0.00025, [0, 0, 0, 1])
        self._add(mujoco.mjtGeom.mjGEOM_SPHERE, [0.0009, 0, 0], [pos[0], pos[1], Z_RIM * 1e-3 + 3e-4], [0, 0, 0, 1])
        for row, a in zip(fr["c"], fr["act"]):
            rad = 0.0007 + 0.0011 * min(1.0, abs(row[7]) / (0.3 * PEG_W)) if a else 0.00045
            self._add(mujoco.mjtGeom.mjGEOM_SPHERE, [rad, 0, 0], row[0:3], [0.9, 0.05, 0.05, 1] if a else [0.45, 0.45, 0.45, 0.9])
        return self.r.render().copy()


# ---------------------------------------------------------------- top view
def draw_top(ax, run, k, ecc, r, az, an_hull, an_edge, crescent_patch_src):
    fr = run["frames"][k]; ax.clear(); ax.set_facecolor("#d9d9d9")
    bore = bore_polygon(az)
    ax.add_patch(Polygon(bore, closed=True, fc="white", ec="#555", lw=1.0, zorder=1))
    for i in range(N_SEG):
        a = 2 * math.pi * i / N_SEG - az; ax.plot([R_VERT * math.cos(a), 40 * math.cos(a)], [R_VERT * math.sin(a), 40 * math.sin(a)], c="#bbb", lw=0.5, zorder=1)
    # analytic contact crescent (initial pose), clipped to outside the bore
    disk = Circle((ecc, 0), r, fc="#2f9e44", alpha=0.28, ec="none", zorder=2)
    outer = np.array([[-60, -60], [60, -60], [60, 60], [-60, 60], [-60, -60]]); hole = np.vstack([bore[::-1], bore[-1:]])
    clip = MPath(np.vstack([outer, hole]), [MPath.MOVETO] + [MPath.LINETO] * 3 + [MPath.CLOSEPOLY] + [MPath.MOVETO] + [MPath.LINETO] * (len(bore) - 1) + [MPath.CLOSEPOLY])
    ax.add_patch(disk); cp = PathPatch(clip, transform=ax.transData, fc="none", ec="none"); ax.add_patch(cp); disk.set_clip_path(cp)
    ax.add_patch(Polygon(an_hull, closed=True, fill=False, ec="#2b8a3e", lw=1.4, ls="--", zorder=3))
    ys = an_hull[:, 1]; ax.plot([an_edge, an_edge], [ys.min() * 0.98, ys.max() * 0.98], c="#2b8a3e", lw=3.0, zorder=4, solid_capstyle="butt")
    # peg bottom face (current pose)
    pos = run["pos"][k] * 1e3; Rm = quat_mat(run["quat"][k]); th = np.linspace(0, 2 * np.pi, 120)
    bot = (Rm @ np.vstack([r * np.cos(th), r * np.sin(th), -PEG_HALF * np.ones_like(th)])).T + pos
    b2 = bot[:, 0:2] @ run["Rz"].T
    ax.add_patch(Polygon(b2, closed=True, fill=False, ec="#e8590c", lw=1.3, zorder=5))
    # engine contacts and support polygon
    if fr["hull"] is not None and len(fr["hull"]) >= 3:
        ax.add_patch(Polygon(fr["hull"], closed=True, fc="#e03131", alpha=0.18, ec="#c92a2a", lw=1.6, zorder=6))
    if fr["edge"] is not None and fr["tilt"] < 10:
        ax.plot([fr["edge"]] * 2, [fr["com"][1] - 4, fr["com"][1] + 4], c="#c92a2a", lw=2.2, zorder=7)
    p2, fn, act = fr["p2"], fr["fn"], fr["act"]
    if len(p2):
        ax.scatter(p2[~act, 0], p2[~act, 1], s=14, facecolors="none", edgecolors="#555", lw=0.8, zorder=8)
        ax.scatter(p2[act, 0], p2[act, 1], s=12 + 160 * np.minimum(1, fn[act] / (0.3 * PEG_W)), c="#e03131", ec="k", lw=0.4, zorder=9)
    ax.plot(*fr["com"], marker="+", ms=16, mew=2.4, c="k", zorder=10)
    ax.set_xlim(-14, 22); ax.set_ylim(-13.5, 13.5); ax.set_aspect("equal"); ax.tick_params(labelsize=7)
    ax.set_xlabel("x' (mm，朝偏移方向)", fontsize=8); ax.set_ylabel("y' (mm)", fontsize=8)


def compose(runs, cell, keys, k, an, r3d, fig_axes, k_slow):
    fig, axes = fig_axes; ecc, r, az, an_hull, an_edge, chord = an
    for col, key in enumerate(keys):
        run = runs[key]; fr = run["frames"][k]
        img = r3d[key].frame(run["pos"][k], run["quat"][k], fr, chord)
        a3 = axes[0][col]; a3.clear(); a3.imshow(img); a3.axis("off")
        name = "SAP Warp" if key == "sap" else run["meta"]["engine"]
        a3.set_title(f"{name}（{run['meta']['candidate']}）", fontsize=10, color=COL[key])
        draw_top(axes[1][col], run, k, ecc, r, az, an_hull, an_edge, None)
        e = fr["edge"]; m = fr["margin"]
        if fr["tilt"] >= 10:
            s = f"已翻倒：倾角 {fr['tilt']:.0f}°"
        elif e is None:
            s = "受力接触点不足 3 个，无支撑多边形"
        else:
            verdict = "重心在支撑多边形内 → 稳" if m > 0 else "重心落在支撑多边形外 → 翻倒"
            s = f"引擎支撑边 x'={e:.2f} mm，重心 x'={fr['com'][0]:.2f} mm，裕度 {m:+.2f} mm：{verdict}"
        axes[1][col].set_title(s, fontsize=8.5, color=("#c92a2a" if (np.isnan(m) or m <= 0) else "#2b8a3e"))
    a = axes[2][0]; a.clear(); b = axes[2][1]; b.clear()
    for key in keys:
        run = runs[key]; lab = "SAP" if key == "sap" else "MuJoCo"
        a.plot(run["t"], run["tilt"], c=COL[key], lw=1.4, label=lab); b.plot(run["t"], run["margin"], c=COL[key], lw=1.2, label=lab)
    t = runs[keys[0]]["t"][k]
    for x in (a, b):
        x.axvline(t, c="k", lw=0.8); x.set_xlim(0, 1.0); x.tick_params(labelsize=7); x.set_xlabel("t (s)", fontsize=8); x.legend(fontsize=7, loc="upper right")
        x.grid(alpha=0.3)
    a.set_ylim(-2, 92); a.set_ylabel("倾角 (deg)", fontsize=8)
    b.axhline(ecc - an_edge, c="#2b8a3e", ls="--", lw=1.2, label="解析裕度"); b.axhline(0, c="k", lw=0.6)
    b.set_ylim(-3, 4); b.set_ylabel("支撑裕度 (mm)\n= 重心 x' − 支撑边 x'", fontsize=8); b.legend(fontsize=7, loc="upper right")
    slow = "慢放 0.075×" if k <= k_slow else "0.75×"
    fig.suptitle(f"{cell['id']}：间隙 c={cell['clearance_mm']} mm，偏移 {ecc:.2f} mm（稳定阈值 {cell['label_basis']['stability_offset_mm']:.2f} mm 的 "
                 f"{cell['label_basis']['offset_over_threshold']:g}×），μ={cell['mu']}    t = {t:5.3f} s   {slow}\n"
                 f"绿 = 解析接触区（月牙）/ 解析支撑多边形，粗绿线 = 解析支撑边 x'={an_edge:.2f} mm；红点 = 受力接触点（大小∝法向力），灰圈 = 零力接触；"
                 f"红框 = 引擎支撑多边形，红竖线 = 引擎支撑边；黑 + = 重心投影",
                 fontsize=9.5)
    fig.canvas.draw(); return np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dir", type=Path, required=True); ap.add_argument("--cells", nargs="+", required=True)
    ap.add_argument("--slow-until", type=float, default=0.6)
    a = ap.parse_args()
    for cid in a.cells:
        runs = {}
        for key, f in (("sap", f"sap_{cid}.npz"), ("mj", f"mj_{cid}.npz")):
            meta = json.loads(str(np.load(a.dir / f)["meta"])); az = math.radians(meta["cell"]["azimuth_deg"]); runs[key] = load(a.dir / f, az)
        cell = runs["sap"]["meta"]["cell"]; az = math.radians(cell["azimuth_deg"]); r = 12.0 - cell["clearance_mm"]; ecc = cell["pose"]["centre_inplane_mm"]
        an_hull, an_edge = analytic_support(ecc, r, az)
        yy = an_hull[np.abs(an_hull[:, 0] - an_edge) < 0.05, 1]; chord = ((an_edge, yy.min()), (an_edge, yy.max()))
        print(cid, f"analytic support edge x'={an_edge:.3f} mm (circle bore: {(ecc ** 2 + 144 - r * r) / (2 * ecc):.3f}), CoM {ecc:.3f}")
        for key in runs:
            act = [f for f in runs[key]["frames"][1:21]]
            e = [f["edge"] for f in act if f["edge"] is not None]
            print(f"  {key}: support edge over t=5..100 ms: median {np.median(e):.2f} mm, range {min(e):.2f}-{max(e):.2f}; final tilt {runs[key]['tilt'][-1]:.1f} deg")
        r3d = {k: Render3D(r * 1e-3, az) for k in runs}
        fig = plt.figure(figsize=(13.0, 11.6), dpi=100)
        gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.1, 0.55], hspace=0.32, wspace=0.12, top=0.9, bottom=0.05, left=0.06, right=0.98)
        axes = [[fig.add_subplot(gs[i, j]) for j in range(2)] for i in range(3)]
        dt = runs["sap"]["t"][1] - runs["sap"]["t"][0]; k_slow = int(round(a.slow_until / dt)); n = min(len(runs["sap"]["t"]), len(runs["mj"]["t"]))
        ks = list(range(0, k_slow + 1)) + list(range(k_slow + 10, n, 10))
        out = a.dir / f"{cid}_support_sap_vs_mujoco.mp4"
        with imageio.get_writer(out, fps=15, codec="libx264", quality=8, macro_block_size=8) as wr:
            for i, k in enumerate(ks):
                img = compose(runs, cell, ["sap", "mj"], k, (ecc, r, az, an_hull, an_edge, chord), r3d, (fig, axes), k_slow)
                wr.append_data(img)
                if k == 4:
                    imageio.imwrite(a.dir / f"{cid}_support_keyframe_t20ms.png", img)
        print("wrote", out, len(ks), "frames")
        plt.close(fig)


if __name__ == "__main__":
    main()
