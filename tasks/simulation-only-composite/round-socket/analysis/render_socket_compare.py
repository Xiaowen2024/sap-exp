#!/usr/bin/env python3
"""Render side-by-side round-socket videos from recorded peg trajectories (SAP vs MuJoCo Warp).

Trajectories come from the engines (100-Hz peg pose); this script only replays them in one MuJoCo scene with the
exact-bore socket meshes (semi-transparent) so both engines are drawn identically.  Each panel: 3D view (top) and a
side view along +y (bottom), with time, centre height, tilt and the engine's peak penetration.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import math
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = REPO_ROOT
ASSETS = ROOT / "tasks/simulation-only-composite/round-socket/assets/round_socket_exact"
FILES = ["socket_base.obj"] + [f"socket_wall_{k:02d}.obj" for k in range(32)]
W, Hh = 480, 360


def scene_xml(radius):
    assets = "".join(f'<mesh name="m{i}" file="{f}"/>' for i, f in enumerate(FILES))
    pieces = "".join(f'<geom type="mesh" mesh="m{i}" rgba=".55 .6 .75 .28" contype="0" conaffinity="0"/>' for i in range(len(FILES)))
    return (f'<mujoco><compiler meshdir="{ASSETS}"/><visual><global offwidth="{W}" offheight="{Hh}"/><headlight ambient=".45 .45 .45" diffuse=".6 .6 .6"/></visual>'
            f'<asset>{assets}</asset><worldbody><light pos="0 -0.3 0.5" dir="0 0.5 -1"/>{pieces}'
            f'<body name="peg" mocap="true"><geom type="cylinder" size="{radius} 0.025" rgba=".95 .45 .1 1" contype="0" conaffinity="0"/>'
            f'<geom type="box" pos="{radius*0.6} 0 0.0" size="{radius*0.35} 0.0008 0.024" rgba=".1 .1 .1 1" contype="0" conaffinity="0"/></body>'
            f'</worldbody></mujoco>')


def tilt_deg(q):
    w, x, y, z = q
    return math.degrees(math.acos(min(1.0, abs(1 - 2 * (x * x + y * y)))))


def panel_frames(pos, quat, clearance_mm, label, sub, stride, k0=0, k1=None):
    m = mujoco.MjModel.from_xml_string(scene_xml(0.012 - clearance_mm * 1e-3)); d = mujoco.MjData(m)
    r = mujoco.Renderer(m, Hh, W)
    c3 = mujoco.MjvCamera(); c3.lookat[:] = [0, 0, 0.05]; c3.distance = 0.20; c3.azimuth = 120; c3.elevation = -22
    cs = mujoco.MjvCamera(); cs.lookat[:] = [0, 0, 0.055]; cs.distance = 0.17; cs.azimuth = 90; cs.elevation = 0   # side view along +y
    frames = []
    font = ImageFont.load_default()
    for k in range(k0, len(pos) if k1 is None else min(k1, len(pos)), stride):
        d.mocap_pos[0] = pos[k]; d.mocap_quat[0] = quat[k]; mujoco.mj_forward(m, d)
        r.update_scene(d, camera=c3); a = r.render().copy()
        r.update_scene(d, camera=cs); b = r.render().copy()
        img = Image.fromarray(np.vstack([a, b])); dr = ImageDraw.Draw(img)
        dr.rectangle([0, 0, W, 44], fill=(255, 255, 255))
        dr.text((6, 3), label, fill=(0, 0, 0), font=font)
        dr.text((6, 17), sub, fill=(60, 60, 60), font=font)
        dr.text((6, 31), f"t={k * 0.01:4.2f}s  z={pos[k, 2] * 1e3:6.1f}mm (seated 35.0)  tilt={tilt_deg(quat[k]):5.1f}deg", fill=(0, 0, 0), font=font)
        frames.append(np.asarray(img))
    return frames


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dir", type=Path, required=True); ap.add_argument("--slow", type=float, default=0.5)
    a = ap.parse_args()
    data = {}
    for f in ("traj_sap.npz", "traj_mj.npz"):
        z = np.load(a.dir / f)
        for k in z.files:
            eng, case, cand, kind = k.split("|"); data.setdefault(case, {}).setdefault((eng, cand), {})[kind] = z[k]
    for case, runs in data.items():
        items = sorted(runs.items(), key=lambda kv: (kv[0][0] != "sap", kv[0][1]))
        # impact window: from 50 ms before the first frame with the peg's centre below 70 mm (any engine) to 400 ms after
        k_hit = min(int(np.argmax(v["pos"][:, 2] < 0.070)) for _, v in items)
        for tag, stride, k0, k1, fps in (("full_0p5x", 2, 0, None, 25), ("impact_0p1x", 1, max(0, k_hit - 5), k_hit + 40, 10)):
            cols = []
            for (eng, cand), v in items:
                c, dur, pb, pw = v["meta"]; name = "SAP Warp" if eng == "sap" else "MuJoCo Warp 4.0"
                cols.append(panel_frames(v["pos"], v["quat"], c, f"{name}: {cand}", f"{case}  c={c:g}mm  peak pen: base {pb:.2f} / wall {pw:.2f} mm", stride, k0, k1))
            n = min(len(x) for x in cols); out = a.dir / f"{case}_sap_vs_mujoco_{tag}.mp4"
            with imageio.get_writer(out, fps=fps, codec="libx264", quality=8, macro_block_size=8) as wr:
                for i in range(n):
                    wr.append_data(np.hstack([x[i] for x in cols]))
            print("wrote", out, f"{n} frames @ {fps} fps")


if __name__ == "__main__":
    main()
