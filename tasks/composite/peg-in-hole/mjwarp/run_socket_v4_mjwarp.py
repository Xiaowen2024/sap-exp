#!/usr/bin/env python3
"""Round socket strict-v4, MuJoCo Warp (exact-bore socket; shared logic in socket_v4_common)."""
from __future__ import annotations
import argparse, json
from pathlib import Path
import mujoco, mujoco_warp as mjw, numpy as np, warp as wp
from native_defaults import native_default_mjcf
import socket_v4_common as sc
ROOT = Path(__file__).resolve().parents[2] if (Path(__file__).resolve().parents[2] / "exp-results").exists() else Path("/root/SIM-EXP")
NCON = 4096


@wp.kernel
def _min_peg_dist(nacon: wp.array(dtype=int), worldid: wp.array(dtype=int), geom: wp.array(dtype=wp.vec2i),
                  dist: wp.array(dtype=float), frame: wp.array(dtype=wp.mat33), peg: int, out: wp.array(dtype=float),
                  out_v: wp.array(dtype=float), out_l: wp.array(dtype=float)):
    i = wp.tid()
    if i < nacon[0]:
        g = geom[i]
        if g[0] == peg or g[1] == peg:
            wp.atomic_min(out, worldid[i], dist[i])
            if wp.abs(frame[i][0, 2]) > 0.5:      # normal = frame row 0; mostly vertical -> base / rim contact
                wp.atomic_min(out_v, worldid[i], dist[i])
            else:                                 # lateral normal -> bore-wall contact
                wp.atomic_min(out_l, worldid[i], dist[i])


def parse_candidate(t):
    if t.strip() == "native default":
        return {"id": "native_default", "native": True}
    kv = dict(p.split("=") for p in t.replace(" ", "").split(","))
    return {"id": t, "native": False, "tc": float(kv["tc"].removesuffix("ms")) * 1e-3, "dr": float(kv["dampratio"]), "d": float(kv["dmax"])}


def xml(c, cell):
    dt, mu, r = cell["dt_ms"] * 1e-3, cell["mu"], sc.peg_radius(cell)
    assets = "".join(f'<mesh name="m{i}" file="{f}"/>' for i, f in enumerate(sc.FILES))
    pieces = "".join(f'<geom name="piece{i}" type="mesh" mesh="m{i}"/>' for i in range(len(sc.FILES)))
    opt = f'<option timestep="{dt}" gravity="0 0 -9.81"' + ("" if c["native"] else ' integrator="implicitfast" cone="elliptic" solver="Newton"') + "/>"
    dflt = f'<default><geom friction="{mu} .005 .0001"' + ("" if c["native"] else f' solref="{c["tc"]} {c["dr"]}" solimp="{c["d"]} {c["d"]} .001 .5 2"') + "/></default>"
    I = sc.peg_inertia(cell)
    if "pose" in cell:                                                   # v6: analytic tilt-plane pose (wxyz)
        p0, q0 = sc.initial_pose(cell)
        peg_open = '<body name="peg" pos="{} {} {}" quat="{} {} {} {}"><freejoint/>'.format(*p0, *q0)
    else:
        peg_open = f'<body name="peg" pos="{cell["offset_mm"] / 1000} 0 {cell["release_height_mm"] / 1000}" euler="0 {cell["tilt_deg"]} 0"><freejoint/>'
    s = (f'<mujoco model="round_socket_v4">{opt}<compiler meshdir="{ROOT / sc.ASSET_DIR}"/>{dflt}<asset>{assets}</asset><worldbody>{pieces}'
         + peg_open +
         f'<inertial pos="0 0 0" mass="{sc.PEG_MASS}" diaginertia="{I[0]} {I[1]} {I[2]}"/>'
         f'<geom name="peg_geom" type="cylinder" size="{r} {sc.PEG_HALF}"/></body></worldbody></mujoco>')
    return native_default_mjcf(s) if c["native"] else s


def simulate(c, cell, init_xy):
    m = mujoco.MjModel.from_xml_string(xml(c, cell)); assert abs(m.opt.timestep - cell["dt_ms"] * 1e-3) < 1e-12
    gm = mjw.put_model(m); R = len(init_xy)
    d = mjw.make_data(m, nworld=R, nconmax=NCON // R + 64, njmax=2048)
    q = d.qpos.numpy(); q[:, 0:2] += init_xy; d.qpos.assign(q.astype(np.float32)); mjw.forward(gm, d)
    peg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "peg_geom")
    mn, fin_d, mv, ml, fv, fl = (wp.zeros(R, dtype=float) for _ in range(6)); cap = d.contact.dist.shape[0]; maxn = 0
    dur = float(cell.get("duration_s", 2.0)); every = max(1, int(round(0.01 / m.opt.timestep))); TP, TQ = [], []
    for k in range(round(dur / m.opt.timestep)):
        if k % every == 0:
            qp = d.qpos.numpy().astype(float); TP.append(qp[:, 0:3].copy()); TQ.append(qp[:, 3:7].copy())   # free joint: pos + wxyz
        mjw.step(gm, d)
        wp.launch(_min_peg_dist, dim=cap, inputs=[d.nacon, d.contact.worldid, d.contact.geom, d.contact.dist, d.contact.frame, peg, mn, mv, ml])
        if k % 200 == 0:
            maxn = max(maxn, int(d.nacon.numpy()[0]))
    wp.launch(_min_peg_dist, dim=cap, inputs=[d.nacon, d.contact.worldid, d.contact.geom, d.contact.dist, d.contact.frame, peg, fin_d, fv, fl])
    qp = d.qpos.numpy().astype(float); TP.append(qp[:, 0:3].copy()); TQ.append(qp[:, 3:7].copy())
    return {"traj_pos": np.stack(TP, axis=1), "traj_quat": np.stack(TQ, axis=1), "final_xyz": qp[:, 0:3], "peak_pen": np.maximum(0, -mn.numpy()), "final_pen": np.maximum(0, -fin_d.numpy()),
            "peak_pen_vertical": np.maximum(0, -mv.numpy()), "peak_pen_lateral": np.maximum(0, -ml.numpy()),
            "finite": np.isfinite(qp).all(axis=1), "truncated": int(maxn >= cap)}


def main():
    global sc
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(); man, sha = sc.load_manifest(a.manifest)
    v6 = man["revision"].startswith("strict-v6")
    if v6:
        import socket_v6_common as sc6
        sc = sc6
    res = sc.execute(man, "mujoco_warp_candidates", parse_candidate, simulate, log=lambda s: print(s, flush=True))
    out = {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha, "engine": "MuJoCo Warp",
           "mujoco_version": mujoco.__version__, "warp_version": wp.config.version, "device": str(wp.get_device()), **res}
    a.out.mkdir(parents=True, exist_ok=True); (a.out / ("mjwarp_round_socket_v6.json" if v6 else "mjwarp_round_socket_v4.json")).write_text(json.dumps(sc.json_safe(out), indent=1) + "\n")


if __name__ == "__main__":
    main()
