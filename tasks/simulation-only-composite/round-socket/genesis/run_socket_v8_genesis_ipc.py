#!/usr/bin/env python3
"""Round-socket strict-v8 (DRAFT manifest), Genesis + IPC pilot.  Scoring is socket_v8_common, unchanged.

What is IPC-specific (every choice is written into each result as IPC_INTERPRETATION):
- geometry: one watertight non-convex socket mesh (outer 32-gon prism with a 32-gon blind bore, the analytic shape of
  assets/round_socket_exact) and the peg as a PEG_SIDES-gon prism whose vertices lie on the nominal cylinder; no convex
  hull, no decomposition.  Both are IPC-only ABD bodies (socket fixed);
- candidates: the manifest's ladder is defined by pose-based peak base penetration, which is 0 for IPC at every setting,
  so the four rungs are replaced by fixed IPC configurations (CANDIDATES, declared order); candidate 1 stays the literal
  native default;
- start gap: IPC cannot start at zero distance.  Poses that the manifest releases in contact are moved by a rigid
  translation (tilt unchanged) until every contact is g0 = d_hat apart (registry rule): rim rests are lifted g0; two-point
  wedges are translated in the tilt plane so both wedge gaps equal g0.  In-air starts are unchanged;
- engine_depth is 0 (IPC reports no penetration) and no contact dump is produced (at the second step the peg is still
  outside the barrier), so the support-edge diagnostic is not available.

Execution is sharded so (candidate, condition) runs can go in parallel:
  --phase selection --cand K [--cells a,b]     selection conditions for candidate K (1..5)
  --phase frozen_B5|frozen_B1 [--cells ...]    frozen matrix with the frozen winner / the native default
  --phase robust_dt_0.25ms|robust_dt_1ms|robust_repeat
  --merge                                      assemble result.json (same layout as socket_v8_common.execute)
  --probe ID --cand K --duration S             timing / start-gap check on one condition (not written as a shard)
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

import genesis_ipc_common as I
import socket_v8_common as sc

CANDIDATES = ["native default",
              "d_hat=0.03mm,eps_v=1e-3,newton_tol=1e-4",
              "d_hat=0.1mm,eps_v=1e-3,newton_tol=1e-4",
              "d_hat=0.01mm,eps_v=1e-3,newton_tol=1e-4",
              "d_hat=0.03mm,eps_v=1e-4,newton_tol=1e-5"]
PEG_SIDES = 128
MESH_KW = {"convexify": False}
MESH_DIR = Path("/dev/shm/socket_v8_ipc_meshes") if Path("/dev/shm").is_dir() else Path(__file__).resolve().parent / "_socket_v8_ipc_meshes"
CONTACT_TOL_MM = 1e-3            # a manifest pose closer than this to the socket counts as "released in contact"

IPC_INTERPRETATION = {
    "status": "strict-v9-draft run under engine_clauses.genesis_ipc (DRAFT manifest)",
    "candidates": CANDIDATES,
    "candidate_rule": "candidate 1 = IPCCouplerOptions() defaults (d_hat 10 mm, eps_v 0.01 m/s, Newton velocity tolerance 0.05 m/s). "
                      "2-5 replace the penetration ladder (unreachable for IPC: pose-based penetration is 0): d_hat 30 / 100 / 10 um at "
                      "eps_v 1e-3, then d_hat 30 um at eps_v 1e-4; Newton tolerance = eps_v / 10.",
    "selection_tie_break": "the manifest rule is applied unchanged; its penetration tie-break is 0 for every IPC candidate, so ties fall "
                           "to declared order. The rest gap is reported (start_gap_mm, final pose) but does not enter selection.",
    "geometry": f"socket: single watertight mesh, 130 vertices / 256 triangles; peg: {PEG_SIDES}-gon prism, vertices on the nominal "
                f"radius (flats {12 * (1 - math.cos(math.pi / PEG_SIDES)) * 1e3:.1f} um inside at r = 12 mm), one vertex line in the tilt plane on each side",
    "inertia": "ABD uniform-density inertia of the prism (the manifest's solid-cylinder inertia to within the faceting)",
    "start_gap": "g0 = d_hat. rim rest (tilt 0, bottom face on the rim top): +g0 in z. two-point wedge: translation in the tilt plane "
                 "with both wedge gaps (socket_v8_common.wedge_gaps) = g0. In-air poses unchanged. The shift is included in the scored "
                 "trajectory (drop / displacement are measured from the shifted start). Applied per rollout, to the pose the rollout is "
                 "released from (manifest pose + its perpendicular perturbation): the nominal shift is solved on the unperturbed pose, then "
                 "a two-point wedge gets the smallest extra rigid translation (3-D, tilt unchanged; least-norm Gauss-Newton) that brings both "
                 "true minimum wedge gaps back to g0. Without it a 0.05-mm perturbation closes a 10-um gap (v8 pilot: initial intersection in "
                 "I_c0p1_wedgehold_g8). Per-rollout shift, extra translation and full signed clearance are recorded in ipc.start_gap.per_rollout.",
    "engine_depth": "0 by construction; spurious-contact and engine-depth columns are not applicable",
    "support_edge": "not available (no force-carrying contact in the second step: the peg starts outside the barrier)",
    "batching": "the R = 8 rollouts of a condition share one IPC scene (one Newton solve), so they are numerically coupled",
    "crash_rule": "libuipc aborts the process on a diverged solve; the condition's rollouts are then non-finite = invalid",
}


# ------------------------------------------------------------------------------------------------ meshes
def _write_obj(path: Path, V: np.ndarray, F: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(f".{time.time_ns()}.tmp")
    tmp.write_text("".join(f"v {x:.12f} {y:.12f} {z:.12f}\n" for x, y, z in V) + "".join(f"f {a + 1} {b + 1} {c + 1}\n" for a, b, c in F))
    tmp.replace(path)


def socket_mesh():
    """Outer 32-gon prism z 0..40 mm with a 32-gon blind bore z 10..40 mm; polygon vertices at k * 11.25 deg (as the asset)."""
    n = sc.N_SIDES; a = np.arange(n) * sc.DTH; cs = np.column_stack([np.cos(a), np.sin(a)])
    ring = lambda rad, z: np.column_stack([rad * cs, np.full(n, z)])
    V = np.vstack([[0, 0, 0], ring(sc.OUTER_VERT, 0), ring(sc.OUTER_VERT, sc.Z_TOP), ring(sc.BORE_VERT, sc.Z_TOP), ring(sc.BORE_VERT, sc.Z_BASE),
                   [0, 0, sc.Z_BASE]]) * 1e-3
    o0, o4, b4, b1, cf = 1, 1 + n, 1 + 2 * n, 1 + 3 * n, 1 + 4 * n
    F = []
    for k in range(n):
        j = (k + 1) % n
        F += [(0, o0 + j, o0 + k),                                                # bottom (-z)
              (o0 + k, o0 + j, o4 + j), (o0 + k, o4 + j, o4 + k),                 # outer side
              (o4 + k, o4 + j, b4 + j), (o4 + k, b4 + j, b4 + k),                 # rim top (+z)
              (b4 + k, b4 + j, b1 + j), (b4 + k, b1 + j, b1 + k),                 # bore wall (normal towards the axis)
              (cf, b1 + k, b1 + j)]                                               # bore floor (+z)
    return V, np.array(F, int)


def peg_mesh(r_mm: float, n: int = PEG_SIDES):
    a = np.arange(n) * 2 * math.pi / n; cs = np.column_stack([np.cos(a), np.sin(a)]) * r_mm
    V = np.vstack([np.column_stack([cs, np.full(n, -sc.PEG_HALF)]), np.column_stack([cs, np.full(n, sc.PEG_HALF)]),
                   [0, 0, -sc.PEG_HALF], [0, 0, sc.PEG_HALF]]) * 1e-3
    cb, ct = 2 * n, 2 * n + 1; F = []
    for k in range(n):
        j = (k + 1) % n
        F += [(k, j, n + j), (k, n + j, n + k), (ct, n + k, n + j), (cb, j, k)]
    return V, np.array(F, int)


def mesh_check(V, F) -> dict:
    """Signed volume and closed-manifold / orientation check (every directed edge once, its reverse once)."""
    t = V[F]; vol = float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6)
    e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]); s = {tuple(x) for x in e}
    return {"volume_m3": vol, "n_verts": int(len(V)), "n_faces": int(len(F)),
            "closed_oriented": bool(len(s) == len(e) and all((b, a) in s for a, b in s))}


def mesh_files(cell) -> tuple[Path, Path]:
    r = sc.peg_radius_mm(cell)
    sp, pp = MESH_DIR / "socket.obj", MESH_DIR / f"peg_r{r:.4f}_n{PEG_SIDES}.obj"
    if not sp.exists():
        _write_obj(sp, *socket_mesh())
    if not pp.exists():
        _write_obj(pp, *peg_mesh(r))
    return sp, pp


def entities(cell) -> dict[str, str]:
    sp, pp = mesh_files(cell); mu = cell["mu"]; inr = sc.peg_inertia(cell)
    return {"socket": I.mjcf(f'<body name="socket" pos="0 0 0"><geom name="socket_geom" type="mesh" mesh="socket" friction="{mu} 0 0"/></body>',
                             meshdir=sp.parent, assets=f'<mesh name="socket" file="{sp.name}"/>'),
            "peg": I.mjcf(f'<body name="peg" pos="0 0 0.2"><freejoint/><inertial pos="0 0 0" mass="{sc.peg_mass(cell)}" '
                          f'diaginertia="{inr[0]} {inr[1]} {inr[2]}"/><geom name="peg_geom" type="mesh" mesh="peg" friction="{mu} 0 0"/></body>',
                          meshdir=pp.parent, assets=f'<mesh name="peg" file="{pp.name}"/>')}


# ------------------------------------------------------------------------------------------------ start gap
def clearance_mm(cell, p_mm, q) -> float:
    """Smallest distance (mm) between the peg (nominal cylinder, sampled) and the analytic socket for a pose outside it:
    peg surface points against the base / bore wall / rim top / bore top edge, and the bore top edge against the peg."""
    r = sc.peg_radius_mm(cell); Rm = sc.quat_to_mat(np.asarray(q, float)[None])[0]
    a = np.linspace(0, 2 * np.pi, 720, endpoint=False); h = np.linspace(-sc.PEG_HALF, sc.PEG_HALF, 201)
    A, H = np.meshgrid(a, h); side = np.stack([r * np.cos(A).ravel(), r * np.sin(A).ravel(), H.ravel()], 1)
    rr = np.linspace(0, r, 41); A2, R2 = np.meshgrid(a, rr)
    caps = np.concatenate([np.stack([R2.ravel() * np.cos(A2).ravel(), R2.ravel() * np.sin(A2).ravel(), np.full(A2.size, s * sc.PEG_HALF)], 1) for s in (-1, 1)])
    W = np.concatenate([side, caps]) @ Rm.T + np.asarray(p_mm, float)
    x, y, z = W[:, 0], W[:, 1], W[:, 2]
    rho = np.hypot(x, y); th = np.arctan2(y, x); k = np.round((th - sc.FACE0) / sc.DTH); proj = rho * np.cos(th - (sc.FACE0 + k * sc.DTH))
    inb = proj < sc.BORE_IN
    d = np.where(inb & (z >= sc.Z_TOP), np.hypot(sc.BORE_IN - proj, z - sc.Z_TOP),
                 np.where(inb, np.minimum(sc.BORE_IN - proj, z - sc.Z_BASE), np.where(proj <= sc.OUTER_IN, z - sc.Z_TOP, np.inf)))
    B = (sc._BORE_RING - np.asarray(p_mm, float)) @ Rm                             # bore top edge in the peg frame
    rb, zb = np.hypot(B[:, 0], B[:, 1]) - r, np.abs(B[:, 2]) - sc.PEG_HALF
    de = np.where((rb > 0) & (zb > 0), np.hypot(rb, zb), np.maximum(rb, zb))
    return float(min(d.min(), de.min()))


def _peg_world(cell, p_mm, q):
    """Nominal-cylinder sample points (side + caps) in world mm and the rotation matrix."""
    r = sc.peg_radius_mm(cell); Rm = sc.quat_to_mat(np.asarray(q, float)[None])[0]
    a = np.linspace(0, 2 * np.pi, 720, endpoint=False); h = np.linspace(-sc.PEG_HALF, sc.PEG_HALF, 201)
    A, H = np.meshgrid(a, h); side = np.stack([r * np.cos(A).ravel(), r * np.sin(A).ravel(), H.ravel()], 1)
    rr = np.linspace(0, r, 41); A2, R2 = np.meshgrid(a, rr)
    caps = np.concatenate([np.stack([R2.ravel() * np.cos(A2).ravel(), R2.ravel() * np.sin(A2).ravel(), np.full(A2.size, s * sc.PEG_HALF)], 1) for s in (-1, 1)])
    return np.concatenate([side, caps]) @ Rm.T + np.asarray(p_mm, float), Rm


def _proj(x, y):
    rho = np.hypot(x, y); th = np.arctan2(y, x); k = np.round((th - sc.FACE0) / sc.DTH); return rho * np.cos(th - (sc.FACE0 + k * sc.DTH))


def _edge_gap(cell, p_mm, Rm):
    """Signed distance (mm) of the bore top edge ring to the peg cylinder (< 0 = the edge is inside the peg)."""
    r = sc.peg_radius_mm(cell); B = (sc._BORE_RING - np.asarray(p_mm, float)) @ Rm
    rb, zb = np.hypot(B[:, 0], B[:, 1]) - r, np.abs(B[:, 2]) - sc.PEG_HALF
    return np.where((rb > 0) & (zb > 0), np.hypot(rb, zb), np.maximum(rb, zb))


def signed_clearance_mm(cell, p_mm, q) -> float:
    """Like clearance_mm, but signed everywhere: a peg point inside the socket wall solid gives minus its exit distance (clearance_mm
    returns z - z_top there, which is not a distance). Used for the per-rollout start-gap check."""
    W, Rm = _peg_world(cell, p_mm, q); x, y, z = W[:, 0], W[:, 1], W[:, 2]; proj = _proj(x, y); inb = proj < sc.BORE_IN
    wall = (~inb) & (proj <= sc.OUTER_IN) & (z < sc.Z_TOP)
    d = np.where(inb & (z >= sc.Z_TOP), np.hypot(sc.BORE_IN - proj, z - sc.Z_TOP),
                 np.where(inb, np.minimum(sc.BORE_IN - proj, z - sc.Z_BASE),
                          np.where(wall, -np.minimum.reduce([proj - sc.BORE_IN, sc.Z_TOP - z, sc.OUTER_IN - proj]),
                                   np.where(proj <= sc.OUTER_IN, z - sc.Z_TOP, np.inf))))
    return float(min(d.min(), _edge_gap(cell, p_mm, Rm).min()))


def _wedge_gaps_full(cell, p_mm, q):
    """The two wedge gaps measured as true minima over the perturbed pose (not only in the nominal tilt plane):
    A = bottom-corner circle of the peg against the bore wall, B = bore top edge against the peg side."""
    r = sc.peg_radius_mm(cell); Rm = sc.quat_to_mat(np.asarray(q, float)[None])[0]; a = np.linspace(0, 2 * np.pi, 2880, endpoint=False)
    C = np.stack([r * np.cos(a), r * np.sin(a), np.full(a.size, -sc.PEG_HALF)], 1) @ Rm.T + np.asarray(p_mm, float)
    return np.array([float((sc.BORE_IN - _proj(C[:, 0], C[:, 1])).min()), float(_edge_gap(cell, p_mm, Rm).min())])


def start_shift_rollouts(cell, g0_mm: float, xy_m) -> dict:
    """Per-rollout start gap (strict-v9 engine_clauses.genesis_ipc.start_gap applied to the pose each rollout is actually released
    from, i.e. including its perturbation). The nominal shift is solved first (start_shift); a two-point wedge is then re-solved per
    rollout with the true minimum gaps (_wedge_gaps_full), translation in the tilt plane only. Every rollout's full signed clearance
    is checked and recorded."""
    sg = start_shift(cell, g0_mm); p0, q0 = sc.initial_pose(cell); P = p0 * 1e3
    az = math.radians(cell.get("azimuth_deg", 0.0)); e = np.array([math.cos(az), math.sin(az), 0.0]); rows = []
    for w, xy in enumerate(np.asarray(xy_m, float) * 1e3):
        Pw = P + np.array([xy[0], xy[1], 0.0]); d = np.asarray(sg["shift_mm"], float)
        if sg["kind"] == "two_point_wedge":
            # smallest extra rigid translation (3-D, tilt unchanged) that brings both true wedge gaps back to g0:
            # least-norm Gauss-Newton step dx = J^T (J J^T)^-1 (g0 - g)
            f_ = lambda dd: _wedge_gaps_full(cell, Pw + dd, q0) - g0_mm; hh = 1e-5
            for _ in range(40):
                f = f_(d)
                if np.abs(f).max() < 1e-7:
                    break
                J = np.column_stack([(f_(d + hh * ax) - f_(d - hh * ax)) / (2 * hh) for ax in np.eye(3)])
                d = d - J.T @ np.linalg.solve(J @ J.T, f)
        cl = signed_clearance_mm(cell, Pw + d, q0) if sg["kind"] != "in_air" else sg["manifest_clearance_mm"]
        rows.append({"rollout": w, "shift_mm": d.tolist(), "extra_vs_nominal_mm": (d - np.asarray(sg["shift_mm"])).tolist(), "clearance_mm": cl})
    sg["per_rollout"] = rows; sg["min_rollout_clearance_mm"] = float(min(r["clearance_mm"] for r in rows))
    sg["min_rollout_clearance_over_g0"] = sg["min_rollout_clearance_mm"] / g0_mm
    return sg


def start_shift(cell, g0_mm: float) -> dict:
    """Rigid translation (mm) that opens every t = 0 contact of the manifest pose to g0; zeros for in-air poses."""
    p0, q0 = sc.initial_pose(cell); P = p0 * 1e3; az = math.radians(cell.get("azimuth_deg", 0.0)); e = np.array([math.cos(az), math.sin(az), 0.0])
    c = cell["clearance_mm"]; tilt = cell["pose"]["tilt_deg"]; c0 = clearance_mm(cell, P, q0)
    out = {"g0_mm": g0_mm, "manifest_clearance_mm": c0, "kind": "in_air", "shift_mm": [0.0, 0.0, 0.0]}
    if c0 > CONTACT_TOL_MM:
        out["shifted_clearance_mm"] = c0
        return out
    if tilt == 0.0:
        d = np.array([0.0, 0.0, g0_mm]); out["kind"] = "rim_rest"
    else:
        gap = lambda s, dz: np.array(sc.wedge_gaps(c, P + s * e + np.array([0.0, 0.0, dz]), q0), float)
        x = np.zeros(2)
        for _ in range(30):
            f = gap(*x) - g0_mm
            if np.abs(f).max() < 1e-9:
                break
            hh = 1e-6; J = np.column_stack([(gap(x[0] + hh, x[1]) - gap(x[0] - hh, x[1])) / (2 * hh), (gap(x[0], x[1] + hh) - gap(x[0], x[1] - hh)) / (2 * hh)])
            x = x - np.linalg.solve(J, f)
        d = x[0] * e + np.array([0.0, 0.0, x[1]]); out["kind"] = "two_point_wedge"; out["wedge_gaps_mm"] = gap(*x).tolist()
    out["shift_mm"] = d.tolist(); out["shifted_clearance_mm"] = clearance_mm(cell, P + d, q0)
    return out


# ------------------------------------------------------------------------------------------------ simulation (child process)
def sim_cell(c, cell, init_xy):
    R = len(init_xy); dt = cell["dt_ms"] * 1e-3; n = round(float(cell["duration_s"]) / dt)
    g0 = I.start_gap(c); sg = start_shift_rollouts(cell, g0 * 1e3, init_xy)
    p0, q0 = sc.initial_pose(cell)
    pos = np.stack([p0 + np.asarray(r["shift_mm"]) * 1e-3 for r in sg["per_rollout"]]); pos[:, 0:2] += init_xy
    xml = entities(cell)
    bodies = {"socket": I.Body(xml["socket"], mu=cell["mu"], mesh_kw=MESH_KW),
              "peg": I.Body(xml["peg"], mu=cell["mu"], mass=sc.peg_mass(cell), mesh_kw=MESH_KW)}
    t0 = time.perf_counter()
    b = I.build(bodies, dt=dt, gravity=(0.0, 0.0, -sc.gravity(cell)), cand=c, n_envs=R, init={"peg": (I.T_from(pos, np.tile(q0, (R, 1))), None)})
    t_build = time.perf_counter() - t0
    T = np.zeros((n + 1, R, 3, 4)); t0 = time.perf_counter()
    for k in range(n + 1):
        T[k] = b.T["peg"][:, :3, :4]
        if k < n:
            b.step()
    t_step = time.perf_counter() - t0
    prov = b.provenance(); b.destroy()
    P = T[..., 3].transpose(1, 0, 2); Q = I.R_to_quat(T[..., :3]).transpose(1, 0, 2)
    for w in range(R):                                   # keep the quaternion sign continuous along each rollout
        flip = np.cumprod(np.where((Q[w, 1:] * Q[w, :-1]).sum(1) < 0, -1.0, 1.0)); Q[w, 1:] *= flip[:, None]
    A = T[..., :3]; ortho = float(np.abs(np.einsum("...ji,...jk->...ik", A, A) - np.eye(3)).max()) if np.isfinite(A).all() else math.nan
    return {"pos": P, "quat": Q,
            "ipc": {"compiled": prov, "start_gap": sg, "wall_build_s": t_build, "wall_step_s": t_step, "s_per_step": t_step / max(n, 1),
                    "steps": n, "R": R, "abd_max_deviation_from_rotation": ortho, "versions": I.versions(),
                    "meshes": {"socket": mesh_check(*socket_mesh()), "peg": mesh_check(*peg_mesh(sc.peg_radius_mm(cell)))}}}


LAST: dict = {}


def simulate(c, cell, init_xy, dump_contacts=False):
    R = len(init_xy); S = round(float(cell["duration_s"]) / (cell["dt_ms"] * 1e-3)) + 1
    r, why = I.ISO.call("run_socket_v8_genesis_ipc", "sim_cell", tag=f"{c['id']}|{cell['id']}", c=c, cell=cell, init_xy=np.asarray(init_xy, float))
    LAST.clear()
    if r is None:
        LAST.update(failed=why)
        return {"pos": np.full((R, S, 3), np.nan), "quat": np.full((R, S, 4), np.nan), "engine_depth": np.zeros((R, S)), "truncated": 0, "contacts": None}
    LAST.update(r["ipc"])
    return {"pos": r["pos"], "quat": r["quat"], "engine_depth": np.zeros((R, S)), "truncated": 0, "contacts": None}


# ------------------------------------------------------------------------------------------------ shards and merge
def cands():
    return [I.parse_candidate(t) for t in CANDIDATES]


def run_shard(man, phase, cand_index, cell, out, log):
    idx = {c["id"]: j for j, c in enumerate(man["conditions"])}; cand = cands()[cand_index - 1]
    path = out / "shards" / phase / f"cand{cand_index}" / f"{cell['id']}.json"
    if path.exists():
        log(f"skip {path} (exists)"); return json.loads(path.read_text())
    r = sc.run_condition(cand, cell, man["seed0"] + idx[cell["id"]], man, simulate, out / "traj" / phase / f"cand{cand_index}", "")
    r["phase"] = phase; r["candidate_index"] = cand_index; r["ipc"] = dict(LAST)
    path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(sc.json_safe(r)) + "\n")
    ipc = r["ipc"]
    log(f"[{phase}] cand{cand_index} {sc.fmt(r)} step={ipc.get('s_per_step', float('nan')) * 1e3:.0f}ms "
        f"start={ipc.get('start_gap', {}).get('kind')} {ipc.get('failed', '')}")
    return r


def load(out, phase, k, cid):
    p = out / "shards" / phase / f"cand{k}" / f"{cid}.json"
    return json.loads(p.read_text()) if p.exists() else None


def selection_summary(man, out):
    sel_ids = man["run_plan"]["1_selection"]["conditions"]; cs = cands(); sel = []
    for i, c in enumerate(cs):
        rs = [load(out, "selection", i + 1, cid) for cid in sel_ids]
        rec = {"index": i + 1, "name": CANDIDATES[i], "candidate": c, "status": "ok", "runnable": all(r is not None for r in rs)}
        if rec["runnable"]:
            pen = [r["pen_total_peak_mm"]["mean"] if r.get("pen_total_peak_mm") else math.inf for r in rs]
            rec.update(conditions=rs, n_correct=sum(r["verdict"] == "correct" for r in rs), n_unstable=sum(r["verdict"] == "unstable" for r in rs),
                       n_correct_rollouts=sum(r["correct_rollouts"] for r in rs), n_rollouts=sum(r["R"] for r in rs),
                       n_inadmissible=sum(r["inadmissible_rests"] for r in rs), mean_peak_pen_mm=float(np.mean(pen)))
        else:
            rec["reason"] = "selection shard missing"
        sel.append(rec)
    return sel


def selection_key(s):
    """strict-v9 rule: most correct rollouts, then fewest inadmissible rests, then lowest penetration, then declared order."""
    return (-s["n_correct_rollouts"], s["n_inadmissible"], s["mean_peak_pen_mm"], s["index"])


def frozen_index(sel, k=None):
    ok = [s for s in sel if s.get("runnable") and (k is None or s["index"] <= k)]
    return min(ok, key=selection_key)["index"] if ok else None


def merge(man, sha, out, log):
    conds = man["conditions"]; sel = selection_summary(man, out)
    res = {"interpretation": sc.INTERPRETATION, "ipc_interpretation": IPC_INTERPRETATION, "protocol": man["protocol"], "task": man["task"],
           "revision": man["revision"], "engine": {"engine": I.ENGINE, "manifest_sha256": sha, "manifest_revision": man["revision"]},
           "candidates": [{"index": i + 1, "name": t, "candidate": c, "status": "ok"} for i, (t, c) in enumerate(zip(CANDIDATES, cands()))],
           "selection": sel, "complete": {"selection": all(s.get("runnable") for s in sel)}}
    if res["complete"]["selection"]:
        res["B_k_frozen_index"] = {f"B{k}": frozen_index(sel, k) for k in range(1, len(sel) + 1)}
        win = frozen_index(sel); res.update(frozen_index=win, frozen_candidate=cands()[win - 1], frozen_name=CANDIDATES[win - 1])
        frozen = [c for c in conds if c["role"] == "frozen"]; res["frozen_matrix"] = {}
        for lab, k in (("B5", win), ("B1", 1)):
            if lab == "B1" and win == 1:
                continue
            rows = [r for r in (load(out, f"frozen_{lab}", k, c["id"]) for c in frozen) if r is not None]
            if rows:
                res["frozen_matrix"][lab] = {"candidate_index": k, "conditions": rows, "n_done": len(rows), "n_total": len(frozen), "tally": sc.tally(rows)}
        rob_ids = man["run_plan"]["3_robustness"]["conditions"]; rob = {}
        for key in ("robust_dt_0.25ms", "robust_dt_1ms", "robust_repeat"):
            rows = [r for r in (load(out, key, win, cid) for cid in rob_ids) if r is not None]
            if rows:
                rob[key] = {"conditions": rows}
        if "robust_repeat" in rob and "B5" in res["frozen_matrix"]:
            first = {r["id"]: r for r in res["frozen_matrix"]["B5"]["conditions"]}; rep = rob["robust_repeat"]["conditions"]
            rob["robust_repeat"].update(verdict_changes=[r["id"] for r in rep if first.get(r["id"], {}).get("verdict") != r["verdict"]],
                                        final_pose_max_diff_mm={r["id"]: sc._pose_diff(first.get(r["id"]), r) for r in rep})
        res["robustness"] = rob
    I.save_json(out / "result.json", res)
    for s in sel:
        if s.get("runnable"):
            log(f"cand{s['index']} {s['name']}: correct {s['n_correct']}/3 unstable {s['n_unstable']} inadmissible {s['n_inadmissible']} "
                + " | ".join(f"{r['id']}={r['verdict']}({r['correct_rollouts']}/{r['R']})" for r in s["conditions"]))
        else:
            log(f"cand{s['index']} {s['name']}: {s.get('reason')}")
    if "frozen_index" in res:
        log(f"B_k {res['B_k_frozen_index']} -> frozen candidate {res['frozen_index']} ({res['frozen_name']})")
        for lab, fm in res["frozen_matrix"].items():
            log(f"frozen {lab} ({fm['n_done']}/{fm['n_total']} conditions) tally {json.dumps(fm['tally'])}")
    return res


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--phase"); ap.add_argument("--cand", type=int); ap.add_argument("--cells"); ap.add_argument("--merge", action="store_true")
    ap.add_argument("--probe"); ap.add_argument("--duration", type=float); ap.add_argument("--list", action="store_true")
    a = ap.parse_args(); man, sha = sc.load_manifest(a.manifest); log = lambda s: print(s, flush=True)
    decl = [d["id"] for d in man.get("candidates", {}).get("genesis_ipc", {}).get("declared_order", [])]
    if decl and decl != CANDIDATES:
        raise SystemExit(f"runner CANDIDATES {CANDIDATES} differ from manifest candidates.genesis_ipc {decl}")
    conds = man["conditions"]; idx = {c["id"]: j for j, c in enumerate(conds)}
    if a.probe:
        cell = dict(conds[idx[a.probe]]); cand = cands()[a.cand - 1]
        if a.duration:
            cell["duration_s"] = a.duration
        r = sc.run_condition(cand, cell, man["seed0"] + idx[cell["id"]], man, simulate)
        log(f"[probe {cell['duration_s']} s] cand{a.cand} {sc.fmt(r)}")
        log(json.dumps(sc.json_safe({k: v for k, v in LAST.items() if k != "versions"}), indent=1)); return
    if a.merge:
        merge(man, sha, a.out, log); return
    ph = a.phase
    if ph == "selection":
        ids = man["run_plan"]["1_selection"]["conditions"]; k = a.cand; over = {}
    else:
        sel = selection_summary(man, a.out); win = frozen_index(sel)
        if not all(s.get("runnable") for s in sel):
            raise SystemExit("selection is not complete; no frozen condition may be run before the configuration is frozen")
        rob = man["run_plan"]["3_robustness"]["conditions"]; fro = [c["id"] for c in conds if c["role"] == "frozen"]
        ids, k, over = {"frozen_B5": (fro, win, {}), "frozen_B1": (fro, 1, {}), "robust_dt_0.25ms": (rob, win, {"dt_ms": 0.25}),
                        "robust_dt_1ms": (rob, win, {"dt_ms": 1.0}), "robust_repeat": (rob, win, {})}[ph]
        if a.cand and a.cand != k:
            raise SystemExit(f"{ph} uses candidate {k}, not {a.cand}")
    if a.cells:
        ids = [c for c in ids if c in a.cells.split(",")]
    if a.list:
        print("\n".join(f"{ph} {k} {c}" for c in ids)); return
    for cid in ids:
        run_shard(man, ph, k, dict(conds[idx[cid]], **over), a.out, log)


if __name__ == "__main__":
    main()
