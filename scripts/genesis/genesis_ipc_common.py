"""Shared Genesis 1.4.6 + IPC (libuipc 0.0.29) adapter for the SIM-EXP protocol tasks (preliminary track).

Physics comes from the task manifest exactly as in the RigidSolver track (same MJCF snippets, one entity per body).
What is IPC-specific (see sim_protocol/genesis_ipc_runner_sop.md):
- every body that touches another is coupled, with an explicit coup_type (the rigid collider only drops a pair when
  both sides carry an explicit IPC coup_type; an implicit one would double-count contact):
  free bodies -> 'ipc_only' (IPC integrates them), fixed bodies / ground -> 'ipc_only' (fixed ABD / IPC ground),
  driven bodies (pinch pads, slope board in task-3, MIT pusher) -> 'two_way_soft_constraint' (Genesis drives the body,
  IPC follows through the bridge's soft transform constraint; the bridge's coupling force is the contact readout);
- ABD mass = rho * V of the IPC surface mesh, so rho = manifest mass / V is set per body; ABD inertia is the
  uniform-density inertia of that mesh (explicit manifest inertia cannot be imposed and is recorded);
- pair friction = geometric mean of coup_friction: every body carries the pair mu (a body whose pairs need different
  values carries mu_pair^2 / mu_other);
- initial pose, initial velocity and external forces are written into IPC (transform / velocity instance attributes,
  AffineBodyExternalBodyForce through an Animator); Genesis-side setters do not reach IPC-only bodies;
- state telemetry is IPC's own (transform and affine velocity from the bridge's ABD accessor); contact load on an
  IPC-only body = m (dv/dt - g) - F_ext (exact for IPC's implicit-Euler update);
- start gap: IPC cannot start at zero distance; every resting contact starts g0 = d_hat apart (registry rule) and the
  tasks' own settle phase absorbs the drop.
"""
from __future__ import annotations

import hashlib
import importlib
import importlib.metadata as metadata
import multiprocessing as mp
import os
import sys
import time
import traceback
import json
import math
import tempfile
from pathlib import Path
from typing import Any

import numpy as np

import genesis_rigid_common as G
from genesis_rigid_common import ROOT, Timer, json_safe, load_manifest, mjcf, np_, save_json  # noqa: F401

REGISTRY = ROOT / "sim_protocol/manifests/genesis_ipc_rerun.json"
ENGINE = "Genesis IPC"
PINNED = {"genesis-world": "1.4.6", "pyuipc": "0.0.29"}
D_HAT_DEFAULT = 0.01          # libuipc 0.0.29 contact.d_hat (resolved default, recorded in the setup audit)
EPS_V_DEFAULT = 0.01
RESISTANCE_DEFAULT = 1e9      # Genesis IPCCouplerOptions.contact_resistance default


def gs_init():
    if metadata.version("pyuipc") != PINNED["pyuipc"]:
        raise RuntimeError(f"pyuipc {metadata.version('pyuipc')} != pinned {PINNED['pyuipc']}")
    return G.gs_init()


def versions() -> dict[str, Any]:
    v = G.versions(); v["pyuipc"] = metadata.version("pyuipc")
    return v


def registry_task(task_id: str, manifest_path: Path):
    reg = json.loads(REGISTRY.read_text())
    spec = next(t for t in reg["tasks"] if t["id"] == task_id)
    man, sha = load_manifest(manifest_path)
    if Path(manifest_path).name != spec["manifest"] or sha != spec["sha256"]:
        raise ValueError(f"{task_id}: manifest name/sha differs from the IPC registry")
    return reg, spec, sha


def registry_id_for(manifest_path: Path) -> str:
    reg = json.loads(REGISTRY.read_text())
    return next(t["id"] for t in reg["tasks"] if t["manifest"] == Path(manifest_path).name)


def candidates_for(spec: dict[str, Any]) -> list[str]:
    if "candidates" in spec:
        return list(spec["candidates"])
    return list(json.loads(REGISTRY.read_text())["default_candidates"])


def with_candidates(man: dict[str, Any], spec: dict[str, Any], key: str = "genesis_ipc_candidates") -> dict[str, Any]:
    m = json.loads(json.dumps(man))
    m["candidate_rule"][key] = candidates_for(spec)
    return m


def header(man: dict[str, Any], sha: str, spec: dict[str, Any]) -> dict[str, Any]:
    return {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
            "engine": ENGINE, "status": "a3 rerun", "registry_revision": json.loads(REGISTRY.read_text())["revision"],
            "versions": versions(), "registry": str(REGISTRY.relative_to(ROOT)),
            "registry_sha256": hashlib.sha256(REGISTRY.read_bytes()).hexdigest(), "ipc_candidates": candidates_for(spec),
            "routes": spec.get("routes", {}), "readouts": spec.get("readouts", {})}


# ------------------------------------------------------------------ candidates
def parse_candidate(text: str, spec: dict[str, Any] | None = None) -> dict[str, Any]:
    """'native default' or comma list of d_hat=1mm, eps_v=1e-3 (m/s), resistance=1e10, newton_tol=0.01."""
    if text.strip() == "native default":
        return {"id": "native_default", "native": True, "text": text}
    c: dict[str, Any] = {"id": text, "native": False, "text": text}
    for part in text.replace(" ", "").split(","):
        k, v = part.split("=")
        if k == "d_hat":
            c["d_hat"] = float(v.removesuffix("mm")) * (1e-3 if v.endswith("mm") else 1.0)
        elif k in ("eps_v", "resistance", "newton_tol", "strength"):
            c[k] = float(v)
        else:
            raise ValueError(f"unknown IPC candidate field {k!r} in {text!r}")
    return c


def d_hat(c: dict[str, Any]) -> float:
    return float(c.get("d_hat", D_HAT_DEFAULT))


def start_gap(c: dict[str, Any]) -> float:
    """Registry rule: resting contacts start one d_hat apart (outside the barrier), whatever the candidate."""
    return d_hat(c)


def coupler_options(c: dict[str, Any]):
    gs = gs_init()
    kw: dict[str, Any] = {}
    if not c.get("native"):
        if "d_hat" in c:
            kw["contact_d_hat"] = c["d_hat"]
        if "eps_v" in c:
            kw["contact_eps_velocity"] = c["eps_v"]
        if "resistance" in c:
            kw["contact_resistance"] = c["resistance"]
        if "newton_tol" in c:
            kw["newton_tolerance"] = c["newton_tol"]
        if "strength" in c:                       # bridge soft-constraint strength (two-way bodies only)
            kw["constraint_strength_translation"] = c["strength"]
            kw["constraint_strength_rotation"] = c["strength"]
    return gs.options.IPCCouplerOptions(**kw)


# ------------------------------------------------------------------ small geometry helpers
def quat_to_R(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, float); q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = (q[..., i] for i in range(4))
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y * y + z * z); R[..., 0, 1] = 2 * (x * y - w * z); R[..., 0, 2] = 2 * (x * z + w * y)
    R[..., 1, 0] = 2 * (x * y + w * z); R[..., 1, 1] = 1 - 2 * (x * x + z * z); R[..., 1, 2] = 2 * (y * z - w * x)
    R[..., 2, 0] = 2 * (x * z - w * y); R[..., 2, 1] = 2 * (y * z + w * x); R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation part of an ABD affine matrix (polar-projected) to wxyz quaternion."""
    R = np.asarray(R, float)
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    out = np.empty(R.shape[:-2] + (4,))
    for idx in np.ndindex(R.shape[:-2]):
        m = R[idx]; t = np.trace(m)
        if t > 0:
            s = 2 * math.sqrt(t + 1); q = [s / 4, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        else:
            i = int(np.argmax(np.diag(m)))
            if i == 0:
                s = 2 * math.sqrt(1 + m[0, 0] - m[1, 1] - m[2, 2]); q = [(m[2, 1] - m[1, 2]) / s, s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s]
            elif i == 1:
                s = 2 * math.sqrt(1 + m[1, 1] - m[0, 0] - m[2, 2]); q = [(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s]
            else:
                s = 2 * math.sqrt(1 + m[2, 2] - m[0, 0] - m[1, 1]); q = [(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4]
        q = np.asarray(q); out[idx] = q * (1 if q[0] >= 0 else -1)
    return out


def T_from(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    pos = np.asarray(pos, float); T = np.zeros(pos.shape[:-1] + (4, 4)); T[..., 3, 3] = 1
    T[..., :3, :3] = quat_to_R(quat); T[..., :3, 3] = pos
    return T


def V_from(T: np.ndarray, v_lin: np.ndarray, w_body: np.ndarray) -> np.ndarray:
    """IPC affine velocity [dA/dt | dp/dt] for world linear velocity and body-frame angular velocity (MuJoCo qvel)."""
    R = T[..., :3, :3]; w = np.einsum("...ij,...j->...i", R, np.asarray(w_body, float))
    W = np.zeros(w.shape[:-1] + (3, 3))
    W[..., 0, 1], W[..., 0, 2], W[..., 1, 2] = -w[..., 2], w[..., 1], -w[..., 0]
    W[..., 1, 0], W[..., 2, 0], W[..., 2, 1] = w[..., 2], -w[..., 1], w[..., 0]
    V = np.zeros(T.shape); V[..., :3, :3] = W @ R; V[..., :3, 3] = v_lin
    return V


def mesh_volume(verts: np.ndarray, faces: np.ndarray) -> float:
    v = np.asarray(verts, float)[np.asarray(faces, int)]
    return float(abs(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum()) / 6.0)


# ------------------------------------------------------------------ scene
MESH_KW = G.MESH_KW


class Body:
    """One physical body.  coup: 'ipc_only' | 'two_way_soft_constraint'.  mass=None for fixed bodies/planes (their
    rho is irrelevant) ; force=True attaches an IPC external body force driven per step."""

    def __init__(self, xml: str, *, mu: float, mass: float | None = None, coup: str = "ipc_only", force: bool = False,
                 ground_contact: bool = True, mesh_kw: dict[str, Any] | None = None):
        self.xml, self.mu, self.mass, self.coup, self.force, self.ground_contact = xml, mu, mass, coup, force, ground_contact
        self.mesh_kw = {**MESH_KW, **(mesh_kw or {})}


class Built:
    def __init__(self, scene, ents, bodies, B, dt, gravity, cand, info):
        self.scene, self.ents, self.bodies, self.B, self.dt, self.cand, self.info = scene, ents, bodies, B, dt, cand, info
        self.g = np.asarray(gravity, float)
        self.cp = scene.sim.coupler
        self.forces = {n: np.zeros((B, 3)) for n, b in bodies.items() if b.force}
        self._applied = {n: np.zeros((B, 3)) for n in self.forces}
        self.T: dict[str, np.ndarray] = {}; self.V: dict[str, np.ndarray] = {}; self.Vprev: dict[str, np.ndarray] = {}
        self.max_rigid_contacts = 0
        self.steps = 0
        self._link: dict[str, Any] = {}
        self._csf = None; self._ptypes: list[str] = []; self._vmap: dict[str, Any] | None = None
        self._read()
        for n in self.T:
            self.Vprev[n] = self.V[n].copy()

    # IPC-side state ---------------------------------------------------------
    def link_of(self, n):
        """The link IPC tracks for entity n (the one with collision geometry; MJCF pads' base link is not it)."""
        if n not in self._link:
            e = self.ents[n]
            cands = [l for l in self.cp._abd_slots_by_link if l.entity is e]
            self._link[n] = cands[0] if cands else None
        return self._link[n]

    def _read(self):
        data = self.cp._abd_data_by_link
        for n, e in self.ents.items():
            link = self.link_of(n)
            if link is not None and link in data:
                self.T[n] = np.stack([d.transform for d in data[link]]).astype(np.float64)
                self.V[n] = np.stack([d.velocity for d in data[link]]).astype(np.float64)
        if self.steps == 0:
            for n, (T, V) in self.info["init"].items():
                self.T[n] = T.copy(); self.V[n] = np.zeros_like(T) if V is None else V.copy()

    def step(self):
        for n in self.forces:
            self._applied[n][:] = self.forces[n]
        for n in self.T:
            self.Vprev[n] = self.V[n].copy()
        self.scene.step(); self.steps += 1
        self._read()

    def pose(self, n):
        T = self.T[n]
        return T[:, :3, 3].copy(), R_to_quat(T[:, :3, :3])

    def vel(self, n):
        """(world linear, world angular, body angular) from the IPC affine velocity."""
        T, V = self.T[n], self.V[n]
        R = T[:, :3, :3]
        W = V[:, :3, :3] @ np.linalg.inv(R)
        w = np.stack([W[:, 2, 1] - W[:, 1, 2], W[:, 0, 2] - W[:, 2, 0], W[:, 1, 0] - W[:, 0, 1]], 1) / 2
        return V[:, :3, 3].copy(), w, np.einsum("bji,bj->bi", R, w)

    def contact_force(self, n):
        """Total contact force ON IPC-only body n over the last step: m (dv/dt - g) - F_ext."""
        a = (self.V[n][:, :3, 3] - self.Vprev[n][:, :3, 3]) / self.dt
        f = self.bodies[n].mass * (a - self.g)
        if n in self._applied:
            f = f - self._applied[n]
        return f

    def coupling_force(self, n):
        """Force the bridge applies to the Genesis side of a two-way body (= contact force carried by the soft
        constraint), world frame, [B,3]."""
        cd = self.cp._coupling_data
        i = cd.links.index(self.link_of(n))
        return np.asarray(cd.out_forces[:, i], float).copy()

    # Direct IPC contact readout ------------------------------------------------
    # libuipc's ContactSystemFeature returns dE/dx of the contact potentials per vertex, for each primitive type
    # ('PT+N', 'EE+N', 'PH+N', ... barrier; '...+F' friction).  The potentials enter the incremental potential scaled by
    # dt^2, so the force on vertex i is f_i = -dE/dx_i / dt^2 (checked against m g at rest in genesis_ipc_cforce_probe).
    # It is evaluated from the current positions, independently of the velocity-based momentum balance above.
    def _vertex_map(self) -> dict[str, Any]:
        """name -> {'off': [B] global vertex offsets, 'nv': int, 'rest': [nv,3] body-frame vertex positions}."""
        if self._vmap is None:
            vm = {}
            for n in self.ents:
                link = self.link_of(n)
                if link is None or link not in self.cp._abd_slots_by_link:
                    continue
                slots = self.cp._abd_slots_by_link[link]
                offs = [int(np.asarray(s.geometry().meta().find("global_vertex_offset").view()).ravel()[0]) for s in slots]
                g0 = slots[0].geometry()
                rest = np.asarray(g0.vertices().find("position").view(), float).reshape(-1, 3)
                vm[n] = {"off": np.asarray(offs, int), "nv": len(rest), "rest": rest}
            self._vmap = vm
        return self._vmap

    def _feature(self):
        if self._csf is None:
            from uipc.core import ContactSystemFeature
            self._csf = self.cp._ipc_world.features().find(ContactSystemFeature)
            self._ptypes = list(self._csf.contact_primitive_types())
        return self._csf

    def vertex_forces(self) -> dict[str, dict[str, np.ndarray]]:
        """Contact force on every vertex of every IPC body after the last step, split into barrier (normal) and friction
        parts: {name: {'N': [B,nv,3], 'F': [B,nv,3]}} in the world frame (N)."""
        from uipc.geometry import Geometry
        f = self._feature(); vm = self._vertex_map()
        top = max(int(v["off"].max()) + v["nv"] for v in vm.values())
        acc = {"N": np.zeros((top + 1, 3)), "F": np.zeros((top + 1, 3))}
        for pt in self._ptypes:
            kind = "F" if pt.endswith("+F") else "N"
            g = Geometry(); f.contact_gradient(pt, g)
            inst = g.instances()
            if inst.size() == 0:
                continue
            idx = np.asarray(inst.find("i").view(), int).ravel()
            grad = np.asarray(inst.find("grad").view(), float).reshape(-1, 3)
            keep = (idx >= 0) & (idx <= top)
            np.add.at(acc[kind], idx[keep], grad[keep])
        s = -1.0 / (self.dt * self.dt)
        out = {}
        for n, v in vm.items():
            rows = v["off"][:, None] + np.arange(v["nv"])[None, :]
            out[n] = {k: s * a[rows] for k, a in acc.items()}
        return out

    def direct_force(self, n: str, side=None, vf=None) -> dict[str, np.ndarray]:
        """Total contact force ON body n from IPC's own potentials, [B,3] for 'N' (barrier), 'F' (friction) and 'total',
        plus 'nv_active' [B] (vertices carrying barrier force).  side: optional function of the body-frame rest vertex
        positions [nv,3] -> bool mask, to keep only the vertices on one face (e.g. lambda p: p[:, 2] < 0 for the force a
        plate receives from below)."""
        vf = vf if vf is not None else self.vertex_forces()
        v = vf[n]
        mask = np.ones(v["N"].shape[1], bool) if side is None else np.asarray(side(self._vertex_map()[n]["rest"]), bool)
        fN, fF = v["N"][:, mask].sum(1), v["F"][:, mask].sum(1)
        act = (np.linalg.norm(v["N"][:, mask], axis=2) > 0).sum(1)
        return {"N": fN, "F": fF, "total": fN + fF, "nv_active": act}

    def rigid_contacts(self) -> int:
        c = self.scene.rigid_solver.collider.get_contacts(as_tensor=True, to_torch=False)
        ga = np.asarray(c["geom_a"]); k = int((ga >= 0).sum(axis=-1).max()) if ga.size else 0
        self.max_rigid_contacts = max(self.max_rigid_contacts, k)
        return k

    def provenance(self) -> dict[str, Any]:
        o = self.cp.options
        return {"dt_s": self.dt, "substeps": 1, "coupler_options": {k: str(v) for k, v in o.model_dump().items()},
                "d_hat_m": d_hat(self.cand), "start_gap_m": start_gap(self.cand),
                "bodies": self.info["bodies"], "pair_friction": self.info["pairs"],
                "ground_pairs_disabled": self.info["ground_off"], "candidate": self.cand,
                "max_rigid_solver_contacts": self.max_rigid_contacts,
                "steps": self.steps}

    def destroy(self):
        try:
            self.scene.destroy()
        except Exception:  # noqa: BLE001
            pass


def _volumes(gs, xml_files: dict[str, Path], kws: dict[str, dict]) -> dict[str, float]:
    """Volume of each entity's collision surface as Genesis hands it to IPC (geom init_verts/init_faces)."""
    sc = gs.Scene(show_viewer=False)
    ents = {n: sc.add_entity(gs.morphs.MJCF(file=str(f), default_armature=None, **kws[n])) for n, f in xml_files.items()}
    out = {}
    for n, e in ents.items():
        vol = 0.0
        for g in e.geoms:
            if g.type != gs.GEOM_TYPE.PLANE and g.n_verts:
                vol += mesh_volume(g.init_verts, g.init_faces)
        out[n] = vol
    try:
        sc.destroy()
    except Exception:  # noqa: BLE001
        pass
    return out


def build(bodies: dict[str, Body], *, dt: float, gravity=(0.0, 0.0, -9.81), cand: dict[str, Any], n_envs: int,
          init: dict[str, tuple[np.ndarray, np.ndarray | None]] | None = None) -> Built:
    """init: name -> (T [B,4,4], V [B,4,4] or None) written into IPC before world init (IPC-only free bodies)."""
    gs = gs_init()
    import uipc
    from uipc.constitution import AffineBodyExternalBodyForce
    init = init or {}
    tmpdir = Path(tempfile.mkdtemp(prefix="gsipc-", dir="/dev/shm" if Path("/dev/shm").is_dir() else None))
    files = {}
    for n, b in bodies.items():
        files[n] = tmpdir / f"{n}.xml"; files[n].write_text(b.xml)
    vols = _volumes(gs, {n: f for n, f in files.items() if bodies[n].mass is not None}, {n: b.mesh_kw for n, b in bodies.items()})
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=dt, substeps=1, gravity=tuple(gravity)),
                     coupler_options=coupler_options(cand), show_viewer=False)
    ents, binfo = {}, {}
    for n, b in bodies.items():
        rho = b.mass / vols[n] if b.mass is not None else 1000.0
        mat = gs.materials.Rigid(rho=rho, friction=b.mu, coup_friction=b.mu, coup_type=b.coup, needs_coup=True)
        ents[n] = scene.add_entity(gs.morphs.MJCF(file=str(files[n]), default_armature=None, **b.mesh_kw), material=mat)
        binfo[n] = {"coup_type": b.coup, "coup_friction": b.mu, "rho": rho, "volume_m3": vols.get(n),
                    "ipc_mass_kg": (rho * vols[n]) if b.mass is not None else None, "manifest_mass_kg": b.mass,
                    "external_force": b.force, "mesh_kw": {k: str(v) for k, v in b.mesh_kw.items()}}
    cp = scene.sim.coupler
    fin = cp._finalize_ipc
    ext_holder: dict[str, Any] = {}
    ground_off: list[str] = []
    B = max(n_envs, 1)
    forces_ref: dict[str, np.ndarray] = {}

    def hook():
        if any(b.force for b in bodies.values()):
            ext = AffineBodyExternalBodyForce(); cp._ipc_constitution_tabular.insert(ext); ext_holder["ext"] = ext
        for n, b in bodies.items():
            links = [l for l in cp._abd_slots_by_link if l.entity is ents[n]]
            if not links:
                continue
            link = links[0]
            slots = cp._abd_slots_by_link[link]
            for env, slot in enumerate(slots):
                geo = slot.geometry()
                if n in init:
                    T, V = init[n]
                    uipc.view(geo.transforms())[0] = T[env]
                    if V is not None:
                        uipc.view(geo.instances().find("velocity"))[:] = V[env]
                if b.force:
                    ext_holder["ext"].apply_to(geo, np.zeros(12), enable=True)
            if b.force:
                for env in range(B):
                    obj = cp._ipc_scene.objects().find(f"rigid_link_{link.idx}_{env}")[0]

                    def anim(info, n=n, env=env):
                        f = forces_ref[n][env] if n in forces_ref else np.zeros(3)
                        for s in info.geo_slots():
                            v = uipc.view(s.geometry().instances().find("external_force"))
                            v[:] = 0; v[0][0:3] = f.reshape(3, 1) if v[0].ndim == 2 else f
                    cp._ipc_animator.insert(obj, anim)
        # driven bodies that must not touch the ground plane (geometry has no such contact in the manifest scene)
        for n, b in bodies.items():
            if not b.ground_contact and ents[n] in cp._ipc_abd_contacts:
                for ge, gelem in cp._ipc_ground_contacts.items():
                    cp._ipc_contact_tabular.insert(gelem, cp._ipc_abd_contacts[ents[n]], 0.0, RESISTANCE_DEFAULT, False)
                ground_off.append(n)
        fin()

    cp._finalize_ipc = hook
    scene.build(n_envs=n_envs)
    pairs = {}
    names = list(bodies)
    for i, a in enumerate(names):
        for bname in names[i + 1:]:
            pairs[f"{a}|{bname}"] = math.sqrt(bodies[a].mu * bodies[bname].mu)
    for n, e in ents.items():
        binfo[n]["geoms"] = [{"type": str(g.type), "n_verts": int(g.n_verts), "n_faces": int(len(g.init_faces)) if g.n_verts else 0}
                             for g in e.geoms]
    info = {"bodies": binfo, "pairs": pairs, "ground_off": ground_off, "init": init}
    b = Built(scene, ents, bodies, B, dt, gravity, cand, info)
    for n in b.forces:
        forces_ref[n] = b._applied[n]
    # Genesis-side pose of IPC-only free bodies mirrors IPC (cosmetic before the first step; IPC overwrites it)
    for n, (T, _) in init.items():
        if bodies[n].coup == "ipc_only" and ents[n].n_dofs == 6:
            p, q = T[:, :3, 3], R_to_quat(T[:, :3, :3])
            ents[n].set_qpos(np.concatenate([p, q], 1).astype(np.float32), zero_velocity=True)
    return b


def finite_rows(*arrays):
    return G.finite_rows(*arrays)


# ------------------------------------------------------------------ crash isolation
# libuipc aborts the whole process (C++ assertion) when a Newton/PCG solve diverges.  Every simulation call therefore
# runs in a persistent spawn-child; a dead child marks that call as failed (its rollouts count as non-finite) and a
# fresh child serves the next call.  The parent never initialises Genesis.
def _worker(conn, paths, errfile=None):
    sys.path[:0] = [p for p in paths if p not in sys.path]
    if errfile:                     # keep the child's C++ stderr (libuipc abort / CUDA messages) for the parent to classify
        fd = os.open(errfile, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644); os.dup2(fd, 2)
    while True:
        msg = conn.recv()
        if msg is None:
            return
        mod, fn, kw = msg
        try:
            conn.send(("ok", getattr(importlib.import_module(mod), fn)(**kw)))
        except Exception:  # noqa: BLE001
            conn.send(("err", traceback.format_exc()))


class Isolated:
    OOM_WAIT_S, OOM_RETRIES = 120, 120         # a CUDA out-of-memory death is an environment failure, not a solver one

    def __init__(self):
        self.p = None; self.conn = None; self.failures: list[dict[str, Any]] = []; self.oom_retries: list[dict[str, Any]] = []
        self.errfile = None

    def _start(self):
        ctx = mp.get_context("spawn")
        # the child must start through the same loader wrapper as the parent (bundled glibc / cuBLAS paths)
        exe = os.environ.get("IPC_CHILD_PY", "/root/autodl-tmp/genesis-runtime/bin/python-genesis")
        if Path(exe).exists():
            ctx.set_executable(exe)
        a, b = ctx.Pipe()
        self.errfile = tempfile.mktemp(prefix="gsipc-child-", suffix=".err")
        self.p = ctx.Process(target=_worker, args=(b, list(sys.path), self.errfile), daemon=True)
        self.p.start(); b.close(); self.conn = a

    def _stop(self):
        if self.p is not None:
            try:
                self.conn.send(None); self.p.join(10)
            except Exception:  # noqa: BLE001
                pass
            if self.p.is_alive():
                self.p.kill()
        self.p = None

    def _err_tail(self) -> str:
        try:
            return Path(self.errfile).read_text(errors="replace")[-3000:] if self.errfile else ""
        except OSError:
            return ""

    @staticmethod
    def _is_oom(text: str) -> bool:
        t = text.lower()
        return "out of memory" in t or "cudaerrormemoryallocation" in t

    def call(self, mod: str, fn: str, tag: str = "", **kw):
        """Returns (result, None) or (None, reason).  A CUDA out-of-memory death (GPU shared with other jobs) is retried
        after OOM_WAIT_S, up to OOM_RETRIES times, and logged in oom_retries; only other deaths count as failures."""
        for attempt in range(self.OOM_RETRIES + 1):
            r, why, oom = self._call_once(mod, fn, tag, **kw)
            if not oom:
                return r, why
            self.oom_retries.append({"call": tag or fn, "attempt": attempt + 1, "time": time.strftime("%Y-%m-%dT%H:%M:%S")})
            print("OOM-RETRY", tag, f"attempt {attempt + 1}; waiting {self.OOM_WAIT_S}s", flush=True)
            time.sleep(self.OOM_WAIT_S)
        why = f"CUDA out of memory after {self.OOM_RETRIES} retries (environment failure, not a solver result)"
        self.failures.append({"call": tag or fn, "reason": why, "environment": True}); print("FAILED", tag, why, flush=True)
        return None, why

    def _call_once(self, mod: str, fn: str, tag: str = "", **kw):
        if self.p is None or not self.p.is_alive():
            self._start()
        self.conn.send((mod, fn, kw))
        try:
            st, val = self.conn.recv()
        except (EOFError, ConnectionResetError, OSError):
            self.p.join(5); code = self.p.exitcode; self.p = None
            err = self._err_tail()
            if self._is_oom(err):
                return None, None, True
            last = [l for l in err.strip().splitlines() if l.strip()][-1:] or [""]
            why = f"IPC process died (exit {code}; libuipc abort, e.g. diverged PCG/Newton solve): {last[0][:200]}"
            self.failures.append({"call": tag or fn, "reason": why, "stderr_tail": err[-1500:]}); print("FAILED", tag, why, flush=True)
            return None, why, False
        if st == "err":
            self._stop()
            if self._is_oom(val):
                return None, None, True
            why = val.strip().splitlines()[-1][:300]
            self.failures.append({"call": tag or fn, "reason": why, "traceback": val[-2000:]}); print("FAILED", tag, why, flush=True)
            return None, why, False
        return val, None, False


ISO = Isolated()


def save_json(path, obj):
    """genesis_rigid_common.save_json plus the isolation log (solver deaths and CUDA out-of-memory retries)."""
    if isinstance(obj, dict):
        obj = {**obj, "isolation_log": {"failures": ISO.failures, "oom_retries": ISO.oom_retries}}
    return G.save_json(path, obj)


def obj_vertices(path: Path) -> np.ndarray:
    return np.array([list(map(float, l.split()[1:4])) for l in open(path) if l.startswith("v ")])


def hull2d(pts: np.ndarray) -> np.ndarray:
    pts = np.unique(np.round(pts, 9), axis=0); pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in pts[::-1]:
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return np.array(lo[:-1] + hi[:-1])


def point_polygon(p: np.ndarray, poly: np.ndarray) -> tuple[float, np.ndarray]:
    """Signed distance (negative inside) from p to convex CCW polygon and the unit direction from p to the polygon."""
    best, cp = np.inf, None
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0, 1); q = a + t * (b - a)
        d = np.linalg.norm(p - q)
        if d < best:
            best, cp = d, q
    inside = all(((poly[(i + 1) % len(poly)] - poly[i])[0] * (p - poly[i])[1]
                  - (poly[(i + 1) % len(poly)] - poly[i])[1] * (p - poly[i])[0]) > 0 for i in range(len(poly)))
    if inside:
        d = poly.mean(0) - p
        return -best, d / max(np.linalg.norm(d), 1e-12)
    d = cp - p
    return best, d / max(np.linalg.norm(d), 1e-12)
