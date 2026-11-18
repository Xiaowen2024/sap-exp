"""Shared Genesis 1.4.6 RigidSolver adapter for the SIM-EXP protocol tasks.

Each task runner builds its scene from the task manifest (never from a tuned MuJoCo MJCF), steps Genesis, and hands
the raw trajectories to the same engine-independent *_common module the SAP and MuJoCo Warp drivers use.  This module
only supplies what is Genesis-specific; see sim_protocol/genesis_rigid_runner_sop.md.

Scene rules enforced here (each one closes a silent-mismatch route found in the 2026-11-14 API probes):
- every physical body is its own entity: Genesis drops self-collision between links of one entity that touch in the
  initial configuration (RigidOptions.enable_neutral_collision=False), which would delete stacked/initially touching
  contacts; separate entities keep RigidOptions at its defaults;
- MJCF snippets carry geometry, mass/inertia, joints and friction only (no solref/solimp, no <pair>, no <option>);
  Genesis ignores <pair>, margin and gap, so contacts are geom-to-geom with Genesis' max friction rule;
- default_armature=None (Genesis otherwise adds 0.1 rotor inertia to revolute/prismatic joints);
- meshes: convex hull, no decimation, no convex decomposition, matching MuJoCo's convex-hull mesh collision;
- dt is the actual step (substeps=1); B=1 sets no numerical contact/solver option.
"""
from __future__ import annotations

import hashlib
import importlib.metadata as metadata
import json
import math
import os
import platform
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np

HERE = Path(__file__).resolve().parent


def _find_root() -> Path:
    cands = [Path(os.environ["SIMEXP_ROOT"])] if os.environ.get("SIMEXP_ROOT") else []
    cands += [HERE.parents[1], Path("/root/SIM-EXP")]
    for c in cands:
        if (c / "exp-results").is_dir() and (c / "sim_protocol/manifests").is_dir():
            return c
    raise RuntimeError("SIM-EXP root not found; set SIMEXP_ROOT")


ROOT = _find_root()
_PROTO = [Path(os.environ["SIMEXP_PROTOCOL_DIR"])] if os.environ.get("SIMEXP_PROTOCOL_DIR") else []
_PROTO += [ROOT / "scripts/protocol_v2", ROOT / "runners"]
PROTOCOL_DIR = next(p for p in _PROTO if (p / "impact_v2_common.py").exists())
for p in (PROTOCOL_DIR, ROOT / "scripts", HERE):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from strict_v3_common import json_safe, load_manifest  # noqa: E402,F401

REGISTRY = ROOT / "sim_protocol/manifests/genesis_rigid_rerun.json"
ENGINE = "Genesis RigidSolver"
PINNED = {"genesis-world": "1.4.6"}
SEED = 20261114
B_FLOOR = 0.02            # 1/s; a declared b=0 candidate uses this damping (Genesis forbids dampratio=0)
_GS = None


# ------------------------------------------------------------------ init / provenance
def gs_init():
    global _GS
    if _GS is not None:
        return _GS
    for name, ver in PINNED.items():
        if metadata.version(name) != ver:
            raise RuntimeError(f"{name} {metadata.version(name)} != pinned {ver}")
    import torch
    import genesis as gs
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA Torch is required; no CPU fallback")
    gs.init(backend=gs.cuda, seed=SEED, logging_level=os.environ.get("GS_LOG", "warning"))
    _GS = gs
    return gs


def versions() -> dict[str, Any]:
    import torch
    out = {n: metadata.version(n) for n in ("genesis-world", "quadrants", "torch", "numpy", "mujoco")}
    out.update(python=sys.version.split()[0], libc=platform.libc_ver(), device=torch.cuda.get_device_name(0))
    return out


def registry_task(task_id: str, manifest_path: Path) -> tuple[dict[str, Any], dict[str, Any], str]:
    reg = json.loads(REGISTRY.read_text())
    spec = next(t for t in reg["tasks"] if t["id"] == task_id)
    man, sha = load_manifest(manifest_path)
    if Path(manifest_path).name != spec["manifest"]:
        raise ValueError(f"{task_id}: registry expects {spec['manifest']}, got {Path(manifest_path).name}")
    if sha != spec["sha256"]:
        raise ValueError(f"{task_id}: manifest sha changed ({sha}); declare a new registry revision first")
    return reg, spec, sha


def registry_id_for(manifest_path: Path) -> str:
    reg = json.loads(REGISTRY.read_text())
    return next(t["id"] for t in reg["tasks"] if t["manifest"] == Path(manifest_path).name)


def with_candidates(man: dict[str, Any], spec: dict[str, Any], key: str = "genesis_rigid_candidates") -> dict[str, Any]:
    """In-memory copy of the manifest whose candidate_rule carries the registry-declared Genesis candidates."""
    m = json.loads(json.dumps(man))
    m["candidate_rule"][key] = list(spec["candidates"])
    return m


def header(man: dict[str, Any], sha: str, spec: dict[str, Any]) -> dict[str, Any]:
    return {"protocol": man["protocol"], "task": man["task"], "revision": man["revision"], "manifest_sha256": sha,
            "engine": ENGINE, "versions": versions(), "registry": str(REGISTRY.relative_to(ROOT)),
            "registry_sha256": hashlib.sha256(REGISTRY.read_bytes()).hexdigest(),
            "genesis_candidates": spec["candidates"], "nonnative_settings": spec.get("nonnative", {}),
            "contact_force_readout": "collider.get_contacts force (on geom_b, world frame); normal load = |f.n|"}


def save_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(json_safe(data), indent=1) + "\n")
    tmp.replace(path)


# ------------------------------------------------------------------ candidates
def parse_candidate(text: str, spec: dict[str, Any]) -> dict[str, Any]:
    """'native default' or comma list of tc=4ms, dr, dmax, dmin, k, b, impratio, noslip, iterations, tolerance,
    cone, integrator.  Registry 'nonnative' settings fill anything a non-native candidate leaves unstated."""
    if text.strip() == "native default":
        return {"id": "native_default", "native": True, "text": text}
    c: dict[str, Any] = {"id": text, "native": False, "text": text, **json.loads(json.dumps(spec.get("nonnative", {})))}
    for part in text.replace(" ", "").split(","):
        k, v = part.split("=")
        if k == "tc":
            c["tc"] = float(v.removesuffix("ms")) * (1e-3 if v.endswith("ms") else 1.0)
        elif k in ("dr", "dmax", "dmin", "k", "b", "impratio", "tolerance"):
            c[k] = float(v)
        elif k in ("noslip", "iterations"):
            c[k] = int(v)
        elif k in ("cone", "integrator"):
            c[k] = v
        else:
            raise ValueError(f"unknown candidate field {k!r} in {text!r}")
    return c


def direct_to_tc_dr(k: float, b: float) -> tuple[float, float]:
    """Exact MuJoCo direct (k, b) -> (timeconst, dampratio): MuJoCo/Genesis use b=2/(dmax tc),
    k=1/(dmax^2 tc^2 dr^2) and the direct form b_dir/dmax, k_dir/dmax^2, so tc=2/b and dr=b/(2 sqrt k)."""
    b = max(b, B_FLOOR)
    return 2.0 / b, b / (2.0 * math.sqrt(k))


def sol_vector(c: dict[str, Any], tc: float | None = None, dr: float | None = None) -> list[float] | None:
    """7-vector (timeconst, dampratio, dmin, dmax, width, mid, power) for a non-native candidate, else None."""
    if c.get("native"):
        return None
    if tc is None:
        if "k" in c:
            tc, dr = direct_to_tc_dr(c["k"], c.get("b", 0.0))
        elif "tc" in c:
            tc, dr = c["tc"], c.get("dr", 1.0)
        else:
            return None                                    # option-only candidate: geoms keep native sol params
    si = list(c.get("solimp", [0.9, 0.95, 0.001, 0.5, 2.0]))
    if "dmax" in c:
        si[1] = c["dmax"]; si[0] = c.get("dmin", c["dmax"])
    return [float(tc), float(dr if dr is not None else 1.0), *map(float, si)]


def rigid_options(c: dict[str, Any]):
    gs = gs_init()
    kw: dict[str, Any] = {}
    if not c.get("native"):
        if "integrator" in c:
            kw["integrator"] = getattr(gs.integrator, c["integrator"])
        if "cone" in c:
            kw["friction_cone"] = getattr(gs.friction_cone, c["cone"])
        for src, dst in (("impratio", "impratio"), ("noslip", "noslip_iterations"),
                         ("iterations", "iterations"), ("tolerance", "tolerance")):
            if src in c:
                kw[dst] = c[src]
    return gs.options.RigidOptions(**kw)


# ------------------------------------------------------------------ scene construction
MESH_KW = dict(convexify=True, decimate=False, decompose_object_error_threshold=float("inf"),
               decompose_robot_error_threshold=float("inf"))


def mjcf(body: str, *, meshdir: Path | None = None, assets: str = "") -> str:
    comp = f'<compiler meshdir="{meshdir}" angle="radian"/>' if meshdir else '<compiler angle="radian"/>'
    return f'<mujoco>{comp}<asset>{assets}</asset><worldbody>{body}</worldbody></mujoco>'


class Built:
    """A built scene: entities by name, provenance, contact helpers."""

    def __init__(self, scene, ents: dict[str, Any], n_envs: int, dt: float, cand: dict[str, Any], xml: dict[str, str]):
        self.scene, self.ents, self.B, self.dt, self.cand, self.xml = scene, ents, n_envs, dt, cand, xml
        self.rs = scene.rigid_solver
        self.geom_owner = {}
        for name, e in ents.items():
            for g in e.geoms:
                self.geom_owner[int(g.idx)] = name
        self.max_contacts_seen = 0

    def geoms_of(self, name: str) -> list[int]:
        return [int(g.idx) for g in self.ents[name].geoms]

    def contacts(self) -> dict[str, np.ndarray]:
        c = self.rs.collider.get_contacts(as_tensor=True, to_torch=False)
        out = {k: np.asarray(v) for k, v in c.items()}
        if out["geom_a"].ndim == 1:                        # n_envs == 0 never used here; keep batch axis anyway
            out = {k: v[None] for k, v in out.items()}
        live = (out["geom_a"] >= 0).sum(axis=1)
        self.max_contacts_seen = max(self.max_contacts_seen, int(live.max()) if live.size else 0)
        return out

    def pair_normal_load(self, a: str, b: str, c: dict[str, np.ndarray] | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Per-env summed normal load |f.n| and contact count between entities a and b."""
        c = self.contacts() if c is None else c
        ga, gb = c["geom_a"], c["geom_b"]
        own = np.vectorize(lambda g: self.geom_owner.get(int(g), ""))(ga) if ga.size else ga
        owb = np.vectorize(lambda g: self.geom_owner.get(int(g), ""))(gb) if gb.size else gb
        m = ((own == a) & (owb == b)) | ((own == b) & (owb == a))
        fn = np.abs(np.sum(c["force"] * c["normal"], axis=-1)) * m if ga.size else np.zeros((self.B, 0))
        return fn.sum(axis=1) if ga.size else np.zeros(self.B), m.sum(axis=1) if ga.size else np.zeros(self.B)

    def force_on(self, target: str, other: str, c: dict[str, np.ndarray] | None = None) -> tuple[np.ndarray, np.ndarray]:
        """Per-env summed world-frame contact force ON entity target FROM entity other, and contact count."""
        c = self.contacts() if c is None else c
        ga, gb = c["geom_a"], c["geom_b"]
        if not ga.size:
            return np.zeros((self.B, 3)), np.zeros(self.B)
        own = np.vectorize(lambda g: self.geom_owner.get(int(g), ""))(ga)
        owb = np.vectorize(lambda g: self.geom_owner.get(int(g), ""))(gb)
        on_b = (owb == target) & (own == other)          # stored force acts on geom_b
        on_a = (own == target) & (owb == other)
        f = c["force"] * on_b[..., None] - c["force"] * on_a[..., None]
        return f.sum(axis=1), (on_a | on_b).sum(axis=1)

    def provenance(self) -> dict[str, Any]:
        import torch

        def arr(x):
            if isinstance(x, (tuple, list)):
                return [arr(y) for y in x]
            return x.detach().cpu().numpy().tolist() if isinstance(x, torch.Tensor) else np.asarray(x).tolist()
        opts = self.rs._options
        ents = {}
        for name, e in self.ents.items():
            d = {"n_links": e.n_links, "n_dofs": e.n_dofs, "links_mass": arr(e.get_links_mass())}
            if e.n_dofs:
                d["dofs_armature"] = arr(e.get_dofs_armature())
                d["dofs_damping"] = arr(e.get_dofs_damping())
            d["links_inertia"] = arr(e.get_links_inertia())
            d["geoms"] = [{"idx": int(g.idx), "type": str(g.type), "n_verts": int(getattr(g, "n_verts", -1))} for g in e.geoms]
            ents[name] = d
        sol = arr(self.rs.get_sol_params())
        return {"dt_s": self.dt, "substeps": 1, "min_contact_timeconst_s": 2.0 * self.dt,
                "rigid_options": {k: str(v) for k, v in opts.model_dump().items()},
                "geoms_friction": arr(self.rs.get_geoms_friction()),
                "geoms_friction_torsional": arr(self.rs.get_geoms_friction_torsional()),
                "geoms_sol_params": sol[0] if np.asarray(sol).ndim == 3 else sol,
                "entities": ents, "candidate": self.cand,
                "xml_sha256": {k: hashlib.sha256(v.encode()).hexdigest() for k, v in self.xml.items()},
                "max_contacts_seen": self.max_contacts_seen}

    def destroy(self) -> None:
        try:
            self.scene.destroy()
        except Exception:                                  # noqa: BLE001  (best effort; a fresh scene follows)
            pass


def build(entities: dict[str, str], *, dt: float, gravity=(0.0, 0.0, -9.81), cand: dict[str, Any], n_envs: int,
          sol: dict[str, list[float] | None] | None = None) -> Built:
    """entities: name -> MJCF string.  sol: optional per-entity 7-vector (applied to all its geoms after build);
    None/absent leaves Genesis' native geom solver parameters (the B=1 rule)."""
    gs = gs_init()
    scene = gs.Scene(sim_options=gs.options.SimOptions(dt=dt, substeps=1, gravity=tuple(gravity)),
                     rigid_options=rigid_options(cand), show_viewer=False)
    ents = {}
    tmpdir = Path(tempfile.mkdtemp(prefix="gsrigid-", dir="/dev/shm" if Path("/dev/shm").is_dir() else None))
    for name, xml in entities.items():
        f = tmpdir / f"{name}.xml"
        f.write_text(xml)
        ents[name] = scene.add_entity(gs.morphs.MJCF(file=str(f), default_armature=None, **MESH_KW))
    scene.build(n_envs=n_envs)
    b = Built(scene, ents, n_envs, dt, cand, entities)
    for name, vec in (sol or {}).items():
        if vec is not None:
            idx = b.geoms_of(name)
            b.rs.set_sol_params(np.tile(np.asarray(vec, dtype=np.float32), (len(idx), 1)), geoms_idx=idx)
    return b


def np_(x) -> np.ndarray:
    import torch
    return x.detach().cpu().numpy() if isinstance(x, torch.Tensor) else np.asarray(x)


def finite_rows(*arrays: np.ndarray) -> np.ndarray:
    """[R] bool: every array finite along all non-env axes (env axis first)."""
    ok = None
    for a in arrays:
        a = np.asarray(a, float)
        f = np.isfinite(a.reshape(a.shape[0], -1)).all(axis=1)
        ok = f if ok is None else ok & f
    return ok


class Timer:
    def __enter__(self):
        self.t0 = time.perf_counter(); return self

    def __exit__(self, *a):
        self.s = time.perf_counter() - self.t0
