#!/usr/bin/env python3
"""Fail fast when a frozen-generalizability manifest misses SOP essentials.

Checks (2026-11-04 revision):
  * protocol, R=20, selection ID disjoint from held-out IDs, unique IDs;
  * at least three held-out dt levels;
  * outcome labels match the declared ``outcome_type`` (binary, multiclass, gate, continuous);
    multiclass three-box labels are recomputed from rigid slip/tip thresholds with margins;
      - "binary"     every held-out cell needs expected retain/release, and every
                     dt level needs both classes (so dt is not confounded with the label);
      - "gate"       per-cell pass/fail gate rate, no retain/release labels allowed;
      - "continuous" continuous loss, no retain/release labels allowed;
  * one to five B=5 candidates per engine, the first being the native default (fewer than five needs a rationale);
  * MuJoCo refsafe: any MuJoCo candidate time constant ``tc=<x>ms`` must be at least
    2*dt for every dt in the manifest, otherwise MuJoCo silently clamps it and the
    dt axis also changes the contact law; duplicate effective candidates are rejected.
"""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)

import argparse
import json
import re
from pathlib import Path


def mujoco_tc_ms(candidate: str) -> float | None:
    m = re.search(r"tc\s*=\s*([0-9.]+)\s*ms", candidate)
    return float(m.group(1)) if m else None


def validate(payload: dict) -> list[str]:
    errors: list[str] = []
    if payload.get("protocol") != "aligned-frozen-generalizability-v1":
        errors.append("protocol must be aligned-frozen-generalizability-v1")
    if payload.get("R") != 20:
        errors.append("R must be exactly 20")
    selection = payload.get("selection")
    conditions = payload.get("held_out_conditions")
    selections = selection if isinstance(selection, list) else [selection]
    for sel in selections:
        if not isinstance(sel, dict) or not sel.get("id") or not sel.get("selection_only"):
            errors.append("every selection cell must have an ID and selection_only=true")
    selections = [s for s in selections if isinstance(s, dict)]
    selection = selections[0] if selections else {}
    if not isinstance(conditions, list) or not conditions:
        errors.append("held_out_conditions is missing")
        conditions = []
    ids = [row.get("id") for row in conditions]
    if any(not value for value in ids) or len(set(ids)) != len(ids):
        errors.append("held-out IDs must be nonempty and unique")
    if any(sel.get("id") in ids for sel in selections):
        errors.append("selection IDs must be disjoint from held-out IDs")
    dts = sorted({row.get("dt_ms") for row in conditions if row.get("dt_ms") is not None})
    if len(dts) < 3:
        errors.append("held-out matrix needs at least three dt values")

    outcome = payload.get("outcome_type")
    if outcome not in ("binary", "multiclass", "gate", "continuous"):
        errors.append("outcome_type must be 'binary', 'multiclass', 'gate', or 'continuous'")
    labelled = [row for row in conditions if "expected" in row]
    if outcome == "binary":
        if any(row.get("expected") not in ("retain", "release") for row in conditions):
            errors.append("binary task: every held-out cell needs expected retain/release")
        for dt in dts:
            classes = {row.get("expected") for row in conditions if row.get("dt_ms") == dt}
            if classes != {"retain", "release"}:
                errors.append(f"binary task: dt={dt} ms must contain both retain and release cells (got {sorted(c for c in classes if c)})")
    elif outcome == "multiclass":
        classes = set(payload.get("classes") or [])
        if len(classes) < 3:
            errors.append("multiclass task must declare its classes")
        for row in list(conditions) + selections:
            if row.get("expected") not in classes:
                errors.append(f"multiclass: {row.get('id')} has expected={row.get('expected')!r} not in {sorted(classes)}")
        for dt in dts:
            got = {row.get("expected") for row in conditions if row.get("dt_ms") == dt}
            if len(got) < 2:
                errors.append(f"multiclass: dt={dt} ms needs at least two outcome classes (got {sorted(c for c in got if c)})")
        if payload.get("task") == "three-box-push":
            from strict_v3_common import box_v4_expected
            for row in list(conditions) + selections:
                lab, _ = box_v4_expected(row)
                if lab != row.get("expected"):
                    errors.append(f"three-box physics: {row.get('id')} analytic label {lab!r} != declared {row.get('expected')!r}")
    elif outcome in ("gate", "continuous") and labelled:
        errors.append(f"{outcome} task: remove retain/release labels from {[r['id'] for r in labelled]}")

    rule = payload.get("candidate_rule", {})
    inherited = payload.get("frozen_from")
    for engine in ("sap_candidates", "mujoco_warp_candidates"):
        candidates = rule.get(engine)
        if inherited:
            # no selection on this task: exactly one candidate, the one frozen by the referenced manifest's run
            want = inherited.get("sap" if engine == "sap_candidates" else "mujoco_warp")
            if not inherited.get("manifest") or not inherited.get("manifest_sha256"):
                errors.append("frozen_from must name the source manifest and its sha256")
            if candidates != [want]:
                errors.append(f"{engine}: with frozen_from, the only candidate must be the inherited one ({want!r})")
            continue
        if not isinstance(candidates, list) or not (1 <= len(candidates) <= 5):
            errors.append(f"{engine} must declare one to five B=5 candidates (B=5 is 'at most five')")
        elif len(candidates) != 5 and not rule.get("rationale"):
            errors.append(f"{engine} declares {len(candidates)} candidates; a rationale is required when fewer than five apply")
        elif candidates and candidates[0] != "native default":
            errors.append(f"{engine}: candidate 1 must be the literal native default (B=1)")
    all_dt = sorted(set(dts) | {sel.get("dt_ms") for sel in selections if sel.get("dt_ms")})
    mj = rule.get("mujoco_warp_candidates") or []
    for dt in all_dt:
        effective = []
        for c in mj:
            tc = mujoco_tc_ms(c)
            if tc is not None and tc < 2 * dt - 1e-12:
                errors.append(f"MuJoCo refsafe: candidate '{c}' has tc={tc} ms < 2*dt={2*dt} ms at dt={dt} ms (silently clamped)")
            effective.append(re.sub(r"tc\s*=\s*[0-9.]+\s*ms", f"tc={max(tc, 2*dt):g}ms", c) if tc is not None else c)
        if len(set(effective)) != len(effective):
            errors.append(f"MuJoCo refsafe: candidates collapse to duplicates at dt={dt} ms: {effective}")
    if not payload.get("fixed", {}).get("pass_gate"):
        errors.append("fixed.pass_gate is missing")
    return errors


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest", type=Path, nargs="+")
    args = parser.parse_args()
    failed = False
    for path in args.manifest:
        errors = validate(json.loads(path.read_text()))
        if errors:
            failed = True
            print(f"INVALID {path}:\n  - " + "\n  - ".join(errors))
        else:
            print(f"valid   {path}")
    raise SystemExit(1 if failed else 0)


if __name__ == "__main__":
    main()
