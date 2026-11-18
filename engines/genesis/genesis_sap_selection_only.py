#!/usr/bin/env python3
"""Run a Genesis SAP task runner with the held-out matrix (and three-box sensitivity) removed: selection B=1..5 only.

User decision 2026-11-18: for the Genesis SAP coupler, run only the selection screen (all five declared candidates) and
report the problems found; no held-out matrix.  The runner itself is unchanged and executed as __main__; only the manifest
returned by genesis_sap_common.load_manifest is edited in memory (held_out_conditions = [], sensitivity cells = []); the
recorded manifest sha256 is that of the unmodified file.  Selection cells already computed hit the shared call cache.

Usage: genesis_sap_selection_only.py <runner.py> --manifest M --out DIR [runner args...]
"""
import json
import runpy
import sys
import time
from pathlib import Path

import genesis_sap_common as G

_orig = G.load_manifest


def _selection_only(path):
    man, sha = _orig(path)
    man = dict(man); man["held_out_conditions"] = []
    if isinstance(man.get("parameter_sensitivity"), dict):
        man["parameter_sensitivity"] = {**man["parameter_sensitivity"], "cells": []}
    return man, sha


G.load_manifest = _selection_only


def main():
    runner = Path(sys.argv[1]); args = sys.argv[2:]
    out = Path(args[args.index("--out") + 1])
    sys.argv = [str(runner)] + args
    t0 = time.time()
    runpy.run_path(str(runner), run_name="__main__")
    for f in out.glob("genesis_sap_*.json"):
        d = json.loads(f.read_text())
        d["complete"] = False
        d["selection_only"] = {"note": "user decision 2026-11-18: selection B=1..5 only; held-out matrix not run",
                               "heldout_conditions_run": 0, "wall_s": time.time() - t0}
        f.write_text(json.dumps(d, indent=1) + "\n")
        print("selection-only artifact", f, flush=True)


if __name__ == "__main__":
    main()
