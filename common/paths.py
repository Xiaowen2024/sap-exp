"""Repository paths and import setup shared by every runner.

Scripts live at different depths under tasks/, engines/, tools/ and common/, so
none of them should compute the repository root with ``parents[N]``.  Each
script starts with the bootstrap below and then uses the constants here::

    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
    from common.paths import REPO_ROOT  # noqa: E402

Importing this module also puts every folder that holds flat (non-package)
modules on ``sys.path``, so the historical flat imports keep working wherever
the importing script lives, e.g. ``import strict_v3_common``,
``import genesis_ipc_common`` or ``importlib.import_module("run_threebox_genesis_sap")``.
Module basenames are unique across those folders (checked by
``tools/check_layout.py``).  The parents of the two real packages
(``real_playback``, ``robodojo_playback``) are added too, so
``python -m real_playback.rp_run`` and ``import robodojo_playback`` work.
"""
from __future__ import annotations

import sys
from functools import lru_cache
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
COMMON = REPO_ROOT / "common"
TASKS = REPO_ROOT / "tasks"
ENGINES = REPO_ROOT / "engines"
TOOLS = REPO_ROOT / "tools"
PROTOCOL = REPO_ROOT / "protocol"
MANIFESTS = PROTOCOL / "manifests"
RESULTS = REPO_ROOT / "exp-results"
EXTERNAL = REPO_ROOT / "external"

CODE_ROOTS = (COMMON, ENGINES, TASKS, TOOLS)
_SKIP = {"__pycache__", "assets", "configs", "sop"}


def _is_package(d: Path) -> bool:
    return (d / "__init__.py").is_file()


@lru_cache(maxsize=None)
def module_dirs() -> tuple[Path, ...]:
    """Folders that hold flat modules, plus the parent folder of each package."""
    found: list[Path] = [COMMON]
    for root in (ENGINES, TASKS, TOOLS):
        stack = [root]
        while stack:
            d = stack.pop()
            subdirs = sorted(c for c in d.iterdir() if c.is_dir() and c.name not in _SKIP and not c.name.startswith("."))
            if any(_is_package(c) for c in subdirs) or any(d.glob("*.py")):
                found.append(d)
            stack.extend(c for c in subdirs if not _is_package(c))
    return tuple(found)


@lru_cache(maxsize=None)
def find_script(name: str) -> Path:
    """Locate a runner by file name (e.g. ``run_pinch_v3_mjwarp.py``) anywhere in the code folders."""
    hits = [d / name for d in module_dirs() if (d / name).is_file()]
    if len(hits) != 1:
        raise FileNotFoundError(f"{name}: expected exactly one match under {', '.join(str(r) for r in CODE_ROOTS)}, got {hits}")
    return hits[0]


def _install() -> None:
    present = set(sys.path)
    # Directly after the script's own folder: these modules must win over stale
    # copies on PYTHONPATH (e.g. an old /root/SIM-EXP/runners directory).
    pos = 1 if sys.path else 0
    for d in reversed(module_dirs()):
        s = str(d)
        if s not in present:
            sys.path.insert(pos, s)


_install()
