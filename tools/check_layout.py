"""Check the code layout invariants that common/paths.py relies on.

    python3 tools/check_layout.py

Fails (exit 1) when
  * two flat modules share a basename (flat imports would become ambiguous),
  * a flat module shadows a standard-library module,
  * a script computes the repository root with ``parents[N]`` instead of REPO_ROOT,
  * a script imports a flat module from another folder without the bootstrap,
  * a script still points at the pre-reorganization ``scripts/`` or ``sim_protocol/`` folders.
No simulator is imported; this is a pure source check.
"""
from __future__ import annotations

import ast
import collections
import re
import sys
import warnings
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.paths import REPO_ROOT, module_dirs  # noqa: E402

OLD = re.compile(r"""(?<![\w-])(ROOT|REPO_ROOT|root)\s*/\s*["'](scripts|sim_protocol)\b""")
PARENTS = re.compile(r"Path\(__file__\)\.resolve\(\)\.parents\[\d\]")


def main() -> int:
    errors: list[str] = []
    dirs = module_dirs()
    by_name: dict[str, list[Path]] = collections.defaultdict(list)
    for d in dirs:
        for p in d.glob("*.py"):
            by_name[p.stem].append(p)
    for name, ps in by_name.items():
        if len(ps) > 1 and name != "__init__":
            errors.append(f"duplicate module name {name}: {[str(p.relative_to(REPO_ROOT)) for p in ps]}")
        if name in sys.stdlib_module_names:
            errors.append(f"{ps[0].relative_to(REPO_ROOT)} shadows the standard-library module {name}")

    for d in dirs:
        for p in d.glob("*.py"):
            rel = p.relative_to(REPO_ROOT)
            src = p.read_text()
            if rel.parts[0] != "common" and p.resolve() != Path(__file__).resolve() and PARENTS.search(src):
                errors.append(f"{rel}: uses Path(__file__).resolve().parents[N]; use REPO_ROOT from common.paths")
            if OLD.search(src):
                errors.append(f"{rel}: refers to the old scripts/ or sim_protocol/ folder")
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    tree = ast.parse(src)
            except SyntaxError as exc:
                errors.append(f"{rel}: syntax error {exc}")
                continue
            imported = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
            imported |= {n.module.split(".")[0] for n in ast.walk(tree)
                         if isinstance(n, ast.ImportFrom) and n.level == 0 and n.module}
            own = {q.stem for q in p.parent.glob("*.py")}
            foreign = sorted(n for n in imported if n in by_name and n not in own)
            if foreign and "common.paths" not in src and not (p.parent / "__init__.py").exists():
                errors.append(f"{rel}: imports {foreign} from other folders without the common.paths bootstrap")

    for e in errors:
        print("ERROR", e)
    print(f"checked {sum(len(v) for v in by_name.values())} modules in {len(dirs)} folders: "
          f"{'OK' if not errors else str(len(errors)) + ' problem(s)'}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
