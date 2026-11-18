import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import sys, json
import genesis_ipc_common as I
import genesis_ipc_cforce_probe as C
if __name__ == "__main__":
    I.gs_init()
    for tol in ("1e-5", "1e-6"):
        x = C.push(0.002, I.parse_candidate("d_hat=1mm,eps_v=1e-3,newton_tol=" + tol), 0.7)
        print("creep", tol, json.dumps(I.json_safe(x)), flush=True)
