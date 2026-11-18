#!/usr/bin/env python3
"""Same declared frozen B=5 static-friction matrix with MuJoCo Warp."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import argparse, importlib.util, json, math
from pathlib import Path
from run_threebox_sap_frozen_generalization import CASES

def module(path: Path):
    s=importlib.util.spec_from_file_location("mj_threebox",path)
    if s is None or s.loader is None: raise RuntimeError(f"cannot import {path}")
    m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m

def safe(value):
    if isinstance(value, float): return value if math.isfinite(value) else None
    if isinstance(value, dict): return {k:safe(v) for k,v in value.items()}
    if isinstance(value, list): return [safe(v) for v in value]
    return value

def main() -> None:
    p=argparse.ArgumentParser(); p.add_argument("--source",type=Path,required=True); p.add_argument("--out",type=Path,required=True); p.add_argument("--repeats",type=int,default=20); a=p.parse_args()
    if a.repeats !=20: raise ValueError("aligned protocol requires R=20")
    m=module(a.source); frozen={"k_pair":1e4,"impratio":10.,"solreffriction_tc":.004,"noslip_iterations":2}; rows=[]
    for i,c in enumerate(CASES):
        xml=m.three_box_xml(dt=c["dt_ms"]*1e-3,mass=c["box_mass_kg"],mu=c["mu"],**frozen)
        r=m.run_three_box_rollouts(backend="mujoco_warp",xml_str=xml,pushed_body_index=1 if c["pushed_body"]=="middle" else 2,force_target_N=c["force_N"],mu=c["mu"],mass=c["box_mass_kg"],dt=c["dt_ms"]*1e-3,n_rollouts=20,seed_base=42000+i)
        r["condition"]=c; rows.append(r); print(c["id"],r["pass_fraction_p_hat"],r["tail_rel_disp_mean_mm"],r["tail_creep_mean_um_s"],flush=True)
    out={"protocol":"aligned-frozen-generalizability-v1","task":"three-box-push","engine":"MuJoCo Warp","B":5,"R_per_condition":20,"frozen_candidate":frozen,
         "axes":{"force_ratio_to_ideal_breakaway":[.5,1.,1.5],"box_mass_kg":[.5,1.,2.],"mu":[.3,.5,.7],"dt_ms":[.25,.5,1.],"pushed_body":["middle","top"]},"seed_formula":"42000 + condition index","conditions":rows}
    a.out.mkdir(parents=True,exist_ok=True); (a.out/"mjwarp_threebox_aligned_frozen_r20.json").write_text(json.dumps(safe(out),indent=2,allow_nan=False)+"\n")
if __name__=="__main__": main()
