#!/usr/bin/env python3
"""Common frozen B=5 static-friction matrix using the private SAP runner."""
from __future__ import annotations
import argparse, importlib.util, json
from pathlib import Path
from typing import Any

G = 9.81
BASE_MU, BASE_MASS, BASE_DT = .5, 1., .0005

def cell(name: str, pushed: str="middle", ratio: float=1.5, mu: float=BASE_MU, mass: float=BASE_MASS, dt: float=BASE_DT) -> dict[str, Any]:
    threshold = (2 if pushed == "middle" else 1) * mu * mass * G
    return {"id": name, "pushed_body": pushed, "force_ratio_to_ideal_breakaway": ratio,
            "force_N": ratio * threshold, "mu": mu, "box_mass_kg": mass, "dt_ms": dt * 1e3}

CASES = [
    cell("middle_ratio_0p5", ratio=.5), cell("middle_ratio_1p0", ratio=1.), cell("middle_ratio_1p5", ratio=1.5),
    cell("mass_0p5kg", mass=.5), cell("mass_2kg", mass=2.),
    cell("mu_0p3", mu=.3), cell("mu_0p7", mu=.7),
    cell("dt_0p25ms", dt=.00025), cell("dt_1ms", dt=.001),
    cell("top_ratio_0p5", pushed="top", ratio=.5), cell("top_ratio_1p0", pushed="top", ratio=1.), cell("top_ratio_1p5", pushed="top", ratio=1.5),
]

def module(path: Path):
    s=importlib.util.spec_from_file_location("private_threebox",path)
    if s is None or s.loader is None: raise RuntimeError(f"cannot import {path}")
    m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m

def main() -> None:
    p=argparse.ArgumentParser(); p.add_argument("--source",type=Path,required=True); p.add_argument("--out",type=Path,required=True); p.add_argument("--device",default="cuda:0"); p.add_argument("--repeats",type=int,default=20); a=p.parse_args()
    if a.repeats != 20: raise ValueError("aligned protocol requires R=20")
    m=module(a.source); m.wp.init(); a.out.mkdir(parents=True,exist_ok=True)
    frozen={"shape_ke_N_m":1e5,"shape_tau_s":.003}; rows=[]
    for i,c in enumerate(CASES):
        m.MU,m.MASS,m.DT=c["mu"],c["box_mass_kg"],c["dt_ms"]*1e-3
        old=m.np.random.default_rng
        m.np.random.default_rng=lambda ignored, _seed=42000+i: old(_seed)
        try: r=m.run_one("B5_frozen",frozen,c["pushed_body"],c["force_N"],a.repeats,a.out,a.device)
        finally: m.np.random.default_rng=old
        r["condition"]=c; rows.append(r); print(c["id"],r["agreement_fraction"],r["slip_fraction"],r["mean_displacement_mm"],flush=True)
    out={"protocol":"aligned-frozen-generalizability-v1","task":"three-box-push","engine":"SAP Warp internal","B":5,"R_per_condition":20,"frozen_candidate":frozen,
         "axes":{"force_ratio_to_ideal_breakaway":[.5,1.,1.5],"box_mass_kg":[.5,1.,2.],"mu":[.3,.5,.7],"dt_ms":[.25,.5,1.],"pushed_body":["middle","top"]},
         "seed_formula":"42000 + condition index", "conditions":rows}
    (a.out/"sap_threebox_aligned_frozen_r20.json").write_text(json.dumps(out,indent=2)+"\n")
if __name__=="__main__": main()
