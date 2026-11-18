#!/usr/bin/env python3
"""Convert released ContactNets toss tensors to physical SI trajectory arrays."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import glob
from pathlib import Path
import numpy as np
import torch

ROOT=REPO_ROOT
SRC=ROOT/'external/contact-nets/data/tosses_processed'
OUT=ROOT/'exp-results/dynamic-impact-v1/cube-toss/contactnets_si.npz'
SCALE=.0524

paths=sorted(glob.glob(str(SRC/'*.pt')),key=lambda p:int(Path(p).stem))
traces=[]; ids=[]
for path in paths:
    a=torch.load(path,map_location='cpu',weights_only=False).detach().cpu().numpy()[0]
    # ContactNets state: position(3), wxyz quaternion(4), linear velocity(3),
    # angular velocity(3), zero control(6); first 13 fields are state.
    traces.append(a[:,:13].astype(np.float64)); ids.append(int(Path(path).stem))
maxlen=max(x.shape[0] for x in traces); packed=np.full((len(traces),maxlen,13),np.nan)
for i,x in enumerate(traces):
    packed[i,:len(x)]=x
packed[:,:,:3]*=SCALE; packed[:,:,7:13]*=SCALE
OUT.parent.mkdir(parents=True,exist_ok=True)
np.savez_compressed(OUT, state=packed,lengths=np.array([x.shape[0] for x in traces]),ids=np.array(ids),
                    scale_m=SCALE,dt_s=1/148,mass_kg=.37,mu=.18,restitution=.125,
                    inertia_kg_m2=.2950002913583124*SCALE**2)
print('wrote',OUT,'trials',len(traces),'length',min(x.shape[0] for x in traces),maxlen)
