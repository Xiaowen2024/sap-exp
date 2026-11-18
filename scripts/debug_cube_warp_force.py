import json
from pathlib import Path
import numpy as np
import mujoco, mujoco_warp as mw
from run_dynamic_impact_cube_budget import DT, SRC, make_model

root=Path(__file__).resolve().parents[1]
src=json.loads((root/'exp-results/dynamic-impact-v1/cube-toss/mjwarp-3.17/b1.json').read_text())
raw=np.load(SRC); x=raw['state'][0,0]
m=make_model(src['frozen_config']); gm=mw.put_model(m); d=mw.make_data(m,nworld=1,nconmax=128,njmax=512)
q=d.qpos.numpy(); v=d.qvel.numpy(); q[0,:7]=x[:7]; v[0,:6]=x[7:13]; d.qpos.assign(q);d.qvel.assign(v);mw.forward(gm,d)
for _ in range(round(3/DT)): mw.step(gm,d)
nac=int(d.nacon.numpy()[0]); print('nac',nac,'nacon',d.nacon.numpy())
print('qfrc_constraint',d.qfrc_constraint.numpy()[0,:6])
print('qfrc_smooth',d.qfrc_smooth.numpy()[0,:6])
print('efc force',d.efc.force.numpy()[0,:20])
print('world',d.contact.worldid.numpy()[:nac]); print('addr',d.contact.efc_address.numpy()[:nac]); print('dist',d.contact.dist.numpy()[:nac])
