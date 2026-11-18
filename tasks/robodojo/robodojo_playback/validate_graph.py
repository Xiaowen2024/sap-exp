"""Cross-check native SAP host and CUDA-graph execution, including contacts."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R


def check(root):
    root=Path(root);checks=[]
    for host,graph in [('sap_drive_r2','sap_drive_graph_r2'),('sap_adapter_host','sap_adapter_graph')]:
        paths=[root/'runs'/name/'trace.npz' for name in (host,graph)]
        a,b=[np.load(p,allow_pickle=False) for p in paths]
        pos,angle,vel=0.,0.,0.
        if not np.allclose(a['time'],b['time'],atol=1e-12,rtol=0):raise ValueError('trace clocks differ')
        for name in a.files:
            if name.endswith('__pose'):
                pos=max(pos,float(np.linalg.norm(a[name][:,:3]-b[name][:,:3],axis=1).max()))
                qa=R.from_quat(a[name][:,[4,5,6,3]]);qb=R.from_quat(b[name][:,[4,5,6,3]])
                angle=max(angle,float(np.rad2deg((qa*qb.inv()).magnitude()).max()))
            elif name.endswith('__velocity'):vel=max(vel,float(np.abs(a[name]-b[name]).max()))
        checks.append({'host':host,'graph':graph,'max_position_m':pos,'max_rotation_deg':angle,'max_twist_component_difference':vel,
                       'trace_sha256':[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths],
                       'pass':bool(pos<=1e-4 and angle<=.05 and vel<=.01)})
    report={'pass':all(c['pass'] for c in checks),'checks':checks,
            'scope':'full no-object trajectory plus first 100ms with real collision bodies; numerical adapter equivalence only'}
    (root/'graph_validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');check(ap.parse_args().root)
