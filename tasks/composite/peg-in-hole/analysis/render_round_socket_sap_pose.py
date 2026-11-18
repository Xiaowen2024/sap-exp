"""Offline render of a recorded SAP shared-CoACD round-socket trajectory."""
from __future__ import annotations
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(next(p for p in _Path(__file__).resolve().parents if (p / "common" / "paths.py").is_file())))
from common.paths import REPO_ROOT  # noqa: E402  (also puts shared module folders on sys.path)
import csv
from pathlib import Path
import imageio.v3 as iio
import mujoco
import numpy as np

ROOT=REPO_ROOT
HERE=ROOT/'mujoco-sim/learn_mujoco/peg_in_hole_experiments'
TRACE=ROOT/'exp-results/peg-in-hole/sap-round-socket-matrix/coacd_shared_r10mm_x1mm/trace.csv'
OUT=ROOT/'exp-results/site-video-demos/round_socket_sap_coacd_pose_raw.mp4'
def main():
 rows=list(csv.DictReader(TRACE.open()));time=np.array([float(x['time_s']) for x in rows]);q=np.array([[float(x[k]) for k in ('x_m','y_m','z_m','qx','qy','qz','qw')] for x in rows])
 source=(HERE/'peg_convex_decomposition.xml').read_text().replace('solref="0.004 1"','solref="0.001 1"')
 assets={'blind_socket.obj':(HERE/'assets/blind_socket.obj').read_bytes()};assets.update({p.name:p.read_bytes() for p in (HERE/'assets/collision').glob('socket_piece_*.obj')})
 model=mujoco.MjModel.from_xml_string(source,assets=assets);data=mujoco.MjData(model);fps=30;frames=[];opt=mujoco.MjvOption();opt.geomgroup[3]=0
 with mujoco.Renderer(model,width=960,height=640) as renderer:
  for j in range(120):
   t=(j/119)*time[-1];pose=np.array([np.interp(t,time,q[:,i]) for i in range(7)]);data.qpos[:3]=pose[:3];data.qpos[3:7]=pose[[6,3,4,5]];mujoco.mj_forward(model,data);renderer.update_scene(data,camera='peg_camera',scene_option=opt);frames.append(renderer.render())
 OUT.parent.mkdir(parents=True,exist_ok=True);iio.imwrite(OUT,frames,fps=fps,codec='libx264',pixelformat='yuv420p');print(OUT)
if __name__=='__main__':main()
