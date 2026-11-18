"""Offline visual replay of SAP three-plate CoACD z telemetry.

The formal SAP trace records z but not orientation or xy, so this clip is
explicitly a z-trajectory visualisation, not a full pose reconstruction.
"""
from __future__ import annotations
import csv
import argparse
from pathlib import Path
import imageio.v3 as iio
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
HERE=ROOT/'mujoco-sim/learn_mujoco/concave_plate_experiments'
BENCHMARK=ROOT/'exp-results/concave-plate/sap-representation-benchmark'

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--representation', choices=('raw_mesh_control', 'coacd_shared'), default='coacd_shared')
    args=parser.parse_args()
    trace=BENCHMARK/f'three_plate_stack_{args.representation}'/'trace.csv'
    suffix='native_mesh' if args.representation == 'raw_mesh_control' else 'coacd'
    out=ROOT/'exp-results/site-video-demos'/f'concave_plate_sap_{suffix}_z_replay_raw.mp4'
    rows=list(csv.DictReader(trace.open()))
    t=np.array([float(r['time_s']) for r in rows])
    z=np.array([[float(r[f'body{i}_z_m']) for i in range(3)] for r in rows])
    m=mujoco.MjModel.from_xml_path(str(HERE/'plate_stack_convex_decomposition.xml'))
    d=mujoco.MjData(m); fps=30; opt=mujoco.MjvOption(); opt.geomgroup[3]=0; frames=[]
    with mujoco.Renderer(m,width=960,height=640) as renderer:
        for j in range(180):
            tt=j/179*t[-1]
            for i in range(3): d.qpos[7*i+2]=np.interp(tt,t,z[:,i])
            mujoco.mj_forward(m,d)
            renderer.update_scene(d,camera='stack_camera',scene_option=opt)
            frames.append(renderer.render())
    out.parent.mkdir(parents=True,exist_ok=True)
    iio.imwrite(out,frames,fps=fps,codec='libx264',pixelformat='yuv420p')
    print(out)

if __name__=='__main__': main()
