"""Render an analytic square-cavity MuJoCo/SAP pose comparison.

The left pane is a native MuJoCo simulation.  The right pane uses the same
analytic MuJoCo visual model only to render the recorded SAP body poses; it
does not run MuJoCo contact dynamics for the SAP pane.
"""
from __future__ import annotations
import csv
from pathlib import Path
import imageio.v3 as iio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[1]
SAP_TRACE=ROOT/'exp-results/peg-in-hole/sap-square-cavity-gap-diagnostics/square_offset_1mm_gap_0.1mm_trace.csv'
OUT=ROOT/'exp-results/site-video-demos/08_square_cavity_offset_labelled.mp4'
DT=.0005
def xml():
 grid = "\n".join(
     [
         f'<geom type="box" pos="{value:.3f} 0 .0002" size=".00018 .15 .0001" '
         'rgba=".70 .70 .70 .70" contype="0" conaffinity="0"/>'
         for value in np.arange(-.14, .141, .02)
     ]
     + [
         f'<geom type="box" pos="0 {value:.3f} .0002" size=".15 .00018 .0001" '
         'rgba=".70 .70 .70 .70" contype="0" conaffinity="0"/>'
         for value in np.arange(-.14, .141, .02)
     ]
 )
 return f'''<mujoco model="square_cavity">
 <option timestep=".0005" gravity="0 0 -9.81" integrator="implicitfast" solver="Newton" iterations="100"/>
 <visual><rgba haze=".95 .95 .95 1"/></visual>
 <asset>
   <texture name="sky" type="skybox" builtin="flat" rgb1=".96 .96 .96" rgb2=".96 .96 .96" width="256" height="256"/>
   <texture name="grid" type="2d" builtin="checker" rgb1=".96 .96 .96" rgb2=".78 .78 .78" width="256" height="256"/>
   <material name="grid" texture="grid" texrepeat="12 12" texuniform="true" reflectance=".05" rgba="1 1 1 1"/>
 </asset>
 <default><geom density="1000" friction=".8 .005 .0001" solref=".001 1" solimp=".9 .95 .001 .5 2" rgba=".2 .46 .82 1"/></default>
 <worldbody>
   <light pos="0 0 .25" directional="true" diffuse=".9 .9 .9" ambient=".35 .35 .35"/>
   <geom type="plane" size=".15 .15 .01" material="grid" rgba="1 1 1 1"/>
   {grid}
   <camera name="cam" pos=".16 -.19 .13" xyaxes=".76 .65 -.0 -.31 .36 .88"/>
   <body>
     <geom type="box" pos=" .017 0 .020" size=".005 .027 .020" rgba=".25 .34 .44 .62"/>
     <geom type="box" pos="-.017 0 .020" size=".005 .027 .020" rgba=".25 .34 .44 .62"/>
     <geom type="box" pos="0 .017 .020" size=".027 .005 .020" rgba=".25 .34 .44 .62"/>
     <geom type="box" pos="0 -.017 .020" size=".027 .005 .020" rgba=".25 .34 .44 .62"/>
     <geom type="box" pos="0 0 .005" size=".022 .022 .005" rgba=".25 .34 .44 .62"/>
   </body>
   <body name="peg" pos=".001 0 .120"><freejoint/><geom type="box" size=".010 .010 .025" rgba=".20 .46 .82 1"/></body>
 </worldbody>
 </mujoco>'''
def annotate(frame,text):
 frame=frame.copy(); frame[:42,:,:]=20
 # Simple bitmap-free label stripe: text is supplied by a separate title card
 # in catalog; colored stripe identifies panes inside the video.
 return frame
def main():
 rows=list(csv.DictReader(SAP_TRACE.open()));st=np.array([float(r['time_s']) for r in rows]);q=np.array([[float(r[k]) for k in ('x_m','y_m','z_m','qx','qy','qz','qw')] for r in rows])
 m=mujoco.MjModel.from_xml_string(xml()); left=mujoco.MjData(m);right=mujoco.MjData(m);fps=30;steps=round(1/(fps*DT));frames=[]
 cam='cam'
 font=ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 18)
 with mujoco.Renderer(m,width=640,height=480) as renderer:
  for n in range(60):
   for _ in range(steps):mujoco.mj_step(m,left)
   renderer.update_scene(left,camera=cam);lf=renderer.render()
   tt=(n+1)/fps; pose=np.array([np.interp(tt,st,q[:,i]) for i in range(7)])
   right.qpos[:3]=pose[:3];right.qpos[3:7]=pose[[6,3,4,5]];mujoco.mj_forward(m,right);renderer.update_scene(right,camera=cam);rf=renderer.render()
   banner=np.zeros((40,1280,3),dtype=np.uint8);banner[:,:640]=np.array([190,99,30],dtype=np.uint8);banner[:,640:]=np.array([79,121,165],dtype=np.uint8)
   frame=np.vstack((banner,np.hstack((lf,rf))))
   label=Image.fromarray(frame); draw=ImageDraw.Draw(label)
   draw.text((20,9),"MuJoCo native — solref 1 ms — dt 0.5 ms",font=font,fill="white")
   draw.text((660,9),"SAP/Warp pose replay — ke 1e4 — tau 3 ms — gap 0.1 mm",font=font,fill="white")
   frame=np.asarray(label)
   # Keep the two-second physical drop readable at ordinary playback speed.
   frames.extend([frame] * 4)
 OUT.parent.mkdir(parents=True,exist_ok=True);iio.imwrite(OUT,frames,fps=fps,codec='libx264',pixelformat='yuv420p');print(OUT)
if __name__=='__main__':main()
