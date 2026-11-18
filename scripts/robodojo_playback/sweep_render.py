"""Render recorded Sweep Blocks states only. Never steps dynamics."""
import os
os.environ.setdefault('MUJOCO_GL','osmesa');os.environ.setdefault('PYOPENGL_PLATFORM','osmesa')
import argparse,json,xml.etree.ElementTree as ET,subprocess
from pathlib import Path
import numpy as np,mujoco as mj
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation as R
import imageio_ffmpeg
from .laptop_render import nums,place,writer

COLORS={'broom':'.93 .85 .31 1','broom_shovel':'.96 .87 .31 1','cube_green':'.45 .77 .12 1','cube_cyan':'.02 .85 .75 1','cube_yellow':'.97 .8 .05 1'}

def build(root):
 sr=root/'scene_v1';sc=json.loads((sr/'scene.json').read_text());data=np.load(root/'reference/numeric.npz');T=data['vision__cam_head__extrinsic_matrix'][0];K=data['vision__cam_head__intrinsic_matrix'];xml=ET.Element('mujoco');ET.SubElement(xml,'compiler',angle='radian');vis=ET.SubElement(xml,'visual');ET.SubElement(vis,'global',offwidth='1920',offheight='960');ET.SubElement(vis,'quality',offsamples='4',shadowsize='2048');ET.SubElement(vis,'headlight',ambient='.65 .65 .65',diffuse='.35 .35 .35',specular='0 0 0');asset=ET.SubElement(xml,'asset');wb=ET.SubElement(xml,'worldbody')
 for kind,a in sc['assets'].items():
  if kind=='base':continue
  for j,p in enumerate(a['pieces']):ET.SubElement(asset,'mesh',name=f'{kind}_{j}',file=str(sr/p))
 ET.SubElement(wb,'geom',type='box',pos='0 -.05 .74',size='.7 .55 .025',rgba='.52 .31 .20 1');ET.SubElement(wb,'light',pos='0 -.3 1.8',dir='0 0 -1',diffuse='.7 .7 .7');ET.SubElement(wb,'camera',name='head',pos=nums(T[:3,3]),quat=nums(R.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]),fovy=str(float(np.degrees(2*np.arctan(240/K[1,1])))))
 for b in sc['bodies']:
  if b['asset']=='base':continue
  body=ET.SubElement(wb,'body',name=b['name'],mocap='true',pos=nums(b['p']),quat=nums(b['q']))
  for j,_ in enumerate(sc['assets'][b['asset']]['pieces']):ET.SubElement(body,'geom',type='mesh',mesh=f"{b['asset']}_{j}",rgba=COLORS.get(b['name'],'.08 .08 .1 1'),contype='0',conaffinity='0')
 m=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));d=mj.MjData(m);ids={b['name']:int(m.body_mocapid[mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,b['name'])]) for b in sc['bodies'] if b['asset']!='base'};return sc,m,d,ids

def label(im,title,t,tail):
 im=Image.fromarray(im);dr=ImageDraw.Draw(im);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16);dr.rectangle((0,0,im.width,48),fill=(20,22,27));dr.text((8,3),title,font=font,fill='white');dr.text((8,25),f'{t:.2f} s'+(' | appended hold' if tail else ''),font=font,fill=(210,210,215));return np.array(im)

def main(root,runs,outname):
 root=Path(root).resolve();out=root/outname;out.mkdir(exist_ok=True);sc,m,d,ids=build(root);ren=mj.Renderer(m,480,640);tr={n:np.load(root/'runs'/n/'dense_trace.npz') for n in runs};duration=min(float(t['time'][-1]) for t in tr.values());n=int(round(duration*25))+1;enc={name:writer(out/(name+'.mp4'),640,480) for name in runs};comb=writer(out/'comparison.mp4',640*(len(runs)+1),480)
 frames=root/'reference/frames'
 if not (frames/'0000.jpg').exists():
  frames.mkdir(exist_ok=True);subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(),'-loglevel','error','-i',str(root/'reference/source.mp4'),'-q:v','2','-start_number','0',str(frames/'%04d.jpg')],check=True)
 samples={0,75,150,200,325,425,500,600,701,n-1};sheets={name:[] for name in runs}
 for k in range(n):
  t=min(k/25,duration);tail=t>sc['source_duration_s'];src=min(k,sc['source_end_frame']);reference=np.array(Image.open(frames/f'{src:04d}.jpg').convert('RGB'));tiles=[label(reference,'RoboDojo source'+(' (last frame held)' if tail else ''),t,tail)]
  for name,trace in tr.items():
   place(m,d,ids,trace,t);ren.update_scene(d,camera='head');im=label(ren.render().copy(),name,t,tail);enc[name].stdin.write(im.tobytes());tiles.append(im)
   if k in samples:sheets[name].append(Image.fromarray(im))
  comb.stdin.write(np.hstack(tiles).tobytes())
  if k==0:Image.fromarray(np.hstack(tiles)).save(out/'poster.jpg')
 for proc in [*enc.values(),comb]:proc.stdin.close();assert proc.wait()==0
 for name,ims in sheets.items():
  sheet=Image.new('RGB',(640*2,480*((len(ims)+1)//2)),'white')
  for j,im in enumerate(ims):sheet.paste(im,((j%2)*640,(j//2)*480))
  sheet.save(out/(name+'_sheet.jpg'))
 ren.close();print('recorded-state videos complete',out,flush=True)
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('--root',required=True);a.add_argument('--runs',nargs='+',required=True);a.add_argument('--out',default='videos');v=a.parse_args();main(v.root,v.runs,v.out)
