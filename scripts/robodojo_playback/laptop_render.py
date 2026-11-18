"""Render recorded states only; OSMesa and ffmpeg, no physics integration."""
import os
os.environ.setdefault('MUJOCO_GL','osmesa');os.environ.setdefault('PYOPENGL_PLATFORM','osmesa')
import argparse,json,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation as R
import mujoco as mj
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg


def nums(v):return ' '.join(str(float(x)) for x in np.ravel(v))

def build(root):
    sr=root/'scene_v1';sc=json.loads((sr/'scene.json').read_text());num=np.load(root/'reference/numeric.npz');T=num['vision__cam_head__extrinsic_matrix'][450];K=num['vision__cam_head__intrinsic_matrix'];r=ET.Element('mujoco');ET.SubElement(r,'compiler',angle='radian');visual=ET.SubElement(r,'visual');ET.SubElement(visual,'global',offwidth='1280',offheight='960');ET.SubElement(visual,'quality',offsamples='4',shadowsize='2048');a=ET.SubElement(r,'asset');w=ET.SubElement(r,'worldbody')
    for kind,asset in sc['assets'].items():
        if kind=='base':continue
        for i,p in enumerate(asset['pieces']):ET.SubElement(a,'mesh',name=f'{kind}_{i}',file=str(sr/p))
    ET.SubElement(visual,'headlight',ambient='.65 .65 .65',diffuse='.35 .35 .35',specular='0 0 0')
    ET.SubElement(w,'geom',type='box',pos='0 -.05 .74',size='.7 .55 .025',rgba='.52 .31 .20 1')
    ET.SubElement(w,'light',pos='0 -.3 1.8',dir='0 0 -1',diffuse='.7 .7 .7');ET.SubElement(w,'light',pos='.7 .5 1.8',dir='-.3 -.3 -1',diffuse='.4 .4 .4')
    ET.SubElement(w,'camera',name='head',pos=nums(T[:3,3]),quat=nums(R.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]),fovy=str(float(np.degrees(2*np.arctan(240/K[1,1])))))
    for sb in sc['static_bodies']:
        for i,_ in enumerate(sc['assets'][sb['asset']]['pieces']):ET.SubElement(w,'geom',type='mesh',mesh=f"{sb['asset']}_{i}",pos=nums(sb['p']),quat=nums(sb['q']),rgba='.07 .07 .08 1' if sb['asset']=='laptop_stand' else ('.72 .72 .74 1' if i==0 else '.57 .32 .18 1'),contype='0',conaffinity='0')
    for b in sc['bodies']:
        if b['asset']=='base':continue
        bn=ET.SubElement(w,'body',name=b['name'],mocap='true',pos=nums(b['p']),quat=nums(b['q']))
        for i,_ in enumerate(sc['assets'][b['asset']]['pieces']):ET.SubElement(bn,'geom',type='mesh',mesh=f"{b['asset']}_{i}",rgba='.92 .93 .95 1' if b['asset']=='laptop' else '.09 .09 .11 1',contype='0',conaffinity='0')
    m=mj.MjModel.from_xml_string(ET.tostring(r,encoding='unicode'));d=mj.MjData(m);ids={b['name']:int(m.body_mocapid[mj.mj_name2id(m,mj.mjtObj.mjOBJ_BODY,b['name'])]) for b in sc['bodies'] if b['asset']!='base'}
    return sc,m,d,ids


def place(m,d,ids,tr,t):
    i=int(np.argmin(abs(tr['time']-t)))
    for n,j in ids.items():d.mocap_pos[j]=tr[n+'__pose'][i,:3];d.mocap_quat[j]=tr[n+'__pose'][i,3:]
    mj.mj_kinematics(m,d);mj.mj_camlight(m,d)


def label(im,title,t,tail=False,source_start=18.):
    p=Image.fromarray(im);d=ImageDraw.Draw(p);font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16);d.rectangle((0,0,p.width,49),fill=(20,22,27));d.text((8,4),title,font=font,fill='white');d.text((8,25),f'{t:.2f} s'+(' | appended hold' if tail else ' | source t='+f'{source_start+t:.2f} s'),font=font,fill=(200,205,210));return np.array(p)


def writer(path,w,h,fps=25):
    return subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(),'-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{w}x{h}','-r',str(fps),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)


def main(root,runs,out_name='videos'):
    root=Path(root).resolve();out=root/out_name;out.mkdir(exist_ok=True);sc,m,d,ids=build(root);renderer=mj.Renderer(m,480,640);traces={n:np.load(root/'runs'/n/'dense_trace.npz') for n in runs};duration=min(float(t['time'][-1]) for t in traces.values());nframes=int(round(duration*25))+1
    enc={n:writer(out/(n+'.mp4'),640,480) for n in runs};combined=writer(out/'comparison.mp4',640*(len(runs)+1),480)
    for f in range(nframes):
        t=min(f/25,duration);sf=min(sc['source_end_frame'],sc['source_start_frame']+int(round(t*25)));reference=np.array(Image.open(root/'reference/segment_frames'/f'{sf:04d}.jpg').convert('RGB'));tiles=[label(reference,'RoboDojo source demo'+(' (last frame held)' if t>sc['source_duration_s'] else ''),t,t>sc['source_duration_s'],sc['source_start_s'])]
        for name,tr in traces.items():
            place(m,d,ids,tr,t);renderer.update_scene(d,camera='head');im=label(renderer.render().copy(),name,t,t>sc['source_duration_s'],sc['source_start_s']);enc[name].stdin.write(im.tobytes());tiles.append(im)
            if f==0 or f==nframes-1:Image.fromarray(im).save(out/(name+('_start.png' if f==0 else '_final.png')))
        combined.stdin.write(np.hstack(tiles).tobytes())
    for proc in [*enc.values(),combined]:proc.stdin.close();assert proc.wait()==0
    camera=mj.MjvCamera();camera.type=mj.mjtCamera.mjCAMERA_FREE;camera.lookat[:]=np.array(sc['static_bodies'][0]['p'])+[0,0,.055];camera.distance=.38;camera.azimuth=-140;camera.elevation=-18
    close=writer(out/'closeup_4x.mp4',640*len(runs),480)
    for t in np.arange(sc.get('closeup_window_s',[1.4,4.])[0],sc.get('closeup_window_s',[1.4,4.])[1]+.001,.01):
        tiles=[]
        for name,tr in traces.items():
            place(m,d,ids,tr,t);renderer.update_scene(d,camera=camera);tiles.append(label(renderer.render().copy(),name+' | 4x slow',t,t>sc['source_duration_s'],sc['source_start_s']))
        close.stdin.write(np.hstack(tiles).tobytes())
    close.stdin.close();assert close.wait()==0;renderer.close();print('rendered recorded states',runs,flush=True)

if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('--root',required=True);a.add_argument('--runs',nargs='+',required=True);a.add_argument('--out-name',default='videos');v=a.parse_args();main(v.root,v.runs,v.out_name)
