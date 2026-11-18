"""Render all simulated states with one common MuJoCo renderer; no integration."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

os.environ.setdefault('MUJOCO_GL','egl')
import mujoco as mj
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from scipy.spatial.transform import Rotation as R
from .common import load
from .run import make_xml,nums


def render(root,revision):
    root=Path(root).resolve();scene,_=load(root/'prepared');data=np.load(root/'reference/numeric.npz',allow_pickle=False)
    T=data['vision__cam_head__extrinsic_matrix'][0];K=data['vision__cam_head__intrinsic_matrix']
    xml=ET.fromstring(make_xml(scene,True));wb=xml.find('worldbody')
    ET.SubElement(wb,'camera',name='head',pos=nums(T[:3,3]),quat=nums(R.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]),
                  fovy=str(np.degrees(2*np.arctan(240/K[1,1]))))
    ET.SubElement(wb,'light',pos='0 -.2 2',dir='0 0 -1',diffuse='.7 .7 .7')
    model=mj.MjModel.from_xml_string(ET.tostring(xml,encoding='unicode'));d=mj.MjData(model)
    for g in range(model.ngeom):
        body=mj.mj_id2name(model,mj.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[g]))
        colors={'bowl0':[.45,.65,.8,1],'bowl1':[.75,.75,.78,1],'bowl2':[.85,.65,.4,1]}
        model.geom_rgba[g]=colors.get(body,[.12,.12,.13,1] if body!='world' else [.4,.28,.18,1])
    slots={b['name']:int(model.jnt_qposadr[i]) for i,b in enumerate(scene['bodies'])}
    renderer=mj.Renderer(model,height=480,width=640)
    traces={};health={}
    for e in ('mujoco','sap','ipc'):
        path=root/'runs'/f'{e}_objects_{revision}'
        if (path/'trace.npz').exists():
            traces[e]=np.load(path/'trace.npz',allow_pickle=False);health[e]=json.loads((path/'health.json').read_text())
    decoder=subprocess.Popen(['ffmpeg','-loglevel','error','-i',str(root/'reference/source_head.mp4'),'-f','rawvideo','-pix_fmt','rgb24','-'],stdout=subprocess.PIPE)
    last=None
    out=root/'comparison.mp4';command=['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pixel_format','rgb24','-video_size','1280x960','-framerate','25','-i','-',
                                    '-an','-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p',str(out)]
    writer=subprocess.Popen(command,stdin=subprocess.PIPE)
    font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),15) if font_path.exists() else ImageFont.load_default()
    def title(image,text):
        image=Image.fromarray(image.copy());draw=ImageDraw.Draw(image);draw.rectangle((0,0,640,30),fill=(20,20,20));draw.text((10,7),text,font=font,fill=(240,240,240));return np.asarray(image)
    for frame in range(int(round((scene['source_duration_s']+scene['tail_s'])*25))+1):
        t=frame/25;raw=decoder.stdout.read(640*480*3);ok=len(raw)==640*480*3
        if ok:last=np.frombuffer(raw,dtype=np.uint8).reshape(480,640,3)
        source=title(last,'Official demo' + (' | last frame held' if not ok else '')+f' | {t:.2f}s')
        tiles=[source]
        for e in ('mujoco','sap','ipc'):
            if e not in traces:
                tiles.append(title(np.full((480,640,3),225,np.uint8),e+' | not run / incomplete'));continue
            tr=traces[e];i=int(np.argmin(abs(tr['time']-t)))
            for name,slot in slots.items():d.qpos[slot:slot+7]=tr[name+'__pose'][i]
            d.qvel[:]=0;mj.mj_forward(model,d);renderer.update_scene(d,camera='head');picture=renderer.render().copy()
            tiles.append(title(picture,e.upper()+f' | same renderer | {t:.2f}s'))
        grid=np.vstack([np.hstack(tiles[:2]),np.hstack(tiles[2:])]);writer.stdin.write(grid.tobytes())
        if frame%50==0:print('render frame',frame,flush=True)
    writer.stdin.close();code=writer.wait();decoder.stdout.close();decoder.wait();renderer.close()
    if code:raise RuntimeError('video encoding failed')
    (root/'render_provenance.json').write_text(json.dumps({'physics':'recorded native states; renderer performs no integration',
       'renderer':'MuJoCo '+mj.__version__,'camera':'official recorded head extrinsics and intrinsic fy',
       'colors':'common bowl-ID colors; source textures omitted','fps':25,
       'limitations':['reduced fingers; arms/palms absent','IPC rendering uses polar rotation; affine strain is separately gated',
                      'source final frame held during appended 1-second simulated tail']},indent=2))
    print(out,flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('root');ap.add_argument('--revision',default='r2');a=ap.parse_args();render(a.root,a.revision)
