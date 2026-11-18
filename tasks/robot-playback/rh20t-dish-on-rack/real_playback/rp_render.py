"""Render recorded RH20T playback states from a real camera (no physics is advanced here).

Every body is a mocap body placed at the recorded pose: dish (convex pieces), two pads
(at their recorded poses), simplified AG-95 fingers / housing / flange attached to the
recorded gripper base, static rack and table.  Camera = calibrated RH20T static camera
(extrinsics from tcp vs tcp_base, focal length of the 1280x720 stream).

usage: python -m real_playback.rp_render --scene S --trace T [--trace2 ...] --label L --cam ID --out out.mp4
"""
from __future__ import annotations

import argparse
import gzip
import json
import os
import pickle
import subprocess
from pathlib import Path

os.environ.setdefault('MUJOCO_GL', 'cgl')
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation as R

from .rh20t_scene import Episode
from .rp_sim import build_assets, place_on_table, qmat, table_pose

W, H = 1280, 720
COL = {'dish': '0.86 0.62 0.24 1', 'rack': '0.78 0.60 0.40 1', 'pad': '0.10 0.10 0.11 1',
       'finger': '0.22 0.22 0.24 1', 'housing': '0.16 0.16 0.18 1', 'flange': '0.55 0.85 0.35 1'}
FINGER_H = 0.045            # finger link above the pad (m)
HOUSING = (0.035, 0.065, 0.045)   # half sizes x, y, z
FLANGE_R, FLANGE_H = 0.042, 0.012


def nums(v):
    return ' '.join(f'{float(x):.9g}' for x in np.ravel(v))


def camera(ep, cam):
    c = ep.cams[cam]
    Rcb, tcb = c['T_cam_base'][:3, :3], c['T_cam_base'][:3, 3]
    Rbc = Rcb.T                                     # OpenCV camera axes in the base frame
    pos = -Rbc @ tcb
    Rmj = Rbc @ np.diag([1., -1., -1.])             # MuJoCo camera looks along -z, y up
    K = c['K'] * 4.                                  # Episode K is for the 320x180 stream
    fovy = float(np.degrees(2 * np.arctan((H / 2) / K[1, 1])))
    return {'pos': pos, 'quat': R.from_matrix(Rmj).as_quat()[[3, 0, 1, 2]], 'fovy': fovy,
            'pp_offset_px': [float(K[0, 2] - W / 2), float(K[1, 2] - H / 2)]}


def build(scene, meta, cam):
    root = ET.Element('mujoco'); ET.SubElement(root, 'compiler', angle='radian')
    vis = ET.SubElement(root, 'visual'); ET.SubElement(vis, 'global', offwidth=str(W), offheight=str(H))
    ET.SubElement(vis, 'quality', shadowsize='4096', offsamples='8')
    ET.SubElement(vis, 'headlight', ambient='.42 .42 .42', diffuse='.45 .45 .45')
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='sky', type='skybox', builtin='gradient', rgb1='.86 .87 .89', rgb2='.62 .63 .66', width='256', height='256')
    ET.SubElement(asset, 'texture', name='mat', type='2d', builtin='flat', rgb1='.12 .42 .36', width='64', height='64')
    ET.SubElement(asset, 'material', name='mat', texture='mat', rgba='.14 .45 .38 1', specular='.05')
    ET.SubElement(asset, 'mesh', name='pad', file=meta['pieces']['pad'])
    for k in ('basket', 'rack'):
        for i, p in enumerate(meta['pieces'][k]):
            ET.SubElement(asset, 'mesh', name=f'{k}{i}', file=p)
    wb = ET.SubElement(root, 'worldbody')
    tc, tR = table_pose(scene); n = tR[:, 2]
    ET.SubElement(wb, 'light', pos=nums(tc + n * 1.6 + [0.1, 0.3, 0]), dir=nums(-n), diffuse='.55 .55 .55', castshadow='true')
    ET.SubElement(wb, 'light', pos=nums(tc + n * 1.2 + [-0.5, -0.4, 0]), dir=nums(-n + [0.3, 0.3, 0]), diffuse='.22 .22 .22', castshadow='false')
    ET.SubElement(wb, 'geom', type='box', size=nums([1.2, 1.2, scene['table']['box_thickness'] / 2]), pos=nums(tc),
                  quat=nums(R.from_matrix(tR).as_quat()[[3, 0, 1, 2]]), material='mat', contype='0', conaffinity='0')
    rp, rq = place_on_table(scene, scene['rack']['xy'], scene['rack']['yaw'])
    for i in range(len(meta['pieces']['rack'])):
        ET.SubElement(wb, 'geom', type='mesh', mesh=f'rack{i}', pos=nums(rp), quat=nums(rq), rgba=COL['rack'], contype='0', conaffinity='0')
    ET.SubElement(wb, 'camera', name='real', pos=nums(cam['pos']), quat=nums(cam['quat']), fovy=str(cam['fovy']))
    dish = ET.SubElement(wb, 'body', name='dish', mocap='true')
    for i in range(len(meta['pieces']['basket'])):        # pieces are COM-centred (build_assets); body origin = COM
        ET.SubElement(dish, 'geom', type='mesh', mesh=f'basket{i}', rgba=COL['dish'], contype='0', conaffinity='0')
    L = scene['pad']['size'][2]; pw = scene['pad']['size'][0] / 2; pt = scene['pad']['size'][1] / 2
    for side in ('left', 'right'):
        b = ET.SubElement(wb, 'body', name=side, mocap='true')
        ET.SubElement(b, 'geom', type='mesh', mesh='pad', rgba=COL['pad'], contype='0', conaffinity='0')
        ET.SubElement(b, 'geom', type='box', size=nums([pw, pt, FINGER_H / 2]), pos=nums([0, 0, -(L / 2 + FINGER_H / 2)]),
                      rgba=COL['finger'], contype='0', conaffinity='0')
    g = ET.SubElement(wb, 'body', name='gripper', mocap='true')    # origin = TCP, tool z points down
    zc = -(L + FINGER_H + HOUSING[2])
    ET.SubElement(g, 'geom', type='box', size=nums(HOUSING), pos=nums([0, 0, zc]), rgba=COL['housing'], contype='0', conaffinity='0')
    ET.SubElement(g, 'geom', type='cylinder', size=nums([FLANGE_R, FLANGE_H / 2]), pos=nums([0, 0, zc - HOUSING[2] - FLANGE_H / 2]),
                  rgba=COL['flange'], contype='0', conaffinity='0')
    ET.SubElement(g, 'geom', type='cylinder', size=nums([0.045, 0.04]), pos=nums([0, 0, zc - HOUSING[2] - FLANGE_H - 0.04]),
                  rgba='.85 .85 .87 1', contype='0', conaffinity='0')
    m = mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    mid = {k: m.body_mocapid[mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, k)] for k in ('dish', 'left', 'right', 'gripper')}
    return m, mid


def place(m, d, mid, s):
    if np.isfinite(s['basket']).all() and np.abs(s['basket'][:3]).max() < 10:
        d.mocap_pos[mid['dish']] = s['basket'][:3]; d.mocap_quat[mid['dish']] = s['basket'][3:]
    else:
        d.mocap_pos[mid['dish']] = [0, 0, -50]
    for k, p in zip(('left', 'right'), s['pads']):
        d.mocap_pos[mid[k]] = p[:3]; d.mocap_quat[mid[k]] = p[3:]
    d.mocap_pos[mid['gripper']] = s['base'][:3]; d.mocap_quat[mid['gripper']] = s['base'][3:]
    mj.mj_forward(m, d)


def label(img, text, sub=None, size=26):
    im = Image.fromarray(img); dr = ImageDraw.Draw(im)
    f = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', size)
    dr.rectangle((0, 0, im.width, size + 20), fill=(18, 18, 20)); dr.text((14, 9), text, font=f, fill=(245, 245, 245))
    if sub:
        fs = ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial.ttf', int(size * .8)); tw = dr.textlength(sub, font=fs)
        dr.text((im.width - tw - 14, 12), sub, font=fs, fill=(200, 200, 205))
    return np.asarray(im)


def writer(path, fps):
    return subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(fps), '-i', '-', '-an',
                             '-c:v', 'libx264', '-preset', 'slow', '-crf', '20', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', required=True); ap.add_argument('--trace', required=True); ap.add_argument('--label', required=True)
    ap.add_argument('--episode', nargs=3, required=True, metavar=('LOWDIM', 'COLOR', 'CALIB'))
    ap.add_argument('--cam', default='105422061350'); ap.add_argument('--fps', type=int, default=25)
    ap.add_argument('--out', required=True); ap.add_argument('--assets', required=True)
    a = ap.parse_args()
    scene = json.loads(Path(a.scene).read_text()); meta = build_assets(scene, a.assets)
    ep = Episode(*a.episode); cam = camera(ep, a.cam)
    m, mid = build(scene, meta, cam); d = mj.MjData(m); rend = mj.Renderer(m, H, W)
    tr = pickle.load(gzip.open(a.trace)); T = np.array([s['t'] for s in tr])
    tc, trel = scene['events']['t_close'], scene['events']['t_release']
    ts = np.arange(scene['t_start'], scene['t_end'] + 1e-9, 1 / a.fps)
    out = Path(a.out); wr = writer(out, a.fps)
    for j, t in enumerate(ts):
        place(m, d, mid, tr[int(np.argmin(abs(T - t)))]); rend.update_scene(d, camera='real')
        sub = f'+{t - tc:5.2f} s' + ('  ·  gripper open, arm held still' if t >= trel else '')
        img = label(rend.render().copy(), a.label, sub); wr.stdin.write(img.tobytes())
        if abs(t - (trel + 1.5)) < 0.5 / a.fps:
            Image.fromarray(img).save(out.with_name(out.stem + '-poster.jpg'), quality=88)
    wr.stdin.close(); wr.wait(); rend.close()
    (out.with_name(out.stem + '-render.json')).write_text(json.dumps({
        'physics': 'recorded native states; the renderer performs no integration', 'renderer': 'MuJoCo ' + mj.__version__,
        'camera': f'RH20T static camera {a.cam}: calibrated extrinsics, 1280x720 focal length (principal-point offset ignored: {cam["pp_offset_px"]} px)',
        'gripper': 'pads at recorded poses; fingers/housing/flange are a simplified AG-95 shape for display only', 'fps': a.fps,
        'window_s': [float(scene['t_start'] - tc), float(scene['t_end'] - tc)]}, indent=1))
    print('wrote', out, len(ts), 'frames')


if __name__ == '__main__':
    main()
