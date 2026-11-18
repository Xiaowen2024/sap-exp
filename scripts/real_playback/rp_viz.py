"""Overlay a simulated rollout on the real RH20T video (runner SOP rule 5).

Every convex piece of the simulated basket, the pads and the rack is projected
into one calibrated static camera and drawn as an outline on the real frame
nearest in time.  Output: an mp4 (real frame + simulated outlines).
"""
from __future__ import annotations

import argparse
import gzip
import json
import pickle
from pathlib import Path

import cv2
import numpy as np

from .rh20t_scene import Episode
from .rp_sim import build_assets, load_obj, place_on_table, qmat


def draw(ep, cam, img, pieces, color, scale):
    for v in pieces:
        uv, z = ep.project(cam, v)
        if (z <= 0).any():
            continue
        h = cv2.convexHull((uv * scale * 4).astype(np.int32))
        cv2.polylines(img, [h], True, color, 1, cv2.LINE_AA, shift=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', required=True); ap.add_argument('--trace', required=True)
    ap.add_argument('--lowdim', required=True); ap.add_argument('--color', required=True); ap.add_argument('--calib', required=True)
    ap.add_argument('--cams', default='105422061350,104122061850'); ap.add_argument('--out', required=True)
    ap.add_argument('--label', default='')
    a = ap.parse_args()
    scene = json.loads(Path(a.scene).read_text())
    tr = pickle.load(gzip.open(a.trace))
    ep = Episode(a.lowdim, a.color, a.calib)
    meta = build_assets(scene, Path(a.out).with_suffix('') .as_posix() + '_assets')
    bk = [load_obj(p)[0] for p in meta['pieces']['basket']]
    rk = [load_obj(p)[0] for p in meta['pieces']['rack']]
    pad = load_obj(meta['pieces']['pad'])[0]
    rp, rq = place_on_table(scene, scene['rack']['xy'], scene['rack']['yaw'])
    Rr = qmat(rq)
    rack_w = [v @ Rr.T + rp for v in rk]
    cams = a.cams.split(',')
    scale = 2
    T = np.array([s['t'] for s in tr])
    vw = None
    caps = {c: cv2.VideoCapture(str(Path(a.color) / f'cam_{c}' / 'color' / 'color.mp4')) for c in cams}
    for k in range(0, len(tr), 5):
        s = tr[k]
        tiles = []
        for c in cams:
            ts = ep.cams[c]['ts'] / 1e3
            j = int(np.argmin(abs(ts - s['t'])))
            caps[c].set(cv2.CAP_PROP_POS_FRAMES, j)
            ok, f = caps[c].read()
            if not ok:
                f = np.zeros((180, 320, 3), np.uint8)
            f = cv2.resize(f, None, fx=scale, fy=scale)
            Rb = qmat(s['basket'][3:])
            draw(ep, c, f, [v @ Rb.T + s['basket'][:3] for v in bk], (255, 255, 0), scale)
            draw(ep, c, f, rack_w, (255, 0, 255), scale)
            for p in s['pads']:
                draw(ep, c, f, [pad @ qmat(p[3:]).T + p[:3]], (0, 0, 255), scale)
            cv2.putText(f, f"{a.label} t={s['t'] - scene['events']['t_close']:+.1f}s (close=0)", (5, 18), 0, .5, (0, 255, 255), 1)
            tiles.append(f)
        img = np.hstack(tiles)
        if vw is None:
            vw = cv2.VideoWriter(a.out, cv2.VideoWriter_fourcc(*'mp4v'), 20, (img.shape[1], img.shape[0]))
        vw.write(img)
    vw.release()
    print('wrote', a.out)


if __name__ == '__main__':
    main()
