"""P1 completion measurements (E2) for RH20T task_0050 cfg2 episodes.

First frame: rack (pose + dims, DE fit) and basket (pose + dims) on the table.
Final frame: basket 6-DoF pose with dims fixed, for the predicate check on
reality (README rule 7).  Views whose rack IoU at the optimum is < 0.3 are
treated as occluded and refit without them.
"""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
import numpy as np
from .rh20t_scene import Episode, masks, carve, components
from .rh20t_fit import *


def run(ep, out, label='basket'):
    rep = {}
    t_first = max(v['ts'][0] for v in ep.cams.values()) / 1e3 + 0.2
    t_last = min(v['ts'][-1] for v in ep.cams.values()) / 1e3 - 0.2
    frames = {c: ep.frame(c, t_first) for c in ep.cams}
    obs_r = {c: masks(f)['rack'] for c, (f, _) in frames.items()}
    obs_b = {c: masks(f)[label] for c, (f, _) in frames.items()}
    # coarse locations from the visual hull
    loc = {}
    for key, lab, ztop in (('rack', 'rack', .15), ('basket', label, .07)):
        loc[key] = None
        for miss in (1, 2):
            pts, _ = carve(ep, t_first, [0.35, -0.4, -0.01], [0.9, 0.4, ztop], voxel=0.004, label=lab, allow_miss=miss, frames=frames)
            comps = components(pts, 0.004, 20)
            if comps:
                loc[key] = comps[0].mean(0)
                break
    rx, ry = loc['rack'][:2]
    best = None
    for yr in [(0, np.pi / 2), (np.pi / 2, np.pi)]:
        B = {'x': (rx - .03, rx + .03), 'y': (ry - .03, ry + .03), 'yaw': yr, 'z0': (0., .04),
             'L': (.12, .24), 'w': (.01, .035), 'h': (.008, .035), 'D': (.07, .12), 'dp': (.006, .016),
             'Hp': (.04, .11), 'sp': (.03, .06), 'dc': (.006, .02)}
        p, ious, _ = fit_de(ep, obs_r, rack_parts, B, maxiter=120)
        good = [c for c, v in ious.items() if v >= .3]
        if len(good) < len(ious) and len(good) >= 4:
            p, ious, _ = fit_de(ep, obs_r, rack_parts, B, views=good, maxiter=120)
        m = np.mean(list(ious.values()))
        if best is None or m > best[0]:
            best = (m, p, ious)
    rep['rack_first'] = {'params': best[1], 'iou': best[2], 'views': list(best[2])}
    bx, by = loc['basket'][:2]
    B = {'x': (bx - .04, bx + .04), 'y': (by - .04, by + .04), 'yaw': (0, np.pi), 'z0': (0., .04),
         'a': (.07, .16), 'b': (.05, .12), 'k': (.5, 1.), 'H': (.02, .08)}
    pb, ib, _ = fit_de(ep, obs_b, basket_parts, B, maxiter=120)
    good = [c for c, v in ib.items() if v >= .5]
    if len(good) < len(ib) and len(good) >= 4:
        pb, ib, _ = fit_de(ep, obs_b, basket_parts, B, views=good, maxiter=120)
    rep['basket_first'] = {'params': pb, 'iou': ib}
    # final frame: basket 6-DoF, dims fixed
    framesL = {c: ep.frame(c, t_last) for c in ep.cams}
    obs_bL = {c: masks(f)[label] for c, (f, _) in framesL.items()}
    pts, _ = carve(ep, t_last, [0.45, -0.1, 0.0], [0.85, 0.35, 0.25], voxel=0.004, label=label, allow_miss=1, frames=framesL)
    comps = components(pts, 0.004, 20)
    c0 = comps[0].mean(0) if comps else np.r_[best[1]['x'], best[1]['y'], .08]
    dims = {k: pb[k] for k in ('a', 'b', 'k', 'H')}
    B6 = {'x': (c0[0] - .1, c0[0] + .1), 'y': (c0[1] - .1, c0[1] + .1), 'z': (-.02, .2),
          'rx': (-np.pi, np.pi), 'ry': (-np.pi, np.pi), 'rz': (-np.pi, np.pi)}
    p6, i6, _ = fit_de(ep, obs_bL, basket_parts6, B6, fixed=dims, maxiter=200, roi_margin=60)
    good = [c for c, v in i6.items() if v >= .4]
    if len(good) < len(i6) and len(good) >= 4:
        p6, i6, _ = fit_de(ep, obs_bL, basket_parts6, B6, fixed=dims, views=good, maxiter=200, roi_margin=60)
    rep['basket_final'] = {'params': p6, 'iou': i6, 'hull_centre': c0.tolist()}
    rep['t_first'], rep['t_last'] = t_first, t_last
    rep['frames_used'] = {'first': {c: tf for c, (_, tf) in frames.items()}, 'last': {c: tf for c, (_, tf) in framesL.items()}}
    Path(out).write_text(json.dumps(rep, indent=1, default=float))
    return rep


def main():
    a = argparse.ArgumentParser()
    a.add_argument('--root', required=True); a.add_argument('--calib', required=True)
    a.add_argument('--episodes', nargs='+', required=True); a.add_argument('--out', required=True); a.add_argument('--label', default='basket')
    args = a.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    for e in args.episodes:
        t = time.time()
        ep = Episode(f'{args.root}/lowdim_out/RH20T_cfg2/{e}', f'{args.root}/color_out/RH20T_cfg2/{e}', args.calib)
        r = run(ep, out / f'{e}.json', args.label)
        print(e, 'rack IoU %.3f basket IoU %.3f final IoU %.3f' % tuple(np.mean(list(r[k]['iou'].values())) for k in ('rack_first', 'basket_first', 'basket_final')), '%.0fs' % (time.time() - t), flush=True)


if __name__ == '__main__':
    main()
