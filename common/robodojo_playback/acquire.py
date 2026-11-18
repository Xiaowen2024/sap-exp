"""Acquire pinned official numeric streams and bounded inspection frames.

This is a data audit, never a completed physics reproduction or scored run.
Large HDF5 and USDZ files are accessed using validated HTTP byte ranges.
"""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request
import zipfile

from .http_file import HTTPFile

REVISION = '1127bda94f172d757386945af3eb3a6754d1a4e6'
REPO = 'RoboDojo-Benchmark/RoboDojo'
BASE = f'https://huggingface.co/datasets/{REPO}/resolve/{REVISION}/'
HEAD0_ONLY = False


def listing(path):
    url = f'https://huggingface.co/api/datasets/{REPO}/tree/{REVISION}/{path}?limit=1000'
    with urllib.request.urlopen(url, timeout=45) as r: return json.load(r)


def get(path, out):
    data = urllib.request.urlopen(BASE + path, timeout=60).read()
    out = Path(out); out.parent.mkdir(parents=True, exist_ok=True); out.write_bytes(data)
    return {'path': path, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}


def episode(task, out, episode_id=0):
    import h5py
    import numpy as np
    root = f'data/RoboDojo/{task}/arx_x5/data'
    path = f'{root}/episode_{episode_id:07d}.hdf5'
    entry = next(x for x in listing(root) if x['path'] == path)
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    numeric, schema = {}, {}
    stream = HTTPFile(BASE + path, entry['size'])
    with h5py.File(stream, 'r') as f:
        def visit(name, obj):
            if isinstance(obj, h5py.Dataset):
                schema[name] = {'shape': list(obj.shape), 'dtype': str(obj.dtype)}
                if name.startswith(('state/', 'action/')) or name.endswith(('frequency', 'intrinsic_matrix', 'extrinsic_matrix')):
                    numeric[name.replace('/', '__')] = obj[()]
        f.visititems(visit)
        instruction = f['instruction'][()].decode()
        n = len(f['state/left_ee_poses'])
        frames = sorted(set([0, n - 1] + list(range(0, n, 25)))) if not HEAD0_ONLY else [0]
        for cam in (('cam_head', 'cam_left_wrist', 'cam_right_wrist') if not HEAD0_ONLY else ('cam_head',)):
            for i in frames:
                data = bytes(f[f'vision/{cam}/colors'][i])
                (out / f'{cam}_{i:04d}.jpg').write_bytes(data)
        np.savez_compressed(out / 'numeric.npz', **numeric)
    report = {'kind': 'official_simulation_demo_audit', 'task': task, 'episode_id': episode_id,
              'repository': REPO, 'revision': REVISION, 'source': entry, 'instruction': instruction,
              'schema': schema, 'inspection_frames': frames, 'http_bytes_read': stream.transferred,
              'missing': ['object poses', 'object identities', 'episode initial layout identifier', 'contact forces'],
              'comparison_complete': False}
    report['numeric_sha256'] = hashlib.sha256((out / 'numeric.npz').read_bytes()).hexdigest()
    (out / 'audit.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k != 'schema'}, indent=2), flush=True)


def asset(category, index, out):
    root = f'Assets/Object/RoboDojo/{category}/{index:05d}'
    entries = listing(root)
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    provenance = [get(root + '/metadata.json', out / 'metadata.json')]
    entry = next(x for x in entries if x['path'].endswith('/object.usdz'))
    stream = HTTPFile(BASE + entry['path'], entry['size'])
    with zipfile.ZipFile(stream) as z:
        print('archive', category, index, [(x.filename, x.file_size) for x in z.infolist()], flush=True)
        # USD layers contain geometry; omit texture payloads. Preserve safe paths.
        for info in z.infolist():
            p = Path(info.filename)
            if p.is_absolute() or '..' in p.parts: raise ValueError('unsafe ZIP path')
            if p.suffix.lower() not in ('.usd', '.usda', '.usdc'): continue
            data = z.read(info)
            dst = out / p; dst.parent.mkdir(parents=True, exist_ok=True); dst.write_bytes(data)
            provenance.append({'archive_member': info.filename, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()})
    report = {'revision': REVISION, 'source': entry, 'members': provenance, 'http_bytes_read': stream.transferred}
    (out / 'source.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--out', required=True)
    sub = ap.add_subparsers(dest='mode', required=True)
    e = sub.add_parser('episode'); e.add_argument('--head0-only', action='store_true'); e.add_argument('task', choices=['stack_bowls', 'insert_tubes', 'deposit_coin', 'store_laptop_and_headphones']); e.add_argument('--id', type=int, default=0)
    a = sub.add_parser('asset'); a.add_argument('category'); a.add_argument('index', type=int)
    args = ap.parse_args()
    if args.mode == 'episode':
        HEAD0_ONLY = args.head0_only; episode(args.task, args.out, args.id)
    else: asset(args.category, args.index, args.out)
