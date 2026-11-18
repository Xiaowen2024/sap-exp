"""P0 inventory on the GPU server; never declares geometric validation."""
from __future__ import annotations

from collections import Counter, defaultdict
from pathlib import Path
import re

import numpy as np

from .io_utils import GateError, read_json, restricted_npy, sha256, write_json

OFFICIAL = 'https://rh20t.github.io/'
API = 'https://github.com/rh20t/rh20t_api'


def stream_summary(value):
    """Describe a transformed stream without choosing a frame or changing rate."""
    if not isinstance(value, dict):
        return {'kind': 'array', 'shape': list(np.shape(value))}
    result = {}
    for serial, rows in value.items():
        if isinstance(rows, list) and rows and isinstance(rows[0], dict):
            times = np.array([r['timestamp'] for r in rows], float)
            fields = sorted(set().union(*(r.keys() for r in rows)))
        elif isinstance(rows, dict):
            times = np.array(sorted(float(t) for t in rows), float)
            fields = sorted(set().union(*(r.keys() for r in rows.values()
                                         if isinstance(r, dict))))
        else:
            result[str(serial)] = {'shape': list(np.shape(rows))}
            continue
        diff = np.diff(times)
        result[str(serial)] = {'samples': len(times), 'fields': fields,
                               'first_native_timestamp': float(times[0]) if len(times) else None,
                               'last_native_timestamp': float(times[-1]) if len(times) else None,
                               'median_native_delta': float(np.median(diff)) if len(diff) else None,
                               'monotone': bool(np.all(diff > 0))}
    return result


def inventory(root: Path, out: Path):
    root = root.resolve()
    episodes = []
    configs = defaultdict(Counter)
    for folder in sorted(root.glob('**/task_0050_*')):
        if not folder.is_dir() or folder.name.endswith('_human'):
            continue
        meta_file = folder / 'metadata.json'
        if not meta_file.is_file():
            continue
        meta = read_json(meta_file)
        match = re.search(r'cfg_(\d+)', folder.name)
        cfg = int(match[1]) if match else None
        # No guessed metadata alias: retain unknown rating as an audit error.
        rating = meta.get('rating')
        if type(rating) is int and 0 <= rating <= 9:
            label = 'excluded_robot_failure' if rating == 0 else 'not_retained' if rating == 1 else 'retained'
        else:
            label = 'unknown_rating'
        configs[str(cfg)][label] += 1
        read_files = {str(meta_file.relative_to(root)): sha256(meta_file)}
        streams, errors = {}, []
        for name in ('tcp_base', 'high_freq_data', 'gripper', 'force_torque_base'):
            path = folder / 'transformed' / (name + '.npy')
            if path.is_file():
                read_files[str(path.relative_to(root))] = sha256(path)
                try:
                    streams[name] = stream_summary(restricted_npy(path))
                except (GateError, ValueError, KeyError) as exc:
                    errors.append(f'{name}: {exc}')
        calibration = folder.parent / 'calib' / str(meta.get('calib', 'MISSING'))
        calibration_files = {}
        if calibration.is_dir():
            for p in sorted(calibration.glob('*.npy')):
                read_files[str(p.relative_to(root))] = sha256(p)
                calibration_files[p.name] = read_files[str(p.relative_to(root))]
        cameras = []
        for camera in sorted(folder.glob('cam_*')):
            ts = camera / 'timestamps.npy'
            if ts.is_file():
                read_files[str(ts.relative_to(root))] = sha256(ts)
                try:
                    restricted_npy(ts)
                except (GateError, ValueError) as exc:
                    errors.append(f'{camera.name}/timestamps: {exc}')
            cameras.append({'serial': camera.name[4:],
                            'color_present': (camera/'color.mp4').is_file() or (camera/'color').is_dir(),
                            'depth_present': (camera/'depth.mp4').is_file() or (camera/'depth').is_dir()})
        episodes.append({'id': folder.name, 'relative_path': str(folder.relative_to(root)),
                         'configuration': cfg, 'rating': rating, 'real_label': label,
                         'metadata': meta, 'streams': streams, 'cameras': cameras,
                         'calibration_files': calibration_files, 'files_read_sha256': read_files,
                         'parse_errors': errors,
                         'geometric_frame_check': 'not performed',
                         'calibration_residual': None, 'admitted': False})
    report = {'phase': 'P0', 'status': 'inventory_only; geometric audit pending',
              'sources': [OFFICIAL, API], 'official_episode_counts_verified': False,
              'episodes_found': len(episodes), 'counts': dict(configs), 'episodes': episodes,
              'gate_passed': False,
              'remaining': ['official release provenance and release completeness',
                            'TCP frame/quaternion/tool offset geometric check',
                            'native time units and high-rate stream validation',
                            'gripper unit/zero and command-vs-observation validation',
                            'force frame/bias/tool-weight audit',
                            'depth alignment and measured calibration residual',
                            'video goal type, rack motion, and real failure stages']}
    if not episodes:
        report['status'] = 'not run: no official task_0050 episodes found'
    write_json(out, report)
    return report
