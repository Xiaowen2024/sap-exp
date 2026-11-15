#!/usr/bin/env python3
"""Small Genesis/IPC environment and input-routing probes, not benchmark results.

No candidate selection or task success claim is made by these tests.
"""
import argparse
import importlib.metadata as metadata
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np
import torch
import genesis as gs


def array(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['rigid', 'ipc', 'hybrid'], required=True)
    ap.add_argument('--probe', choices=['drop', 'velocity', 'force'], default='drop')
    ap.add_argument('--n-envs', type=int, default=1)
    ap.add_argument('--input-api', choices=['genesis', 'ipc'], default='genesis')
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    versions = {name: metadata.version(name) for name in
                ('genesis-world', 'pyuipc', 'quadrants', 'torch', 'numpy')}
    assert versions['genesis-world'] == '1.4.6'
    assert versions['pyuipc'] == '0.0.29'
    assert torch.cuda.is_available(), 'CUDA Torch must be available; no CPU fallback'
    gs.init(backend=gs.cuda, seed=20261114, logging_level='warning')
    print('Initialized CUDA', args.mode, args.probe, flush=True)
    options = gs.options.SimOptions(dt=.002, substeps=1,
                                    gravity=(0, 0, -9.81) if args.probe == 'drop' else (0, 0, 0))
    kw = {'sim_options': options, 'show_viewer': False}
    if args.mode != 'rigid':
        kw['coupler_options'] = gs.options.IPCCouplerOptions()
    scene = gs.Scene(**kw)
    if args.probe == 'drop':
        scene.add_entity(gs.morphs.Plane(), material=gs.materials.Rigid(friction=.3, coup_friction=.3))
    material_kw = {'rho': 1000., 'friction': .3, 'coup_friction': .3}
    if args.mode == 'ipc':
        material_kw['coup_type'] = 'ipc_only'
    elif args.mode == 'hybrid':
        material_kw['coup_type'] = 'two_way_soft_constraint'
    box = scene.add_entity(gs.morphs.Box(pos=(0, 0, .12), size=(.04, .04, .04)),
                           material=gs.materials.Rigid(**material_kw))
    if args.input_api == 'ipc':
        if args.mode != 'ipc' or args.probe == 'drop':
            raise ValueError('Direct IPC input probe requires IPC-only velocity/force mode')
        import uipc
        coupler = scene.sim.coupler
        finalize = coupler._finalize_ipc

        def finalize_with_declared_input():
            # Pinned Genesis 1.4.6 adapter hook, before the IPC world is initialized.
            # The actual input is set using public libuipc state/constitution APIs.
            if args.probe == 'force':
                from uipc.constitution import AffineBodyExternalBodyForce
                external = AffineBodyExternalBodyForce()
                coupler._ipc_constitution_tabular.insert(external)
            for slot in coupler._abd_slots_by_link[box.base_link]:
                geom = slot.geometry()
                if args.probe == 'velocity':
                    velocity = np.zeros((4, 4))
                    velocity[0, 3] = .2
                    uipc.view(geom.instances().find('velocity'))[:] = velocity
                else:
                    force = np.zeros(12)
                    force[0] = .064
                    external.apply_to(geom, force, enable=True)
            finalize()

        coupler._finalize_ipc = finalize_with_declared_input
    start = time.perf_counter()
    scene.build(n_envs=args.n_envs)
    print('Built scene', args.mode, args.probe, flush=True)
    if args.probe == 'velocity' and args.input_api == 'genesis':
        box.set_dofs_velocity(np.array([.2, 0, 0, 0, 0, 0]))
    positions, velocities = [], []
    steps = 300 if args.probe == 'drop' else 50
    for _ in range(steps+1):
        positions.append(array(box.get_pos()).reshape(-1, 3).copy())
        velocities.append(array(box.get_dofs_velocity()).copy())
        if len(positions) <= steps:
            if args.probe == 'force' and args.input_api == 'genesis':
                # Per-step world-frame wrench; 0.064 kg and 0.064 N give a_x=1 m/s^2.
                force = np.tile(np.array([.064, 0, 0]), (args.n_envs, 1, 1))
                box.apply_links_external_wrench(force=force, links_idx_local=[0])
            scene.step()
    positions = np.asarray(positions)
    velocities = np.asarray(velocities)
    finite = bool(np.isfinite(positions).all() and np.isfinite(velocities).all())
    delta = positions[-1]-positions[0]
    result = {'kind': 'environment_probe_not_task_result', 'versions': versions,
              'python': sys.version, 'libc': platform.libc_ver(),
              'backend': str(gs.backend), 'torch_device': str(gs.device),
              'mode': args.mode, 'probe': args.probe, 'n_envs': args.n_envs,
              'input_api': args.input_api,
              'dt_s': .002, 'steps': steps, 'finite': finite,
              'initial_position': positions[0].tolist(), 'final_position': positions[-1].tolist(),
              'delta_position': delta.tolist(), 'wall_s': time.perf_counter()-start,
              'sim_options': options.model_dump(),
              'rigid_velocity_getter_note': 'IPC-only path may zero Genesis-side velocities; validate before using as telemetry'}
    sim = getattr(scene, 'sim', getattr(scene, '_sim', None))
    if sim is not None:
        coupler = sim.coupler
        result['coupler_class'] = type(coupler).__name__
        result['coupler_active'] = bool(coupler.is_active) if hasattr(coupler, 'is_active') else None
        if args.mode != 'rigid':
            result['has_rigid_ipc_coupling'] = bool(coupler.has_any_rigid_coupling)
            assert result['coupler_class'] == 'IPCCoupler' and result['has_rigid_ipc_coupling']
    if args.probe == 'drop':
        result['passed'] = bool(finite and np.all(delta[:, 2] < -.05)
                                and np.all(positions[-1, :, 2] > .018)
                                and np.all(positions[-1, :, 2] < .05))
    else:
        duration = steps*.002
        expected = .2*duration if args.probe == 'velocity' else .5*duration**2
        result['expected_displacement_x_m'] = expected
        result['passed'] = bool(finite and np.all(np.abs(delta[:, 0]-expected) <= max(.0002,.05*expected)))
        result['interpretation'] = ('input_route_supported' if result['passed'] else
                                     'native_input_route_does_not_reproduce_requested_motion; adapter required')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, default=str)+'\n')
    print(json.dumps({k: result[k] for k in ['mode','probe','n_envs','finite','passed','final_position']},indent=2))
    if args.probe == 'drop' and not result['passed']:
        raise RuntimeError('Contact environment probe failed')


if __name__ == '__main__':
    main()
