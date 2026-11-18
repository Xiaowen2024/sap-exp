"""Shared body-local mesh catalog and physical MJCF, with explicit contact pairs."""
from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree as ET
import itertools
import numpy as np
from scipy.spatial.transform import Rotation

from .assets import convex_mesh, save_obj, build_plate
from .io_utils import sha256, write_json


def numbers(x):
    return ' '.join(format(float(v), '.17g') for v in np.asarray(x).ravel())


def wxyz(q):
    return np.asarray(q)[[3, 0, 1, 2]]


def box_mesh(half):
    return convex_mesh(np.array(list(itertools.product((-1, 1), repeat=3))) * half)


def catalog(scene, output):
    output = Path(output).resolve()
    plate = build_plate(scene, output/'plate')
    padv, padf = box_mesh(np.asarray(scene['pad']['size_m'])/2)
    save_obj(output/'pad.obj', padv, padf)
    members = []
    for i, member in enumerate(scene['rack']['members']):
        if member['type'] == 'box':
            v, f = box_mesh(np.asarray(member['size_m'])/2)
        elif member['type'] == 'cylinder':
            n = member['sectors']
            theta = np.arange(n)*2*np.pi/n
            v, f = convex_mesh([[member['radius_m']*np.cos(a), member['radius_m']*np.sin(a), z]
                                for a in theta for z in (-member['height_m']/2, member['height_m']/2)])
        else:
            raise ValueError('rack members must be declared boxes or cylinders')
        path = output/f'rack_{i:03d}.obj'
        save_obj(path, v, f)
        members.append({'path': str(path), 'sha256': sha256(path),
                        'position_m': member['position_m'], 'quaternion_xyzw': member['quaternion_xyzw']})
    p = plate['profiles'][scene['plate']['profile']]
    result = {'plate': {**p, 'pieces': [{**f, 'path': str(output/'plate'/f['path'])} for f in p['pieces']]},
              'pad': {'path': str(output/'pad.obj'), 'sha256': sha256(output/'pad.obj')},
              'rack': members}
    write_json(output/'catalog.json', result)
    return result


def make_mjcf(scene, assets, base_pose, width, *, objects=True, candidate=None):
    """B=1 has no solver/contact numerical attributes. Friction is physical.

    Friction is set on explicit pairs so pair combiners cannot change it.
    All surface files come from the same catalog in each engine.
    """
    root = ET.Element('mujoco', model='real_playback_v1')
    ET.SubElement(root, 'compiler', angle='radian', inertiafromgeom='false')
    ET.SubElement(root, 'option', timestep=str(scene['dt_s']), gravity=numbers(scene['drive']['gravity_m_s2']))
    ET.SubElement(root, 'size', nconmax=str(scene['contact_capacity']), njmax=str(scene['jacobian_capacity']))
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'mesh', name='pad_mesh', file=assets['pad']['path'])
    wb = ET.SubElement(root, 'worldbody')
    base = ET.SubElement(wb, 'body', name='base', pos=numbers(base_pose[:3]), quat=numbers(wxyz(base_pose[3:])))
    ET.SubElement(base, 'freejoint', name='base_free')
    drive = scene['drive']
    ET.SubElement(base, 'inertial', mass=str(drive['base_mass_kg']), pos='0 0 0', diaginertia=numbers(drive['base_inertia_kg_m2']))
    pad_half = np.asarray(scene['pad']['size_m'])/2
    for side, sign in (('left', -1), ('right', 1)):
        body = ET.SubElement(base, 'body', name=side, pos=numbers([sign*pad_half[0], 0, 0]))
        ET.SubElement(body, 'joint', name=side+'_slide', type='slide', axis=numbers([sign, 0, 0]),
                      limited='true', range=numbers([0, scene['pad']['stroke_m']/2]))
        ET.SubElement(body, 'inertial', pos='0 0 0', mass=str(scene['pad']['mass_kg']),
                      diaginertia=numbers(scene['pad']['mass_kg']/3*(np.sum(pad_half**2)-pad_half**2)))
        ET.SubElement(body, 'geom', name=side+'_pad', type='mesh', mesh='pad_mesh', contype='0', conaffinity='0')
    if objects:
        ET.SubElement(wb, 'geom', name='table', type='plane', size='2 2 .01', pos=numbers([0,0,scene['table_z_m']]), contype='0', conaffinity='0')
        plate = scene['plate']
        body = ET.SubElement(wb, 'body', name='plate', pos=numbers(plate['position_m']), quat=numbers(wxyz(plate['quaternion_xyzw'])))
        ET.SubElement(body, 'freejoint', name='plate_free')
        I = np.asarray(assets['plate']['inertia_per_kg_m2'])*plate['mass_kg']
        ET.SubElement(body, 'inertial', pos='0 0 0', mass=str(plate['mass_kg']),
                      fullinertia=numbers([I[0,0], I[1,1], I[2,2], I[0,1], I[0,2], I[1,2]]))
        plate_names = []
        for i, piece in enumerate(assets['plate']['pieces']):
            name = f'plate_{i:04d}'
            ET.SubElement(asset, 'mesh', name=name+'_mesh', file=piece['path'])
            ET.SubElement(body, 'geom', name=name, type='mesh', mesh=name+'_mesh', contype='0', conaffinity='0')
            plate_names.append(name)
        rack = ET.SubElement(wb, 'body', name='rack', pos=numbers(scene['rack']['position_m']), quat=numbers(wxyz(scene['rack']['quaternion_xyzw'])))
        rack_names = []
        for i, piece in enumerate(assets['rack']):
            name = f'rack_{i:03d}'
            ET.SubElement(asset, 'mesh', name=name+'_mesh', file=piece['path'])
            ET.SubElement(rack, 'geom', name=name, type='mesh', mesh=name+'_mesh',
                          pos=numbers(piece['position_m']), quat=numbers(wxyz(piece['quaternion_xyzw'])), contype='0', conaffinity='0')
            rack_names.append(name)
        contact = ET.SubElement(root, 'contact')
        def pair(a,b,mu):
            attrs = {'geom1': a, 'geom2': b, 'friction': numbers([mu, mu, 0, 0, 0]), 'condim': '3'}
            if candidate and not candidate['native']:
                attrs['solref'] = numbers([candidate['parameters']['tc_s'],1])
            ET.SubElement(contact, 'pair', **attrs)
        for p in plate_names:
            for name in ('left_pad', 'right_pad'):
                pair(p, name, scene['mu_pad_plate'])
            for name in rack_names+['table']:
                pair(p, name, scene['mu_plate_environment'])
        for pad in ('left_pad', 'right_pad'):
            for name in rack_names+['table']:
                pair(pad, name, scene['mu_pad_environment'])
    if candidate and not candidate['native']:
        opt = root.find('option')
        opt.set('impratio', str(candidate['parameters']['impratio']))
        opt.set('noslip_iterations', str(candidate['parameters']['noslip_iterations']))
    return ET.tostring(root, encoding='unicode')
