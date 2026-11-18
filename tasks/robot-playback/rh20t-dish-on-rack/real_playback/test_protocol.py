"""Logical/safety tests only; these do not run or score a physics episode."""
from __future__ import annotations

import copy
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from .assets import convex_mesh, mass_properties, plate_pieces
from .completion import backproject, plane_fit, reprojection_residual, scaled_intrinsics
from .drive import Playback, base_wrench, closed_segments
from .io_utils import GateError, digest, restricted_npy, verified_file, sha256
from .manifest import validate
from .matrix import (CELL_IDS, apply_cell, cell_result, episode_result, permitted_claim,
                     perturbations, select_candidate)
from .metrics import retained_predicate
from .observer import sat_overlap
from .scene import box_mesh


class ProtocolTests(unittest.TestCase):
    def test_restricted_official_numpy_dict(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp)/'stream.npy'
            data = {'base': [{'timestamp': 1001, 'tcp': np.array([0.,1.,2.,1.,0.,0.,0.])}]}
            np.save(file, data)
            loaded = restricted_npy(file)
            np.testing.assert_array_equal(loaded['base'][0]['tcp'],data['base'][0]['tcp'])

    def test_pickle_payload_cannot_execute(self):
        with tempfile.TemporaryDirectory() as tmp:
            marker = Path(tmp)/'EXECUTED'
            class Malicious:
                def __reduce__(self):
                    return os.system, ('touch '+str(marker),)
            file = Path(tmp)/'malicious.npy'
            np.save(file, np.array(Malicious(), dtype=object))
            with self.assertRaises(GateError):
                restricted_npy(file)
            self.assertFalse(marker.exists())

    def test_hash_and_path_guards(self):
        with tempfile.TemporaryDirectory() as tmp:
            file = Path(tmp)/'input.json'
            file.write_text('{}')
            spec = {'path':'input.json','sha256':sha256(file)}
            self.assertEqual(verified_file(tmp,spec),file.resolve())
            file.write_text('{"changed":true}')
            with self.assertRaises(GateError):
                verified_file(tmp,spec)
            with self.assertRaises(GateError):
                verified_file(tmp,{'path':'../input.json','sha256':'x'})

    def test_design_draft_never_runs(self):
        with self.assertRaisesRegex(GateError,'not executable'):
            validate({'status':'draft','R':20})

    def test_paired_perturbations(self):
        a,b = perturbations(91100),perturbations(91100)
        np.testing.assert_array_equal(a,b)
        self.assertEqual(len(np.unique(a,axis=0)),20)
        self.assertTrue(np.all(abs(a[:,:2])<=.0005))
        self.assertNotEqual(digest(a.tolist()),digest(perturbations(91101).tolist()))

    def rows(self, n, label='retained', invalid=0):
        return [{'rollout':i,'valid':i>=invalid,'outcome':'retained' if i<n else 'not_retained',
                 'real_label':label,'flags':[]} for i in range(20)]

    def test_thresholds_invalid_denominator_and_real_failures(self):
        self.assertEqual(cell_result(self.rows(18))['status'],'pass')
        self.assertEqual(cell_result(self.rows(2))['status'],'fail')
        self.assertEqual(cell_result(self.rows(17))['status'],'mixed')
        self.assertEqual(cell_result(self.rows(20,invalid=3))['status'],'mixed')
        self.assertEqual(cell_result(self.rows(2,label='not_retained'))['correct'],18)
        self.assertEqual(cell_result(self.rows(18)[:-1])['status'],'not run')
        duplicate = self.rows(20);duplicate[-1]['rollout']=0
        self.assertEqual(cell_result(duplicate)['status'],'not run')

    def test_all_nineteen_cells_required(self):
        cells = {k:{'status':'pass'} for k in CELL_IDS}
        self.assertEqual(episode_result(cells),'reproduces')
        cells['mass_low']['status']='fail'
        self.assertEqual(episode_result(cells),'sensitive')
        del cells['nominal']
        self.assertEqual(episode_result(cells),'not run')

    def test_failure_label_does_not_become_task_success_claim(self):
        self.assertIn('real task failure',permitted_claim('reproduces','fails','not_retained'))
        self.assertIn('completion suspect',permitted_claim('fails','fails','retained'))
        self.assertIn('no ranking',permitted_claim('sensitive','fails','retained'))

    def test_selection_uses_no_real_diagnostics(self):
        candidates = [{'candidate_index':0,'preflight_passed':True,'wall_s':5.,
                       'episodes':{'sel':self.rows(20)}},
                      {'candidate_index':1,'preflight_passed':True,'wall_s':1.,
                       'episodes':{'sel':self.rows(18)},'stalled_width_error':0}]
        self.assertEqual(select_candidate(candidates,['sel']),0)
        candidates[0]['episodes']['sel'][0]['flags']=['deep_penetration']
        candidates[1]['episodes']['sel']=self.rows(20)
        self.assertEqual(select_candidate(candidates,['sel']),1)
        self.assertIsNone(select_candidate(candidates,['missing']))

    def test_width_state_machine_and_no_invented_release(self):
        t=np.arange(8.)
        self.assertEqual(closed_segments(t,[.08,.08,.077,.012,.012,.012,.016,.08]),[(2.,6.)])
        with self.assertRaises(GateError):
            closed_segments(t,[.08,.08,.077,.012,.012,.012,.012,.012])

    def test_world_wrench_caps_and_gravity_feedforward(self):
        cfg={'K_t_N_m':1e4,'K_r_Nm_rad':300.,'base_mass_kg':1.,
             'base_inertia_kg_m2':[.001,.002,.003],'force_cap_N':50.,'torque_cap_Nm':5.,
             'gravity_m_s2':[0,0,-9.81]}
        pose=np.r_[np.zeros(3),[0,0,0,1]]
        target=np.r_[[1,0,0],Rotation.from_euler('z',90,degrees=True).as_quat()]
        wrench=base_wrench(pose,np.zeros(6),target,np.zeros(6),cfg)
        np.testing.assert_allclose(wrench,[50,0,9.81,0,0,5],atol=1e-10)

    def test_rgbd_geometry_and_intrinsics_scaling(self):
        K=scaled_intrinsics([[400,0,200],[0,400,100],[0,0,1]],(400,200),(200,100))
        np.testing.assert_allclose(K,[[200,0,100],[0,200,50],[0,0,1]])
        depth=np.ones((2,2));T=np.eye(4);T[:3,3]=[1,2,3]
        p=backproject(depth,np.eye(3),T,np.ones((2,2),bool))
        np.testing.assert_allclose(p[0],[1,2,4])
        grid=np.array([[x,y,.2] for x in np.linspace(0,1,10) for y in np.linspace(0,1,10)])
        self.assertLess(plane_fit(grid)['rms_m'],1e-12)
        p=np.array([[0,0,1],[1,0,1],[0,1,1.]])
        self.assertEqual(reprojection_residual(p,p[:,:2],np.eye(3),np.eye(4))['rms_px'],0)

    def test_volume_inertia_and_overlap(self):
        v,f=box_mesh(np.array([1,2,3.]))
        vol,c,I=mass_properties(v,f)
        self.assertAlmostEqual(vol,48.)
        np.testing.assert_allclose(c,[0,0,0],atol=1e-12)
        np.testing.assert_allclose(np.diag(I)/vol,[13/3,10/3,5/3],atol=1e-10)
        self.assertAlmostEqual(sat_overlap(v,f,v+[1.5,0,0],f),.5)
        self.assertEqual(sat_overlap(v,f,v+[2.1,0,0],f),0)
        pieces=plate_pieces([[0,-.001,.001],[.1,-.001,.001]],24)
        volume=sum(mass_properties(a,b)[0] for a,b in pieces)
        self.assertAlmostEqual(volume,24*.5*.1**2*np.sin(2*np.pi/24)*.002)

    def test_goal_is_rack_local_not_real_final_pose(self):
        goal={'rack_position_m':[1,0,0],'rack_quaternion_xyzw':[0,0,0,1],
              'footprint_min_xy_m':[-.2,-.2],'footprint_max_xy_m':[.2,.2],
              'plate_normal_axis_rack':[0,0,1],'normal_cone_half_angle_rad':.1}
        state={'plate_position_m':[1,0,.1],'plate_quaternion_xyzw':[0,0,0,1],
               'rack_contact':True,'pad_contacts':[False,False],
               'tail_max_linear_speed_m_s':.001,'tail_max_angular_speed_rad_s':.01}
        self.assertTrue(retained_predicate(state,goal))
        state['pad_contacts'][0]=True
        self.assertFalse(retained_predicate(state,goal))


if __name__ == '__main__':
    unittest.main()
