"""Physical/unit checks for the existing-trace evaluator, without simulators."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from .evaluate_stack_existing import window_stats


class ExistingEvaluationTests(unittest.TestCase):
    def trace(self,v):
        t=np.linspace(0,1,101);p=np.column_stack((t[:,None]*np.asarray(v),np.tile([1.,0.,0.,0.],(len(t),1))))
        return {'time':t,'bowl__pose':p,'bowl__velocity':np.tile([*v,0.,0.,.2],(len(t),1))}

    def test_constant_physical_motion_and_units(self):
        s=window_stats(self.trace([.003,.004,0]),'bowl',0,1)
        self.assertAlmostEqual(s['linear_rms_mm_s'],5.)
        self.assertAlmostEqual(s['angular_rms_rad_s'],.2)
        self.assertAlmostEqual(s['com_net_drift_mm'],5.)
        self.assertAlmostEqual(s['com_path_length_mm'],5.)

    def test_speed_rms_is_coordinate_invariant(self):
        v=np.array([.003,.004,.002]);rot=Rotation.from_rotvec([.6,-.4,.2])
        a=window_stats(self.trace(v),'bowl',0,1);b=window_stats(self.trace(rot.apply(v)),'bowl',0,1)
        self.assertAlmostEqual(a['linear_rms_mm_s'],b['linear_rms_mm_s'])

    def test_missing_window_is_not_silently_scored(self):
        with self.assertRaises(ValueError):window_stats(self.trace([0,0,0]),'bowl',.005,1.)


if __name__=='__main__':unittest.main()
