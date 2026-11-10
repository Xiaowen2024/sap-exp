"""GPU integration checks, including a real deliberately undersized contact buffer."""
import unittest

import mujoco
import mujoco_warp as mjw
import numpy as np
import warp as wp

from mujoco_capacity_audit import CapacityAudit


@unittest.skipUnless(wp.is_cuda_available(), "requires the evaluation GPU")
class CapacityAuditTests(unittest.TestCase):
    def test_flags_and_transient_nonfinite_are_latched(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>')
        data = mjw.make_data(model, nworld=3, nconmax=8, njmax=32)
        audit = CapacityAudit(data, 3)
        data.overflow.assign(np.array([0, 1, 512], dtype=np.int32))
        q = data.qpos.numpy(); q[2, 0] = np.nan; data.qpos.assign(q)
        audit.record(data)
        data.overflow.zero_(); q[2, 0] = 0; data.qpos.assign(q); audit.record(data)
        bad, details = audit.result()
        self.assertEqual(bad.tolist(), [False, True, False])
        self.assertEqual(details['overflow_flags_by_world'], [0, 1, 512])
        self.assertEqual(details['finite_by_world'], [True, True, False])

    def test_engine_contact_overflow_is_detected(self):
        model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom type="plane" size="2 2 .1"/><body pos="0 0 .09"><freejoint/><geom type="box" size=".1 .1 .1"/></body></worldbody></mujoco>')
        gm = mjw.put_model(model)
        data = mjw.make_data(model, nworld=2, nconmax=1, njmax=128)
        audit = CapacityAudit(data, 2)
        mjw.step(gm, data); audit.record(data)
        bad, details = audit.result()
        self.assertTrue(bad.all())
        self.assertGreater(details['max_active_contacts'], details['allocated_contacts'])


if __name__ == '__main__':
    unittest.main()
