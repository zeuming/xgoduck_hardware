import copy
import unittest
from install_native_control import closed_unarmed_connect_failure

class ClosedProbeTests(unittest.TestCase):
    def setUp(self):
        self.motion=dict(phase='fault',last_operation=dict(action='connect',state='failed'),
            closed_proof=dict(serial_closed=True,release_confirmed=False,
                request_counts={'1':2,'6':2,'20':1,'25':1,'19':2,'3':1,'30':1,'32':34},
                before=dict(mode='off',armed=False,torque_ids=[]),
                after=dict(mode='off',armed=False,torque_ids=[])))
    def test_closed_off_connect_with_missing_position_is_update_only_eligible(self):
        self.assertTrue(closed_unarmed_connect_failure(self.motion))
        self.assertFalse(self.motion['closed_proof']['release_confirmed'])
    def test_any_arm_target_or_maintenance_request_refuses(self):
        for kind in ('4','5','31','40'):
            value=copy.deepcopy(self.motion);value['closed_proof']['request_counts'][kind]=1
            with self.subTest(kind=kind):self.assertFalse(closed_unarmed_connect_failure(value))
    def test_unclosed_unknown_or_previously_armed_state_refuses(self):
        for patch in ({'serial_closed':False},{'before':None},{'after':dict(mode='off',armed=True,torque_ids=[])}):
            value=copy.deepcopy(self.motion);value['closed_proof'].update(patch)
            self.assertFalse(closed_unarmed_connect_failure(value))
    def test_runtime_motion_fault_cannot_use_connect_probe_exception(self):
        self.motion['last_operation']['action']='start'
        self.assertFalse(closed_unarmed_connect_failure(self.motion))

if __name__=='__main__':unittest.main()
