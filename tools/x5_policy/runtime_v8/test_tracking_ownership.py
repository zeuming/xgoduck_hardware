import copy, unittest
import test_native_workbench as fixture

class TrackingOwnershipTests(fixture.NativeWorkbenchTests):
    # The existing suite runs separately; retain just its fixture and body helper.
    def test_tracking_requires_current_owner_principal_generation_and_fixed_payload(self):
        self.workbench.motion_action(self.body('start', mode='hold', support_confirmed=True))
        owner = copy.deepcopy(self.workbench.control_owner)
        before = len(self.runtime.calls)
        for changes in ({'support_confirmed':False}, {'support_confirmed':1},
                        {'control_id':fixture.OTHER}, {'generation':-1}, {'id':23},
                        {'amplitude':10}, {'duration':30}, {'mode':'walk'},
                        {'joint_id':True}, {'joint_id':14.0}, {'joint_id':'14'},
                        {'joint_id':None}, {'joint_id':10}, {'joint_id':24}, {'joint_id':30}):
            body = self.body('tracking-probe', support_confirmed=True)
            body.update(changes)
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.workbench.motion_action(body)
        with self.assertRaises(ValueError):
            self.workbench.motion_action(self.body('tracking-probe', support_confirmed=True), principal='other')
        self.assertEqual(len(self.runtime.calls), before)
        def receipt(body):
            self.runtime.calls.append(('receipt', copy.deepcopy(body)))
            h = self.runtime.motion['hardware']
            return dict(action_receipt_protocol=1, action='tracking-probe',
                connection_sequence=1, generation=h['generation'], mode=h['mode'],
                connected=True, release_confirmed=False)
        self.runtime.action_receipt = receipt
        result = self.workbench.motion_action(self.body('tracking-probe', support_confirmed=True, joint_id=14), live=True)
        self.assertEqual(result['mode'], 'hold')
        self.assertEqual(self.workbench.control_owner, owner)
        self.assertEqual(self.runtime.calls[-1][0], 'receipt')
        self.assertEqual(self.runtime.calls[-1][1]['joint_id'], 14)

# Avoid rerunning inherited regression methods in this focused module.
for _name in tuple(vars(fixture.NativeWorkbenchTests)):
    if _name.startswith('test_'):
        setattr(TrackingOwnershipTests, _name, None)
