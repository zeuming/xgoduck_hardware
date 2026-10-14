"""Native page-owner and runtime dispatch checks with existing CPU fixtures."""
import unittest
import test_native_workbench as wbtests
from test_native_workbench import CLIENT, OTHER
import test_native_runtime as rttests


class WebTests(unittest.TestCase):
    setUp=wbtests.NativeWorkbenchTests.setUp
    tearDown=wbtests.NativeWorkbenchTests.tearDown
    body=wbtests.NativeWorkbenchTests.body

    def test_start_uses_owner_then_only_owner_can_send_targets(self):
        result=self.workbench.motion_action(self.body(joint_follow=True,clearance_confirmed=True),live=True,principal='a')
        self.assertEqual(result['control_owner']['id'],CLIENT)
        target=self.body('joint-target',joint_id=21,angle_deg=-3,sequence=1)
        for changes,principal in ((dict(control_id=OTHER),'a'),({},'b'),(dict(generation=-1),'a'),(dict(sequence=True),'a')):
            with self.subTest(changes=changes,principal=principal),self.assertRaises(ValueError):
                self.workbench.motion_action(dict(target,**changes),principal=principal)
        self.workbench.motion_action(target,principal='a')
        self.assertEqual(self.runtime.calls[-1][1]['action'],'joint-target')

    def test_speed_parameter_reaches_runtime_and_invalid_values_do_not(self):
        self.workbench.motion_action(self.body(joint_follow=True,clearance_confirmed=True),live=True,principal='a')
        target=self.body('joint-target',joint_id=34,angle_deg=3,sequence=1,speed_deg_s=50)
        count=len(self.runtime.calls)
        for speed in (0,51,True,'20',float('nan')):
            with self.subTest(speed=speed),self.assertRaises(ValueError):
                self.workbench.motion_action(dict(target,speed_deg_s=speed),principal='a')
        self.assertEqual(len(self.runtime.calls),count)
        self.workbench.motion_action(target,principal='a')
        self.assertEqual(self.runtime.calls[-1][1]['speed_deg_s'],50)

    def test_start_needs_explicit_clearance_and_off_mode(self):
        for changes in (dict(clearance_confirmed=False),dict(joint_follow=False,clearance_confirmed=True),dict(mode='walk',clearance_confirmed=True)):
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                self.workbench.motion_action(self.body(**dict(dict(joint_follow=True),**changes)))
        self.assertEqual(self.runtime.calls,[])
        self.runtime.motion['hardware']['mode']='hold'
        with self.assertRaises(ValueError):self.workbench.motion_action(self.body(joint_follow=True,clearance_confirmed=True))

    def test_hidden_page_stops_follow_but_retains_normal_hold_rule(self):
        self.workbench.motion_action(self.body(joint_follow=True,clearance_confirmed=True),live=True)
        h=self.runtime.motion['hardware'];h['hold_control_protocol']=2
        with self.assertRaises(ValueError):self.workbench.motion_action(self.body('stop',stop_source='page-hidden'),live=True)
        h['joint_follow']={'active':True}
        self.workbench.motion_action(self.body('stop',stop_source='page-hidden'),live=True)
        self.assertEqual(h['mode'],'off')

    def test_implicit_stop_never_interrupts_normal_x5_owned_modes(self):
        self.workbench.motion_action(self.body(mode='walk'),live=True,principal='a')
        calls=len(self.runtime.calls)
        for source in ('page-hidden','pagehide','workspace-leave','window-blur'):
            with self.subTest(source=source),self.assertRaises(ValueError):
                self.workbench.motion_action(self.body('stop',stop_source=source),live=True,principal='a')
        self.assertEqual(len(self.runtime.calls),calls)
        self.assertEqual(self.runtime.motion['hardware']['mode'],'walk')

    def test_follow_implicit_stop_requires_same_page_principal_and_generation(self):
        self.workbench.motion_action(self.body(joint_follow=True,clearance_confirmed=True),live=True,principal='a')
        self.runtime.motion['hardware']['joint_follow']={'active':True}
        calls=len(self.runtime.calls)
        for changes,principal in (({'control_id':OTHER},'a'),({},'b'),({'generation':-1},'a'),({'stop_source':'unknown'},'a')):
            with self.subTest(changes=changes,principal=principal),self.assertRaises(ValueError):
                body=self.body('stop',stop_source='page-hidden');body.update(changes)
                self.workbench.motion_action(body,live=True,principal=principal)
        self.assertEqual(len(self.runtime.calls),calls)
        result=self.workbench.motion_action(self.body('stop',stop_source='page-hidden'),live=True,principal='a')
        self.assertTrue(result['release_confirmed'])
        self.assertEqual(result['mode'],'off')


class RuntimeTests(unittest.TestCase):
    # Reuse fixture setup, not the unrelated baseline test cases.
    setUp=rttests.NativeRuntimeTests.setUp
    tearDown=rttests.NativeRuntimeTests.tearDown
    create=rttests.NativeRuntimeTests.create
    wait_for=rttests.NativeRuntimeTests.wait_for

    def test_receipt_enable_target_and_context_stop_flags(self):
        runtime=self.create();s=runtime.connect();generation=s['hardware']['generation']
        result=runtime.action_receipt(dict(action='start',mode='hold',joint_follow=True,
            support_confirmed=True,clearance_confirmed=True,generation=generation))
        generation=result['generation']
        ctx=runtime.control_context()['hardware']
        self.assertEqual(ctx['hold_control_protocol'],1)
        self.assertTrue(ctx['joint_follow']['active'])
        result=runtime.action(dict(action='joint-target',generation=generation,joint_id=10,angle_deg=2,sequence=1,speed_deg_s=20))
        self.assertTrue(result['target_accepted'])
        self.assertTrue(runtime._controller.status()['joint_follow']['active'])
        self.assertEqual(runtime._controller.status()['joint_follow']['speed_deg_s'],20)
        runtime.action_receipt(dict(action='stop',generation=generation,stop_source='button'))
        self.assertFalse(runtime.control_context()['hardware']['joint_follow']['active'])


if __name__=='__main__':unittest.main()
