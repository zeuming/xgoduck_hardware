"""Local controller integration, fake hardware only; no robot connection."""
import copy
import inspect
from pathlib import Path
import shutil
import tempfile
import threading
import unittest
import textwrap
import types
from unittest import mock

from native_gamepad import GamepadControl, ControlMonitor, PRINCIPAL, HOLD_PRINCIPAL
from native_command_runtime import CommandRuntime
from native_runtime import NativeRuntime
import native_workbench as web
from test_native_workbench import Runtime, SOURCE, CLIENT


class GamepadTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        shutil.copyfile(SOURCE, root/'index.html')
        self.runtime = Runtime()
        self.runtime.motion['hardware'].update(mode='hold', hold_ready=True, armed=True)
        self.wb = web.NativeWorkbench(self.runtime, web_root=root, authentication_required=False)
        # Match prepare_official_runtime.py's deployed command envelope while
        # retaining the real generation/ownership/parameter checks in this fixture.
        source = textwrap.dedent(inspect.getsource(web.NativeWorkbench.motion_action))
        source = source.replace("(('forward', .2), ('side', .1), ('yaw', .5))",
                                "(('forward', .4), ('side', .3), ('yaw', 1.))")
        namespace = dict(vars(web))
        exec(compile(source, '<official-workbench-test-fixture>', 'exec'), namespace)
        self.wb.motion_action = types.MethodType(namespace['motion_action'], self.wb)
        self.wb.control_owner = dict(id=CLIENT, principal='browser', generation=5)
        self.now = 100.
        self.input = dict(connected=True, session='a'*32, sequence=0,
                          axes=[0,0,0,0,-32767,-32767,0,0], buttons=[0]*15, events=[])
        self.pad = GamepadControl(self.wb, reader=lambda: copy.deepcopy(self.input), clock=lambda:self.now, standalone=False, shared_buttons=False)

    def request(self, action='gamepad-enable', **extra):
        body=dict(action=action, page_generation=self.wb.generation,
                  generation=self.runtime.motion['hardware']['generation'], control_id=CLIENT,
                  support_confirmed=True)
        body.update(extra)
        return self.pad.request(body, 'browser')

    def event(self, kind, number, value, *, age=0):
        self.input['sequence'] += 1
        self.input['buttons' if kind==1 else 'axes'][number] = value
        self.input['events'].append(dict(sequence=self.input['sequence'], kind=kind,
                                         number=number, value=value, age_ms=age))
        self.input['events']=self.input['events'][-64:]

    def mode(self, name):
        self.runtime.motion['hardware']['mode']=name

    def calls(self, action):
        return [v for kind,v in self.runtime.calls if v.get('action')==action]

    def test_enable_transfers_only_hold_owner_without_any_motor_action(self):
        result=self.request()
        self.assertTrue(result['motion']['gamepad_control']['active'])
        self.assertEqual(self.runtime.calls, [])
        self.assertEqual(self.wb.control_owner['principal'], PRINCIPAL)
        self.assertNotEqual(self.wb.control_owner['id'], CLIENT)

    def test_enable_rejects_walk_stale_generation_missing_support_other_owner(self):
        for changes in ({'generation':4},{'support_confirmed':False},{'control_id':'2'*32}):
            with self.subTest(changes=changes), self.assertRaises(ValueError): self.request(**changes)
        for mode in ('walk',):
            self.mode(mode)
            with self.subTest(mode=mode), self.assertRaises(ValueError): self.request()
        self.assertEqual(self.runtime.calls,[])

    def test_enable_rejects_held_inputs_and_unready_hold(self):
        self.input['buttons'][7]=1
        with self.assertRaises(ValueError): self.request()
        self.input['buttons'][7]=0
        self.runtime.motion['hardware']['hold_ready']=False
        with self.assertRaises(ValueError): self.request()

    def test_old_browser_cannot_drive_or_renew_after_transfer(self):
        self.request()
        for action,extra in [('heartbeat',{}),('start',dict(mode='walk',support_confirmed=True)),
                             ('stop',dict(stop_source='page-hidden'))]:
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.wb.motion_action(dict(action=action, generation=5,page_generation=self.wb.generation,
                                           control_id=CLIENT,**extra),principal='browser',live=True)
        self.assertEqual(self.runtime.calls,[])

    def test_off_takeover_does_not_arm_and_a_alone_requests_default_stance(self):
        self.mode('off');self.runtime.motion['hardware']['armed']=False
        self.wb.control_owner=None
        self.request();self.assertEqual(self.runtime.calls,[])
        self.pad.poll_once();self.assertIsNotNone(self.pad.session)
        self.event(1,0,1);self.pad.poll_once()
        self.assertEqual([x['mode'] for x in self.calls('start')],['hold'])
        self.pad.poll_once();self.assertEqual(len(self.calls('start')),1)

    def test_off_takeover_rejects_armed_state_and_foreign_owner(self):
        self.mode('off')
        with self.assertRaises(ValueError):self.request()
        self.runtime.motion['hardware']['armed']=False
        self.wb.control_owner['principal']='other'
        with self.assertRaises(ValueError):self.request()
        self.assertEqual(self.runtime.calls,[])

    def test_x_alone_starts_zero_once(self):
        self.request();self.event(1,3,1);self.pad.poll_once()
        self.assertEqual([x['mode'] for x in self.calls('start')],['walk'])
        self.pad.poll_once();self.assertEqual(len(self.calls('start')),1)
        self.assertEqual(self.calls('forward-trial'),[])

    def test_a_requests_guarded_hold_from_walk_without_handoff(self):
        self.request();self.mode('walk');self.event(1,0,1);self.pad.poll_once()
        self.assertEqual(self.calls('start')[0]['mode'],'hold')
        self.assertIsNotNone(self.pad.session);self.assertEqual(self.calls('stop'),[])

    def test_y_alone_requests_forward_and_holding_never_repeats(self):
        self.request();self.mode('walk');self.event(1,4,1)
        self.pad.poll_once();self.pad.poll_once()
        self.assertEqual(len(self.calls('command')),1)
        self.assertEqual(self.calls('command')[0]['forward'],.3)

    def test_y_again_or_b_returns_zero_and_forward_can_be_used_again(self):
        self.request();self.mode('walk')
        for value in (1,0,1,0,1):
            self.event(1,4,value);self.pad.poll_once()
        self.assertEqual([c['forward'] for c in self.calls('command')],[.3,0.,.3])
        self.event(1,1,1);self.pad.poll_once()
        self.assertEqual(self.calls('command')[-1]['forward'],0)
        self.assertEqual(self.calls('forward-trial'),[])

    def test_hold_heartbeat_remains_local_without_browser(self):
        self.request();self.pad.poll_once();self.pad.heartbeat_once()
        self.assertEqual(len(self.calls('heartbeat')),1)

    def test_forward_uses_existing_rejection_without_retry(self):
        self.request();self.mode('walk');self.runtime.failure='跟随未到位'
        self.event(1,4,1);self.pad.poll_once();self.pad.poll_once()
        self.assertIn('跟随未到位',self.pad.message);self.assertEqual(len(self.calls('command')),1)
        self.assertEqual(self.calls('stop'),[])

    def test_sticks_shoulders_and_releases_never_trigger_motion(self):
        self.request();self.mode('walk')
        for event in ((1,7,1),(2,1,-32767),(2,7,-32767),(1,7,0),(2,1,0),(2,7,0)):
            self.event(*event);self.pad.poll_once()
        self.assertEqual(self.runtime.calls,[])

    def test_b_returns_zero_without_stop_or_handoff(self):
        self.request();self.mode('walk');self.event(1,1,1);self.pad.poll_once()
        self.assertEqual(self.calls('command')[0]['forward'],0)
        self.assertIsNotNone(self.pad.session);self.assertEqual(self.calls('stop'),[])
        self.assertEqual(self.calls('start'),[])

    def test_start_button_returns_hold_and_web_owner_without_stop(self):
        self.request();self.mode('walk');self.event(1,11,1);self.pad.poll_once()
        self.assertEqual(self.calls('start')[0]['mode'],'hold')
        self.assertIsNone(self.pad.session);self.assertEqual(self.wb.control_owner['id'],CLIENT)
        self.assertEqual(self.wb.control_owner['generation'],6);self.assertEqual(self.calls('stop'),[])

    def test_select_release_precedes_other_buttons(self):
        self.request();self.mode('walk');self.event(1,4,1);self.event(1,10,1)
        self.pad.poll_once();self.assertEqual(len(self.calls('stop')),1)
        self.assertEqual(self.calls('forward-trial'),[]);self.assertIsNone(self.pad.session)

    def test_multiple_action_buttons_do_not_chain_start_and_forward(self):
        self.request();self.event(1,0,1);self.event(1,3,1);self.event(1,4,1)
        self.pad.poll_once();self.assertEqual(self.runtime.calls,[])
        self.assertIn('多个动作键',self.pad.message)

    def test_input_loss_returns_hold_and_reconnect_never_auto_enables(self):
        self.request();self.mode('walk');self.input['connected']=False;self.pad.poll_once()
        self.assertEqual(self.calls('start')[0]['mode'],'hold');self.assertEqual(self.calls('stop'),[])
        self.input['connected']=True;self.pad.poll_once();self.assertIsNone(self.pad.session)

    def test_reader_error_is_retained_after_receiver_recovers(self):
        self.request();self.mode('walk')
        self.input.update(connected=False,error='Input reader stale',age_ms=650.)
        self.pad.poll_once()
        self.assertIn('Input reader stale',self.pad.message)
        self.assertIn('交给手柄',self.pad.message)
        self.input.update(connected=True,error=None,age_ms=1.)
        self.pad.poll_once()
        diagnostic=self.pad.status()['last_input_failure']
        self.assertEqual(diagnostic['reason'],'Input reader stale')
        self.assertEqual(diagnostic['age_ms'],650.)
        self.assertIsNone(self.pad.session)
        self.assertEqual(self.calls('stop'),[])

    def test_event_gap_stale_event_session_change_and_silent_state_change_revoke(self):
        for fault in ('gap','stale','session','state','huge'):
            with self.subTest(fault=fault):
                self.setUp();self.request();self.mode('walk')
                if fault=='gap': self.input['sequence']=2
                if fault=='huge': self.input['sequence']=10**15
                if fault=='stale': self.event(1,7,1,age=301)
                if fault=='session': self.input['session']='b'*32
                if fault=='state': self.input['buttons'][7]=1
                self.pad.poll_once();self.assertIsNone(self.pad.session)
                self.assertEqual(self.calls('forward-trial'),[])

    def test_heartbeat_comes_from_x5_without_http_and_stalled_input_does_not_renew(self):
        self.request();self.mode('walk');self.pad.poll_once();self.pad.heartbeat_once()
        self.assertEqual(len(self.calls('heartbeat')),1)
        self.assertEqual(self.calls('heartbeat')[0]['control_id'],self.pad.session['id'])
        self.now+=.201;self.pad.heartbeat_once();self.assertEqual(len(self.calls('heartbeat')),1)

    def test_stop_from_any_authenticated_page_still_wins_and_never_restarts(self):
        self.request();self.mode('walk')
        self.wb.motion_action(dict(action='stop', stop_source='button',control_id='2'*32,
            page_generation=self.wb.generation,generation=5),principal='other',live=True)
        self.pad.poll_once();self.pad.heartbeat_once();self.assertIsNone(self.pad.session)
        self.assertEqual(self.calls('start'),[])

    def test_recovery_failure_does_not_forge_hold_success_or_keep_heartbeat(self):
        self.request();self.mode('walk');self.runtime.failure='反馈过期'
        self.input['connected']=False
        with self.assertRaises(ValueError): self.pad.poll_once()
        self.assertIsNone(self.pad.session);self.assertIn('恢复站姿未确认',self.pad.message)
        self.pad.heartbeat_once();self.assertEqual(self.calls('heartbeat'),[])
        self.assertEqual(self.wb.control_owner['principal'], PRINCIPAL)
        CommandRuntime(self.wb, ControlMonitor()).tick()
        self.assertEqual(self.calls('heartbeat'),[])

    def test_originless_recovered_hold_has_runtime_heartbeat_owner(self):
        self.request();self.pad.origin=None;self.mode('walk')
        self.input['connected']=False;self.pad.poll_once()
        self.assertIsNone(self.pad.session)
        self.assertEqual(self.wb.control_owner['principal'], HOLD_PRINCIPAL)
        runtime=CommandRuntime(self.wb, ControlMonitor())
        runtime.tick()
        self.assertEqual(len(self.calls('heartbeat')),1)
        self.assertEqual(self.calls('start')[0]['mode'],'hold')
        self.assertEqual(self.calls('stop'),[])

    def test_stop_racing_recovery_handoff_cannot_restore_an_owner(self):
        self.request();self.pad.origin=None;self.mode('walk')
        original=self.pad._action
        def stop_after_hold(*args, **kwargs):
            result=original(*args, **kwargs)
            self.wb.motion_action(dict(action='stop', stop_source='button',control_id='2'*32,
                page_generation=self.wb.generation),principal='other',live=True)
            return result
        self.pad._action=stop_after_hold
        self.input['connected']=False;self.pad.poll_once()
        self.assertIsNone(self.pad.session)
        self.assertIsNone(self.wb.control_owner)
        CommandRuntime(self.wb, ControlMonitor()).tick()
        self.assertEqual(self.calls('heartbeat'),[])
        self.assertEqual(len(self.calls('stop')),1)

    def test_disable_requires_original_page_and_recovers_before_handoff(self):
        self.request();self.mode('walk')
        with self.assertRaises(ValueError): self.request('gamepad-disable',control_id='2'*32)
        value=self.request('gamepad-disable')
        self.assertFalse(value['motion']['gamepad_control']['active'])
        self.assertEqual(value['motion']['hardware']['mode'],'hold')

    def test_changed_connection_never_applies_inputs_or_recovers_into_new_session(self):
        self.request();self.mode('walk');self.runtime.motion['connection_sequence']+=1
        self.event(1,7,1);self.event(2,1,-32767);self.pad.poll_once()
        self.assertIsNone(self.pad.session);self.assertEqual(self.runtime.calls,[])


class StandaloneTests(unittest.TestCase):
    event = GamepadTests.event
    mode = GamepadTests.mode
    def setUp(self):
        GamepadTests.setUp(self)
        self.pad.standalone = True
        self.mode('off')
        self.runtime.motion['hardware']['armed'] = False
        self.wb.control_owner = None

    def calls(self, action):
        return [v for item in self.runtime.calls if len(item) == 2
                for kind,v in [item] if v.get('action') == action]

    def test_boot_baseline_and_first_a_without_browser(self):
        self.pad.poll_once()
        self.assertTrue(self.pad.status()['standalone_ready'])
        self.assertEqual(self.runtime.calls, [])
        self.event(1,0,1);self.pad.poll_once();self.pad.heartbeat_once()
        self.assertEqual([c['mode'] for c in self.calls('start')], ['hold'])
        self.assertEqual(len(self.calls('heartbeat')), 1)
        self.assertFalse(self.pad.status()['browser_setup_required'])

    def test_disconnected_first_x_connects_then_starts_once(self):
        self.runtime.motion.update(phase='closed',owns_connection=False)
        original=self.runtime.connect
        def connect():
            result=original()
            self.runtime.motion.update(phase='ready',owns_connection=True)
            return result
        self.runtime.connect=connect
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once();self.pad.poll_once()
        self.assertEqual(self.runtime.calls[0], ('connect',))
        self.assertEqual([c['mode'] for c in self.calls('start')], ['walk'])

    def test_boot_held_a_does_not_replay(self):
        self.event(1,0,1);self.pad.poll_once();self.pad.poll_once()
        self.assertEqual(self.runtime.calls, [])
        self.event(1,0,0);self.pad.poll_once()
        self.event(1,0,1);self.pad.poll_once()
        self.assertEqual(len(self.calls('start')),1)

    def test_standby_never_takes_browser_owner_but_select_stops(self):
        self.wb.control_owner=dict(id=CLIENT,principal='browser',generation=5)
        self.pad.poll_once();self.event(1,0,1);self.pad.poll_once()
        self.assertEqual(self.runtime.calls,[])
        self.event(1,0,0);self.pad.poll_once()
        self.event(1,10,1);self.event(1,3,1);self.pad.poll_once()
        self.assertEqual(len(self.calls('stop')),1)
        self.assertEqual(self.calls('start'),[])

    def test_stop_release_and_new_a_work_without_webpage(self):
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        self.event(1,3,0);self.event(1,10,1);self.pad.poll_once()
        self.event(1,10,0);self.pad.poll_once()
        self.event(1,0,1);self.pad.poll_once()
        self.assertEqual([c['mode'] for c in self.calls('start')],['walk','hold'])

    def test_reconnect_requires_neutral_then_fresh_press(self):
        self.pad.poll_once();self.input['connected']=False;self.pad.poll_once()
        self.input.update(connected=True,session='b'*32)
        self.event(1,3,1);self.pad.poll_once();self.pad.poll_once()
        self.assertEqual(self.runtime.calls,[])
        self.event(1,3,0);self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        self.assertEqual(len(self.calls('start')),1)

    def test_stop_during_connect_prevents_delayed_start(self):
        self.runtime.motion.update(phase='closed',owns_connection=False)
        original=self.runtime.connect
        def connect():
            result=original();self.runtime.motion.update(phase='ready',owns_connection=True)
            self.event(1,10,1)
            return result
        self.runtime.connect=connect
        self.pad.poll_once();self.event(1,0,1);self.pad.poll_once()
        self.assertEqual(self.calls('start'),[])
        self.assertIn('输入已变化',self.pad.message)

    def test_brief_press_release_during_slow_connect_still_starts_once(self):
        self.runtime.motion.update(phase='closed',owns_connection=False)
        original=self.runtime.connect
        def connect():
            result=original();self.runtime.motion.update(phase='ready',owns_connection=True)
            self.event(1,0,0)
            for event in self.input['events']:event['age_ms']=1000
            return result
        self.runtime.connect=connect
        self.pad.poll_once();self.event(1,0,1);self.pad.poll_once();self.pad.poll_once()
        self.assertEqual([c['mode'] for c in self.calls('start')],['hold'])

    def test_unopened_trust_without_maintenance_uses_connection_review_and_closes(self):
        import threading
        session=types.SimpleNamespace(lock=threading.RLock(),generation=20,
            maintenance_fault_hold=False,recovery_recheck={'category':'trust'},
            _maintenance_can_reconcile_unopened_trust=lambda:False,
            connection_check_blocked_reason=lambda:None,
            feedback_recheck={'phase':'passed'},connection_check={'phase':'passed'})
        def end():session.recovery_recheck=None
        service=types.SimpleNamespace(session=session,end_for_motion=mock.Mock(side_effect=end))
        self.wb.calibration_service=service;self.wb.calibration_action=mock.Mock()
        self.pad._review_unopened_startup_fault(self.input)
        self.wb.calibration_action.assert_called_once_with('/api/check-connection',
            dict(purpose='joints',generation=20),principal=PRINCIPAL)
        service.end_for_motion.assert_called_once()

    def test_start_stays_locally_owned_in_standalone(self):
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        self.event(1,11,1);self.pad.poll_once();self.pad.heartbeat_once()
        self.assertEqual(self.calls('start')[-1]['mode'],'hold')
        self.assertIsNotNone(self.pad.session)
        self.assertEqual(self.wb.control_owner['principal'],PRINCIPAL)

    def test_only_unopened_trust_review_runs_and_failed_review_never_starts(self):
        import threading
        session=types.SimpleNamespace(lock=threading.RLock(),generation=20,
            maintenance_fault_hold=True,recovery_recheck={'category':'trust'},
            _maintenance_can_reconcile_unopened_trust=lambda:True,
            maintenance=types.SimpleNamespace(snapshot=lambda:dict(owns_connection=False,busy=False,error='反馈缺失')))
        self.wb.calibration_service=types.SimpleNamespace(session=session)
        self.wb.calibration_action=mock.Mock()
        with self.assertRaisesRegex(ValueError,'反馈缺失'):
            self.pad._review_unopened_startup_fault(self.input)
        self.wb.calibration_action.assert_called_once_with('/api/servo-maintenance',
            dict(action='review',generation=20),principal=PRINCIPAL)
        self.assertEqual(self.calls('start'),[])
        session._maintenance_can_reconcile_unopened_trust=lambda:False
        self.wb.calibration_action.reset_mock()
        self.pad._review_unopened_startup_fault(self.input)
        self.wb.calibration_action.assert_not_called()

    def test_review_pass_preserves_input_and_changed_input_cancels_start(self):
        import threading
        session=types.SimpleNamespace(lock=threading.RLock(),generation=20,
            maintenance_fault_hold=False,recovery_recheck=None,
            _maintenance_can_reconcile_unopened_trust=lambda:True,
            maintenance=types.SimpleNamespace(snapshot=lambda:dict(owns_connection=False,busy=False)))
        self.wb.calibration_service=types.SimpleNamespace(session=session)
        self.wb.calibration_action=mock.Mock()
        self.pad._review_unopened_startup_fault(copy.deepcopy(self.input))
        saved=copy.deepcopy(self.input)
        self.event(1,10,1)
        with self.assertRaisesRegex(ValueError,'输入已变化'):
            self.pad._review_unopened_startup_fault(saved)
        self.assertEqual(self.calls('start'),[])


class SharedButtonTests(unittest.TestCase):
    event = GamepadTests.event
    mode = GamepadTests.mode
    calls = StandaloneTests.calls

    def setUp(self):
        GamepadTests.setUp(self)
        self.pad.shared_buttons = True
        self.pad.standalone = True
        self.wb.shared_model_buttons = True
        # Exercise the real runtime projection on a fake controller, rather
        # than returning snapshot fields that its public contract omits.
        def context():
            h = self.runtime.motion['hardware'] or {}
            controller=types.SimpleNamespace(_lock=threading.RLock(),
                _connected=h.get('connected',False),_generation=h.get('generation'),
                _mode=h.get('mode'),_pending_stop=h.get('pending_stop',False),
                _joint_follow=h.get('joint_follow'),_official_firmware=True,
                _age_locked=lambda:.01,
                _state=dict(online_mask=32767,fault_mask=0,armed=h.get('armed',False),
                            torque_mask=0 if h.get('release_confirmed') else 32767,cookie=0))
            host=types.SimpleNamespace(lock=threading.RLock(),_controller=controller,
                _connection_sequence=self.runtime.motion['connection_sequence'],
                phase=self.runtime.motion['phase'],motion=types.SimpleNamespace(ALL_MASK=32767,FRESH_SECONDS=.15),
                _owns_locked=lambda:self.runtime.motion['owns_connection'],
                _remember_timing=lambda *args:None)
            return NativeRuntime.control_context(host)
        self.runtime.control_context = context

    def web_action(self, action, **extra):
        return self.wb.motion_action(dict(action=action,page_generation=self.wb.generation,
            generation=self.runtime.motion['hardware']['generation'],control_id=CLIENT,**extra),
            principal='browser',live=True)

    def test_web_started_walk_then_pad_y_b_then_web_command(self):
        self.mode('walk');self.pad.poll_once()
        self.event(1,4,1);self.pad.poll_once()
        self.assertEqual(self.calls('command')[-1]['forward'],.3)
        self.assertEqual(self.wb.control_owner['id'],CLIENT)
        self.event(1,1,1);self.pad.poll_once()
        self.assertEqual(self.calls('command')[-1]['forward'],0)
        self.web_action('command',forward=.2,side=0,yaw=0)
        self.assertEqual(self.calls('command')[-1]['forward'],.2)

    def test_pad_start_then_other_browser_start_and_pad_command(self):
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        self.assertEqual(self.wb.control_owner['principal'],PRINCIPAL)
        self.web_action('start',mode='hold',support_confirmed=True)
        self.assertEqual(self.wb.control_owner['principal'],'browser')
        self.event(1,3,0);self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        self.assertEqual(self.calls('start')[-1]['mode'],'walk')
        self.web_action('command',forward=0,side=0,yaw=0)
        self.assertEqual(self.calls('command')[-1]['forward'],0)

    def test_foreign_browser_generation_still_rejected_and_joint_follow_not_shared(self):
        self.mode('walk');self.runtime.motion['hardware']['joint_follow']={'active':True}
        with self.assertRaises(ValueError):
            self.wb.motion_action(dict(action='command',page_generation=self.wb.generation,
                generation=5,control_id='2'*32,forward=0,side=0,yaw=0),principal='other',live=True)
        self.runtime.motion['hardware']['joint_follow']={'active':False}
        with self.assertRaises(ValueError):
            self.wb.motion_action(dict(action='start',page_generation=self.wb.generation,
                generation=4,control_id='2'*32,mode='hold',support_confirmed=True),principal='other',live=True)
        self.assertEqual(self.runtime.calls,[])

    def test_pad_heartbeat_tracks_current_owner_without_overwriting_it(self):
        self.mode('walk');self.pad.poll_once();self.event(1,1,1);self.pad.poll_once()
        owner=copy.deepcopy(self.wb.control_owner)
        self.pad.heartbeat_once()
        self.assertEqual(self.calls('heartbeat')[-1]['control_id'],CLIENT)
        self.assertEqual(self.wb.control_owner,owner)
        self.assertTrue(self.pad.status()['shared_model_buttons'])

    def test_a_after_off_fault_does_not_get_blocked_by_old_browser_owner(self):
        self.mode('off');self.runtime.motion['hardware'].update(armed=False,generation=6,error='old fault')
        self.pad.poll_once();self.event(1,0,1);self.pad.poll_once()
        self.assertEqual(self.calls('start')[-1]['mode'],'hold')
        self.assertNotIn('其他会话',self.pad.message)

    def test_actual_context_without_armed_field_allows_released_off_a(self):
        self.mode('off');self.runtime.motion['hardware']['armed']=False
        self.assertNotIn('armed',self.runtime.control_context()['hardware'])
        self.pad.poll_once();self.event(1,0,1);self.pad.poll_once()
        self.assertEqual([c['mode'] for c in self.calls('start')],['hold'])

    def test_off_without_confirmed_release_remains_blocked(self):
        self.mode('off');self.runtime.motion['hardware']['release_confirmed']=False
        self.pad.poll_once();self.event(1,0,1);self.pad.poll_once()
        self.assertEqual(self.calls('start'),[])
        self.assertIn('扭矩状态未就绪',self.pad.message)

    def test_l1_connects_once_without_start_or_arm_and_a_then_starts(self):
        self.mode('off');self.runtime.motion['hardware']['armed']=False
        self.runtime.motion.update(phase='closed',owns_connection=False)
        original=self.runtime.connect
        def connect():
            result=original();self.runtime.motion.update(phase='ready',owns_connection=True)
            return result
        self.runtime.connect=connect
        self.pad.poll_once();self.event(1,6,1);self.pad.poll_once();self.pad.poll_once()
        self.assertEqual(self.runtime.calls,[('connect',)])
        self.assertIsNone(self.pad.session)
        self.event(1,6,0);self.pad.poll_once();self.event(1,0,1);self.pad.poll_once()
        self.assertEqual([c['mode'] for c in self.calls('start')],['hold'])

    def test_r1_disconnects_web_walk_and_does_not_restart_held_key(self):
        self.mode('walk');self.pad.poll_once();self.event(1,7,1)
        self.pad.poll_once();self.pad.poll_once()
        self.assertEqual(self.runtime.calls,[('disconnect',)])
        self.assertIsNone(self.pad.session)
        self.assertIsNone(self.wb.control_owner)

    def test_r1_disconnects_active_pad_session_and_stops_heartbeat(self):
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        self.event(1,3,0);self.pad.poll_once();self.event(1,7,1);self.pad.poll_once()
        self.pad.heartbeat_once()
        self.assertEqual(self.runtime.calls[-1],('disconnect',))
        self.assertIsNone(self.pad.session)
        self.assertEqual(self.calls('heartbeat'),[])

    def test_connection_keys_do_not_chain_with_start_and_select_wins(self):
        self.mode('off');self.pad.poll_once()
        self.event(1,6,1);self.event(1,0,1);self.pad.poll_once()
        self.assertEqual(self.runtime.calls,[])
        self.event(1,10,1);self.event(1,7,1);self.pad.poll_once()
        self.assertEqual(len(self.calls('stop')),1)
        self.assertNotIn(('disconnect',),self.runtime.calls)

    def test_y_reads_current_web_velocity_not_omitted_context_command(self):
        self.mode('walk');self.runtime.motion['hardware']['command']=dict(forward=.2,side=0,yaw=0)
        self.pad.poll_once();self.event(1,4,1);self.pad.poll_once()
        self.assertEqual(self.calls('command')[-1]['forward'],0)

    def test_old_heartbeat_does_not_undo_concurrent_new_mode_or_revoke_session(self):
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        session=self.pad.session;original=self.runtime.action
        def raced(body):
            if body['action']=='heartbeat':
                self.web_action('start',mode='hold',support_confirmed=True)
                raise ValueError('Motion state changed; refresh before starting')
            return original(body)
        self.runtime.action=raced
        self.pad.heartbeat_once()
        self.assertIs(self.pad.session,session)
        self.assertEqual([c['mode'] for c in self.calls('start')],['walk','hold'])
        self.assertEqual(self.pad.heartbeat_count,0)
        self.runtime.action=original;self.pad.heartbeat_once()
        self.assertEqual(self.pad.heartbeat_count,1)

    def test_heartbeat_failure_without_new_generation_is_not_hidden(self):
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        self.runtime.failure='UART disconnected'
        with self.assertRaisesRegex(ValueError,'UART disconnected'):
            self.pad.heartbeat_once()
        self.assertEqual(self.pad.heartbeat_count,0)

    def test_retired_heartbeat_cannot_revoke_a_new_input_session(self):
        self.pad.poll_once();self.event(1,3,1);self.pad.poll_once()
        replacement=dict(self.pad.session,id='c'*32)
        def retired(body):
            self.pad.session=replacement
            raise ValueError('Motion state changed; refresh before starting')
        self.runtime.action=retired;self.pad.heartbeat_once()
        self.assertIs(self.pad.session,replacement)
        self.assertEqual(self.pad.heartbeat_count,0)


class MonitorTests(unittest.TestCase):
    def test_rejected_browser_heartbeat_is_not_reported_as_renewal(self):
        now=[10.]
        monitor=ControlMonitor(clock=lambda:now[0])
        secret='do-not-record-credentials'
        monitor.call('x5_local',dict(action='heartbeat',generation=2,control_id=secret),lambda:None)
        now[0]+=.1
        monitor.call('x5_local',dict(action='heartbeat',generation=2),lambda:None)
        def rejected():raise ValueError('Old page owner')
        with self.assertRaisesRegex(ValueError,'Old page owner'):
            monitor.call('browser',dict(action='heartbeat',generation=1),rejected)
        value=monitor.snapshot()
        self.assertEqual(value['sources']['browser']['heartbeat_accepted'],0)
        self.assertEqual(value['sources']['browser']['rejected'],1)
        self.assertEqual(value['sources']['x5_local']['heartbeat_accepted'],2)
        self.assertAlmostEqual(value['sources']['x5_local']['maximum_heartbeat_gap_ms'],100.)
        self.assertNotIn(secret,str(value))

    def test_callback_result_and_exception_preserved_with_bounded_events(self):
        monitor=ControlMonitor()
        result=object()
        self.assertIs(monitor.call('browser',dict(action='start'),lambda:result),result)
        for index in range(100):
            monitor.call('browser',dict(action='other-'+str(index)),lambda:None)
        self.assertEqual(len(monitor.snapshot()['recent_requests']),64)
        self.assertEqual(len(monitor.snapshot(compact=True)['recent_requests']),8)
        self.assertEqual(set(monitor.snapshot()['sources']['browser']['actions']),{'start','other'})


if __name__=='__main__': unittest.main()
