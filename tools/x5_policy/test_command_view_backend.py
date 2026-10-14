"""Backend lifetime, admission and STOP routing with fake hardware only."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

P=Path(__file__).resolve().parents[2]/'output/duck_sim/command-view-20261012'
sys.path.insert(0,str(P/'candidate/host/service'))
from test_native_workbench import Runtime,SOURCE,CLIENT
import native_workbench as web
from native_gamepad import ControlMonitor,GamepadControl,PRINCIPAL
from native_command_runtime import CommandRuntime


class CommandView(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        Path(self.tmp.name,'index.html').write_bytes(SOURCE.read_bytes())
        self.r=Runtime();self.w=web.NativeWorkbench(self.r,web_root=self.tmp.name,authentication_required=False)
        self.monitor=ControlMonitor();self.local=CommandRuntime(self.w,self.monitor)

    def body(self,action,**fields):
        h=self.r.motion.get('hardware') or {}
        return dict(action=action,control_id=CLIENT,page_generation=self.w.generation,generation=h.get('generation',5),**fields)

    def start(self):
        self.w.motion_action(self.body('start',mode='walk'),principal='browser',live=True)

    def test_no_commands_without_explicit_successful_start(self):
        for _ in range(3):self.local.tick()
        self.assertEqual(self.r.calls,[])
        self.r.failure='ARM register40';
        with self.assertRaises(ValueError):self.start()
        before=len(self.r.calls);self.local.tick();self.assertEqual(len(self.r.calls),before)

    def test_completed_start_continues_with_no_browser_requests(self):
        self.start();before=len(self.r.calls)
        for _ in range(8):self.local.tick()
        self.assertEqual([v['action'] for _,v in self.r.calls[before:]],['heartbeat']*8)
        self.assertEqual(self.monitor.snapshot()['sources']['x5_runtime']['heartbeat_accepted'],8)

    def test_rendered_status_does_not_renew_control(self):
        self.start();before=len(self.r.calls)
        for _ in range(5):self.w.snapshot(live=True)
        self.assertEqual(len(self.r.calls),before)

    def test_input_owners_and_stale_generations_are_not_renewed(self):
        self.start();owner=copy.deepcopy(self.w.control_owner);before=len(self.r.calls)
        self.w.control_owner['principal']=PRINCIPAL;self.local.tick()
        self.w.control_owner=owner;self.r.motion['hardware']['joint_follow']={'active':True};self.local.tick()
        self.r.motion['hardware']['joint_follow']={'active':False};self.w.control_owner['generation']-=1;self.local.tick()
        self.assertEqual(len(self.r.calls),before)

    def test_explicit_stop_wins_no_automatic_restart(self):
        self.start();self.w.motion_action(self.body('stop',stop_source='button'),principal='other-page',live=True)
        before=len(self.r.calls);self.local.tick();self.assertEqual(len(self.r.calls),before)
        self.assertIsNone(self.w.control_owner)

    def test_old_browser_lifecycle_stops_rejected_in_backend(self):
        self.start();before=len(self.r.calls)
        for source in ('page-hidden','pagehide','workspace-leave','window-blur'):
            with self.subTest(source=source),self.assertRaises(ValueError):
                self.w.motion_action(self.body('stop',stop_source=source),principal='browser',live=True)
        self.assertEqual(len(self.r.calls),before)

    def test_closed_stop_recovery_decided_by_backend(self):
        self.r.motion.update(phase='fault',owns_connection=False,hardware=None,
            closed_proof=dict(serial_closed=True,release_confirmed=False))
        def release():
            self.r.calls.append(('release',{}))
            self.r.motion['closed_proof']['release_confirmed']=True
            return self.r.snapshot()
        self.r.release_torque=release
        self.w.motion_action(self.body('stop',stop_source='button'),live=True)
        self.assertEqual([k for k,_ in self.r.calls],['release'])
        self.w.motion_action(self.body('stop',stop_source='button'),live=True)
        self.assertEqual([k for k,_ in self.r.calls],['release'])

    def test_backend_still_rejects_out_of_range_stale_and_foreign_command(self):
        self.start();before=len(self.r.calls)
        for fields in ({'forward':999},{'generation':-1},{'control_id':'2'*32}):
            body=self.body('command',forward=.3,side=0,yaw=0);body.update(fields)
            with self.subTest(fields=fields),self.assertRaises(ValueError):self.w.motion_action(body,principal='browser',live=True)
        self.assertEqual(len(self.r.calls),before)

    def test_failed_gamepad_recovery_cannot_be_renewed_by_command_service(self):
        self.start();owner=copy.deepcopy(self.w.control_owner)
        pad=GamepadControl(self.w);pad.origin=owner
        pad.session=dict(id='a'*32,connection=self.r.motion['connection_sequence'],input_session='b'*32)
        self.w.control_owner=dict(owner,id=pad.session['id'],principal=PRINCIPAL)
        self.r.failure='actual feedback unavailable'
        with self.assertRaises(ValueError):pad._revoke('input lost',recover=True)
        self.assertIsNone(self.w.control_owner)
        before=len(self.r.calls);self.local.tick();self.assertEqual(len(self.r.calls),before)


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CommandView))
    (P/'backend-tests.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,errors=[str(v) for v in result.errors+result.failures]),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
