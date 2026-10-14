"""Dual-model protocol and real CPU inference checks; no UART or motion."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock

p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
root=a.candidate.resolve()
sys.path[:0]=[str(root/'service'),str(root/'policy'),str(Path(__file__).resolve().parent)]
import infer
from model_sets import ModelSetMixin
from policy_worker import PolicyWorker,allowed_worker_command
import native_workbench as web
from test_native_workbench import Runtime,CLIENT,SOURCE
import numpy as np
import onnxruntime as ort


class Harness(ModelSetMixin):
    def __init__(self,directory=root):
        self.directory=directory
        self.deployment=json.loads((root/'service/deployment.json').read_text())
        self.lock=threading.RLock();self.phase='idle';self._closed=False
        self._controller=None;self._namespace=None;self.closed_proof=None
        self._policy_worker=None;self.owns=False;self.sessions={'unchanged':object()};self.events=[]
    def _owns_locked(self):return self.owns
    def _serial_closed(self,*args):return False
    def _record(self,name,value):self.events.append(value)


class Models(unittest.TestCase):
    def test_default_and_legacy_request_keep_original_and_do_not_load(self):
        h=Harness();old=h.sessions
        h._select_model_set(None);h._select_model_set('original')
        self.assertIs(old,h.sessions);self.assertEqual(h.model_set_status()['model_set'],'original')
        self.assertEqual(h.events,[])

    def test_real_trained_and_original_inference_and_hash_mapping(self):
        h=Harness();h._select_model_set('trained')
        self.assertIsNone(h._policy_worker,'keep installed synchronous inference')
        observation=np.zeros((1,61),np.float32);observation[0,5]=-1
        for group in ['trained','original']:
            h._select_model_set(group)
            directory=root/('policy/models' if group=='original' else 'policy/model-sets/trained')
            for name in infer.NAMES:
                path=directory/f'xgoduck_{name}.onnx'
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),h.deployment['model_hashes'][name])
                options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
                reference=ort.InferenceSession(str(path),options,providers=['CPUExecutionProvider'])
                expected=reference.run(None,{reference.get_inputs()[0].name:observation})[0]
                np.testing.assert_array_equal(infer.infer(h.sessions[name],observation),expected)
        self.assertEqual([x['model_set'] for x in h.events],['trained','original'])
        self.assertTrue(all(x['serial_open_attempts']==x['motor_start_attempted']==0 for x in h.events))

    def test_unknown_active_and_unclosed_connections_do_not_touch_models(self):
        h=Harness();old=h.sessions
        for value in ['../../elsewhere','bad',[],True]:
            with self.assertRaises(ValueError):h._select_model_set(value)
        with mock.patch.object(infer,'session',side_effect=AssertionError('should not load')):
            h.owns=True
            with self.assertRaisesRegex(ValueError,'停止并断开'):h._select_model_set('trained')
            h.owns=False;h._controller=object()
            with self.assertRaisesRegex(ValueError,'旧串口'):h._select_model_set('trained')
        self.assertIs(old,h.sessions);self.assertEqual(h.events,[])

    def test_corrupt_member_rejects_entire_set_and_retains_old_sessions(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp).resolve()
            shutil.copytree(root/'policy/model-sets',directory/'policy/model-sets')
            (directory/'policy/model-sets/trained/xgoduck_pick.onnx').write_bytes(b'corrupt')
            h=Harness(directory);old=h.sessions
            with self.assertRaisesRegex(ValueError,'校验失败'):h._select_model_set('trained')
            self.assertIs(old,h.sessions);self.assertEqual(h.model_set_status()['model_set'],'original')
            self.assertEqual(h.events,[])

    def test_existing_worker_uses_selected_weights_and_path_allowlist(self):
        h=Harness();old=mock.Mock();h._policy_worker=old
        h._select_model_set('trained')
        self.addCleanup(h._policy_worker.close)
        old.close.assert_called_once_with()
        obs=np.zeros((1,61),np.float32);obs[0,5]=-1
        for name in infer.NAMES:
            output,_=h._policy_worker.run(name,obs)
            np.testing.assert_array_equal(output,infer.infer(h.sessions[name],obs))
        command=[sys.executable,'-I','-B',str(root/'policy/policy_worker.py'),'--worker','7',str(root/'policy/model-sets/trained')]
        self.assertTrue(allowed_worker_command(sys.executable,command,root))
        command[-1]='/tmp/other-models';self.assertFalse(allowed_worker_command(sys.executable,command,root))

    def test_worker_start_failure_does_not_commit_model_selection(self):
        h=Harness();old=mock.Mock();h._policy_worker=old;sessions=h.sessions
        with mock.patch('policy_worker.PolicyWorker',side_effect=RuntimeError('startup failed')):
            with self.assertRaisesRegex(RuntimeError,'startup failed'):h._select_model_set('trained')
        self.assertIs(h.sessions,sessions);self.assertIs(h._policy_worker,old)
        old.close.assert_not_called();self.assertEqual(h.events,[])


class Protocol(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        Path(self.tmp.name,'index.html').write_bytes(SOURCE.read_bytes())
        self.runtime=Runtime()
        def connect(model_set=None):
            self.runtime.calls.append(('connect',model_set));return self.runtime.snapshot()
        self.runtime.connect=connect
        self.w=web.NativeWorkbench(self.runtime,web_root=self.tmp.name,authentication_required=False)
    def body(self,**fields):return dict(action='connect',page_generation=self.w.generation,control_id=CLIENT,**fields)
    def test_each_button_routes_model_set_without_start_command(self):
        for name in ['original','trained']:
            self.w.motion_action(self.body(model_set=name),principal='browser',live=True)
        self.assertEqual(self.runtime.calls,[('connect','original'),('connect','trained')])
    def test_invalid_group_rejected_before_runtime_or_calibration(self):
        with self.assertRaisesRegex(ValueError,'模型组'):
            self.w.motion_action(self.body(model_set='elsewhere'),principal='browser',live=True)
        self.assertEqual(self.runtime.calls,[])
    def test_legacy_connect_uses_current_group(self):
        self.w.motion_action(self.body(),principal='browser',live=True)
        self.assertEqual(self.runtime.calls,[('connect',None)])


suite=unittest.TestSuite(unittest.defaultTestLoader.loadTestsFromTestCase(t) for t in [Models,Protocol])
result=unittest.TextTestRunner(verbosity=2).run(suite)
a.output.write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,hardware_access=False,
    failures=[str(x) for x in result.errors+result.failures]),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
