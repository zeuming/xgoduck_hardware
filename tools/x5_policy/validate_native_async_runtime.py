"""Native lifecycle regression with an explicit fake inference process factory."""
import sys
if __name__ == '__main__':
    import argparse
    from pathlib import Path
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,required=True)
    args=parser.parse_args();root=args.source.resolve()
    sys.path[:0]=[str(root/'service'),str(root/'policy')]
    import production_motion
    sys.modules['motion']=production_motion
import threading
import time
import motion
from unittest import mock
import unittest
import numpy as np
import policy_worker
from test_native_runtime import NativeRuntimeTests as ExistingRuntimeTests

class CPUFixtureWorker:
    def __init__(self,*args):self.closed=False;self.seq=0;self.pending={}
    def run(self,name,obs,timeout=.1):
        session=sys.modules['infer'].session(name)
        return session.run(None,{'obs':obs})[0],.1
    def submit(self,name,obs):
        self.seq+=1;self.pending[self.seq]=self.run(name,obs);return self.seq
    def take(self,request_id):return self.pending.pop(request_id,None)
    def discard(self,request_id):self.pending.pop(request_id,None)
    def status(self):return dict(alive=not self.closed,backend='explicit_fake_process')
    def close(self):self.closed=True

class AsyncRuntimeTests(ExistingRuntimeTests):
    def setUp(self):
        super().setUp()
        self.worker_patch=mock.patch.object(policy_worker,'PolicyWorker',CPUFixtureWorker)
        self.worker_patch.start();self.addCleanup(self.worker_patch.stop)
    def test_actual_one_shot_cycle_stop_interrupts_inference_and_discards_target(self):
        runtime=self.create();state=runtime.connect()
        runtime.action(dict(action='start',mode='hold',support_confirmed=True,
            generation=state['hardware']['generation']))
        controller=runtime._controller
        with controller._lock:controller._hold_begin-=4
        self.wait_for(lambda:runtime.snapshot()['hardware']['hold_ready'])
        state=runtime.snapshot();waiting=threading.Event()
        def deferred(request_id):waiting.set();return None
        with mock.patch.object(runtime._policy_worker,'take',side_effect=deferred):
            running=runtime.action(dict(action='start',mode='walk',support_confirmed=True,
                generation=state['hardware']['generation']))
            self.assertTrue(waiting.wait(1.))
            before=sum(kind==motion.TARGET for kind,_ in self.transports[0].requests)
            began=time.monotonic()
            runtime.action(dict(action='stop',generation=running['hardware']['generation']))
            self.assertLess(time.monotonic()-began,.4)
            self.assertFalse(self.transports[0].armed)
        time.sleep(.05)
        self.assertEqual(sum(kind==motion.TARGET for kind,_ in self.transports[0].requests),before)
        self.assertEqual(controller.status()['mode'],'off')
        self.assertIsNone(controller._policy_job)

    def test_worker_attached_before_uart_loop_starts(self):
        runtime=self.create();runtime.connect()
        self.assertIs(runtime._controller._policy_worker,runtime._policy_worker)
    def test_worker_is_closed_after_runtime_disconnect(self):
        runtime=self.create();runtime.close();self.assertTrue(runtime._policy_worker.closed)
    def test_worker_startup_failure_has_no_uart_authority(self):
        with mock.patch.object(policy_worker,'PolicyWorker',side_effect=RuntimeError('CPU startup failed')):
            with self.assertRaisesRegex(RuntimeError,'CPU startup failed'):self.create()
        self.assertEqual(self.transports,[]);self.assertEqual(self.factory_calls,0)

if __name__ == '__main__':
    suite=unittest.TestLoader().loadTestsFromTestCase(AsyncRuntimeTests)
    result=unittest.TextTestRunner().run(suite)
    raise SystemExit(not result.wasSuccessful())
