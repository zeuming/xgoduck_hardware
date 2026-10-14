"""Real ONNX process and bounded IPC tests; no controller or hardware devices."""
import hashlib
from pathlib import Path
import signal
import os
import sys
import time
import unittest
import numpy as np
from policy_worker import PolicyWorker,PolicyWorkerError,allowed_worker_command

MODELS=Path(__file__).resolve().parents[2]/'output/x5_policy_bundle/models'

@unittest.skipUnless((MODELS/'xgoduck_walk.onnx').exists(),'Reviewed local model fixture absent')
class WorkerTests(unittest.TestCase):
    def setUp(self):
        self.hashes={'walk':hashlib.sha256((MODELS/'xgoduck_walk.onnx').read_bytes()).hexdigest()}
        self.worker=PolicyWorker(MODELS,self.hashes)
        self.addCleanup(self.worker.close)
        self.obs=np.zeros((1,61),np.float32)
    def test_cpu_process_matches_direct_onnx(self):
        import onnxruntime as ort
        opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
        opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
        session=ort.InferenceSession(str(MODELS/'xgoduck_walk.onnx'),sess_options=opts,providers=['CPUExecutionProvider'])
        for i in range(5):
            self.obs[0,0]=i*.01
            raw,ms=self.worker.run('walk',self.obs)
            np.testing.assert_array_equal(raw,session.run(None,{session.get_inputs()[0].name:self.obs})[0])
            self.assertGreaterEqual(ms,0.)
        self.assertNotEqual(self.worker.process.pid,os.getpid())
    def test_pending_cannot_grow_queue_and_discard_does_not_relabel_old_result(self):
        os.kill(self.worker.process.pid,signal.SIGSTOP)
        first=self.worker.submit('walk',self.obs)
        for _ in range(10):self.assertIsNone(self.worker.submit('walk',self.obs))
        self.worker.discard(first)
        self.assertIsNone(self.worker.submit('walk',self.obs))
        os.kill(self.worker.process.pid,signal.SIGCONT)
        deadline=time.monotonic()+1.
        second=None
        while second is None and time.monotonic()<deadline:
            second=self.worker.submit('walk',self.obs);time.sleep(.002)
        self.assertIsNotNone(second);self.assertNotEqual(first,second)
        self.assertEqual(self.worker.discarded,1)
    def test_crash_no_autorestart(self):
        pid=self.worker.process.pid;self.worker.process.kill();self.worker.process.wait(timeout=1.)
        with self.assertRaises(PolicyWorkerError):self.worker.submit('walk',self.obs)
        self.assertEqual(self.worker.process.pid,pid)
    def test_invalid_observation_not_sent(self):
        self.obs[0,0]=float('nan')
        with self.assertRaises(PolicyWorkerError):self.worker.submit('walk',self.obs)
        self.assertEqual(self.worker.submitted,0)
    def test_close_reaps_child(self):
        self.worker.close();self.assertIsNotNone(self.worker.process.poll())
    def test_preflight_timeout_returns_and_abandons_result(self):
        os.kill(self.worker.process.pid,signal.SIGSTOP)
        with self.assertRaises(PolicyWorkerError):self.worker.run('walk',self.obs,timeout=.025)
        self.assertEqual(self.worker.discarded,1)
        os.kill(self.worker.process.pid,signal.SIGCONT)

class LaunchGateTests(unittest.TestCase):
    def test_only_exact_cpu_entry_and_model_directory_are_allowed(self):
        root=Path('/dev/shm/native-test')
        command=[sys.executable,'-I','-B',str(root/'policy/policy_worker.py'),'--worker','7',str(root/'policy/models')]
        self.assertTrue(allowed_worker_command(sys.executable,command,root))
        for index,value in ((0,'sh'),(3,'/tmp/other.py'),(4,'--execute'),(5,'2'),(6,'/other/models')):
            changed=command.copy();changed[index]=value
            self.assertFalse(allowed_worker_command(sys.executable,changed,root))

if __name__=='__main__':unittest.main()
