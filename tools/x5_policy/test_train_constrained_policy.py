"""Focused runner accounting and offline-boundary regression tests."""
import os
from pathlib import Path
import subprocess
import sys
import unittest

import torch

from train_constrained_policy import assert_state_identical, require_budget, allowed_training_device


class TrainingRunnerTests(unittest.TestCase):
    def test_accelerator_permission_does_not_allow_robot_devices(self):
        for path in ('/dev/nvidia0','/dev/nvidiactl','/dev/nvidia-uvm','/dev/nvidia-caps/nvidia-cap1'):
            self.assertTrue(allowed_training_device(path,True))
            self.assertFalse(allowed_training_device(path,False))
        for path in ('/dev/ttyUSB0','/dev/ttyACM0','/dev/mem','/dev/nvidia0/other','/dev/nvidia-invalid'):
            self.assertFalse(allowed_training_device(path,True))
    def test_bounded_smoke_counts_environment_work(self):
        self.assertEqual(require_budget(2,16,1,1,1,32),32)
        with self.assertRaisesRegex(ValueError,'exceeds'):
            require_budget(2,16,2,1,1,32)

    def test_invalid_optimizer_batches_and_counts_rejected(self):
        for args in ((2,16,1,1,3,32),(1,1,1,1,1,32),(True,16,1,1,1,32),(2,16,1,0,1,32)):
            with self.assertRaises(ValueError):require_budget(*args)

    def test_checkpoint_comparison_checks_moments_beyond_step_count(self):
        before={'state':{0:{'step':torch.tensor(2.),'exp_avg':torch.tensor([0.3,-0.1])}},
                'param_groups':[{'lr':0.001,'params':[0]}]}
        after={'state':{0:{'step':torch.tensor(2.),'exp_avg':torch.tensor([0.3,-0.1])}},
               'param_groups':[{'lr':0.001,'params':[0]}]}
        assert_state_identical(before,after)
        after['state'][0]['exp_avg'][0]=0.4
        with self.assertRaises(AssertionError):assert_state_identical(before,after)

    def test_checkpoint_comparison_checks_hyperparameters(self):
        with self.assertRaises(ValueError):assert_state_identical({'lr':0.001},{'lr':0.002})
        with self.assertRaises(ValueError):assert_state_identical([1,2],[1])

    def test_offline_guards_reject_network_and_robot_device_access(self):
        script='''
import os,socket
from train_constrained_policy import install_offline_guards
install_offline_guards()
assert os.environ['WANDB_MODE']=='disabled'
assert os.environ['WANDB_DISABLED']=='true'
for action in (lambda: socket.create_connection(('127.0.0.1',9)),
               lambda: socket.socket().connect(('127.0.0.1',9)),
               lambda: socket.socket().connect_ex(('127.0.0.1',9)),
               lambda: socket.socket(type=socket.SOCK_DGRAM).sendto(b'x',('127.0.0.1',9))):
    try: action()
    except RuntimeError: pass
    else: raise AssertionError('network access was not rejected')
try: open('/dev/ttyUSB0','rb')
except PermissionError: pass
else: raise AssertionError('robot device open was not rejected')
'''
        environment=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parent))
        result=subprocess.run([sys.executable,'-c',script],env=environment,capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
