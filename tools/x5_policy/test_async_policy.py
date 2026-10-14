"""No hardware: asynchronous model latency must not suspend sensor polling."""
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
import motion
from policy_worker import PolicyWorkerError
from test_motion import FakeClock, FakePolicy, FakeTransport, ready_config


class DeferredWorker:
    def __init__(self):
        self.requests = []
        self.results = {}
        self.dead = False
        self.discarded = []
    def submit(self, name, observation):
        if self.dead: raise PolicyWorkerError('worker exited')
        self.requests.append((name, observation.copy()))
        return len(self.requests)
    def take(self, request_id):
        if self.dead: raise PolicyWorkerError('worker exited')
        return self.results.pop(request_id, None)
    def discard(self, request_id): self.discarded.append(request_id)
    def complete(self, request_id, value=.01):
        self.results[request_id] = np.full((1, 14), value, np.float32), 12.
    def status(self): return dict(alive=not self.dead)


class AsyncPolicyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        path = Path(self.tmp.name)/'calibration.json'
        path.write_text(json.dumps(ready_config()))
        self.clock = FakeClock()
        self.transport = FakeTransport('fake')
        self.policy = FakePolicy()
        self.c = motion.MotionController({'walk':self.policy}, path,
            transport_factory=lambda p:self.transport, clock=self.clock)
        self.c._connect_once()
        self.c.start_mode('hold', support_confirmed=True)
        for _ in range(155):
            self.clock.advance(.02); self.c._cycle()
        self.c.start_mode('walk', support_confirmed=True)
        self.worker = DeferredWorker()
        # Fixture installs the test executor after synchronous preflight.
        self.c._policy_worker = self.worker
        self.c._cycle()
        self.baseline = len(self.transport.requests)
    def step(self, dt=.02):
        self.clock.advance(dt); self.c.heartbeat(); self.c._cycle()
    def targets(self):
        return sum(k==motion.TARGET for k,p in self.transport.requests[self.baseline:])
    def test_pending_model_keeps_polling_without_targets_or_action_history_updates(self):
        calls = self.policy.calls
        for _ in range(4): self.step()
        self.assertEqual(self.policy.calls,calls)
        self.assertEqual([k for k,p in self.transport.requests[self.baseline:]], [motion.STATE]*4)
        self.assertEqual(len(self.worker.requests),1)
        np.testing.assert_array_equal(self.c._previous_action,0)
        self.assertLess(self.c._age_locked(),.001)
    def test_completed_result_commits_once_then_pipelines_latest_observation(self):
        self.worker.complete(1)
        self.step()
        self.assertEqual(self.targets(),1)
        self.assertEqual(len(self.worker.requests),2)
        np.testing.assert_allclose(self.c._previous_action,.01)
        self.assertEqual(self.c._policy_job['observation_sample'], self.c._last_sample)
        np.testing.assert_array_equal(self.c._policy_job['previous'],self.c._previous_action)
        self.step()
        self.assertEqual(self.targets(),1)
    def test_old_observation_cannot_be_refreshed_by_new_feedback(self):
        self.worker.complete(1)
        self.step(.11)
        self.assertEqual(self.targets(),0)
        self.assertEqual(self.worker.discarded,[1])
        self.assertEqual(self.c._late_policy_frames,1)
        self.assertLess(self.c._age_locked(),.001)
        np.testing.assert_array_equal(self.c._previous_action,0)
    def test_stop_invalidates_inflight_result(self):
        self.c.stop();self.worker.complete(1);self.step()
        self.assertEqual(self.targets(),0)
        self.assertEqual(self.c._mode,'off')
    def test_command_change_and_change_back_still_invalidates_old_result(self):
        self.c.command(.1,0,0);self.c.command(0,0,0)
        self.worker.complete(1);self.step()
        self.assertEqual(self.targets(),0)
        self.assertIn(1,self.worker.discarded)
    def test_fault_not_hidden_by_pending_inference(self):
        self.transport.fault=1;self.step()
        self.assertEqual(self.c._mode,'off');self.assertFalse(self.transport.armed)
        self.assertEqual(self.targets(),0)
    def test_frozen_feedback_not_hidden_by_pending_inference(self):
        self.transport.freeze_sample=True;self.step(.11)
        self.assertEqual(self.c._mode,'off');self.assertFalse(self.transport.armed)
    def test_worker_death_is_explicit_error_not_synchronous_fallback(self):
        calls=self.policy.calls;self.worker.dead=True
        with self.assertRaisesRegex(motion.MotionError,'worker exited'): self.step()
        self.assertEqual(self.policy.calls,calls);self.assertEqual(self.targets(),0)
    def test_original_imu_age_is_part_of_model_deadline(self):
        self.c._cancel_policy_job()
        self.transport.imu_age_us=70000
        self.step()
        request_id=len(self.worker.requests)
        self.worker.complete(request_id)
        self.step(.04)
        self.assertEqual(self.targets(),0)
        self.assertIn(request_id,self.worker.discarded)
    def test_hold_transition_cancels_old_pending_job(self):
        self.c.start_mode('hold',support_confirmed=True)
        self.worker.complete(1);self.step()
        self.assertIn(1,self.worker.discarded)
        self.assertIsNone(self.c._policy_job)
        np.testing.assert_array_equal(self.c._previous_action,0)
    def test_stalled_model_does_not_renew_firmware_target_watchdog(self):
        original=self.transport.request
        last_target=self.clock()
        def watchdog(kind,payload=b''):
            if self.transport.armed and self.clock()-last_target>=.25:
                self.transport.armed=False;self.transport.torque=0;self.transport.cookie=0
            return original(kind,payload)
        self.transport.request=watchdog
        arms=sum(k==motion.ARM for k,p in self.transport.requests)
        for _ in range(15):self.step()
        self.assertEqual(self.c._mode,'off')
        self.assertFalse(self.transport.armed)
        self.assertEqual(self.targets(),0)
        self.assertEqual(sum(k==motion.ARM for k,p in self.transport.requests),arms)
    def test_attach_cannot_switch_backend_with_hardware_open(self):
        with self.assertRaises(ValueError):self.c.attach_policy_worker(self.worker)

if __name__ == '__main__':unittest.main()
