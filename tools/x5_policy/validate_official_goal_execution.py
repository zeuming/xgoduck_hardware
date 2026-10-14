"""Exercise the complete pinned host candidate with device access denied.

Uses the owner validator CLI (--source and --output). Firmware bytes/guards are
tested separately with the actual C++ command handler, not this ideal plant.
"""
import json
import struct
import time
import unittest
import numpy as np
import validate_serial_owner as base
import validate_deferred_control as deferred
from causal_test_transport import PairedFakeTransport

_legacy_request = PairedFakeTransport.request


def position_request(self, kind, payload=b''):
    if kind == 24:
        self.requests.append((kind,payload))
        failure = getattr(self, 'execution_cap_failure', None)
        if isinstance(failure, BaseException): raise failure
        return bytes([24,0]) if failure is None else failure
    if kind == base.motion.TARGET and len(payload) == 35:
        if payload[-1] != 1: raise ValueError('Unknown execution profile')
        answer = _legacy_request(self,kind,payload[:34])
        self.requests[-1] = (kind,payload)
        return answer
    return _legacy_request(self,kind,payload)


PairedFakeTransport.request = position_request


class ProfileTests(base.OwnerTests):
    def test_profile_selected_on_real_control_loop_and_normal_return(self):
        self.walk()
        goals=[p for k,p in self.t.requests if k==base.motion.TARGET]
        self.assertTrue(any(len(p)==34 for p in goals))
        self.assertEqual(goals[-1][-1:],b'\x01')
        self.assertEqual(len(goals[-1]),35)
        self.assertTrue(self.c.status()['policy_goal_execution']['firmware_capability_confirmed'])
        before=len(self.t.requests)
        self.c._trial_began=time.monotonic()-base.ZERO_WINDOW_S
        self.cycles(4)
        self.assertEqual(self.c._mode,'hold')
        goals=[p for k,p in self.t.requests[before:] if k==base.motion.TARGET]
        self.assertTrue(goals)
        self.assertTrue(all(len(p)==34 for p in goals))
        self.assertTrue(self.t.armed)

    def test_old_or_corrupt_firmware_cannot_start_or_silently_downgrade(self):
        self.c.close()
        for refusal in (bytes([24,1]),b'',bytes([24,0,1]),TimeoutError('caps timeout')):
            with self.subTest(refusal=str(refusal)):
                transport=base.GuardedTransport('fake-only');transport.execution_cap_failure=refusal
                c=base.motion.MotionController({'walk':base.FakePolicy()},self.cfg,
                    transport_factory=lambda _:transport,allow_fixed_head_walk=True,serial_owner=True)
                try:
                    with self.assertRaises(Exception):c._connect_once()
                    self.assertFalse(c._connected)
                    self.assertFalse(c._policy_position_negotiated)
                    self.assertFalse(transport.armed)
                    self.assertFalse(any(k in (base.motion.ARM,base.motion.TARGET) for k,p in transport.requests))
                finally:c.close()

    def test_reconnect_must_negotiate_again(self):
        self.assertTrue(self.c._policy_position_negotiated)
        self.c._disconnect(ValueError('link lost'))
        self.assertFalse(self.c._policy_position_negotiated)
        self.t.execution_cap_failure=bytes([24,1])
        with self.assertRaises(Exception):self.c._connect_once()
        self.assertFalse(self.c._policy_position_negotiated)

    def test_synchronous_mode_has_identical_profile_and_takeover(self):
        clock=base.FakeClock();t=PairedFakeTransport('fake-sync')
        c=base.motion.MotionController({'walk':base.FakePolicy()},self.cfg,
            transport_factory=lambda _:t,clock=clock,allow_fixed_head_walk=True,serial_owner=False)
        try:
            c._connect_once();c.capture_head_clearance(clearance_confirmed=True,expected_generation=c._generation)
            c.start_mode('hold',support_confirmed=True)
            for _ in range(160):clock.advance();c._cycle()
            self.assertTrue(c.status()['hold_ready'])
            stance=c._raw_targets_locked(c._fixed_stance_locked())
            c.start_mode('walk',support_confirmed=True)
            # First takeover frame has weight zero, so changing execution
            # contract cannot jump straight to the unblended neural target.
            c.heartbeat(minimal=True);c._cycle()
            first=[p for k,p in t.requests if k==base.motion.TARGET][-1]
            self.assertEqual(len(first),35)
            self.assertEqual(list(struct.unpack_from('<I15H',first)[1:]),stance)
            for _ in range(110):clock.advance();c.heartbeat(minimal=True);c._cycle()
            self.assertEqual(c._mode,'walk')
            self.assertEqual(c.status()['candidate_profile']['zero_takeover']['policy_weight'],1.)
            np.testing.assert_array_equal(c._previous_action,np.zeros(14))
            self.assertTrue(all(p[-1:]==b'\x01' for k,p in t.requests if k==base.motion.TARGET and len(p)==35))
        finally:c.close()

    def test_out_of_range_policy_is_rejected_before_enqueue(self):
        self.walk()
        self.c._io_wait_until(time.monotonic()+.04)
        before=len(self.t.target_times)
        self.c.sessions['walk'].output[0,3]=100.
        with self.assertRaises(base.motion.MotionError):self.c._cycle()
        self.assertEqual(len(self.t.target_times),before)


if __name__=='__main__':
    suite=unittest.TestSuite([
        unittest.defaultTestLoader.loadTestsFromTestCase(ProfileTests),
        unittest.defaultTestLoader.loadTestsFromTestCase(deferred.DeferredTests)])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    report=dict(tests=result.testsRun,passed=result.wasSuccessful(),hardware_access=False,
        failures=[str(x) for x in result.failures+result.errors],source=str(base.a.source.resolve()))
    base.a.output.write_text(json.dumps(report,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
