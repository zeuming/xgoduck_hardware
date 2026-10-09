"""Actual controller and native ownership paths with fake physical feedback."""
import copy, math, struct, unittest
import numpy as np
import motion
from tracking_probe import AMPLITUDE_DEG, DURATION_S, COMPLETE, offset_at
import test_native_zero_adapter as fixture

class TrackingTests(unittest.TestCase):
    setUp = fixture.NativeZeroAdapterTests.setUp
    tearDown = fixture.NativeZeroAdapterTests.tearDown
    prepare = fixture.NativeZeroAdapterTests.prepare
    controller = fixture.NativeZeroAdapterTests.controller

    def advance(self, c, seconds, heartbeat=True):
        for _ in range(round(seconds / .02)):
            c._clock.advance(.02)
            if heartbeat and c._mode != 'off':
                c.heartbeat(expected_generation=c._generation, minimal=True)
            c._cycle()

    def ready(self):
        c = self.controller()
        c._connect_once()
        c.capture_head_clearance(clearance_confirmed=True, expected_generation=c._generation)
        c.start_mode('hold', support_confirmed=True, expected_generation=c._generation)
        self.advance(c, 3.2)
        self.assertTrue(c.status()['tracking_probe']['allowed'])
        return c

    def trigger(self, c, joint_id=23):
        return c.tracking_probe(expected_generation=c._generation, support_confirmed=True, joint_id=joint_id)

    def test_scheduling_has_no_motion_or_actor_and_only_knee_changes(self):
        c = self.ready()
        try:
            targets = []
            before = list(c._transport.requests)
            baseline = c._fixed_stance_locked()
            original = copy.deepcopy(c._config)
            self.trigger(c)
            self.assertEqual(c._transport.requests, before)
            for p in c.sessions.values():
                p.run = lambda *args, **kw: self.fail('HOLD diagnostic must not infer')
            for _ in range(260):
                self.advance(c, .02)
                if c._last_executed_target is not None:
                    targets.append(c._last_executed_target.copy())
            self.assertEqual(c._mode, 'off')
            self.assertFalse(c._transport.armed)
            self.assertEqual(c._state['torque_mask'], 0)
            self.assertEqual(c.status()['tracking_probe']['phase'], 'completed')
            for target in targets:
                for i, (a, b) in enumerate(zip(target, baseline)):
                    if i == motion.MOTOR_IDS.index(23):
                        self.assertGreaterEqual(a - b, -1e-6)
                        self.assertLessEqual(a - b, 3.000001)
                    else:
                        self.assertAlmostEqual(a, b, places=6)
            self.assertAlmostEqual(max(t[8] - baseline[8] for t in targets), 3., places=5)
            self.assertAlmostEqual(targets[-1][8], baseline[8], places=5)
            self.assertFalse(any(k == motion.ARM for k, _ in c._transport.requests[len(before):]))
            self.assertEqual(c._config, original)
            trace = c.completed_trial_traces()[-1]
            self.assertFalse(trace['truncated'])
            self.assertEqual(trace['frames'][0]['event'], 'tracking-start')
            self.assertTrue(all(tuple(f['command']) == (0., 0., 0.) for f in trace['frames']))
            self.assertEqual(trace['reason'], COMPLETE)
        finally:
            c.close()

    def test_generation_support_mode_and_duplicate_requests_fail_without_TARGET(self):
        c = self.ready()
        try:
            for kw in (dict(expected_generation=c._generation - 1, support_confirmed=True),
                       dict(expected_generation=c._generation, support_confirmed=False),
                       dict(expected_generation=True, support_confirmed=True)):
                before = list(c._transport.requests)
                with self.assertRaises(motion.MotionError):
                    c.tracking_probe(**kw)
                self.assertEqual(c._transport.requests, before)
            self.trigger(c)
            with self.assertRaises(motion.MotionError):
                self.trigger(c)
            with self.assertRaises(motion.MotionError):
                c.start_mode('walk', support_confirmed=True, expected_generation=c._generation)
            self.advance(c, .5)
            c.stop()
            self.assertEqual(c.status()['tracking_probe']['phase'], 'cancelled')
            self.assertFalse(c._transport.armed)
            with self.assertRaises(motion.MotionError):
                self.trigger(c)
        finally:
            c.close()

    def test_stale_heartbeat_and_initial_tilt_cannot_schedule(self):
        for failure in ('heartbeat', 'feedback', 'tilt', 'torque'):
            c = self.ready()
            try:
                if failure == 'heartbeat': c._heartbeat = c._clock() - .401
                if failure == 'feedback': c._sample_received = c._clock() - .101
                if failure == 'torque': c._state['torque_mask'] = 0
                if failure == 'tilt':
                    c._filtered.grav = [math.sin(math.radians(8.1)), 0., -math.cos(math.radians(8.1))]
                before = list(c._transport.requests)
                with self.assertRaises(motion.MotionError): self.trigger(c)
                self.assertEqual(c._transport.requests, before)
                self.assertIsNone(c._tracking)
            finally: c.close()

    def test_heartbeat_loss_stops_before_another_target_including_background_hold(self):
        c = self.ready()
        try:
            self.trigger(c)
            self.advance(c, .2)
            c.heartbeat(expected_generation=c._generation, minimal=True, background=True)
            before = sum(k == motion.TARGET for k, _ in c._transport.requests)
            c._clock.advance(.401)
            c._cycle()
            self.assertEqual(c._mode, 'off')
            self.assertFalse(c._transport.armed)
            self.assertEqual(sum(k == motion.TARGET for k, _ in c._transport.requests), before)
            self.assertEqual(c.status()['tracking_probe']['phase'], 'cancelled')
        finally: c.close()

    def test_actual_range_and_lost_torque_cancel_before_target(self):
        for failure in ('range', 'torque'):
            c = self.ready()
            try:
                self.trigger(c)
                self.advance(c, .2)
                if failure == 'range': c._transport.positions[8] = self.calibration['joints'][8]['min'] - 1
                else: c._transport.armed = False
                before = sum(k == motion.TARGET for k, _ in c._transport.requests)
                self.advance(c, .02)
                self.assertEqual(c._mode, 'off')
                self.assertFalse(c._transport.armed)
                self.assertEqual(sum(k == motion.TARGET for k, _ in c._transport.requests), before)
            finally: c.close()

    def test_deadline_crossing_before_final_commit_discards_target(self):
        c = self.ready()
        try:
            self.trigger(c)
            self.advance(c, 4.96)
            original = c._require_feedback_locked
            def delayed():
                original()
                c._clock.advance(.06)
                c._sample_received = c._clock()
                c._last_imu_received = c._clock()
            c._require_feedback_locked = delayed
            before = sum(k == motion.TARGET for k, _ in c._transport.requests)
            c._control_once()
            self.assertEqual(c._mode, 'off')
            self.assertFalse(c._transport.armed)
            self.assertEqual(sum(k == motion.TARGET for k, _ in c._transport.requests), before)
        finally: c.close()

    def test_display_view_cannot_cancel_live_diagnostic(self):
        from status_snapshot import capture_display_view
        c = self.ready()
        try:
            self.trigger(c)
            with c._lock: view = capture_display_view(c)
            view._tracking['phase'] = 'cancelled'
            self.assertEqual(c._tracking['phase'], 'running')
        finally: c.close()

    def test_active_eight_degree_tilt_stops_before_target(self):
        c = self.ready()
        try:
            self.trigger(c)
            self.advance(c, .2)
            c._tilt_locked = lambda: 8.01
            before = sum(k == motion.TARGET for k, _ in c._transport.requests)
            self.advance(c, .02)
            self.assertEqual(c._mode, 'off')
            self.assertFalse(c._transport.armed)
            self.assertEqual(sum(k == motion.TARGET for k, _ in c._transport.requests), before)
        finally: c.close()

    def test_allowlisted_joint_alone_moves_and_returns_before_auto_stop(self):
        for sid in (14, 11, 21, 23):
            with self.subTest(joint=sid):
                c = self.ready()
                try:
                    baseline = c._fixed_stance_locked()
                    before = list(c._transport.requests)
                    self.trigger(c, sid)
                    self.assertEqual(c._transport.requests, before)
                    self.assertEqual(c.status()['tracking_probe']['id'], sid)
                    index = motion.MOTOR_IDS.index(sid)
                    peaks = []
                    for _ in range(250):
                        self.advance(c, .02)
                        if c._last_executed_target is not None:
                            target = c._last_executed_target.copy()
                            peaks.append(target[index] - baseline[index])
                            for i in range(15):
                                if i != index:
                                    self.assertAlmostEqual(target[i], baseline[i], places=6)
                    self.assertAlmostEqual(max(peaks), 3., places=5)
                    self.assertAlmostEqual(peaks[-1], 0., places=5)
                    self.advance(c, .04)
                    self.assertEqual(c._mode, 'off')
                    self.assertEqual(c._state['torque_mask'], 0)
                    self.assertEqual(c.status()['tracking_probe']['phase'], 'completed')
                    self.assertEqual(c.completed_trial_traces()[-1]['reason'], COMPLETE)
                    self.assertFalse(any(k == motion.ARM for k, _ in c._transport.requests[len(before):]))
                finally:
                    c.close()

    def test_unapproved_joint_types_and_mid_probe_switch_are_rejected(self):
        c = self.ready()
        try:
            for sid in (True, False, 14., '14', None, 10, 12, 13, 22, 24, 30, 34, -1, 999):
                before = list(c._transport.requests)
                with self.subTest(joint=sid), self.assertRaises(motion.MotionError):
                    self.trigger(c, sid)
                self.assertEqual(c._transport.requests, before)
                self.assertIsNone(c._tracking)
            self.trigger(c, 14)
            for sid in (11, 21, 23, 14):
                before = list(c._transport.requests)
                with self.assertRaises(motion.MotionError):
                    self.trigger(c, sid)
                self.assertEqual(c._transport.requests, before)
                self.assertEqual(c._tracking['id'], 14)
            self.advance(c, .3)
            c.stop()
            self.assertEqual(c.status()['tracking_probe']['phase'], 'cancelled')
            self.advance(c, .02)
            self.assertEqual(c._state['torque_mask'], 0)
        finally:
            c.close()

    def test_selected_endpoint_range_is_rechecked_without_affecting_other_joint(self):
        c = self.ready()
        try:
            # Isolate the per-joint endpoint gate; calibration authority is
            # independently tested and otherwise rejects any changed config.
            c._standing_reference_valid_locked = lambda: True
            index = motion.MOTOR_IDS.index(14)
            baseline = c._raw_targets_locked(c._fixed_stance_locked(), mode='hold')[index]
            c._config['joints'][index]['min'] = baseline - 20
            status = c.status()['tracking_probe']
            self.assertFalse(status['allowed_by_joint']['14'])
            self.assertTrue(status['allowed_by_joint']['23'])
            before = list(c._transport.requests)
            with self.assertRaises(motion.MotionError):
                self.trigger(c, 14)
            self.assertEqual(c._transport.requests, before)
            self.assertIsNone(c._tracking)
        finally:
            c.close()

    def test_schedule_boundaries(self):
        for t, expected in ((0., 0.), (1.2, 3.), (2.7, 3.), (3.9, 0.), (5., 0.)):
            self.assertAlmostEqual(offset_at(t), expected)
        for t in (-1., float('nan'), float('inf')):
            with self.assertRaises(ValueError): offset_at(t)

if __name__ == '__main__': unittest.main()
