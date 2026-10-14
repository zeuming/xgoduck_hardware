"""All-model page/gamepad integration. Fake runtime/input; no device access."""
import unittest

from native_gamepad import BEHAVIOR_BUTTONS, PRINCIPAL
from test_native_gamepad import SharedButtonTests


class AllModelButtonTests(SharedButtonTests):
    def setUp(self):
        super().setUp()
        original = self.runtime.action_receipt
        def receipt(body):
            result = original(body)
            h = self.runtime.motion['hardware']
            # The real controller reports torque state after ARM/STOP; the
            # generic workbench fixture only changes mode and generation.
            if body['action'] in ('start', 'stop'):
                h['armed'] = h['mode'] != 'off'
                h['release_confirmed'] = h['mode'] == 'off'
            return result
        self.runtime.action_receipt = receipt

    def test_every_behavior_key_selects_its_model_and_confirms_only_that_press(self):
        for button, mode in BEHAVIOR_BUTTONS.items():
            with self.subTest(mode=mode):
                self.setUp()
                self.pad.poll_once()
                self.event(1, button, 1)
                self.pad.poll_once()
                starts = self.calls('start')
                self.assertEqual([row['mode'] for row in starts], [mode])
                self.assertIs(starts[0]['behavior_confirmed'], True)
                self.assertEqual(self.wb.control_owner['principal'], PRINCIPAL)
                self.pad.poll_once()
                self.assertEqual(len(self.calls('start')), 1)
                self.pad.heartbeat_once()
                self.assertEqual(len(self.calls('heartbeat')), 1)

    def test_page_and_gamepad_can_start_each_model_from_the_other_source(self):
        for mode in BEHAVIOR_BUTTONS.values():
            with self.subTest(mode=mode):
                self.setUp()
                self.pad.poll_once()
                self.event(1, 3, 1)
                self.pad.poll_once()
                self.web_action('start', mode=mode, behavior_confirmed=True)
                self.assertEqual(self.calls('start')[-1]['mode'], mode)
                self.assertEqual(self.wb.control_owner['principal'], 'browser')
                self.pad.heartbeat_once()
                self.assertEqual(self.calls('heartbeat')[-1]['control_id'], self.wb.control_owner['id'])

    def test_select_stops_each_behavior_started_by_page_without_model_restart(self):
        for mode in BEHAVIOR_BUTTONS.values():
            with self.subTest(mode=mode):
                self.setUp()
                self.mode(mode)
                self.pad.poll_once()
                self.event(1, 10, 1)
                self.event(1, 8, 1)
                self.pad.poll_once()
                self.assertEqual(len(self.calls('stop')), 1)
                self.assertEqual(self.calls('start'), [])

    def test_input_loss_during_each_single_action_stops_without_hold_or_restart(self):
        for button, mode in BEHAVIOR_BUTTONS.items():
            with self.subTest(mode=mode):
                self.setUp()
                self.pad.poll_once()
                self.event(1, button, 1)
                self.pad.poll_once()
                self.input['connected'] = False
                self.pad.poll_once()
                self.assertEqual(len(self.calls('stop')), 1)
                self.assertEqual([row['mode'] for row in self.calls('start')], [mode])
                self.assertIsNone(self.pad.session)
                self.pad.heartbeat_once()
                self.assertEqual(self.calls('heartbeat'), [])

    def test_finished_episode_and_held_key_do_not_restart(self):
        self.pad.poll_once()
        self.event(1, 9, 1)
        self.pad.poll_once()
        self.mode('off')
        self.runtime.motion['hardware'].update(armed=False, release_confirmed=True)
        self.pad.poll_once()
        self.pad.heartbeat_once()
        self.assertEqual([row['mode'] for row in self.calls('start')], ['pick'])
        self.assertEqual(self.calls('heartbeat'), [])
        self.event(1, 9, 0)
        self.pad.poll_once()
        self.event(1, 9, 1)
        self.pad.poll_once()
        self.assertEqual([row['mode'] for row in self.calls('start')], ['pick', 'pick'])

    def test_rejected_behavior_key_is_not_retried(self):
        self.pad.poll_once()
        self.runtime.failure = 'ID30 target outside saved range'
        self.event(1, 8, 1)
        self.pad.poll_once()
        self.pad.poll_once()
        self.assertEqual(len(self.calls('start')), 1)
        self.assertIn('ID30 target outside saved range', self.pad.message)
        self.assertEqual(self.calls('stop'), [])

    def test_behavior_key_does_not_chain_during_another_single_action(self):
        self.pad.poll_once()
        self.event(1, 9, 1)
        self.pad.poll_once()
        self.event(1, 14, 1)
        self.pad.poll_once()
        self.assertEqual([row['mode'] for row in self.calls('start')], ['pick'])
        self.assertIn('尚未结束', self.pad.message)

    def test_multiple_operation_keys_and_initial_held_behavior_are_ignored(self):
        self.pad.poll_once()
        self.event(1, 8, 1)
        self.event(1, 9, 1)
        self.pad.poll_once()
        self.assertEqual(self.calls('start'), [])
        self.setUp()
        self.event(1, 14, 1)
        self.pad.poll_once()
        self.assertEqual(self.calls('start'), [])

    def test_stance_keys_during_single_action_do_not_chain_or_issue_implicit_stop(self):
        for button in (0, 3, 4, 11):
            with self.subTest(button=button):
                self.setUp()
                self.pad.poll_once()
                self.event(1, 9, 1)
                self.pad.poll_once()
                self.event(1, button, 1)
                self.pad.poll_once()
                self.assertEqual([row['mode'] for row in self.calls('start')], ['pick'])
                self.assertEqual(self.calls('stop'), [])

    def test_short_behavior_press_released_during_connect_is_one_request(self):
        self.mode('off')
        self.runtime.motion['hardware']['armed'] = False
        self.runtime.motion.update(phase='closed', owns_connection=False)
        original = self.runtime.connect
        def connect():
            result = original()
            self.runtime.motion.update(phase='ready', owns_connection=True)
            self.event(1, 8, 0)
            return result
        self.runtime.connect = connect
        self.pad.poll_once()
        self.event(1, 8, 1)
        self.pad.poll_once()
        self.assertEqual([row['mode'] for row in self.calls('start')], ['getup'])
        self.pad.poll_once()
        self.assertEqual(len(self.calls('start')), 1)


if __name__ == '__main__':
    unittest.main()
