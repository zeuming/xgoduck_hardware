"""STOP must acquire the control lock while ordinary policy work is in flight."""
import threading
import unittest

from current_pose_probe import controller_class


class ProbeLockScopeTests(unittest.TestCase):
    def test_inactive_diagnostic_does_not_hold_an_outer_policy_lock(self):
        entered, release, stop_acquired = threading.Event(), threading.Event(), threading.Event()
        class Parent:
            def __init__(self):
                self._lock = threading.RLock()
            def _control_once(self):
                entered.set()
                if not release.wait(2):
                    raise AssertionError('Test did not release parent control')
        cls = controller_class(Parent, ValueError, [10,11], 3, 5)
        c = cls()
        errors = []
        def control():
            try: c._control_once()
            except BaseException as e: errors.append(e)
        def stop():
            with c._lock: stop_acquired.set()
        runner = threading.Thread(target=control)
        stopper = threading.Thread(target=stop)
        runner.start()
        try:
            self.assertTrue(entered.wait(1))
            stopper.start()
            self.assertTrue(stop_acquired.wait(.5), 'Inactive diagnostic blocked STOP')
        finally:
            release.set()
            runner.join(2)
            if stopper.ident is not None: stopper.join(2)
        self.assertEqual(errors, [])


if __name__ == '__main__':
    unittest.main()
