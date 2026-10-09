from pathlib import Path
import sys, unittest
D = Path(__file__).resolve().parent
R = D.parents[2]
sys.path[:0] = [str(D/'source/policy'), str(D/'source/service'), str(D),
               str(R/'tools/x5_policy'), str(R/'tools/x5_recovery')]
import test_native_zero_adapter as fixture
fixture.fixture.motion_workbench.WORKER = (D/'source/service/worker-classes.py').read_text()
suite = unittest.defaultTestLoader.loadTestsFromNames([
    'test_tracking_probe', 'test_forward_trial', 'test_forward_boundary', 'test_native_zero_adapter',
    'test_phase_timings', 'test_status_snapshot', 'test_takeover',
    'test_native_workbench', 'test_native_full_workbench', 'test_tracking_ownership', 'test_forward_tilt'])
result = unittest.TextTestRunner(verbosity=1).run(suite)
raise SystemExit(not result.wasSuccessful())
