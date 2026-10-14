"""Passive per-motor feedback evidence, using the existing memory-only fixture.

Run the six original wire/history tests as well as the provenance cases. This
loads fixture definitions without running their independent test runner.
"""
import ast
import json
from pathlib import Path
import unittest

fixture_path = Path(__file__).with_name('test_policy_evidence.py')
tree = ast.parse(fixture_path.read_text())
definitions = []
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == 'result' for target in node.targets):
        break
    definitions.append(node)
namespace = {'__file__': str(fixture_path), '__name__': 'feedback_evidence_fixture'}
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(fixture_path), 'exec'), namespace)
Base = namespace['EvidenceTests']
Worker = namespace['Worker']
motion = namespace['motion']
args = namespace['a']


class FeedbackProvenanceTests(Base):
    def test_partial_scan_ages_are_preserved_without_changing_control(self):
        ages = [0] * 15
        ages[2] = 61000
        self.c._state['position_ages_us'] = ages
        self.c._state['online_mask'] = motion.ALL_MASK & ~(1 << 2)
        self.c.heartbeat(expected_generation=self.c._generation)
        self.c._control_once()
        row = self.c._policy_trace[-1]
        observed = row['input_feedback_provenance']
        self.assertEqual(observed['position_ages_us'][2], 61000)
        self.assertFalse(observed['online_mask'] & (1 << 2))
        self.assertTrue(row['target_write_completed'])
        self.assertFalse(row['motor_execution_confirmed'])
        before = observed['raw_positions'][2]
        self.c._state['positions'][2] += 5
        ages[2] = 0
        self.assertEqual(observed['position_ages_us'][2], 61000)
        self.assertEqual(observed['raw_positions'][2], before)
        self.assertEqual(self.c._trace[-1]['feedback_provenance']['position_ages_us'][2], 61000)

    def test_async_input_and_commit_keep_distinct_feedback_provenance(self):
        worker = Worker()
        self.c._policy_worker = worker
        self.c._state['position_ages_us'] = [60000] * 15
        self.c._control_once()
        sample = self.c._policy_job['observation_sample']
        worker.complete(1, .02)
        self.step()
        row = self.c._policy_trace[-1]
        original = row['input_feedback_provenance']
        latest = row['commit_feedback_provenance']
        self.assertEqual(original['state_sample_us'], sample)
        self.assertEqual(original['position_ages_us'], [60000] * 15)
        self.assertNotEqual(latest['state_sample_us'], sample)
        self.assertEqual(latest['position_ages_us'], self.c._state['position_ages_us'])
        self.assertNotEqual(latest['position_ages_us'], original['position_ages_us'])

    def test_missing_per_motor_ages_remain_unknown(self):
        self.c._state.pop('position_ages_us')
        evidence = self.c._snapshot_policy_feedback_locked()
        self.assertIsNone(evidence['position_ages_us'])
        self.assertEqual(evidence['scope'], 'received_state_fields_not_motor_execution_proof')


result = unittest.TextTestRunner(verbosity=2).run(
    unittest.defaultTestLoader.loadTestsFromTestCase(FeedbackProvenanceTests))
args.output.write_text(json.dumps(dict(tests=result.testsRun, errors=len(result.errors),
    failures=len(result.failures), passed=result.wasSuccessful(), hardware_access=False,
    deployment=False), indent=2) + '\n')
raise SystemExit(not result.wasSuccessful())
