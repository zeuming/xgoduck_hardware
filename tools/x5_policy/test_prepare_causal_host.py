"""Reproducible offline assembly refuses unreviewed source and patch inputs."""
import hashlib
from pathlib import Path
import tempfile
import unittest

from prepare_causal_host import prepare

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT/'output/duck_sim/adaptation-20261011/candidate'
PATCH = ROOT/'output/duck_sim/repair-20261011/timing/patch-v2/policy/candidate_controller.py'


class AssemblyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_combined_output_matches_tested_candidate_and_preserves_model_calibration(self):
        dest = self.root/'combined'
        report = prepare(SOURCE,dest,PATCH)
        tested = ROOT/'output/duck_sim/repair-20261011/host-causal-optimized'
        for name,digest in report['after'].items():
            data = (dest/'policy'/name).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(),digest)
            self.assertEqual(data,(tested/'policy'/name).read_bytes())
        for path in ('calibration.json','models/xgoduck_walk.onnx'):
            self.assertEqual((dest/'policy'/path).read_bytes(),(SOURCE/'policy'/path).read_bytes())
        self.assertFalse(report['installed'])
        self.assertFalse(report['walking_validated'])
        self.assertFalse(report['thresholds_changed'])

    def test_unreviewed_patch_rejects_before_output_creation(self):
        patch = self.root/'candidate.py'
        patch.write_bytes(PATCH.read_bytes()+b'\n# changed\n')
        dest = self.root/'rejected'
        with self.assertRaisesRegex(ValueError,'diagnostic-cost patch differs'):
            prepare(SOURCE,dest,patch)
        self.assertFalse(dest.exists())

    def test_changed_baseline_rejects_before_output_creation(self):
        source = self.root/'source';(source/'policy').mkdir(parents=True)
        (source/'policy/production_motion.py').write_text('changed')
        dest = self.root/'rejected'
        with self.assertRaisesRegex(ValueError,'baseline differs'):
            prepare(source,dest)
        self.assertFalse(dest.exists())

    def test_existing_output_is_never_overwritten(self):
        dest = self.root/'exists';dest.mkdir()
        marker = dest/'keep';marker.write_text('existing work')
        with self.assertRaisesRegex(ValueError,'must be new'):
            prepare(SOURCE,dest)
        self.assertEqual(marker.read_text(),'existing work')


if __name__ == '__main__':
    unittest.main()
