import hashlib
from pathlib import Path
import tempfile
import unittest

import prepare_causal_feedback as candidate


class PrepareTest(unittest.TestCase):
    def test_exact_pinned_base_and_reproducible_source(self):
        with tempfile.TemporaryDirectory() as tmp:
            one,two=Path(tmp)/'one',Path(tmp)/'two'
            first=candidate.prepare(one);second=candidate.prepare(two)
            self.assertEqual(first,second)
            self.assertEqual(first['changed_firmware_files'],['firmware/main.cpp','firmware/motion_protocol.h'])
            self.assertEqual((one/'candidate.patch').read_bytes(),(two/'candidate.patch').read_bytes())
            self.assertFalse(first['hardware_accessed']);self.assertFalse(first['flashed'])
            with self.assertRaises(ValueError):candidate.prepare(one)

    def test_transforms_refuse_unpinned_shape(self):
        with self.assertRaises(ValueError):candidate.transform_main('unknown source')

    def test_gains_slew_and_retry_unchanged(self):
        before=(candidate.BASE/'firmware/main.cpp').read_text()
        after=candidate.transform_main(before)
        for begin,end in [('motion::Error prepareRunGains()', 'void sampleServos();'),
                          ('bool canRetryPosition(', '\nvoid sampleServos()')]:
            if begin not in before:
                continue
            first=before.index(begin);stop=before.index(end,first)
            other=after.index(begin);otherstop=after.index(end,other)
            self.assertEqual(before[first:stop],after[other:otherstop])
        self.assertEqual(before.count('kServoDeadlineUs = 1500'),after.count('kServoDeadlineUs = 1500'))
        # Safety predicate and existing retry path are byte-for-byte retained.
        a=before[before.index('    if (canRetryPosition(reply'):before.index('    positionReadEvidence[i].record(reply')]
        b=after[after.index('    if (canRetryPosition(reply'):after.index('    positionReadEvidence[i].record(reply')]
        self.assertEqual(a,b)


if __name__=='__main__':unittest.main()
