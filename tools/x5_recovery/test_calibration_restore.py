"""Local-only checks of strict, identity-bound calibration restoration."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import calibration_restore as restore


class RestoreTests(unittest.TestCase):
    def fixture(self):
        ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34]
        return dict(version=1, mechanical_reference_confirmed=True,
                    joints=[dict(id=sid, zero=2000 + i, min=None, max=None,
                                 sign=-1, direction_confirmed=False) for i, sid in enumerate(ids)],
                    imu=dict(axes=['x', 'y', 'z'], axes_confirmed=False,
                             calibrated=False, bias_dps=[0., 0., 0.]))

    def record(self, cfg=None):
        raw = json.dumps(self.fixture() if cfg is None else cfg)
        return dict(snapshot_utf8=raw, sha256=hashlib.sha256(raw.encode()).hexdigest(),
                    boot_id='bound-boot', cid='bound-cid', serial_device={'path': '/dev/mock-only'})

    def validate(self, record):
        return restore.validate_restore_record(record, boot_id='bound-boot', cid='bound-cid',
                                               serial_device={'path': '/dev/mock-only'})

    def test_valid_roundtrip_and_embedded_runtime_match(self):
        namespace = {}
        exec(compile(restore.runtime_source(), '<restore-runtime>', 'exec'), namespace)
        cfg = self.validate(self.record())
        self.assertEqual(cfg, self.fixture())
        self.assertEqual(namespace['validate_restore_record'](self.record(), boot_id='bound-boot',
                         cid='bound-cid', serial_device={'path': '/dev/mock-only'}), cfg)

    def test_digest_binding_and_duplicate_keys_rejected(self):
        for key, value in [('sha256', '0' * 64), ('boot_id', 'other'), ('cid', 'other'),
                           ('serial_device', {'path': '/dev/other'})]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                record = self.record(); record[key] = value; self.validate(record)
        record = self.record()
        record['snapshot_utf8'] = record['snapshot_utf8'].replace('"version": 1', '"version": 1, "version": 1')
        record['sha256'] = hashlib.sha256(record['snapshot_utf8'].encode()).hexdigest()
        with self.assertRaises(ValueError): self.validate(record)

    def test_closed_schema_types_ranges_and_axes(self):
        invalid = []
        for edit in (lambda c: c.update(extra=True), lambda c: c.update(version=True),
                     lambda c: c['joints'][0].update(id=True), lambda c: c['joints'][0].update(sign=True),
                     lambda c: c['joints'][0].update(zero=4096), lambda c: c['joints'][0].update(zero=True),
                     lambda c: c['joints'][0].update(min=3000, max=3500),
                     lambda c: c['imu'].update(axes=['x', 'y', '-z']),
                     lambda c: c['imu'].update(axes=['x', 'x', 'z']),
                     lambda c: c['imu'].update(bias_dps=[float('nan'), 0., 0.]),
                     lambda c: c['imu'].update(bias_dps=[True, 0., 0.]),
                     lambda c: c['imu'].update(calibrated=True)):
            cfg = copy.deepcopy(self.fixture()); edit(cfg); invalid.append(cfg)
        for i, cfg in enumerate(invalid):
            with self.subTest(case=i), self.assertRaises(ValueError): self.validate(self.record(cfg))

    def test_private_file_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'calibration.json'
            raw = self.record()['snapshot_utf8']; path.write_text(raw); path.chmod(0o600)
            digest = hashlib.sha256(raw.encode()).hexdigest()
            review = dict(expected_boot_id='bound-boot', expected_cid='bound-cid',
                          serial_device={'path': '/dev/mock-only'})
            self.assertEqual(restore.read_restore_record(path, digest, review)['sha256'], digest)
            link = Path(directory) / 'link.json'; link.symlink_to(path)
            with self.assertRaises(ValueError): restore.read_restore_record(link, digest, review)
            path.chmod(0o644)
            with self.assertRaises(ValueError): restore.read_restore_record(path, digest, review)


if __name__ == '__main__':
    unittest.main()
