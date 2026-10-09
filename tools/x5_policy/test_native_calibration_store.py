"""CPU-only persistence tests; all writes are to private temporary directories."""
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'x5_recovery'))
import native_calibration_store as store


BASE = Path(__file__).resolve().parents[2] / 'output/x5_recovery/replacement-board-20261004T011959+0800/native-workbench-20261006-01'


class NativeCalibrationStoreTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        self.config = json.loads((BASE / 'calibration-latest.json').read_text())
        self.provenance = json.loads((BASE / 'joint-zero-provenance.json').read_text())
        self.sha = store.digest(store.calibration_bytes(self.config))
        self.directory = self.root / 'persistent'
        self.subject = store.NativeCalibrationStore(self.directory, self.config, self.provenance, source_sha256=self.sha)

    def tearDown(self):
        self.temporary.cleanup()

    def changed(self):
        value = copy.deepcopy(self.config)
        value['imu']['bias_dps'][0] += .01
        return value

    def test_loading_verified_seed_does_not_create_or_write_persistent_directory(self):
        self.assertFalse(self.directory.exists())
        state = self.subject.load()
        self.assertFalse(state['persistent'])
        self.assertEqual(state['revision'], 0)
        self.assertEqual(state['calibration'], self.config)
        self.assertEqual(state['zero_provenance'], self.provenance)
        self.assertEqual(state['calibration_sha256'], self.sha)

    def test_explicit_save_atomically_persists_pair_and_survives_new_process_state(self):
        changed = self.changed()
        self.assertIsNone(self.subject.commit(changed, self.provenance))
        path = self.directory / store.STATE_NAME
        self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        self.assertEqual(self.directory.stat().st_mode & 0o777, 0o700)
        self.assertEqual(list(self.directory.iterdir()), [path])
        fresh = store.NativeCalibrationStore(self.directory, self.config, self.provenance, source_sha256=self.sha)
        state = fresh.load()
        self.assertTrue(state['persistent'])
        self.assertEqual(state['revision'], 1)
        self.assertEqual(state['calibration'], changed)
        self.assertEqual(state['zero_provenance'], self.provenance)

    def test_unreviewed_zero_change_is_rejected_before_writes(self):
        changed = copy.deepcopy(self.config)
        changed['joints'][0]['zero'] += 1
        with self.assertRaises(ValueError):
            self.subject.commit(changed, self.provenance)
        self.assertFalse(self.directory.exists())

    def test_authorized_selected_zero_save_requires_full_unmodified_chain(self):
        changed = copy.deepcopy(self.config)
        joint = changed['joints'][0]
        joint.update(zero=joint['zero'] + 1, min=None, max=None, direction_confirmed=False)
        ledger = copy.deepcopy(self.provenance)
        previous = ledger['entries'][-1]['sha256'] if ledger['entries'] else None
        entry = dict(sequence=len(ledger['entries']) + 1, id=joint['id'], generation=1234,
                     previous_sha256=previous, before=self.config, after=changed)
        entry['sha256'] = store.digest(store.canonical_bytes(entry))
        ledger['entries'].append(entry)
        self.subject.commit(changed, ledger)
        self.assertEqual(self.subject.load()['calibration'], changed)
        self.assertEqual(self.subject.load()['zero_provenance'], ledger)
        forged = copy.deepcopy(ledger)
        forged['entries'][-1]['after']['imu']['bias_dps'][0] += .1
        with self.assertRaises(ValueError):
            self.subject.commit(changed, forged)

    def test_disk_digest_chain_and_seed_identity_tampering_are_rejected(self):
        self.subject.commit(self.changed(), self.provenance)
        path = self.directory / store.STATE_NAME
        original = path.read_bytes()
        for mutation in ('calibration', 'calibration_sha256', 'source_calibration_sha256', 'revision'):
            value = json.loads(original)
            if mutation == 'calibration':
                value['calibration']['imu']['bias_dps'][0] += .1
            elif mutation == 'revision':
                value['revision'] = True
            else:
                value[mutation] = '0' * 64
            path.write_text(json.dumps(value))
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                store.NativeCalibrationStore(self.directory, self.config, self.provenance, source_sha256=self.sha)
            path.write_bytes(original)

    def test_second_stale_store_cannot_overwrite_newer_explicit_save(self):
        other = store.NativeCalibrationStore(self.directory, self.config, self.provenance, source_sha256=self.sha)
        self.subject.commit(self.changed(), self.provenance)
        with self.assertRaisesRegex(ValueError, '另一会话'):
            other.commit(self.config, self.provenance)
        self.assertEqual(self.subject.load()['revision'], 1)

    def test_failed_write_or_promotion_preserves_old_atomic_pair(self):
        self.subject.commit(self.config, self.provenance)
        original = (self.directory / store.STATE_NAME).read_bytes()
        with mock.patch.object(store.os, 'replace', side_effect=OSError('fake promotion failure')):
            with self.assertRaises(OSError):
                self.subject.commit(self.changed(), self.provenance)
        self.assertEqual((self.directory / store.STATE_NAME).read_bytes(), original)
        self.assertEqual(len(list(self.directory.iterdir())), 1)
        with mock.patch.object(store.os, 'fsync', side_effect=OSError('fake file fsync failure')):
            with self.assertRaises(OSError):
                self.subject.commit(self.changed(), self.provenance)
        self.assertEqual((self.directory / store.STATE_NAME).read_bytes(), original)

    def test_directory_sync_failure_after_promotion_reports_saved_warning(self):
        self.subject.commit(self.config, self.provenance)
        original = store.os.fsync
        def fsync(fd):
            if os.fstat(fd).st_mode & 0o170000 == 0o040000:
                raise OSError('fake directory fsync failure')
            return original(fd)
        with mock.patch.object(store.os, 'fsync', side_effect=fsync):
            warning = self.subject.commit(self.changed(), self.provenance)
        self.assertIn('校准已保存', warning)
        self.assertEqual(self.subject.load()['calibration'], self.changed())

    def test_private_permissions_symlinks_and_hardlinks_are_rejected(self):
        self.subject.commit(self.config, self.provenance)
        path = self.directory / store.STATE_NAME
        path.chmod(0o644)
        with self.assertRaises(ValueError):
            self.subject.load()
        path.chmod(0o600)
        linked = self.root / 'copy.json'
        os.link(path, linked)
        with self.assertRaises(ValueError):
            self.subject.load()
        linked.unlink()
        original = path.read_bytes()
        path.unlink()
        linked.write_bytes(original)
        linked.chmod(0o600)
        path.symlink_to(linked)
        with self.assertRaises(OSError):
            self.subject.load()

    def test_persistent_mutation_scope_is_exact_and_active_only_during_commit(self):
        temporary = self.directory / ('.calibration-state.' + 'a' * 32 + '.json')
        self.assertFalse(store.allows_persistent_mutation(temporary, 'write'))
        original = store.os.replace
        checks = []
        def replace(source, destination):
            checks.append(store.allows_persistent_mutation(source, 'rename'))
            checks.append(store.allows_persistent_mutation(destination, 'rename'))
            self.assertFalse(store.allows_persistent_mutation(self.directory / store.STATE_NAME, 'write'))
            self.assertFalse(store.allows_persistent_mutation('/etc/config', 'write'))
            return original(source, destination)
        with mock.patch.object(store.os, 'replace', side_effect=replace):
            self.subject.commit(self.changed(), self.provenance)
        self.assertEqual(checks, [True, True])
        self.assertFalse(store.allows_persistent_mutation(temporary, 'write'))

    def test_wrong_source_digest_and_noncanonical_directory_are_rejected(self):
        with self.assertRaises(ValueError):
            store.NativeCalibrationStore(self.directory, self.config, self.provenance, source_sha256='0' * 64)
        with self.assertRaises(ValueError):
            store.NativeCalibrationStore(Path('relative'), self.config, self.provenance, source_sha256=self.sha)


class RAMCalibrationAuthorityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name).resolve()
        (self.root / 'policy').mkdir(mode=0o700)
        (self.root / 'service').mkdir(mode=0o700)
        self.config = json.loads((BASE / 'calibration-latest.json').read_text())
        raw = store.calibration_bytes(self.config)
        path = self.root / 'policy/calibration.json'
        path.write_bytes(raw)
        path.chmod(0o600)
        self.subject = store.RAMCalibrationAuthority(self.root, store.digest(raw))

    def tearDown(self):
        self.temporary.cleanup()

    def test_explicit_publish_updates_exact_mac_revision_and_calibration(self):
        first = self.subject.verify()
        changed = copy.deepcopy(self.config)
        changed['imu']['bias_dps'][0] += .1
        raw = store.calibration_bytes(changed)
        result = self.subject.publish(raw)
        self.assertEqual(result, store.digest(raw))
        self.assertNotEqual(result, first)
        self.assertEqual(self.subject.verify(), result)
        record = json.loads(self.subject.authority_path.read_text())
        self.assertEqual(record['revision'], 1)
        spec = self.subject.child_spec()
        message = json.dumps({'revision': 1, 'sha256': result}, sort_keys=True, separators=(',', ':')).encode()
        import hmac
        self.assertEqual(record['mac'], hmac.new(bytes.fromhex(spec['calibration_authority_key']), message, hashlib.sha256).hexdigest())
        self.assertEqual(len(spec['calibration_authority_key']), 128)

    def test_arbitrary_calibration_or_authority_rewrite_fails_closed(self):
        original = self.subject.path.read_bytes()
        self.subject.path.write_bytes(b'{}')
        with self.assertRaises(ValueError):
            self.subject.verify()
        with self.assertRaises(ValueError):
            self.subject.publish(original)
        self.subject.path.write_bytes(original)
        record = json.loads(self.subject.authority_path.read_text())
        record['sha256'] = store.digest(b'{}')
        self.subject.authority_path.write_text(json.dumps(record))
        with self.assertRaises(ValueError):
            self.subject.verify()

    def test_lock_replacement_and_busy_lock_are_bounded_failures(self):
        descriptor = os.open(self.subject.lock_path, os.O_RDWR)
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        try:
            with self.assertRaisesRegex(ValueError, '锁超时'):
                self.subject.verify()
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)
        self.subject.lock_path.unlink()
        self.subject.lock_path.write_bytes(b'')
        self.subject.lock_path.chmod(0o600)
        with self.assertRaisesRegex(ValueError, '锁身份'):
            self.subject.verify()

    def test_authority_promotion_failure_restores_old_ram_pair(self):
        original = self.subject.path.read_bytes()
        old_record = self.subject.authority_path.read_bytes()
        actual = self.subject._atomic
        calls = []
        def fail_once(path, raw):
            calls.append(path)
            if path == self.subject.authority_path and len(calls) == 2:
                raise OSError('fake authority promotion failure')
            return actual(path, raw)
        changed = copy.deepcopy(self.config)
        changed['imu']['bias_dps'][0] += .1
        with mock.patch.object(self.subject, '_atomic', side_effect=fail_once), self.assertRaises(OSError):
            self.subject.publish(store.calibration_bytes(changed))
        self.assertEqual(self.subject.path.read_bytes(), original)
        self.assertEqual(self.subject.authority_path.read_bytes(), old_record)
        self.assertEqual(self.subject.verify(), store.digest(original))


if __name__ == '__main__':
    unittest.main()
