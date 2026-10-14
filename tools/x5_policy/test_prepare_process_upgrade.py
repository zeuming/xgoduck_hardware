"""Verify upgrade bytes against the retained real installation, without I/O."""
import ast
import json
from pathlib import Path
import tempfile
import unittest

import prepare_process_upgrade as app

SOURCE = Path(__file__).resolve().parents[2] / 'tmp/standing-audit-20261008/awake-start-feedback-stale-20261008T114618Z/software-fix/installation.tar.gz'


class UpgradeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.destination = Path(cls.tmp.name) / 'upgrade'
        cls.report = app.prepare(SOURCE, cls.destination)
        cls.before_outer = app.unpack(SOURCE.read_bytes())
        cls.after_outer = app.unpack((cls.destination / 'installation.tar.gz').read_bytes())
        cls.before = app.unpack(cls.before_outer['native-bundle.tar.gz'])
        cls.after = app.unpack(cls.after_outer['native-bundle.tar.gz'])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_model_candidate_UI_calibration_and_launcher_are_byte_identical(self):
        for name, data in self.before.items():
            if name.startswith(('policy/', 'web/')):
                self.assertEqual(data, self.after[name], name)
        for name in ('native_service.py', 'persistent_boot.py'):
            self.assertEqual(self.before_outer[name], self.after_outer[name])
        self.assertTrue(self.report['nonzero_commands_constructor_locked'])
        self.assertFalse(self.report['automatic_motion_start'])

    def test_manifest_seal_and_current_process_source_are_consistent(self):
        seal = json.loads(self.after_outer['installation.json'])
        for name, digest in seal['files'].items():
            self.assertEqual(app.digest(self.after_outer[name]), digest)
        manifest = json.loads(self.after_outer['deployment.json'])
        self.assertEqual(app.digest(self.after_outer['native-bundle.tar.gz']), manifest['archive_sha256'])
        for name, info in manifest['files'].items():
            self.assertEqual(app.digest(self.after[name]), info['sha256'], name)
        self.assertEqual(self.after['service/native_process.py'], (app.HERE / 'native_process.py').read_bytes())

    def test_bootstrap_preserves_exact_constructor_and_uses_split_process_entrypoint(self):
        def constructor(raw):
            tree = ast.parse(raw)
            return next(ast.dump(node) for node in tree.body if isinstance(node, ast.Assign)
                and any(isinstance(target, ast.Name) and target.id == 'runtime' for target in node.targets))
        self.assertEqual(constructor(self.before['service/bootstrap.py']),
                         constructor(self.after['service/bootstrap.py']))
        self.assertIn(b'native_process.serve(runtime,', self.after['service/bootstrap.py'])
        self.assertIn(b'native_process.allowed_web_command', self.after['service/bootstrap.py'])
        self.assertIn(b"self.snapshot(compact=True)", self.after['service/native_runtime.py'])

    def test_current_compact_runtime_and_standing_restore_are_not_replaced(self):
        current = Path(__file__).resolve().parents[2] / 'output/x5_recovery/replacement-board-20261004T011959+0800/native-service-install-compact-history-46h_wvb1/installation.tar.gz'
        directory = Path(self.tmp.name) / 'current-upgrade'
        report = app.prepare(current, directory)
        before = app.unpack(app.unpack(current.read_bytes())['native-bundle.tar.gz'])
        after = app.unpack(app.unpack((directory / 'installation.tar.gz').read_bytes())['native-bundle.tar.gz'])
        self.assertEqual(set(report['changed_inner_members']), {
            'service/bootstrap.py', 'service/native_workbench.py', 'service/native_process.py',
            'service/gamepad_web.py', 'service/gamepad_panel.html', 'service/native_gamepad.py',
            'service/system_health.py', 'service/system_health_panel.html'})
        for name in ('service/native_runtime.py', 'service/worker-classes.py', 'policy/motion.py'):
            self.assertEqual(before[name], after[name], name)


if __name__ == '__main__':
    unittest.main()
