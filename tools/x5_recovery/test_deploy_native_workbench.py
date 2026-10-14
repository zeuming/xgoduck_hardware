"""Pure CPU deployment tests; no SSH, socket bind, UART or device writes."""
import ast
import contextlib
import copy
import errno
import hashlib
import inspect
import json
import os
from pathlib import Path
import stat
import socket
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'x5_policy'))
import native_calibration_store as storage
import motion
import native_runtime
import native_process
import native_transports

import deploy_native_workbench as app
import feedback_stream
import motion_workbench
from transport_config import TransportConfig


BASE = app.HERE.parents[1] / 'output/x5_recovery/replacement-board-20261004T011959+0800'
FREEZE = BASE / 'native-workbench-20261006-01'
PROFILE = BASE / 'feedback-wifi-reboot-reviewed-20261006-01.json'
PASSWORD = 'test_only_random_safe_' + 'x' * 32


def load(path):
    return json.loads(path.read_text())


class NativeDeploymentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.expected = load(FREEZE / 'quiescence-check/expectations.json')
        cls.proof = load(FREEZE / 'quiescence-check/proof.json')
        # The CPU fixture keeps historical identity metadata but exercises the
        # current source validator; it is never an actual deployment receipt.
        canonical = app._canonical_expected(cls.expected, PROFILE, feedback_stream.WIFI_RECONNECT_PROFILE_SHA256)
        canonical = json.loads(json.dumps(canonical, sort_keys=True, allow_nan=False))
        check_source = (app.feedback_connection_check.runtime_source() + '\nEXPECTED = ' + repr(canonical) +
                        '\nraise SystemExit(check_connection_metadata(EXPECTED))\n').encode()
        cls.proof['source_sha256'] = hashlib.sha256(check_source).hexdigest()
        cls.config = load(FREEZE / 'calibration-latest.json')
        cls.snapshot = load(FREEZE / 'startup-state.json')
        cls.zeros = load(FREEZE / 'joint-zero-provenance.json')
        cls.transport = TransportConfig(kind='wifi', host='192.168.1.11', interface='en0')
        cls.arguments = dict(expected=cls.expected, reviewed_profile_path=PROFILE,
            reviewed_profile_sha256=feedback_stream.WIFI_RECONNECT_PROFILE_SHA256,
            quiescence_proof=cls.proof, transport_config=cls.transport, access_password=PASSWORD)
        # A real archive is built from already saved local models/dependencies.
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No child allowed')):
            cls.archive, cls.deployment = app.build_bundle(cls.snapshot, cls.config, cls.zeros, **cls.arguments)
            cls.source = app.remote_source(cls.expected, cls.deployment,
                reviewed_profile_path=PROFILE,
                reviewed_profile_sha256=feedback_stream.WIFI_RECONNECT_PROFILE_SHA256,
                transport_config=cls.transport)
        cls.members = app.archive_tools.inspect_tar(cls.archive, cls.deployment['files'])

    def test_current_four_models_calibration_full_ui_and_guards_are_exact(self):
        files = self.members
        self.assertEqual(hashlib.sha256(self.archive).hexdigest(), self.deployment['archive_sha256'])
        self.assertEqual(64, len(self.deployment['calibration_sha256']))
        self.assertEqual(motion_workbench.calibration_bytes(self.config), files['policy/calibration.json'])
        self.assertEqual(feedback_stream.runtime_source().encode(), files['service/guard-runtime.py'])
        self.assertEqual((app.HERE / 'feedback_calibration.html').read_bytes(), files['web/index.html'])
        self.assertEqual({'walk', 'getup', 'pick', 'roulade'}, set(self.deployment['model_hashes']))
        for name, digest in self.deployment['model_hashes'].items():
            self.assertEqual(digest, hashlib.sha256(files['policy/models/xgoduck_' + name + '.onnx']).hexdigest())
        for name in ('zero-reference.png', 'positive-direction.png'):
            self.assertEqual((app.HERE.parent / 'x5_policy/assets' / name).read_bytes(), files['web/assets/' + name])
        self.assertEqual(self.zeros, json.loads(files['service/joint-zero-provenance.json']))
        self.assertEqual(self.proof['serial_node'], self.deployment['serial_node_review'])
        self.assertEqual(self.proof['serial_path'], self.deployment['serial_path_review'])
        self.assertTrue(self.deployment['full_workbench'])
        self.assertEqual('/var/lib/xgoduck-workbench', self.deployment['persistent_calibration_directory'])
        for name in app.CALIBRATION_MODULES:
            self.assertEqual((app.HERE / (name + '.py')).read_bytes(), files['service/' + name + '.py'])
        for name in ('native_calibration', 'native_transports', 'native_calibration_store'):
            self.assertEqual((app.HERE.parent / 'x5_policy' / (name + '.py')).read_bytes(),
                             files['service/' + name + '.py'])

    def test_access_password_is_private_bundle_only_and_template_has_no_live_claims(self):
        self.assertNotIn(PASSWORD, json.dumps(self.deployment))
        self.assertEqual({'password': PASSWORD}, json.loads(self.members['service/access-secret.json']))
        template = json.loads(self.members['service/startup-snapshot.json'])
        self.assertNotIn('token', template)
        for field in ('feedback', 'body_imu', 'motion', 'maintenance'):
            self.assertIsNone(template[field])
        self.assertEqual('closed', template['session']['status'])
        self.assertFalse(template['session']['recovery_allowed'])
        self.assertIsNone(template['session']['closed_proof'])

    def test_authentication_mode_is_explicit_bool_and_consistent_with_child_and_ready(self):
        self.assertIs(self.deployment['auth_required'], True)
        runtime = json.loads(self.members['service/deployment.json'])
        self.assertIs(runtime['auth_required'], True)
        self.assertIn("authentication_required=deployment['auth_required']", app.CHILD_BOOTSTRAP)
        anonymous = dict(self.deployment, auth_required=False)
        ready = dict(self.ready(), auth_required=False)
        self.assertEqual(ready, app.validate_ready(ready, 123, '/dev/shm/xgoduck-native-test', anonymous, self.expected))
        for manifest in (self.deployment, anonymous):
            mismatch = dict(self.ready(), auth_required=not manifest['auth_required'])
            with self.assertRaises(ValueError):
                app.validate_ready(mismatch, 123, '/dev/shm/xgoduck-native-test', manifest, self.expected)

    def test_restart_snapshot_preserves_fault_and_identity_holds_without_live_ownership(self):
        snapshot=copy.deepcopy(self.snapshot)
        snapshot['maintenance']=dict(fault_hold=True,identity_changed=True,
            error='USB disconnected; actual feedback review still required',owns_connection=True)
        value=app._snapshot_template(snapshot,self.config)
        self.assertIsNone(value['maintenance'])
        holds=value['native_recovery_holds']
        self.assertTrue(holds['maintenance-fault-hold.json']['requires_feedback_review'])
        self.assertTrue(holds['maintenance-identity-hold.json']['requires_physical_mapping_review'])
        self.assertEqual(holds['maintenance-fault-hold.json']['reason'],snapshot['maintenance']['error'])
        self.assertEqual(snapshot['maintenance']['owns_connection'],True)
        for mode in ('False', 0, None):
            with self.assertRaisesRegex(ValueError, 'explicit boolean'):
                app.build_bundle(self.snapshot, self.config, self.zeros, **self.arguments, authentication_required=mode)

    def test_explicit_anonymous_bundle_preserves_exact_calibration_and_models(self):
        with mock.patch.object(subprocess, 'Popen', side_effect=AssertionError('No child allowed')):
            archive, deployment = app.build_bundle(self.snapshot, self.config, self.zeros,
                **self.arguments, authentication_required=False)
            source = app.remote_source(self.expected, deployment, reviewed_profile_path=PROFILE,
                reviewed_profile_sha256=feedback_stream.WIFI_RECONNECT_PROFILE_SHA256,
                transport_config=self.transport)
        members = app.archive_tools.inspect_tar(archive, deployment['files'])
        self.assertIs(deployment['auth_required'], False)
        self.assertIs(json.loads(members['service/deployment.json'])['auth_required'], False)
        self.assertEqual(self.deployment['calibration_sha256'], deployment['calibration_sha256'])
        self.assertEqual(self.deployment['model_hashes'], deployment['model_hashes'])
        self.assertTrue("authentication_required=deployment['auth_required']" in members['service/bootstrap.py'].decode())
        compile(source, '<anonymous-native-parent>', 'exec')

    def test_only_two_reviewed_classes_without_worker_entrypoint_or_parent_lease(self):
        actual = ast.parse(self.members['service/worker-classes.py'])
        source = ast.parse(motion_workbench.WORKER)
        self.assertEqual(['ReviewedTransport', 'OneShotController'], [node.name for node in actual.body])
        for node in actual.body:
            original = next(part for part in ast.walk(source) if isinstance(part, ast.ClassDef) and part.name == node.name)
            self.assertEqual(ast.dump(original), ast.dump(node))
        child = self.members['service/bootstrap.py'].decode()
        self.assertNotIn('prctl', child)
        self.assertNotIn('getppid', child)
        self.assertNotIn('run_reader()', child)
        self.assertIn('auto_connect=not offline_startup', child)
        self.assertEqual(self.deployment['startup_mode'],'strict')
        self.assertNotIn('start_mode(', child)

    def test_holder_definition_preparation_runs_once_before_models_and_hardware_factory(self):
        tree=ast.parse(app.CHILD_BOOTSTRAP)
        matches=[(i,node) for i,node in enumerate(tree.body) if isinstance(node,ast.Expr)
            and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute)
            and node.value.func.attr=='prepare_holder_runtime']
        self.assertEqual(len(matches),1)
        index,node=matches[0]
        self.assertEqual(ast.unparse(node.value),'native_transports.prepare_holder_runtime(guard_source)')
        compile_index=next(i for i,part in enumerate(tree.body) if isinstance(part,ast.Assign)
                           and any(isinstance(target,ast.Name) and target.id=='guard_code' for target in part.targets))
        factory_index=next(i for i,part in enumerate(tree.body) if isinstance(part,ast.FunctionDef)
                           and part.name=='connection_guard_factory')
        runtime_index=next(i for i,part in enumerate(tree.body) if isinstance(part,ast.Assign)
                           and any(isinstance(target,ast.Name) and target.id=='runtime' for target in part.targets))
        self.assertEqual(index,compile_index+1)
        self.assertLess(index,factory_index);self.assertLess(index,runtime_index)
        prepare=mock.Mock()
        namespace={'native_transports':types.SimpleNamespace(prepare_holder_runtime=prepare),
                   'guard_source':'immutable reviewed definitions'}
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<bootstrap-static-preparation>','exec'),namespace)
        prepare.assert_called_once_with('immutable reviewed definitions')

    def test_bootstrap_explicitly_selects_hold_only_hip_bias_without_model_opt_in(self):
        # Execute only the packaged constructor call with a fake runtime. The
        # deployment selects the reviewed HOLD candidate without bypassing any default gate or
        # starting a model, opening UART or executing the rest of bootstrap.
        tree = ast.parse(self.members['service/bootstrap.py'])
        assignments = [node for node in tree.body if isinstance(node, ast.Assign)
                       and any(isinstance(target, ast.Name) and target.id == 'runtime'
                               for target in node.targets)]
        self.assertEqual(len(assignments), 1)
        runtime_call = assignments[0]
        self.assertEqual(ast.unparse(runtime_call.value.func), 'native_runtime.NativeRuntime')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'service').mkdir()
            classes = 'reviewed worker classes fixture'
            (root / 'service/worker-classes.py').write_text(classes)
            for offline in (False, True):
                with self.subTest(offline_startup=offline):
                    constructor = mock.Mock(return_value=object())
                    guards = {'reviewed': 'guard fixture'}
                    startup_fault = {'reason': 'explicit offline hardware fault'} if offline else None
                    namespace = dict(native_runtime=types.SimpleNamespace(NativeRuntime=constructor),
                                     namespace=guards, root=root, deployment=self.deployment,
                                     calibration=self.config, startup_fault=startup_fault,
                                     offline_startup=offline)
                    exec(compile(ast.Module(body=[runtime_call], type_ignores=[]),
                                 '<packaged-native-runtime-constructor>', 'exec'), namespace)
                    constructor.assert_called_once_with(guards, str(root), self.deployment,
                        self.config, classes, startup_fault=startup_fault['reason'] if offline else None,
                        allow_fixed_head_walk=False, hold_hip_pitch_bias_deg=-2.0)
                    self.assertIs(namespace['runtime'], constructor.return_value)

    def test_walk_opt_in_is_deployment_only_and_manual_head_capture_is_not_required(self):
        for constructor in (native_runtime.NativeRuntime, motion.MotionController):
            with self.subTest(constructor=constructor.__name__):
                self.assertIs(inspect.signature(constructor).parameters['allow_fixed_head_walk'].default,
                              False)
                self.assertEqual(inspect.signature(constructor).parameters['hold_hip_pitch_bias_deg'].default, 0)
        self.assertIs(inspect.signature(native_runtime.NativeRuntime).parameters[
            'require_head_clearance'].default, False)

    def test_bad_or_stale_quiescence_is_rejected_before_packaging(self):
        for change in ({'generation': self.expected['generation'] + 1}, {'serial_holders': [99]},
                       {'ssh_exit_code': 255}, {'identity_checked': False}, {'serial_open_attempts': 1}):
            proof = dict(self.proof, **change)
            arguments = dict(self.arguments, quiescence_proof=proof)
            with mock.patch.object(motion_workbench, 'prepare_bundle', side_effect=AssertionError('Must reject early')):
                with self.assertRaises(ValueError):
                    app.build_bundle(self.snapshot, self.config, self.zeros, **arguments)

    def test_default_usb_and_unreviewed_zero_changes_are_rejected(self):
        args = dict(self.arguments, transport_config=None)
        with self.assertRaisesRegex(ValueError, 'explicit reviewed Wi-Fi'):
            app.build_bundle(self.snapshot, self.config, self.zeros, **args)
        config = copy.deepcopy(self.config)
        config['joints'][0]['zero'] += 1
        with self.assertRaises(ValueError):
            app._validate_zero_provenance(self.zeros, config)

    def ready(self, *, connected=False):
        hardware = (dict(connected=True, mode='off', release_confirmed=True) if connected else None)
        return dict(schema_version=1, kind='native_service_ready', deployment_generation=self.expected['generation'],
            pid=123, address='192.168.1.11', port=8767, listener_fd=8, listener_inode=999,
            auth_required=True, web_ready=True, controller_ready=connected,
            ram_directory='/dev/shm/xgoduck-native-test', ram_directory_node=[1, 22],
            calibration_sha256=self.deployment['calibration_sha256'], model_hashes=self.deployment['model_hashes'],
            calibration_store=dict(source_calibration_sha256=self.deployment['source_calibration_sha256'],
                revision=0,persistent=False,calibration_sha256=self.deployment['calibration_sha256'],
                zero_provenance_sha256=self.deployment['source_zero_provenance_sha256']),
            motor_start_attempted=False, automatic_reconnect=False,
            startup_fault=None,
            motion=dict(phase='ready' if connected else 'fault', hardware=hardware,
                closed_proof={'serial_closed': True, 'release_confirmed': False}, error=None if connected else 'IMU invalid'))

    def test_real_fault_web_can_be_ready_without_inventing_motor_readiness(self):
        for connected in (False, True):
            ready = self.ready(connected=connected)
            self.assertEqual(ready, app.validate_ready(ready, 123, '/dev/shm/xgoduck-native-test', self.deployment, self.expected))
        forged = self.ready()
        forged['controller_ready'] = True
        with self.assertRaisesRegex(ValueError, 'invent'):
            app.validate_ready(forged, 123, '/dev/shm/xgoduck-native-test', self.deployment, self.expected)

    def test_ready_wrong_pid_port_schema_models_and_hardware_are_rejected(self):
        for change in ({'pid': 124}, {'port': 8765}, {'model_hashes': {}}, {'web_ready': False},
                       {'listener_inode': 0}, {'extra': True}, {'motor_start_attempted': True},
                       {'automatic_reconnect': True}, {'ram_directory_node': [True, 22]}):
            with self.assertRaises(ValueError):
                app.validate_ready(dict(self.ready(), **change), 123, '/dev/shm/xgoduck-native-test', self.deployment, self.expected)
        ready = self.ready(); ready['motion']['hardware'] = ['not a status']
        with self.assertRaises(ValueError):
            app.validate_ready(ready, 123, '/dev/shm/xgoduck-native-test', self.deployment, self.expected)

    def test_ready_authorized_persistent_calibration_has_exact_seed_receipt(self):
        value = self.ready()
        value['calibration_sha256'] = 'a' * 64
        value['calibration_store'].update(revision=2,persistent=True,calibration_sha256='a'*64)
        self.assertEqual(value,app.validate_ready(value,123,'/dev/shm/xgoduck-native-test',self.deployment,self.expected))
        for change in ({'revision':True},{'revision':0},{'persistent':False},
                       {'source_calibration_sha256':'b'*64},{'calibration_sha256':'c'*64},
                       {'zero_provenance_sha256':'invalid'},{'extra':True}):
            bad = copy.deepcopy(value)
            bad['calibration_store'].update(change)
            with self.assertRaises(ValueError):
                app.validate_ready(bad,123,'/dev/shm/xgoduck-native-test',self.deployment,self.expected)

    def test_explicit_offline_bundle_has_no_hardware_proof_or_automatic_connection(self):
        arguments=dict(self.arguments,quiescence_proof=None,offline_startup=True)
        with mock.patch.object(subprocess,'Popen',side_effect=AssertionError('CPU packaging only')):
            archive,manifest=app.build_bundle(self.snapshot,self.config,self.zeros,**arguments)
            source=app.remote_source(self.expected,manifest,reviewed_profile_path=PROFILE,
                reviewed_profile_sha256=feedback_stream.WIFI_RECONNECT_PROFILE_SHA256,transport_config=self.transport)
        self.assertEqual(manifest['startup_mode'],'metadata_only_fault_page')
        for field in ('quiescence_proof','serial_path_review','serial_node_review'):
            self.assertIsNone(manifest[field])
        compile(source,'<offline-metadata-only-native-parent>','exec')
        members=app.archive_tools.inspect_tar(archive,manifest['files'])
        self.assertEqual(members['policy/calibration.json'],self.members['policy/calibration.json'])
        self.assertEqual(manifest['model_hashes'],self.deployment['model_hashes'])
        child=members['service/bootstrap.py'].decode()
        self.assertIn('startup_fault=startup_fault[\'reason\'] if offline_startup else None',child)
        self.assertIn('auto_connect=not offline_startup',child)
        for mode in (0,'True',None):
            with self.assertRaisesRegex(ValueError,'explicit boolean'):
                app.build_bundle(self.snapshot,self.config,self.zeros,**dict(self.arguments,offline_startup=mode))
        with self.assertRaisesRegex(ValueError,'must not reuse'):
            app.build_bundle(self.snapshot,self.config,self.zeros,**self.arguments,offline_startup=True)

    def test_offline_readiness_cannot_certify_hardware_or_clear_actual_fault(self):
        manifest=dict(self.deployment,startup_mode='metadata_only_fault_page')
        ready=self.ready()
        reason='Unreviewed USB disconnect fault'
        ready['startup_fault']=dict(schema_version=1,kind='native_offline_startup_fault',reason=reason,
            proof_sha256='a'*64,hardware_gate_passed=False,serial_open_attempts=0,automatic_retry=False)
        ready['motion'].update(startup_fault=reason,owns_connection=False,
            closed_proof=dict(serial_closed=True,release_confirmed=False,serial_open_attempts=0,
                              request_counts={},no_uart_opened=True))
        self.assertEqual(app.validate_ready(ready,123,'/dev/shm/xgoduck-native-test',manifest,self.expected),ready)
        for change in ({'hardware_gate_passed':True},{'serial_open_attempts':1},{'proof_sha256':'bad'},
                       {'automatic_retry':True},{'reason':''}):
            bad=copy.deepcopy(ready);bad['startup_fault'].update(change)
            with self.assertRaises(ValueError):
                app.validate_ready(bad,123,'/dev/shm/xgoduck-native-test',manifest,self.expected)
        for change in ({'phase':'ready'},{'hardware':{'connected':True}},{'owns_connection':True},
                       {'startup_fault':None}):
            bad=copy.deepcopy(ready);bad['motion'].update(change)
            with self.assertRaises(ValueError):
                app.validate_ready(bad,123,'/dev/shm/xgoduck-native-test',manifest,self.expected)
        with self.assertRaises(ValueError):
            app.validate_ready(ready,123,'/dev/shm/xgoduck-native-test',self.deployment,self.expected)

    def test_parent_has_actual_socket_and_ram_node_checks_only_own_child_termination(self):
        compile(self.source, '<pure-native-source>', 'exec')
        parent = app.REMOTE_PARENT
        self.assertIn("link!='socket:['+str(ready['listener_inode'])+']'", parent)
        self.assertIn("ready['ram_directory_node']!=pin['ram_directory_node']", parent)
        self.assertIn('start_new_session=True', parent)
        self.assertIn('child.terminate()', parent)
        self.assertNotIn('child.kill(', parent)
        self.assertNotIn('prctl', parent)
        self.assertIn('child_closure_unknown=child is not None', parent)
        self.assertIn('parent_serial_open_attempts=0', parent)
        self.assertIn('closed=motion.get(\'closed_proof\')', app.CHILD_BOOTSTRAP)
        self.assertIn('for evidence in (hardware,closed)', app.CHILD_BOOTSTRAP)

    def test_source_rejects_different_transport_generation_and_node(self):
        for change in ({'generation': self.expected['generation'] + 1}, {'serial_node_review': {}},
                       {'serial_path_review': '/dev/ttyUSB9'}, {'automatic_reconnect': True}):
            with self.assertRaises(ValueError):
                app.remote_source(self.expected, dict(self.deployment, **change), reviewed_profile_path=PROFILE,
                    reviewed_profile_sha256=feedback_stream.WIFI_RECONNECT_PROFILE_SHA256, transport_config=self.transport)

    def test_audit_denies_persistent_writes_block_devices_aliases_and_other_processes(self):
        tree = ast.parse(app.CHILD_BOOTSTRAP)
        audit = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'audit')
        namespace = dict(os=os, stat=stat, root=Path('/dev/shm/xgoduck-native-test'),
                         uart='/dev/ttyUSB0', uart_identity=(5, 100, os.makedev(188, 0)),
                         immutable_files={'/dev/shm/xgoduck-native-test/policy/models/xgoduck_walk.onnx'},
                         calibration_storage=storage, native_transports=native_transports,
                         native_process=native_process)
        exec(compile(ast.Module(body=[audit], type_ignores=[]), '<pure-audit>', 'exec'), namespace)
        check = namespace['audit']
        check('open', ('/dev/shm/xgoduck-native-test/native-evidence/test.json', 'w', os.O_WRONLY))
        node = types.SimpleNamespace(st_mode=stat.S_IFCHR | 0o600, st_dev=5, st_ino=100,
                                     st_rdev=os.makedev(188, 0))
        with mock.patch.object(os, 'stat', return_value=node):
            check('open', ('/dev/ttyUSB0', None, os.O_RDWR))
        check('subprocess.Popen', ('dmesg', ['dmesg', '--color=never']))
        frontend = [sys.executable, '-I', '-B', str(namespace['root']/'service/native_process.py'),
                    '--frontend', '3', '4', '5', '6']
        check('subprocess.Popen', (sys.executable, frontend, None, None))
        with self.assertRaises(PermissionError):
            check('subprocess.Popen', (sys.executable, frontend + ['extra'], None, None))
        for args in (('/etc/config', 'w', os.O_WRONLY), ('/dev/mmcblk1', 'rb', os.O_RDONLY),
                     ('/dev/ttyUSB1', 'rb', os.O_RDONLY)):
            with self.assertRaises(PermissionError): check('open', args)
        with mock.patch.object(os.path, 'realpath', return_value='/dev/ttyUSB1'):
            with self.assertRaises(PermissionError): check('open', ('/tmp/serial-alias', 'rb', os.O_RDONLY))
        with self.assertRaises(PermissionError): check('subprocess.Popen', ('python3', ['python3']))
        with self.assertRaises(PermissionError): check('os.system', ('true',))
        command = native_transports.child_command(b'pass\n')
        check('subprocess.Popen', (sys.executable, command, None, None))
        for changed in (command[:-1]+[str(os.getpid()+1)], command[:-3]+['9999999']+command[-2:]):
            with self.assertRaises(PermissionError): check('subprocess.Popen',(sys.executable,changed,None,None))
        with self.assertRaises(PermissionError): check('subprocess.Popen',(sys.executable,command,None,{'PATH':'/tmp'}))
        with self.assertRaises(PermissionError): check('open',('/dev/shm/xgoduck-native-test/policy/models/xgoduck_walk.onnx','w',os.O_WRONLY))
        with self.assertRaises(PermissionError): check('os.rename',('/tmp/calibration.json','/etc/config',-1,-1))
        with self.assertRaises(PermissionError): check('os.mkdir',('/var/lib/unrelated',0o700,-1))

    def test_actual_private_store_commit_passes_audit_but_other_mutations_fail(self):
        audit = next(node for node in ast.parse(app.CHILD_BOOTSTRAP).body
                     if isinstance(node,ast.FunctionDef) and node.name=='audit')
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary).resolve()
            ram = base/'ram';ram.mkdir()
            subject = storage.NativeCalibrationStore(base/'private',self.config,self.zeros,
                        source_sha256=self.deployment['calibration_sha256'])
            namespace = dict(os=os,stat=stat,root=ram,uart=None,uart_identity=None,
                            immutable_files=set(),calibration_storage=storage,native_transports=native_transports,
                            native_process=native_process)
            exec(compile(ast.Module(body=[audit],type_ignores=[]),'<audit-store-commit>','exec'),namespace)
            active={'value':True}
            def bounded_hook(event,args):
                if active['value']:
                    namespace['audit'](event,args)
            # Audit hooks cannot be removed; this narrowly scoped hook is inert
            # after the test, including for unittest/temporary-directory cleanup.
            sys.addaudithook(bounded_hook)
            try:
                changed=copy.deepcopy(self.config);changed['imu']['bias_dps'][0]+=.01
                subject.commit(changed,self.zeros)
                self.assertEqual(subject.load()['calibration'],changed)
                with self.assertRaises(PermissionError):
                    (subject.directory/storage.STATE_NAME).write_text('{}')
                with self.assertRaises(PermissionError):
                    (base/'unrelated').mkdir()
            finally:
                active['value']=False

    def test_explicit_writer_orders_preflight_commit_reload_and_blocks_partial_failure(self):
        writer = next(node for node in ast.parse(app.CHILD_BOOTSTRAP).body
                      if isinstance(node,ast.FunctionDef) and node.name=='calibration_writer')
        for failure in (False,True):
            with self.subTest(reload_failure=failure),tempfile.TemporaryDirectory() as temporary:
                root=Path(temporary).resolve();(root/'service').mkdir();(root/'policy').mkdir()
                calibration_path=root/'policy/calibration.json'
                calibration_path.write_bytes(motion_workbench.calibration_bytes(self.config));calibration_path.chmod(0o600)
                subject=storage.NativeCalibrationStore(root/'persistent',self.config,self.zeros,
                        source_sha256=self.deployment['calibration_sha256'])
                authority=storage.RAMCalibrationAuthority(root,self.deployment['calibration_sha256'])
                operations=[]
                class FakeRuntime:
                    def prepare_calibration_reload(_self,config):operations.append('preflight')
                    def reload_calibration(_self,config,provenance):
                        operations.append('reload')
                        self.assertTrue(subject.load()['persistent'])
                        if failure:raise ValueError('CPU injected RAM refusal')
                        return authority.publish(storage.calibration_bytes(config))
                commit=subject.commit
                def marked_commit(config,provenance):
                    operations.append('commit');return commit(config,provenance)
                namespace=dict(runtime=FakeRuntime(),store=subject,loaded_calibration=subject.load(),
                               calibration_reload_fault=None)
                exec(compile(ast.Module(body=[writer],type_ignores=[]),'<explicit-native-writer>','exec'),namespace)
                changed=copy.deepcopy(self.config);changed['imu']['bias_dps'][0]+=.01
                with mock.patch.object(subject,'commit',side_effect=marked_commit):
                    warning=namespace['calibration_writer'](changed,self.zeros)
                self.assertEqual(operations,['preflight','commit','reload'])
                self.assertEqual(subject.load()['calibration'],changed)
                self.assertEqual(namespace['loaded_calibration']['calibration'],changed)
                if failure:
                    self.assertIn('已持久保存',warning);self.assertIn('运动已锁定',warning)
                    self.assertEqual(warning,namespace['calibration_reload_fault'])
                    self.assertEqual(authority.verify(),self.deployment['calibration_sha256'])
                else:
                    self.assertIsNone(warning)
                    self.assertEqual(authority.verify(),storage.digest(storage.calibration_bytes(changed)))


class NativeConnectionFactoryTests(unittest.TestCase):
    @contextlib.contextmanager
    def factory_fixture(self):
        """Actual bootstrap closure and reviewed serial cache; fake sysfs only."""
        tree = ast.parse(feedback_stream.runtime_source())
        names = ('bind_serial', 'serial_identity', 'serial_path', 'serial_proof')
        nodes = [next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                      and node.name == name) for name in names]
        source = '\n'.join(ast.unparse(node) for node in nodes)
        fixture = types.ModuleType('_native_factory_cpu_fixture')
        model = types.SimpleNamespace(inode=100, path='/dev/ttyUSB0', holders=[],
            health_error=None, health_calls=0, isolated_scans=[], mounts='config-row 1 179:1 / /boot/config rw\n'
                'shm-row 1 0:1 / /dev/shm rw\n')
        class InventoryPath:
            def __init__(self, value): self.value = value
            def glob(self, pattern):
                assert self.value == '/sys/class/tty' and pattern == 'ttyUSB*'
                return [types.SimpleNamespace(name=Path(model.path).name)]
        def identity(path):
            if path != model.path: raise FileNotFoundError(path)
            node = types.SimpleNamespace(st_mode=stat.S_IFCHR | 0o600, st_dev=5,
                st_ino=model.inode, st_rdev=os.makedev(188, int(Path(path).name[6:])))
            actual = dict(path=path, vid='1a86', pid='7523', serial_number='', sysfs_name='1-1.3')
            return node, actual
        def validate_health(expected):
            model.health_calls += 1
            if model.health_error: raise ValueError(model.health_error)
            if expected != fixture.fixed_expected: raise ValueError('Reviewed expectation mutated')
        fixture.Path, fixture._usb_serial_identity = InventoryPath, identity
        fixture._holders = lambda node, deadline: copy.deepcopy(model.holders)
        fixture.bounded_read = lambda path, limit: model.mounts
        fixture.validate_health, fixture.ProbeError = validate_health, ValueError
        expected = dict(generation=123, discover_serial=True,
            serial=dict(vid='1a86', pid='7523', serial_number='', path='/dev/ttyUSB0', sysfs_name='1-1.3'))
        fixture.fixed_expected = copy.deepcopy(expected)
        guard_source = ('import json,os,stat\nimport _native_factory_cpu_fixture as fixture\n'
            'from _native_factory_cpu_fixture import Path,_usb_serial_identity,_holders,bounded_read,ProbeError\n'
            '_SERIAL_BINDINGS={}\n' + source + '\n'
            'def health_snapshot(expected):\n'
            '    fixture.validate_health(expected)\n'
            '    _,identity=serial_proof(expected)\n'
            '    return {"serial":identity}\n'
            'def bounded_dmesg(): return "stable raw kernel"\n')
        bootstrap = ast.parse(app.CHILD_BOOTSTRAP)
        functions = [next(node for node in bootstrap.body if isinstance(node, ast.FunctionDef)
                          and node.name == name) for name in ('audit', 'connection_guard_factory')]
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary).resolve(); (root / 'policy').mkdir()
            raw = b'{"saved_calibration":"unchanged"}\n'
            (root / 'policy/calibration.json').write_bytes(raw)
            info = root.stat()
            namespace = dict(os=os, stat=stat, time=time, json=json, hashlib=hashlib,
                root=root, expected=expected, uart=None, uart_identity=None,
                calibration_reload_fault=None, guard_source=guard_source, immutable_files=set(),
                calibration_storage=storage,native_transports=native_transports,
                initial_connection_gate_pending=True,
                last_completed_ram_health=None,
                pin=dict(serial_node_posix=dict(dev=5, ino=100, rdev=os.makedev(188, 0)),
                    shm_mount=[model.mounts.splitlines()[1]], ram_directory_node=[info.st_dev, info.st_ino]),
                deployment=dict(serial_path_review='/dev/ttyUSB0',
                    config_mount_review=[model.mounts.splitlines()[0]],
                    calibration_sha256=hashlib.sha256(raw).hexdigest()),
                guard_code=compile(guard_source, '<cpu-reviewed-guards>', 'exec'))
            def isolated_holders(source,rdev,deadline,directory):
                self.assertEqual(source,guard_source)
                self.assertEqual(directory,root/'holder-evidence')
                self.assertEqual(rdev,os.makedev(188,int(Path(model.path).name[6:])))
                model.isolated_scans.append((rdev,deadline))
                return fixture._holders(rdev,deadline)
            namespace['native_transports']=types.SimpleNamespace(isolated_holders=isolated_holders,
                prepare_usb_recovery=lambda *args:None,
                pinned_uart_checker=native_transports.pinned_uart_checker,
                allowed_child_command=native_transports.allowed_child_command)
            namespace['authority']=types.SimpleNamespace(
                verify=lambda:hashlib.sha256((root/'policy/calibration.json').read_bytes()).hexdigest(),
                child_spec=lambda:dict(calibration_path=str(root/'policy/calibration.json'),
                    calibration_sha256=namespace['deployment']['calibration_sha256']))
            namespace['publish_calibration']=lambda raw: hashlib.sha256(raw).hexdigest()
            exec(compile(ast.Module(body=functions, type_ignores=[]), '<actual-bootstrap-factory>', 'exec'), namespace)
            with mock.patch.dict(sys.modules, {'_native_factory_cpu_fixture': fixture}):
                yield namespace, model, identity

    def test_explicit_usb_gate_rebinds_stable_node_and_publishes_metadata_only_receipt(self):
        with self.factory_fixture() as (namespace,model,_):
            proof={'kernel_prefix':'stable raw kernel'}
            namespace['native_transports'].prepare_usb_recovery=lambda *args:proof
            namespace['native_transports'].install_usb_recovery=lambda ns,source,value:source
            model.inode=101
            gate=namespace['connection_guard_factory'](usb_recovery=True)
            self.assertIs(namespace['native_usb_recovery'],proof)
            self.assertFalse(namespace['initial_connection_gate_pending'])
            self.assertGreaterEqual(model.health_calls,3)
            rows=list((namespace['root']/'usb-recovery-evidence').glob('*.json'))
            self.assertEqual(len(rows),1)
            receipt=json.loads(rows[0].read_text())
            self.assertEqual(receipt['observation_count'],3)
            self.assertEqual(receipt['serial_open_attempts'],0)
            self.assertFalse(receipt['motion_resumed'])
            self.assertEqual(receipt['serial_node'][1],101)
            self.assertEqual(gate['EXPECTED'],namespace['expected'])

    def test_initial_quiescence_pin_and_later_connection_pins_have_independent_closures(self):
        with self.factory_fixture() as (namespace, model, _):
            factory = namespace['connection_guard_factory']
            old = factory(initial=True)
            self.assertTrue(namespace['initial_connection_gate_pending'])
            factory()
            self.assertFalse(namespace['initial_connection_gate_pending'])
            old['ram_health']()
            model.inode = 200
            with self.assertRaisesRegex(ValueError, 'serial device changed'): old['ram_health']()
            fresh = factory()
            self.assertIsNot(old['_SERIAL_BINDINGS'], fresh['_SERIAL_BINDINGS'])
            self.assertIsNot(old['bind_serial'].__globals__, fresh['bind_serial'].__globals__)
            self.assertIsNot(old['EXPECTED'], fresh['EXPECTED'])
            self.assertEqual(old['EXPECTED'], fresh['EXPECTED'])
            fresh['ram_health']()
            self.assertEqual(namespace['uart_identity'], (5, 200, os.makedev(188, 0)))
            model.path, model.inode = '/dev/ttyUSB1', 300
            newest = factory()
            newest['ram_health']()
            self.assertEqual(namespace['uart'], '/dev/ttyUSB1')
            with self.assertRaises((ValueError, FileNotFoundError)): fresh['ram_health']()
            with self.assertRaisesRegex(ValueError, 'initial UART'): factory(initial=True)

    def test_failed_kernel_or_holder_gate_keeps_previous_audit_allowance(self):
        with self.factory_fixture() as (namespace, model, _):
            factory = namespace['connection_guard_factory']; factory(initial=True); factory()
            old = namespace['uart_identity']
            model.inode = 200; model.health_error = 'New unreviewed kernel fault'
            with self.assertRaisesRegex(ValueError, 'unreviewed kernel fault'): factory()
            self.assertEqual(namespace['uart_identity'], old)
            model.health_error = None; model.holders = [{'pid': 99}]
            with self.assertRaisesRegex(ValueError, 'already has an owner'): factory()
            self.assertEqual(namespace['uart_identity'], old)
            model.holders = []; self.assertEqual(factory()['EXPECTED'], namespace['expected'])

    def test_config_ram_and_calibration_gates_are_checked_before_audit_commit(self):
        for failure in ('CONFIG', 'RAM', 'calibration', 'boot'):
            with self.subTest(failure=failure), self.factory_fixture() as (namespace, model, _):
                factory = namespace['connection_guard_factory']; factory(initial=True); factory()
                old = namespace['uart_identity']; model.inode = 200
                if failure == 'CONFIG': model.mounts = model.mounts.replace('config-row', 'changed-config-row')
                elif failure == 'RAM': model.mounts = model.mounts.replace('shm-row', 'changed-shm-row')
                elif failure == 'calibration': (namespace['root'] / 'policy/calibration.json').write_bytes(b'{}')
                else: model.health_error = 'Reviewed boot/root/card/model identity changed'
                with self.assertRaises(ValueError): factory()
                self.assertEqual(namespace['uart_identity'], old)

    def test_audit_accepts_only_current_explicit_path_and_fixed_inode(self):
        with self.factory_fixture() as (namespace, model, identity):
            factory = namespace['connection_guard_factory']; factory(initial=True); factory()
            check = namespace['audit']
            real_stat = os.stat
            def fake_stat(path, *args, **kwargs):
                if str(path).startswith('/dev/ttyUSB'): return identity(str(path))[0]
                return real_stat(path, *args, **kwargs)
            with mock.patch.object(os, 'stat', side_effect=fake_stat):
                check('open', ('/dev/ttyUSB0', None, os.O_RDWR))
                model.inode = 200
                with self.assertRaisesRegex(PermissionError, 'identity changed'):
                    check('open', ('/dev/ttyUSB0', None, os.O_RDWR))
                model.path = '/dev/ttyUSB1'
                with self.assertRaisesRegex(PermissionError, 'Unreviewed'):
                    check('open', ('/dev/ttyUSB1', None, os.O_RDWR))
                factory()
                check('open', ('/dev/ttyUSB1', None, os.O_RDWR))
                with self.assertRaises(PermissionError): check('open', ('/dev/ttyUSB0', None, os.O_RDWR))
                with mock.patch.object(os.path, 'realpath', return_value='/dev/ttyUSB1'):
                    with self.assertRaises(PermissionError): check('open', ('/tmp/alias', 'rb', os.O_RDONLY))

    def test_model_loading_window_cannot_consume_or_replace_first_runtime_pin(self):
        with self.factory_fixture() as (namespace, model, _):
            factory = namespace['connection_guard_factory']; factory(initial=True)
            for path, inode in (('/dev/ttyUSB0', 200), ('/dev/ttyUSB1', 100)):
                model.path, model.inode = path, inode
                with self.assertRaisesRegex(ValueError, 'initial UART'): factory()
                self.assertTrue(namespace['initial_connection_gate_pending'])
                self.assertEqual(namespace['uart'], '/dev/ttyUSB0')
                self.assertEqual(namespace['uart_identity'], (5, 100, os.makedev(188, 0)))
            model.path, model.inode = '/dev/ttyUSB0', 100
            model.health_error = 'New unreviewed kernel fault'
            with self.assertRaisesRegex(ValueError, 'unreviewed kernel fault'): factory()
            self.assertTrue(namespace['initial_connection_gate_pending'])
            model.health_error = None; factory()
            self.assertFalse(namespace['initial_connection_gate_pending'])
            model.inode = 200; factory()
            self.assertEqual(namespace['uart_identity'], (5, 200, os.makedev(188, 0)))

    def test_parent_isolated_scan_preserves_exact_rdev_deadline_and_guard_source(self):
        with self.factory_fixture() as (namespace,model,_):
            original_source=namespace['guard_source']
            with mock.patch.object(time,'monotonic',return_value=100.):
                guard=namespace['connection_guard_factory'](initial=True)
            self.assertEqual(model.isolated_scans,[(os.makedev(188,0),100.2)]*2)
            guard['_holders'](os.makedev(188,0),123.456)
            self.assertEqual(model.isolated_scans[-1],(os.makedev(188,0),123.456))
            self.assertEqual(namespace['guard_source'],original_source)
            self.assertEqual(guard['NATIVE_RUNTIME_SOURCE'],original_source)
            # Only the parent's function slot changes. Its pinned source is the
            # original reviewed implementation used by reader/metadata children.
            self.assertNotIn('isolated_holders',guard['NATIVE_RUNTIME_SOURCE'])

    def test_retained_live_health_allows_own_reader_but_new_factory_still_checks_owners(self):
        with self.factory_fixture() as (namespace,model,_):
            self.assertIsNone(namespace['last_completed_ram_health'])
            guard=namespace['connection_guard_factory'](initial=True)
            completed=namespace['last_completed_ram_health']
            self.assertIs(completed,guard['ram_health'])
            scans=len(model.isolated_scans)
            model.holders=[{'pid':123,'fd':'4'}]
            completed()
            self.assertEqual(len(model.isolated_scans),scans)
            with self.assertRaisesRegex(ValueError,'already has an owner'):
                namespace['connection_guard_factory']()
            self.assertIs(namespace['last_completed_ram_health'],completed)
            self.assertEqual(len(model.isolated_scans),scans+1)
            model.health_error='Unreviewed kernel fault after completed gate'
            with self.assertRaisesRegex(ValueError,'Unreviewed kernel fault'):
                completed()


class NativeAddressProbeTests(unittest.TestCase):
    def test_probe_reuses_time_wait_but_refuses_live_listener_and_closes(self):
        tree=ast.parse(app.REMOTE_PARENT)
        node=next(part for part in tree.body if isinstance(part,ast.FunctionDef)
                  and part.name=='probe_native_address')
        for state in ('time_wait','active_listener'):
            with self.subTest(state=state):
                operations=[]
                class FakeSocket:
                    def setsockopt(self,level,option,value):
                        operations.append(('option',level,option,value))
                    def bind(self,address):
                        operations.append(('bind',address))
                        if state=='active_listener':
                            raise OSError(errno.EADDRINUSE,'Address already in use')
                        if operations[0]!=('option',socket.SOL_SOCKET,socket.SO_REUSEADDR,1):
                            raise OSError(errno.EADDRINUSE,'TIME_WAIT requires address reuse')
                    def close(self):operations.append(('close',))
                namespace={'socket':types.SimpleNamespace(AF_INET=socket.AF_INET,SOCK_STREAM=socket.SOCK_STREAM,
                    SOL_SOCKET=socket.SOL_SOCKET,SO_REUSEADDR=socket.SO_REUSEADDR,
                    socket=mock.Mock(return_value=FakeSocket()))}
                exec(compile(ast.Module(body=[node],type_ignores=[]),'<pure-native-address-probe>','exec'),namespace)
                if state=='active_listener':
                    with self.assertRaises(OSError) as raised:
                        namespace['probe_native_address']('192.168.1.11',8767)
                    self.assertEqual(raised.exception.errno,errno.EADDRINUSE)
                else:
                    namespace['probe_native_address']('192.168.1.11',8767)
                namespace['socket'].socket.assert_called_once_with(socket.AF_INET,socket.SOCK_STREAM)
                self.assertEqual(operations,[('option',socket.SOL_SOCKET,socket.SO_REUSEADDR,1),
                                              ('bind',('192.168.1.11',8767)),('close',)])
                self.assertNotIn('SO_REUSEPORT',ast.unparse(node))


class NativeOfflineMetadataTests(unittest.TestCase):
    def fixture(self):
        tree=ast.parse(app.REMOTE_PARENT)
        nodes=[next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name==name)
               for name in ('offline_parent_metadata','parent_health')]
        mounts=('root 1 179:2 / / rw - ext4 /dev/root rw\n'
                'config 1 179:1 / /boot/config rw - vfat /dev/mmcblk1p1 rw\n'
                'shm 1 0:1 / /dev/shm rw - tmpfs tmpfs rw\n')
        data={'/proc/self/mountinfo':mounts,'/proc/sys/kernel/random/boot_id':'boot',
              '/sys/block/mmcblk1/device/cid':'cid','/proc/device-tree/model':b'X5\0',
              '/proc/swaps':'Filename Type Size Used Priority\n','/sys/block/mmcblk1/device/type':'SD'}
        expected=dict(boot_id='boot',cid='cid',root_mount_line=mounts.splitlines()[0],model='X5')
        def validate_os(metadata,review):
            for name in expected:
                if metadata[name]!=review[name]:raise ValueError('Fixed board identity differs')
        forbidden=mock.Mock(side_effect=AssertionError('No strict hardware grant in offline metadata'))
        namespace=dict(json=json,hashlib=hashlib,stat=stat,time=time,ProbeError=ValueError,
            EXPECTED=expected,DEPLOY=dict(startup_mode='metadata_only_fault_page',
                config_mount_review=[mounts.splitlines()[1]]),startup_fault=None,
            bounded_read=lambda path,limit,binary=False:data[path],
            bounded_dmesg=lambda:'936 USB disconnect device 4',validate_os=validate_os,
            validate_kernel=mock.Mock(side_effect=ValueError('Unreviewed USB disconnect device 4')),
            bind_serial=mock.Mock(side_effect=ValueError('ESP32 absent')),
            serial_proof=forbidden,_holders=forbidden,health_snapshot=forbidden,
            os=types.SimpleNamespace(stat=lambda *a,**k:types.SimpleNamespace(st_mode=stat.S_IFBLK),
                major=lambda value:179,minor=lambda value:2))
        namespace['os'].stat=lambda *a,**k:types.SimpleNamespace(st_mode=stat.S_IFBLK,st_rdev=123)
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'<offline-parent-metadata>','exec'),namespace)
        return namespace,data,forbidden

    def test_actual_fault_metadata_retains_kernel_and_missing_uart_without_health_grant(self):
        namespace,data,forbidden=self.fixture()
        node,shm=namespace['parent_health']()
        self.assertIsNone(node)
        self.assertEqual(shm,[data['/proc/self/mountinfo'].splitlines()[2]])
        fault=namespace['startup_fault']
        self.assertIn('Unreviewed USB disconnect',fault['reason'])
        self.assertEqual(fault['kernel_log'],'936 USB disconnect device 4')
        self.assertEqual(fault['kernel_sha256'],hashlib.sha256(fault['kernel_log'].encode()).hexdigest())
        self.assertIsNone(fault['serial_metadata'])
        self.assertEqual(fault['serial_error'],'ESP32 absent')
        self.assertFalse(fault['hardware_gate_passed']);self.assertFalse(fault['uart_granted'])
        self.assertEqual(fault['serial_open_attempts'],0)
        forbidden.assert_not_called()

    def test_offline_metadata_still_rejects_fixed_identity_and_config_changes(self):
        for change in ('boot','CONFIG','SHM'):
            namespace,data,forbidden=self.fixture()
            if change=='boot':data['/proc/sys/kernel/random/boot_id']='different-boot'
            elif change=='CONFIG':data['/proc/self/mountinfo']=data['/proc/self/mountinfo'].replace('config 1','changed 1')
            else:data['/proc/self/mountinfo']=data['/proc/self/mountinfo'].replace('rw - tmpfs','ro - tmpfs')
            with self.assertRaises(ValueError):namespace['parent_health']()
            forbidden.assert_not_called()

    def test_offline_child_initialization_never_invokes_hardware_factory(self):
        tree=ast.parse(app.CHILD_BOOTSTRAP)
        branch=next(node for node in tree.body if isinstance(node,ast.If)
                    and isinstance(node.test,ast.Name) and node.test.id=='offline_startup')
        forbidden=mock.Mock(side_effect=AssertionError('Startup hardware gate forbidden'))
        fault=dict(hardware_gate_passed=False,serial_open_attempts=0,reason='Actual USB disconnect',serial_metadata=None)
        namespace=dict(offline_startup=True,startup_fault=fault,deployment={'startup_mode':'metadata_only_fault_page'},
            guard_code=compile('class ProbeError(RuntimeError): pass\ndef serial_proof(expected): raise RuntimeError("strict guard")','<guard>','exec'),
            expected={'generation':123},json=json,connection_guard_factory=forbidden,
            publish_calibration=lambda raw:None,guard_source='original immutable guard',last_completed_ram_health=None)
        exec(compile(ast.Module(body=[branch],type_ignores=[]),'<offline-native-init>','exec'),namespace)
        forbidden.assert_not_called()
        self.assertEqual(namespace['namespace']['EXPECTED'],{'generation':123})
        self.assertEqual(namespace['namespace']['NATIVE_RUNTIME_SOURCE'],'original immutable guard')
        with self.assertRaisesRegex(RuntimeError,'Actual USB disconnect'):
            namespace['namespace']['ram_health']()
        forbidden.assert_not_called()
        completed=mock.Mock(return_value={'boot_id':'actual live checked boot'})
        namespace['last_completed_ram_health']=completed
        self.assertEqual(namespace['namespace']['ram_health'](),{'boot_id':'actual live checked boot'})
        completed.assert_called_once_with()
        forbidden.assert_not_called()


if __name__ == '__main__':
    unittest.main()
