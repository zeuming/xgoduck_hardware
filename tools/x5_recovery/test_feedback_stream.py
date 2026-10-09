"""Pure reader checks: all hardware, processes and emitted events are faked."""
import copy
import base64
import ast
import hashlib
import json
import os
from pathlib import Path
import struct
import tempfile
import types
import unittest
from unittest import mock

import feedback_stream


def runtime():
    namespace = {}
    exec(compile(feedback_stream.runtime_source(), "<reader-test-definitions>", "exec"), namespace)
    return namespace


def expected_fixture(namespace):
    lines = (["[    0.000000] Booting Linux"]
             + [item["line"] for item in feedback_stream.REVIEWED_HISTORICAL_FAULTS]
             + ["[ 3009.000000] usb 1-1.3: new full-speed USB device number 4 using xhci-hcd"])
    return {"boot_id": "dcf6255f-2afb-4262-8c96-4a00bc0d8cab",
            "cid": "035344534b33324785d55cb687017c00",
            "root_mount_line": "21 1 179:2 / / rw,relatime shared:1 - ext4 /dev/root rw",
            "model": "D-Robotics RDK X5 V1.0",
            "serial": {"path": "/dev/ttyUSB0", "vid": "1a86", "pid": "7523",
                       "sysfs_name": "1-1.3", "serial_number": ""},
            "kernel_prefix": "\n".join(lines)+"\n",
            "historical_faults": [{"line": line, "reason": reason} for line in lines
                                  if (reason := namespace["classify_log"](line))],
            "generation": 1837609}


def payload(namespace, *, sample=1000000, imu=1001000, online=0,
            fault=0, torque=0, armed=0, cookie=0, accel=None,
            imu_ok=1, last_error=0, configured=0):
    values = [sample, imu, online, fault, torque, armed, imu_ok]
    values += [2000]*15 + [0]*15 + (accel or [0.,0.,9.8]) + [0.,0.,0.]
    values += [cookie,last_error,configured]
    return namespace["STATE_PACKET"].pack(*values)


def state(namespace, **values):
    return namespace["_decode_state"](payload(namespace, **values))


class ReaderTests(unittest.TestCase):
    def setUp(self):
        self.ns = runtime()
        self.expected = expected_fixture(self.ns)
        self.Error = self.ns["ProbeError"]

    def test_voltage_firmware_return_is_exact_and_keeps_future_fault_guards(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-servo-voltage-usb-return-reviewed-20261006-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.SERVO_VOLTAGE_RETURN_PROFILE_SHA256)
        review = profile['review']['servo_voltage_firmware_usb_return_review']
        self.assertTrue(profile['user_confirmed_reconnect'])
        self.assertFalse(review['battery_connected'])
        self.assertTrue(review['prior_historical_faults_preserved'])
        self.assertFalse(review['generic_prefix_loss_exemption'])
        self.assertEqual(profile['historical_faults'][:-1], feedback_stream.NATIVE_USB_THIRD_RETURN_HISTORICAL_FAULTS)
        self.assertEqual(review['arm_requests'], 0)
        self.assertEqual(review['target_requests'], 0)
        expected = self.profile_expected(profile)
        expected['reviewed_reconnect_proof'] = dict(feedback_stream.SERVO_VOLTAGE_RETURN_PROOF)
        result = self.ns['validate_kernel'](profile['kernel_prefix'], expected)
        self.assertFalse(result['historical_faults_cleared'])
        for kernel in (profile['kernel_prefix'].split('\n', 1)[1],
                       profile['kernel_prefix'] + '[12000.000000] usb 1-1.4: USB disconnect, device number 12\n',
                       profile['kernel_prefix'] + '[12000.000001] mmc1: error -5\n'):
            with self.subTest(kernel=kernel[-100:]), self.assertRaises(self.Error):
                self.ns['validate_kernel'](kernel, expected)

    def test_voltage_return_cannot_change_review_or_grant_motion(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-servo-voltage-usb-return-reviewed-20261006-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.SERVO_VOLTAGE_RETURN_PROFILE_SHA256)
        for key, value in (('servo_motion_authorized', True), ('user_confirmed_reconnect', False),
                           ('historical_faults', profile['historical_faults'][:-1])):
            with self.subTest(key=key), self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile, key: value})
        changed = copy.deepcopy(profile)
        changed['review']['servo_voltage_firmware_usb_return_review']['battery_connected'] = True
        with self.assertRaises(ValueError):
            feedback_stream._validate_reviewed_profile(changed)

    def reviewed_profile(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-reconnect-reviewed-20261004-01.json'
        return path, feedback_stream.load_reviewed_profile(path, feedback_stream.RECONNECT_PROFILE_SHA256)

    def test_hello_timeout_keeps_phase_and_never_retries_or_accepts_other_firmware(self):
        for reply, error, reason in ((None, 'Response deadline expired', 'ESP32 handshake response timed out'),
                                    (None, 'Response CRC mismatch', 'Response CRC mismatch'),
                                    (b'other-firmware', None, 'Unknown motion firmware identity'),
                                    (b'XGODUCK_MOTION_1', None, None)):
            exchange = mock.Mock(return_value=reply, side_effect=self.Error(error) if error else None)
            metadata = {}
            with self.subTest(reason=reason), mock.patch.dict(self.ns, _exchange=exchange):
                if reason:
                    with self.assertRaisesRegex(self.Error, reason):
                        self.ns['_hello_feedback'](71, metadata)
                else:
                    self.ns['_hello_feedback'](71, metadata)
            self.assertEqual(exchange.call_count, 1)
            self.assertEqual(exchange.call_args.args[:3], (71, 1, 1))
            self.assertIs(exchange.call_args.args[-1], metadata)

    def profile_expected(self, profile):
        return {**{key: copy.deepcopy(profile[key]) for key in (
            'boot_id','cid','root_mount_line','model','serial','kernel_prefix','historical_faults')},
            'generation': self.expected['generation']}

    def test_explicit_reconnect_profile_source_and_future_faults(self):
        path, profile = self.reviewed_profile()
        expected = self.profile_expected(profile)
        with self.assertRaises(ValueError): feedback_stream.remote_source(expected)
        source = feedback_stream.remote_source(expected, reviewed_profile_path=path,
                                               reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)
        compile(source, '<reviewed-reader>', 'exec')
        self.assertNotIn(profile['review']['human_context'], source)
        self.assertEqual(source.count(repr(profile['kernel_prefix'])), 1)
        expected['reviewed_reconnect_proof'] = dict(feedback_stream.RECONNECT_PROOF)
        validate = self.ns['validate_kernel']
        proof = validate(profile['kernel_prefix'], expected)
        self.assertEqual(len(proof['historical_faults']), 7)
        self.assertFalse(proof['historical_faults_cleared'])
        for tail in ('[ 9000.000000] mmc1: error -5\n',
                     '[ 9000.000001] usb 1-1.3: USB disconnect, device number 5\n',
                     '[ 9000.000002] txrx_err:<aicwf_sdio_wakeup,1312>: sdio wakeup fail\n',
                     profile['historical_faults'][-1]['line']+'\n'):
            with self.subTest(tail=tail), self.assertRaises(self.Error):
                validate(profile['kernel_prefix']+tail, expected)
        with self.assertRaises(self.Error):
            validate(profile['kernel_prefix'].split('\n',1)[1], expected)
        changed = copy.deepcopy(expected)
        changed['reviewed_reconnect_proof']['profile_sha256'] = '0'*64
        with self.assertRaises(self.Error): validate(profile['kernel_prefix'], changed)

    def test_reconnect_requires_both_arguments_and_exact_all_identity_baseline_fields(self):
        path, profile = self.reviewed_profile()
        expected = self.profile_expected(profile)
        for kwargs in ({'reviewed_profile_path':path},
                       {'reviewed_profile_sha256':feedback_stream.RECONNECT_PROFILE_SHA256}):
            with self.assertRaises(ValueError): feedback_stream.remote_source(expected, **kwargs)
        changes = {'root_mount_line':expected['root_mount_line'].replace('21 1','22 1'),
                   'model':'D-Robotics RDK X5 V1.1',
                   'serial':{**expected['serial'],'serial_number':'changed'},
                   'kernel_prefix':expected['kernel_prefix']+'[ 9000.0] innocuous extra line\n',
                   'historical_faults':expected['historical_faults'][:-1]}
        for key, value in changes.items():
            with self.subTest(key=key), self.assertRaises(ValueError):
                feedback_stream.remote_source({**expected,key:value}, reviewed_profile_path=path,
                    reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)
        default = feedback_stream.remote_source(self.expected)
        self.assertNotIn('reviewed_reconnect_proof\':',default)

    def test_profile_private_immutable_read_and_exact_bytes(self):
        path, _ = self.reviewed_profile()
        original = path.read_bytes()
        sha = feedback_stream.RECONNECT_PROFILE_SHA256
        with tempfile.TemporaryDirectory() as directory:
            private = Path(directory)/'profile.json'
            private.write_bytes(original)
            private.chmod(0o600)
            self.assertEqual(len(feedback_stream.load_reviewed_profile(private,sha)['historical_faults']),7)
            with self.assertRaises(ValueError): feedback_stream.load_reviewed_profile(private,'0'*64)
            with self.assertRaises(ValueError): feedback_stream.load_reviewed_profile('profile.json',sha)
            private.write_bytes(original+b' ')
            with self.assertRaisesRegex(ValueError,'reviewed hash'): feedback_stream.load_reviewed_profile(private,sha)
            private.write_bytes(original)
            private.chmod(0o644)
            with self.assertRaisesRegex(ValueError,'private'): feedback_stream.load_reviewed_profile(private,sha)
            private.chmod(0o600)
            link = Path(directory)/'symlink.json'
            link.symlink_to(private)
            with self.assertRaises(ValueError): feedback_stream.load_reviewed_profile(link,sha)
            hard = Path(directory)/'hard.json'
            os.link(private,hard)
            with self.assertRaisesRegex(ValueError,'singly linked'): feedback_stream.load_reviewed_profile(private,sha)
            hard.unlink()
            info = private.stat()
            changed = types.SimpleNamespace(**{key:getattr(info,key) for key in (
                'st_dev','st_ino','st_mode','st_uid','st_nlink','st_size','st_mtime_ns','st_ctime_ns')})
            changed.st_mtime_ns += 1
            with mock.patch.object(feedback_stream.os,'fstat',side_effect=[info,changed]), \
                    self.assertRaisesRegex(ValueError,'changed while'):
                feedback_stream.load_reviewed_profile(private,sha)
            private.write_bytes(b'x'*(feedback_stream.MAX_PROFILE_BYTES+1))
            with self.assertRaisesRegex(ValueError,'bounded'): feedback_stream.load_reviewed_profile(private,sha)

    def test_profile_schema_and_exact_seven_cannot_widen_scope(self):
        _, profile = self.reviewed_profile()
        changes = {'schema_version':True, 'kernel_line_count':963,
                   'kernel_prefix_sha256':'0'*64, 'allowed_requests':['HELLO','STATE','ARM'],
                   'servo_arm_authorized':True, 'storage_health_certified':True,
                   'kernel_prefix':profile['kernel_prefix'].replace('USB disconnect','USB disconnected',1),
                   'historical_faults':profile['historical_faults']+[profile['historical_faults'][-1]],
                   'review':'unbounded opaque instructions'}
        for key,value in changes.items():
            with self.subTest(key=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile,key:value})
        altered = copy.deepcopy(profile)
        altered['historical_faults'][6]['line'] = altered['historical_faults'][6]['line'].replace('8544.480306','8544.480307')
        with self.assertRaises(ValueError): feedback_stream._validate_reviewed_profile(altered)

    def test_feedback_only_aic_negative_result_timeout_and_transfer_failure_stop(self):
        actual='[ 9890.432382] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff'
        self.assertIsNone(self.ns['classify_log'](actual))
        lines=(actual,
               '[ 9900.1] aicsdio: read ret: -5',
               '[ 9900.2] aicwf_sdio_read: timeout waiting for response',
               '[ 9900.3] aic8800: timed out ETIMEDOUT',
               '[ 9900.4] aicwf_sdio_bus: write failed',
               '[ 9900.5] aicsdio: rc=-19')
        for line in lines:
            with self.subTest(line=line):
                self.assertEqual(self.ns['classify_feedback_log'](line),
                    'Feedback guard reported an AIC SDIO WiFi driver failure')
                with self.assertRaises(self.Error) as raised:
                    self.ns['validate_kernel'](self.expected['kernel_prefix']+line+'\n',self.expected)
                self.assertIn(line,raised.exception.metadata['new_kernel_fault_evidence']['faults'][0]['line'])
                changed={**self.expected,'kernel_prefix':self.expected['kernel_prefix']+line+'\n'}
                with self.assertRaisesRegex(self.Error,'baseline faults changed'):
                    self.ns['validate_kernel'](changed['kernel_prefix'],changed)
        path,profile=self.reviewed_profile()
        expected=self.profile_expected(profile)
        expected['reviewed_reconnect_proof']=dict(feedback_stream.RECONNECT_PROOF)
        with self.assertRaises(self.Error):self.ns['validate_kernel'](profile['kernel_prefix']+actual+'\n',expected)
        feedback_stream.rx_capture_source(self.profile_expected(profile),reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)

    def test_aic_feedback_supplement_requires_source_and_explicit_failure(self):
        ordinary=('[ 9.0] aicsdio: aicwf_sdio_reg_init',
                  '[ 9.1] aicsdio: write retry:  1',
                  '[ 9.2] aicbsp: sdio_err:<aicwf_sdio_bus_pwrctl,1402>: bus down',
                  '[ 9.3] aicwf_sdio_read: ret=0',
                  '[ 9.4] aicwf_sdio_read: ret=-0',
                  '[ 9.5] aicwf_sdio_read: ret=110',
                  '[ 9.6] unrelated driver: ret=-110, timeout',
                  '[ 9.7] application: transfer failed',
                  '[ 9.8] AICWFDBG(LOGERROR) invalid cmd: loss_enable')
        for line in ordinary:
            with self.subTest(line=line):self.assertIsNone(feedback_stream.additional_feedback_fault(line))
        historical=feedback_stream.RECONNECT_HISTORICAL_FAULTS[-1]
        self.assertEqual(self.ns['classify_feedback_log'](historical['line']),historical['reason'])

    def test_second_fixed_profile_preserves_eight_and_rejects_every_future_fault(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-reconnect-reviewed-20261004-02.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RECONNECT2_PROFILE_SHA256)
        self.assertEqual(len(profile['kernel_prefix'].splitlines()),963)
        self.assertEqual(profile['historical_faults'][:7],feedback_stream.RECONNECT_HISTORICAL_FAULTS)
        expected=self.profile_expected(profile)
        source=feedback_stream.remote_source(expected,reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.RECONNECT2_PROFILE_SHA256)
        self.assertIn(repr(feedback_stream.RECONNECT2_PROOF),source)
        compile(source,'<second-reviewed-reader>','exec')
        expected['reviewed_reconnect_proof']=dict(feedback_stream.RECONNECT2_PROOF)
        proof=self.ns['validate_kernel'](profile['kernel_prefix'],expected)
        self.assertEqual(proof['historical_faults'],feedback_stream.RECONNECT2_HISTORICAL_FAULTS)
        self.assertFalse(proof['historical_faults_cleared'])
        for tail in (profile['historical_faults'][-1]['line']+'\n',
                     '[ 9999.0] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                     '[ 9999.1] aicsdio: read timeout\n',
                     '[ 9999.2] usb 1-1.3: USB disconnect, device number 6\n',
                     '[ 9999.3] mmc1: error -5\n'):
            with self.subTest(tail=tail),self.assertRaises(self.Error):
                self.ns['validate_kernel'](profile['kernel_prefix']+tail,expected)
        for change in ({'kernel_prefix':profile['kernel_prefix'].split('\n',1)[1]},
                       {'historical_faults':profile['historical_faults'][:-1]},
                       {'reviewed_reconnect_proof':dict(feedback_stream.RECONNECT_PROOF)}):
            changed={**expected,**change}
            with self.subTest(change=next(iter(change))),self.assertRaises(self.Error):
                self.ns['validate_kernel'](changed['kernel_prefix'],changed)

    def test_two_review_variants_cannot_be_crossed_or_used_as_default_or_network_scope(self):
        firstpath,first=self.reviewed_profile()
        secondpath=feedback_stream.EVIDENCE_DIR/'feedback-reconnect-reviewed-20261004-02.json'
        second=feedback_stream.load_reviewed_profile(secondpath,feedback_stream.RECONNECT2_PROFILE_SHA256)
        for path,sha in ((firstpath,feedback_stream.RECONNECT2_PROFILE_SHA256),
                         (secondpath,feedback_stream.RECONNECT_PROFILE_SHA256)):
            with self.subTest(path=path.name),self.assertRaises(ValueError):
                feedback_stream.load_reviewed_profile(path,sha)
        with self.assertRaises(ValueError):feedback_stream.remote_source(self.profile_expected(second))
        with self.assertRaises(ValueError):
            feedback_stream.remote_source(self.profile_expected(second),reviewed_profile_path=firstpath,
                reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)
        for profile in (first,second):
            with self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile,'scope':'network_only_reviewed_reconnect'})
        feedback_stream.remote_source(self.expected)
        feedback_stream.remote_source(self.profile_expected(first),reviewed_profile_path=firstpath,
            reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)

    def test_second_profile_requires_completed_exact_network_evidence_and_whole_kernel(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-reconnect-reviewed-20261004-02.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RECONNECT2_PROFILE_SHA256)
        changes={'wifi_power_save_after':'on','wifi_power_save_set_attempts':True,
                 'wifi_volatile_setting_result_sha256':'0'*64,
                 'uart_open_attempts_in_network_recovery':1,
                 'new_faults_during_network_recovery':1,
                 'original_serial_readable_cause_confirmed':True}
        for key,value in changes.items():
            with self.subTest(key=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile,'review':{**profile['review'],key:value}})
        for key,value in ({'historical_faults':profile['historical_faults']+[profile['historical_faults'][-1]],
                           'kernel_line_count':964,
                           'kernel_prefix':profile['kernel_prefix']+'[ 9999.4] harmless extra\n'}).items():
            with self.subTest(key=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile,key:value})

    def test_aic_tx_message_retry_failure_and_negative_buffer_storm_fail_closed(self):
        actual='[10906.977561] sdio_err:<aicwf_sdio_tx_msg,1854>: tx msg fc retry fail:-1, 512'
        buffer='[10906.977500] buffer_cnt=-1'
        self.assertIsNone(self.ns['classify_log'](actual))
        self.assertIsNone(self.ns['classify_log'](buffer))
        self.assertEqual(self.ns['classify_feedback_log'](actual),
                         'Feedback guard reported an AIC SDIO WiFi driver failure')
        self.assertEqual(self.ns['classify_feedback_log'](buffer),
                         'Feedback guard reported a negative driver buffer count')
        for line in (actual,buffer,
                     '[10907.0] txrx_err:<aicwf_sdio_tx_msg,1854>: operation failed',
                     '[10907.1] sdio_err:<aicwf_sdio_tx_msg,1854>: retry failure'):
            with self.subTest(line=line),self.assertRaises(self.Error):
                self.ns['validate_kernel'](self.expected['kernel_prefix']+line+'\n',self.expected)
        path=feedback_stream.EVIDENCE_DIR/'feedback-reconnect-reviewed-20261004-02.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RECONNECT2_PROFILE_SHA256)
        expected=self.profile_expected(profile)
        expected['reviewed_reconnect_proof']=dict(feedback_stream.RECONNECT2_PROOF)
        with self.assertRaises(self.Error):
            self.ns['validate_kernel'](profile['kernel_prefix']+buffer+'\n'+actual+'\n',expected)
        with self.assertRaisesRegex(self.Error,'history changed or lost'):
            self.ns['validate_kernel']((buffer+'\n'+actual+'\n')*900,expected)
        for line in ('[10.0] buffer_cnt=0','[10.1] buffer_cnt=1','[10.2] buffer_cnt=-0',
                     '[10.3] aicbsp: sdio_err:<aicwf_sdio_bus_pwrctl,1402>: bus down',
                     '[10.4] sdio_err:<aicwf_sdio_tx_msg,1854>: retry: 1'):
            with self.subTest(ordinary=line):self.assertIsNone(feedback_stream.additional_feedback_fault(line))
        feedback_stream.remote_source(self.expected)
        firstpath,first=self.reviewed_profile()
        feedback_stream.remote_source(self.profile_expected(first),reviewed_profile_path=firstpath,
            reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)
        feedback_stream.remote_source(self.profile_expected(profile),reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.RECONNECT2_PROFILE_SHA256)

    def test_explicit_single_reboot_profile_two_unresolved_warnings_and_no_future_faults(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-reboot-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.REBOOT_PROFILE_SHA256)
        self.assertEqual(profile['boot_id'],feedback_stream.REBOOT_BOOT_ID)
        self.assertEqual(len(profile['kernel_prefix'].splitlines()),922)
        self.assertEqual(profile['historical_faults'],feedback_stream.REBOOT_HISTORICAL_FAULTS)
        expected=self.profile_expected(profile)
        source=feedback_stream.remote_source(expected,reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.REBOOT_PROFILE_SHA256)
        assignment=next(node for node in ast.parse(source).body if isinstance(node,ast.Assign)
                        and any(isinstance(target,ast.Name) and target.id=='EXPECTED' for target in node.targets))
        emitted=ast.literal_eval(assignment.value)
        self.assertEqual(emitted['reviewed_reconnect_proof'],feedback_stream.REBOOT_PROOF)
        self.assertEqual(emitted['boot_id'],feedback_stream.REBOOT_BOOT_ID)
        proof=self.ns['validate_kernel'](profile['kernel_prefix'],emitted)
        self.assertEqual(proof['historical_faults'],feedback_stream.REBOOT_HISTORICAL_FAULTS)
        self.assertFalse(proof['historical_faults_cleared'])
        for tail in (profile['historical_faults'][0]['line']+'\n',
                     profile['historical_faults'][1]['line']+'\n',
                     '[ 30.0] mmc1: error -5\n','[ 30.1] usb 1-1.3: USB disconnect, device number 4\n',
                     '[ 30.2] sdio_err:<aicwf_sdio_tx_msg,1854>: tx msg fc retry fail:-1, 512\n',
                     '[ 30.3] buffer_cnt=-1\n'):
            with self.subTest(tail=tail),self.assertRaises(self.Error):
                self.ns['validate_kernel'](profile['kernel_prefix']+tail,emitted)
        with self.assertRaisesRegex(self.Error,'history changed or lost'):
            self.ns['validate_kernel'](profile['kernel_prefix'].split('\n',1)[1],emitted)
        changed={**emitted,'boot_id':feedback_stream.REVIEWED_BOOT_ID}
        with self.assertRaises(self.Error):self.ns['validate_kernel'](changed['kernel_prefix'],changed)

    def test_cold_reconnect_uses_exact_review_and_requires_serial_discovery(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-cold-reconnect-reviewed-20261005-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.COLD_RECONNECT_PROFILE_SHA256)
        expected = self.profile_expected(profile)
        with self.assertRaises(ValueError):
            feedback_stream.remote_source(expected, reviewed_profile_path=path,
                reviewed_profile_sha256=feedback_stream.COLD_RECONNECT_PROFILE_SHA256)
        expected['discover_serial'] = True
        source = feedback_stream.remote_source(expected, reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.COLD_RECONNECT_PROFILE_SHA256)
        assignment = next(node for node in ast.parse(source).body if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == 'EXPECTED' for target in node.targets))
        emitted = ast.literal_eval(assignment.value)
        proof = self.ns['validate_kernel'](profile['kernel_prefix'], emitted)
        self.assertEqual(emitted['boot_id'], feedback_stream.COLD_RECONNECT_BOOT_ID)
        self.assertEqual(proof['historical_faults'], feedback_stream.COLD_RECONNECT_HISTORICAL_FAULTS)
        self.assertFalse(proof['historical_faults_cleared'])
        self.assertEqual(len(profile['review']['prior_boot_historical_faults']), 13)
        self.assertFalse(profile['review']['prior_boot_faults_cleared'])
        for tail in ('[ 800.0] mmc1: error -5\n',
                     '[ 800.1] usb 1-1.4: USB disconnect, device number 3\n',
                     profile['historical_faults'][0]['line'] + '\n'):
            with self.subTest(tail=tail), self.assertRaises(self.Error):
                self.ns['validate_kernel'](profile['kernel_prefix'] + tail, emitted)

    def test_cold_reconnect_does_not_allow_an_unreviewed_boot_or_edited_history(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-cold-reconnect-reviewed-20261005-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.COLD_RECONNECT_PROFILE_SHA256)
        with self.assertRaises(ValueError):
            feedback_stream.load_reviewed_profile(path, feedback_stream.USB_PORT_MOVE_ESP32_PROFILE_SHA256)
        for change in ({'boot_id': '00000000-0000-4000-8000-000000000001'},
                       {'historical_faults': []}, {'servo_arm_authorized': True},
                       {'kernel_prefix': profile['kernel_prefix'] + '[800.1] harmless\n'},
                       {'review': {**profile['review'], 'prior_boot_historical_faults': []}},
                       {'review': {**profile['review'], 'diagnostic_uart_opened': True}}):
            with self.subTest(change=next(iter(change))), self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile, **change})
        expected = {**self.profile_expected(profile), 'discover_serial': True}
        with self.assertRaises(ValueError): feedback_stream.remote_source(expected)
        with self.assertRaises(self.Error):
            self.ns['validate_kernel'](profile['kernel_prefix'], expected)

    def test_reboot_needs_its_fixed_artifact_and_never_expands_default_or_legacy_boots(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-reboot-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.REBOOT_PROFILE_SHA256)
        expected=self.profile_expected(profile)
        with self.assertRaises(ValueError):feedback_stream.remote_source(expected)
        default_new={**self.expected,'boot_id':feedback_stream.REBOOT_BOOT_ID}
        with self.assertRaises(ValueError):feedback_stream.remote_source(default_new)
        self.ns['validate_kernel'](self.expected['kernel_prefix'],self.expected)
        with self.assertRaises(self.Error):self.ns['validate_kernel'](default_new['kernel_prefix'],default_new)
        firstpath,first=self.reviewed_profile()
        for wrong_sha in (feedback_stream.RECONNECT_PROFILE_SHA256,feedback_stream.RECONNECT2_PROFILE_SHA256):
            with self.subTest(sha=wrong_sha),self.assertRaises(ValueError):
                feedback_stream.load_reviewed_profile(path,wrong_sha)
        with self.assertRaises(ValueError):
            feedback_stream.remote_source({**self.profile_expected(first),'boot_id':feedback_stream.REBOOT_BOOT_ID},
                reviewed_profile_path=firstpath,reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)
        for change in ({'boot_id':'00000000-0000-4000-8000-000000000001'},
                       {'cid':'0'*32},{'root_mount_line':expected['root_mount_line'].replace('rw,relatime','ro,relatime')},
                       {'serial':{**expected['serial'],'sysfs_name':'1-1.4'}},
                       {'historical_faults':[]}, {'generation':True}):
            with self.subTest(change=next(iter(change))),self.assertRaises(ValueError):
                feedback_stream.remote_source({**expected,**change},reviewed_profile_path=path,
                    reviewed_profile_sha256=feedback_stream.REBOOT_PROFILE_SHA256)

    def test_reboot_review_preserves_prior_faults_exact_diagnostics_and_false_repair_flags(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-reboot-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.REBOOT_PROFILE_SHA256)
        for key in feedback_stream.REBOOT_REQUIRED_REVIEW:
            changed=copy.deepcopy(profile)
            changed['review'].pop(key)
            with self.subTest(missing=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile(changed)
        for key,value in ({'filesystem_repaired':True,'prior_boot_faults_cleared':True,
                           'diagnostic_uart_opened':True,'wifi_settings_modified_in_this_update':True,
                           'prior_boot_historical_faults':feedback_stream.RECONNECT2_HISTORICAL_FAULTS[:-1],
                           'diagnostic_result_sha256':'0'*64,'human_context':'unconfirmed restart'}).items():
            with self.subTest(changed=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile,'review':{**profile['review'],key:value}})
        for change in ({'kernel_line_count':923},
                       {'kernel_prefix':profile['kernel_prefix']+'[30.5] harmless extra\n'},
                       {'historical_faults':profile['historical_faults']+[profile['historical_faults'][0]]},
                       {'filesystem_warnings_cleared':True},{'scope':'network_only_reviewed_reconnect'}):
            with self.subTest(change=next(iter(change))),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile,**change})

    def test_returned_esp32_fixed_same_boot_profile_keeps_old_warnings_and_future_fault_stops(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-returned-esp32-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        self.assertEqual(profile['boot_id'],feedback_stream.REBOOT_BOOT_ID)
        self.assertEqual(len(profile['kernel_prefix'].splitlines()),935)
        self.assertEqual(profile['historical_faults'],feedback_stream.RETURNED_ESP32_HISTORICAL_FAULTS)
        self.assertEqual(profile['historical_faults'][:2],feedback_stream.REBOOT_HISTORICAL_FAULTS)
        expected=self.profile_expected(profile)
        source=feedback_stream.remote_source(expected,reviewed_profile_path=path,
            reviewed_profile_sha256=feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        assignment=next(node for node in ast.parse(source).body if isinstance(node,ast.Assign)
                        and any(isinstance(target,ast.Name) and target.id=='EXPECTED' for target in node.targets))
        emitted=ast.literal_eval(assignment.value)
        self.assertEqual(emitted['reviewed_reconnect_proof'],feedback_stream.RETURNED_ESP32_PROOF)
        proof=self.ns['validate_kernel'](profile['kernel_prefix'],emitted)
        self.assertFalse(proof['historical_faults_cleared'])
        self.assertEqual(proof['historical_faults'],profile['historical_faults'])
        for tail in (profile['historical_faults'][2]['line']+'\n',
                     '[ 7000.0] usb 1-1.3: USB disconnect, device number 5\n',
                     '[ 7000.1] usb 1-1: USB disconnect, device number 2\n',
                     '[ 7000.2] mmc1: error -5\n',
                     '[ 7000.3] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n',
                     profile['historical_faults'][0]['line']+'\n'):
            with self.subTest(tail=tail),self.assertRaises(self.Error):
                self.ns['validate_kernel'](profile['kernel_prefix']+tail,emitted)
        with self.assertRaises(self.Error):
            self.ns['validate_kernel'](profile['kernel_prefix'].split('\n',1)[1],emitted)
        with self.assertRaises(ValueError):feedback_stream.remote_source(expected)
        with self.assertRaises(ValueError):
            feedback_stream.load_reviewed_profile(path,feedback_stream.REBOOT_PROFILE_SHA256)

    def test_returned_esp32_requires_every_old_review_field_and_exact_new_nested_review(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-returned-esp32-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        for key in feedback_stream.REBOOT_REQUIRED_REVIEW:
            changed=copy.deepcopy(profile);changed['review'].pop(key)
            with self.subTest(old_missing=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile(changed)
        for key in feedback_stream.RETURNED_ESP32_REQUIRED_REVIEW:
            changed=copy.deepcopy(profile);changed['review']['same_boot_usb_reconnect_review'].pop(key)
            with self.subTest(new_missing=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile(changed)
        for key,value in (('unsafe_feedback_hold_cleared',True),('prior_faults_cleared',True),
                          ('old_boot_pin_refusal_retained',1),('diagnostic_result_sha256','0'*64),
                          ('reviewed_usb_events',[]),('extra_generic_usb_allowance',True)):
            changed=copy.deepcopy(profile);changed['review']['same_boot_usb_reconnect_review'][key]=value
            with self.subTest(new_changed=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile(changed)
        for key,value in (('filesystem_repaired',True),('prior_boot_faults_cleared',True)):
            changed=copy.deepcopy(profile);changed['review'][key]=value
            with self.subTest(old_changed=key),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile(changed)

    def test_maintenance_firmware_return_keeps_exact_history_and_future_fault_stops(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-esp32-reviewed-20261004-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.MAINTENANCE_ESP32_PROFILE_SHA256)
        old = feedback_stream.load_reviewed_profile(
            feedback_stream.EVIDENCE_DIR / 'feedback-returned-esp32-reviewed-20261004-01.json',
            feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        self.assertTrue(profile['kernel_prefix'].startswith(old['kernel_prefix']))
        self.assertEqual(profile['historical_faults'][:-1], old['historical_faults'])
        self.assertEqual(len(profile['kernel_prefix'].splitlines()), 944)
        expected = self.profile_expected(profile)
        expected['reviewed_reconnect_proof'] = dict(feedback_stream.MAINTENANCE_ESP32_PROOF)
        self.assertFalse(self.ns['validate_kernel'](profile['kernel_prefix'], expected)['historical_faults_cleared'])
        for tail in (profile['historical_faults'][-1]['line'] + '\n',
                     '[ 69000.0] usb 1-1.3: USB disconnect, device number 5\n',
                     '[ 69000.1] mmc1: error -5\n',
                     '[ 69000.2] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n'):
            with self.subTest(tail=tail), self.assertRaises(self.Error):
                self.ns['validate_kernel'](profile['kernel_prefix'] + tail, expected)
        source = feedback_stream.remote_source(self.profile_expected(profile),
            reviewed_profile_path=path, reviewed_profile_sha256=feedback_stream.MAINTENANCE_ESP32_PROFILE_SHA256)
        self.assertNotIn(profile['review']['maintenance_firmware_usb_reconnect_review']['human_context'], source)

    def test_maintenance_firmware_return_requires_every_old_and_new_review_field(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-esp32-reviewed-20261004-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.MAINTENANCE_ESP32_PROFILE_SHA256)
        for field in feedback_stream.MAINTENANCE_ESP32_REQUIRED_REVIEW:
            changed = copy.deepcopy(profile)
            changed['review']['maintenance_firmware_usb_reconnect_review'].pop(field)
            with self.subTest(missing=field), self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile(changed)
        for mutate in (lambda x: x['review'].pop('same_boot_usb_reconnect_review'),
                       lambda x: x['review'].pop('diagnostic_result_sha256'),
                       lambda x: x.update(servo_motion_authorized=True),
                       lambda x: x['review']['maintenance_firmware_usb_reconnect_review'].update(
                           unsafe_feedback_hold_cleared=True),
                       lambda x: x.update(historical_faults=x['historical_faults'][1:])):
            changed = copy.deepcopy(profile)
            mutate(changed)
            with self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile(changed)
        with self.assertRaises(ValueError):
            feedback_stream.load_reviewed_profile(path, feedback_stream.RETURNED_ESP32_PROFILE_SHA256)

    def test_write_ack_firmware_return_preserves_history_and_rejects_future_faults_or_tampering(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-write-ack-reviewed-20261004-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.WRITE_ACK_ESP32_PROFILE_SHA256)
        previous = feedback_stream.load_reviewed_profile(
            feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-esp32-reviewed-20261004-01.json',
            feedback_stream.MAINTENANCE_ESP32_PROFILE_SHA256)
        self.assertTrue(profile['kernel_prefix'].startswith(previous['kernel_prefix']))
        self.assertEqual(profile['historical_faults'][:-1], previous['historical_faults'])
        expected = self.profile_expected(profile)
        expected['reviewed_reconnect_proof'] = dict(feedback_stream.WRITE_ACK_ESP32_PROOF)
        self.ns['validate_kernel'](profile['kernel_prefix'], expected)
        for tail in ('[ 71000.0] usb 1-1.3: USB disconnect, device number 6\n',
                     '[ 71000.1] mmc1: error -5\n'):
            with self.assertRaises(self.Error): self.ns['validate_kernel'](profile['kernel_prefix'] + tail, expected)
        for name, required in (('maintenance_write_ack_usb_reconnect_review', feedback_stream.WRITE_ACK_ESP32_REQUIRED_REVIEW),
                               ('maintenance_firmware_usb_reconnect_review', feedback_stream.MAINTENANCE_ESP32_REQUIRED_REVIEW)):
            for field in required:
                changed = copy.deepcopy(profile); changed['review'][name].pop(field)
                with self.subTest(review=name, missing=field), self.assertRaises(ValueError):
                    feedback_stream._validate_reviewed_profile(changed)
        changed = copy.deepcopy(profile); changed['review']['maintenance_write_ack_usb_reconnect_review']['unsafe_feedback_hold_cleared'] = True
        with self.assertRaises(ValueError): feedback_stream._validate_reviewed_profile(changed)
        changed = copy.deepcopy(profile); changed['historical_faults'].pop(0)
        with self.assertRaises(ValueError): feedback_stream._validate_reviewed_profile(changed)

    def test_center_trace_return_keeps_old_faults_and_only_exact_nonfault_line_eviction(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-center-trace-reviewed-20261005-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.CENTER_TRACE_ESP32_PROFILE_SHA256)
        old = feedback_stream.load_reviewed_profile(
            feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-write-ack-reviewed-20261004-01.json',
            feedback_stream.WRITE_ACK_ESP32_PROFILE_SHA256)
        removed, retained = old['kernel_prefix'].split('\n', 1)
        self.assertTrue(profile['kernel_prefix'].startswith(retained))
        self.assertEqual(profile['historical_faults'][:-1], old['historical_faults'])
        self.assertEqual(profile['review']['maintenance_center_trace_usb_reconnect_review']['evicted_nonfault_lines'], [removed])
        expected = self.profile_expected(profile)
        expected['reviewed_reconnect_proof'] = dict(feedback_stream.CENTER_TRACE_ESP32_PROOF)
        self.ns['validate_kernel'](profile['kernel_prefix'], expected)
        for changed_log in (profile['kernel_prefix'] + '[ 73000.0] usb 1-1.3: USB disconnect, device number 7\n',
                            profile['kernel_prefix'] + '[ 73000.1] mmc1: error -5\n',
                            profile['kernel_prefix'].split('\n', 1)[1]):
            with self.assertRaises(self.Error): self.ns['validate_kernel'](changed_log, expected)
        for name, required in (('maintenance_center_trace_usb_reconnect_review', feedback_stream.CENTER_TRACE_ESP32_REQUIRED_REVIEW),
                               ('maintenance_write_ack_usb_reconnect_review', feedback_stream.WRITE_ACK_ESP32_REQUIRED_REVIEW),
                               ('maintenance_firmware_usb_reconnect_review', feedback_stream.MAINTENANCE_ESP32_REQUIRED_REVIEW),
                               ('same_boot_usb_reconnect_review', feedback_stream.RETURNED_ESP32_REQUIRED_REVIEW)):
            for field in required:
                changed = copy.deepcopy(profile); changed['review'][name].pop(field)
                with self.subTest(review=name, missing=field), self.assertRaises(ValueError):
                    feedback_stream._validate_reviewed_profile(changed)
        changed = copy.deepcopy(profile); changed['historical_faults'].pop(0)
        with self.assertRaises(ValueError): feedback_stream._validate_reviewed_profile(changed)

    def test_center_holding_return_preserves_faults_and_exact_six_line_eviction(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-center-holding-reviewed-20261005-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.CENTER_HOLDING_ESP32_PROFILE_SHA256)
        old = feedback_stream.load_reviewed_profile(
            feedback_stream.EVIDENCE_DIR / 'feedback-maintenance-center-trace-reviewed-20261005-01.json',
            feedback_stream.CENTER_TRACE_ESP32_PROFILE_SHA256)
        lines = old['kernel_prefix'].splitlines(keepends=True)
        self.assertTrue(profile['kernel_prefix'].startswith(''.join(lines[6:])))
        self.assertEqual(profile['historical_faults'][:-1], old['historical_faults'])
        review = profile['review']['maintenance_center_holding_usb_reconnect_review']
        self.assertEqual(review['evicted_nonfault_lines'], [line.rstrip('\n') for line in lines[:6]])
        self.assertEqual(review['retained_prior_line_count'], len(lines) - 6)
        expected = self.profile_expected(profile)
        expected['reviewed_reconnect_proof'] = dict(feedback_stream.CENTER_HOLDING_ESP32_PROOF)
        self.ns['validate_kernel'](profile['kernel_prefix'], expected)
        for log in (profile['kernel_prefix'] + '[ 75000.0] usb 1-1.3: USB disconnect, device number 8\n',
                    profile['kernel_prefix'] + '[ 75000.1] mmc1: error -5\n',
                    profile['kernel_prefix'].split('\n', 1)[1]):
            with self.assertRaises(self.Error): self.ns['validate_kernel'](log, expected)
        for name, required in (('maintenance_center_holding_usb_reconnect_review', feedback_stream.CENTER_HOLDING_ESP32_REQUIRED_REVIEW),
                               ('maintenance_center_trace_usb_reconnect_review', feedback_stream.CENTER_TRACE_ESP32_REQUIRED_REVIEW)):
            for field in required:
                changed = copy.deepcopy(profile); changed['review'][name].pop(field)
                with self.subTest(review=name, missing=field), self.assertRaises(ValueError):
                    feedback_stream._validate_reviewed_profile(changed)
        changed = copy.deepcopy(profile); changed['historical_faults'].pop(0)
        with self.assertRaises(ValueError): feedback_stream._validate_reviewed_profile(changed)

    def test_arm_feedback_return_pins_exact_update_and_preserves_all_old_faults(self):
        path = feedback_stream.EVIDENCE_DIR / 'feedback-motion-arm-fresh-reviewed-20261005-01.json'
        profile = feedback_stream.load_reviewed_profile(path, feedback_stream.ARM_FRESH_ESP32_PROFILE_SHA256)
        self.assertEqual(profile['historical_faults'][:-1], feedback_stream.CENTER_HOLDING_ESP32_HISTORICAL_FAULTS)
        review = profile['review']['motion_arm_fresh_feedback_usb_reconnect_review']
        self.assertEqual(10,len(review['evicted_nonfault_lines']))
        self.assertEqual(954,review['retained_prior_line_count'])
        self.assertFalse(profile['servo_motion_authorized'])
        expected = self.profile_expected(profile)
        expected['reviewed_reconnect_proof'] = dict(feedback_stream.ARM_FRESH_ESP32_PROOF)
        self.ns['validate_kernel'](profile['kernel_prefix'],expected)
        for log in (profile['kernel_prefix']+'[ 79000.0] usb 1-1.3: USB disconnect, device number 9\n',
                    profile['kernel_prefix']+'[ 79000.1] mmc1: error -5\n',
                    profile['kernel_prefix'].split('\n',1)[1]):
            with self.assertRaises(self.Error): self.ns['validate_kernel'](log,expected)
        for name,required in [('motion_arm_fresh_feedback_usb_reconnect_review',feedback_stream.ARM_FRESH_ESP32_REQUIRED_REVIEW),
                              ('maintenance_center_holding_usb_reconnect_review',feedback_stream.CENTER_HOLDING_ESP32_REQUIRED_REVIEW)]:
            for field in required:
                changed = copy.deepcopy(profile); changed['review'][name].pop(field)
                with self.subTest(review=name,missing=field),self.assertRaises(ValueError):
                    feedback_stream._validate_reviewed_profile(changed)
        changed=copy.deepcopy(profile);changed['historical_faults'].pop(0)
        with self.assertRaises(ValueError):feedback_stream._validate_reviewed_profile(changed)

    def test_arm_write_reply_return_retains_exact_history_and_does_not_authorize_motion(self):
        path = feedback_stream.EVIDENCE_DIR/'feedback-motion-arm-write-reply-reviewed-20261005-01.json'
        profile = feedback_stream.load_reviewed_profile(path,feedback_stream.ARM_WRITE_REPLY_ESP32_PROFILE_SHA256)
        self.assertEqual(profile['historical_faults'][:-1],feedback_stream.ARM_FRESH_ESP32_HISTORICAL_FAULTS)
        self.assertFalse(profile['servo_motion_authorized'])
        review=profile['review']['motion_arm_write_reply_usb_reconnect_review']
        self.assertEqual(len(review['evicted_nonfault_lines']),10)
        self.assertEqual(review['retained_prior_line_count'],953)
        expected=self.profile_expected(profile)
        expected['reviewed_reconnect_proof']=dict(feedback_stream.ARM_WRITE_REPLY_ESP32_PROOF)
        self.ns['validate_kernel'](profile['kernel_prefix'],expected)
        for log in (profile['kernel_prefix']+'[ 81000.0] usb 1-1.3: USB disconnect, device number 10\n',
                    profile['kernel_prefix']+'[ 81000.1] mmc1: error -5\n',profile['kernel_prefix'].split('\n',1)[1]):
            with self.assertRaises(self.Error):self.ns['validate_kernel'](log,expected)
        for name,required in [('motion_arm_write_reply_usb_reconnect_review',feedback_stream.ARM_WRITE_REPLY_ESP32_REQUIRED_REVIEW),
                              ('motion_arm_fresh_feedback_usb_reconnect_review',feedback_stream.ARM_FRESH_ESP32_REQUIRED_REVIEW)]:
            for field in required:
                changed=copy.deepcopy(profile);changed['review'][name].pop(field)
                with self.subTest(review=name,missing=field),self.assertRaises(ValueError):
                    feedback_stream._validate_reviewed_profile(changed)
        changed=copy.deepcopy(profile);changed['historical_faults'].pop(0)
        with self.assertRaises(ValueError):feedback_stream._validate_reviewed_profile(changed)

    def test_returned_esp32_proof_never_changes_boot_identity_or_motion_permissions(self):
        path=feedback_stream.EVIDENCE_DIR/'feedback-returned-esp32-reviewed-20261004-01.json'
        profile=feedback_stream.load_reviewed_profile(path,feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        expected=self.profile_expected(profile)
        for change in ({'boot_id':feedback_stream.REVIEWED_BOOT_ID}, {'cid':'0'*32},
                       {'serial':{**expected['serial'],'sysfs_name':'1-1.4'}}, {'historical_faults':[]}):
            with self.subTest(change=next(iter(change))),self.assertRaises(ValueError):
                feedback_stream.remote_source({**expected,**change},reviewed_profile_path=path,
                    reviewed_profile_sha256=feedback_stream.RETURNED_ESP32_PROFILE_SHA256)
        for change in ({'allowed_requests':['HELLO','STATE','ARM']},{'servo_motion_authorized':True},
                       {'filesystem_warnings_cleared':True}):
            with self.subTest(change=next(iter(change))),self.assertRaises(ValueError):
                feedback_stream._validate_reviewed_profile({**profile,**change})

    def test_generation_and_obsolete_settings_rejected_before_remote_source(self):
        feedback_stream.remote_source(self.expected)
        for change in ({"generation": True}, {"generation": 0}, {"generation": "uuid"},
                       {"duration_seconds": 3600}, {"control_end": "ARM\n"},
                       {"cid": "fe34325344313647200000052e013c00"}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                feedback_stream.remote_source({**self.expected, **change})

    def test_no_default_or_configurable_absolute_lifetime(self):
        self.assertNotIn('duration_seconds',feedback_stream.validate_expected(self.expected))
        for duration in (None,0,900,3600,3601,True,'3600'):
            with self.subTest(duration=duration),self.assertRaises(ValueError):
                feedback_stream.validate_expected({**self.expected,'duration_seconds':duration})

    def test_fixed_control_message_complete_and_fragmented_end(self):
        channel=self.ns['ControlChannel'](71)
        clock=[0.]
        chunks=[b'END_',b'FEEDBACK\n']
        with mock.patch.object(self.ns['time'],'monotonic',side_effect=lambda:clock[0]), \
                mock.patch.object(self.ns['select'],'select',return_value=([71],[],[])), \
                mock.patch.object(self.ns['os'],'read',side_effect=lambda fd,count:chunks.pop(0)) as reads:
            self.assertFalse(channel.poll())
            clock[0]=.05
            self.assertTrue(channel.poll())
            self.assertTrue(channel.poll())
            self.assertEqual(reads.call_count,2)
            self.assertTrue(all(call.args[0]==71 and 0<call.args[1]<=64 for call in reads.call_args_list))
        self.assertEqual(self.ns['CONTROL_END'],feedback_stream.CONTROL_END)

    def test_fixed_control_rejects_eof_arbitrary_or_trailing_commands_and_stalled_prefix(self):
        for chunk in (b'',b'ARM\n',b'END_FEEDBACK\nARM\n',feedback_stream.CONTROL_END*2,b'E'*64):
            with self.subTest(chunk=chunk):
                channel=self.ns['ControlChannel'](71)
                with mock.patch.object(self.ns['select'],'select',return_value=([71],[],[])), \
                        mock.patch.object(self.ns['os'],'read',return_value=chunk),self.assertRaises(self.Error):
                    channel.poll()
        channel=self.ns['ControlChannel'](71)
        clock=[0.]
        with mock.patch.object(self.ns['time'],'monotonic',side_effect=lambda:clock[0]), \
                mock.patch.object(self.ns['select'],'select',return_value=([71],[],[])), \
                mock.patch.object(self.ns['os'],'read',return_value=b'END_') as reads:
            self.assertFalse(channel.poll())
            clock[0]=.201
            with self.assertRaisesRegex(self.Error,'did not complete'):channel.poll()
            self.assertEqual(reads.call_count,1)

    def test_source_generation_has_no_device_open_or_evidence_main_execution(self):
        with mock.patch("os.open", side_effect=AssertionError("device open during generation")), \
                mock.patch("subprocess.Popen", side_effect=AssertionError("process during generation")):
            source = feedback_stream.remote_source(self.expected)
            compile(source, "<remote-reader>", "exec")
        self.assertNotIn("def get_feedback(", source)
        self.assertNotIn("one_shot_passive_os_metadata", source)

    def test_health_retains_exact_faults_and_rejects_new_duplicate_or_lost_history(self):
        validate = self.ns["validate_kernel"]
        proof = validate(self.expected["kernel_prefix"]+"[ 1002.000000] normal information\n", self.expected)
        self.assertFalse(proof["historical_faults_cleared"])
        self.assertEqual(proof["historical_faults"], self.expected["historical_faults"])
        for changed in (self.expected["kernel_prefix"]+"[ 1002.000000] mmc1: error -5\n",
                        self.expected["kernel_prefix"]+self.expected["historical_faults"][2]["line"]+"\n",
                        self.expected["kernel_prefix"].split("\n",1)[1],
                        self.expected["kernel_prefix"].replace("device number 3", "device number 8")):
            with self.subTest(changed=changed[-100:]), self.assertRaises(self.Error):
                validate(changed, self.expected)

    def test_cached_baseline_still_matches_every_line_and_classifies_every_new_tail(self):
        validate=self.ns['validate_kernel']
        classifier=mock.Mock(wraps=self.ns['classify_log'])
        self.ns['classify_log']=classifier
        prefix=self.expected['kernel_prefix']
        validate(prefix,self.expected)
        self.assertEqual(classifier.call_count,len(prefix.splitlines()))
        classifier.reset_mock()
        tail='[ 4000.000000] ordinary status\n'
        validate(prefix+tail,self.expected)
        self.assertEqual(classifier.call_args_list,[mock.call(tail.strip())])
        classifier.reset_mock()
        with self.assertRaises(self.Error):
            validate(prefix+tail+'[ 4001.000000] Buffer I/O error on dev mmcblk1p2\n',self.expected)
        self.assertEqual(classifier.call_count,2)
        for changed in (prefix.split('\n',1)[1],
                        prefix.replace('corrupted or uncleanly', 'silently damaged and uncleanly')):
            classifier.reset_mock()
            with self.assertRaises(self.Error):validate(changed,self.expected)
            self.assertEqual(classifier.call_count,0)
        # Changing the caller's expected baseline cannot reuse the valid cache.
        changed=prefix.replace('corrupted or uncleanly', 'silently damaged and uncleanly')
        with self.assertRaises(self.Error):
            validate(changed,{**self.expected,'kernel_prefix':changed})
        altered=copy.deepcopy(self.expected)
        altered['historical_faults'][0]['reason']='pretend harmless'
        with self.assertRaises(self.Error):validate(prefix,altered)

    def test_new_fault_error_keeps_bounded_source_and_driver_context_without_relabeling(self):
        prefix=self.expected['kernel_prefix']
        tail=('\n'.join([
            '[ 8518.691466] usb 1-1: USB disconnect, device number 2',
            '[ 8518.691521] usb 1-1.3: USB disconnect, device number 4',
            '[ 8518.931957] usb 2-1: USB disconnect, device number 2',
            '[ 8544.480306] txrx_err:<aicwf_sdio_wakeup,1312>: sdio wakeup fail',
            '[ 8544.486285] aicsdio: write retry:  1'])+'\n')
        with self.assertRaises(self.Error) as captured:
            self.ns['validate_kernel'](prefix+tail,self.expected)
        evidence=captured.exception.metadata['new_kernel_fault_evidence']
        self.assertEqual(evidence['new_fault_count'],4)
        self.assertFalse(evidence['fault_details_truncated'])
        self.assertTrue(all(fault['source']=='new_kernel_tail' for fault in evidence['faults']))
        sdio=evidence['faults'][3]
        self.assertIn('aicwf_sdio_wakeup',sdio['line'])
        self.assertEqual(sdio['classifier_reason'],self.ns['classify_log'](sdio['line']))
        self.assertTrue(any('aicsdio: write retry' in line['line'] for line in sdio['context']))
        self.assertIn('classifier result requires source review',str(captured.exception))
        self.assertNotIn('TF card damaged',str(captured.exception))
        self.assertEqual(evidence['faults'][0]['kernel_line_number'],len(prefix.splitlines())+1)

    def test_fault_context_is_bounded_escaped_and_never_folds_replayed_history(self):
        prefix=self.expected['kernel_prefix']
        replay=self.expected['historical_faults'][0]['line']
        long_fault='[ 9000.000001] Buffer I/O error '+('x'*10000)+'\x1b[31m'
        faults=[replay]+[long_fault]*5
        with self.assertRaises(self.Error) as captured:
            self.ns['validate_kernel'](prefix+'\n'.join(faults)+'\n',self.expected)
        evidence=captured.exception.metadata['new_kernel_fault_evidence']
        self.assertEqual(evidence['new_fault_count'],6)
        self.assertEqual(len(evidence['faults']),4)
        self.assertTrue(evidence['fault_details_truncated'])
        self.assertEqual(evidence['faults'][0]['source'],'new_kernel_tail')
        self.assertEqual(evidence['faults'][0]['line'],replay)
        self.assertTrue(evidence['faults'][1]['line_truncated'])
        self.assertTrue(all(len(fault['line'])<=512 and len(fault['context'])<=4
                            and all(len(row['line'])<=512 for row in fault['context'])
                            for fault in evidence['faults']))
        self.assertLess(len(str(captured.exception)),13000)
        # Control characters even near the beginning must remain JSON escaped.
        with self.assertRaises(self.Error) as short:
            self.ns['validate_kernel'](prefix+'[ 9001.0] I/O error \x1b[31m\n',self.expected)
        self.assertNotIn('\x1b',str(short.exception))
        self.assertIn('\\u001b',str(short.exception))

    def test_identity_rejects_root_readonly_swap_card_or_boot_change(self):
        metadata = {key:self.expected[key] for key in ("boot_id","cid","root_mount_line","model")}
        metadata["swaps"] = "Filename\tType\tSize\tUsed\tPriority"
        self.ns["validate_os"](metadata, self.expected)
        for change in ({"root_mount_line":metadata["root_mount_line"].replace("rw", "ro")},
                       {"swaps":metadata["swaps"]+"\n/dev/mmcblk1p3 partition 100 0 -1"},
                       {"cid":"f"*32}, {"boot_id":"new"}):
            with self.subTest(change=change), self.assertRaises(self.Error):
                self.ns["validate_os"]({**metadata,**change}, self.expected)

    def test_state_gate_allows_unpowered_imu_and_complete_powered_feedback(self):
        for online in (0,self.ns["ALL_MASK"]):
            tracker = self.ns["StateTracker"]()
            self.assertTrue(tracker.accept(state(self.ns,online=online),0))
            self.assertTrue(tracker.accept(state(self.ns,online=online,sample=1020000,imu=1021000),.02))
        for changes in ({"online":1}, {"fault":1}, {"torque":1}, {"armed":1}, {"cookie":1},
                        {"sample":1000000,"imu":400000}, {"accel":[float("nan"),0,9.8]}):
            with self.subTest(changes=changes), self.assertRaises(self.Error):
                self.ns["StateTracker"]().accept(state(self.ns,**changes),0)

    def test_timestamp_wrap_is_valid_duplicate_not_emitted_frozen_or_reset_rejected(self):
        tracker = self.ns["StateTracker"]()
        initial = state(self.ns,sample=0xfffff000,imu=0xfffff500)
        self.assertTrue(tracker.accept(initial,0))
        self.assertFalse(tracker.accept(copy.deepcopy(initial),.02))
        self.assertTrue(tracker.accept(state(self.ns,sample=0x3e20,imu=0x4320),.04))
        with self.assertRaises(self.Error):
            tracker.accept(state(self.ns,sample=0x3000,imu=0x3500),.06)
        tracker = self.ns["StateTracker"]()
        tracker.accept(state(self.ns),0)
        with self.assertRaises(self.Error):
            tracker.accept(state(self.ns),.5)

    def test_framing_exact_crc_and_sequence_and_no_input_recovery(self):
        ns = self.ns
        sequence = 11
        body = payload(ns)
        header = ns["HEADER"].pack(b"XD",1,0x82,len(body),sequence)
        crc = struct.pack("<H",ns["_crc16"](header+body))
        sent = []
        def exchange(raw_header, tail, readable=False):
            ns["_read_exact"] = mock.Mock(side_effect=[raw_header,tail])
            ns["_write_once"] = lambda fd,packet,deadline,metadata:sent.append(packet)
            metadata = {"request_types_attempted":[],"reply_types_received":[],"request_bytes_written":0}
            with mock.patch.object(ns["select"],"select", return_value=([91],[],[]) if readable else ([],[],[])):
                return ns["_exchange"](91,2,sequence,ns["time"].monotonic()+2,metadata)
        self.assertEqual(exchange(header,body+crc),body)
        self.assertEqual(sent[-1],ns["_frame"](2,sequence))
        with self.assertRaisesRegex(self.Error,"CRC"):
            exchange(header,body+b"\0\0")
        with self.assertRaisesRegex(self.Error,"header"):
            exchange(ns["HEADER"].pack(b"XD",1,0x82,len(body),sequence-1),body+crc)
        previous_writes = len(sent)
        with self.assertRaisesRegex(self.Error,"Unexpected serial input"):
            exchange(header,body+crc,readable=True)
        self.assertEqual(len(sent),previous_writes)
        for kind in (3,4,5,6):
            with self.assertRaises(self.Error): ns["_frame"](kind,1)

    def test_kernel_capture_handles_independent_eof_and_rejects_stderr_or_oversize(self):
        class Pipe:
            def __init__(self,fd):self.fd,self.closed=fd,False
            def fileno(self):return self.fd
            def close(self):self.closed=True
        class Child:
            def __init__(self):
                self.stdout,self.stderr=Pipe(201),Pipe(202)
                self.returncode=None
                self.killed=False
            def poll(self):return self.returncode
            def wait(self,timeout=None):self.returncode=0;return 0
            def kill(self):self.killed=True
        for stdout,stderr,raises in ((b"[ 0.0] Kernel\n",b"",False),
                                     (b"Kernel\n",b"dmesg denied",True),
                                     (b"x"*(feedback_stream.MAX_KERNEL_BYTES+1),b"",True)):
            with self.subTest(raises=raises,stderr=stderr):
                child=Child()
                chunks={201:[stdout,b""],202:[stderr,b""] if stderr else [b""]}
                with mock.patch.object(self.ns["subprocess"],"Popen",return_value=child), \
                        mock.patch.object(self.ns["select"],"select",side_effect=lambda reads,*args:(reads,[],[])), \
                        mock.patch.object(self.ns["os"],"read",side_effect=lambda fd,count:chunks[fd].pop(0)):
                    if raises:
                        with self.assertRaises(self.Error):self.ns["bounded_dmesg"]()
                    else:
                        self.assertEqual(self.ns["bounded_dmesg"](),stdout.decode())
                self.assertTrue(child.stdout.closed and child.stderr.closed)
                if len(stdout)>feedback_stream.MAX_KERNEL_BYTES:self.assertTrue(child.killed)

    def fake_run(self, *, unsafe=False, open_fails=False, pipe_fails=False, health_fails=False,
                 modem_changes=False, holder_stage=None, end_at=.10,
                 control_problem=None, parent_gone=False, payload_changes=None,
                 reply_failure_at=None, health_factory=None):
        ns = self.ns
        if health_factory is None:ns.pop('reader_health_watcher',None)
        else:ns['reader_health_watcher']=health_factory
        events, requests, opened, closed = [], [], [], []
        clock = [0.]
        sequence_state = [0]
        proof = {"kernel_log":self.expected["kernel_prefix"], "historical_faults_cleared":False}
        class FakeControl:
            def __init__(self,fd):pass
            def poll(control):
                if control_problem and clock[0]>=.04:raise self.Error(control_problem)
                return end_at is not None and not health_fails and clock[0]>=end_at
        def fake_health(expected,fd=None):
            if health_fails and clock[0] >= .25:raise self.Error("new USB fault")
            return proof
        def fake_open(*args):
            opened.append(args)
            if open_fails:raise OSError("UART unavailable")
            return 91
        def fake_exchange(fd,kind,sequence,deadline,metadata):
            self.assertAlmostEqual(deadline-clock[0],.20)
            requests.append((kind,sequence))
            metadata["request_types_attempted"].append(kind)
            metadata["reply_types_received"].append(0x80+kind)
            metadata["request_bytes_written"] += len(ns["_frame"](kind,sequence))
            if kind==1:return b"XGODUCK_MOTION_1"
            sequence_state[0]+=1
            if sequence_state[0] == reply_failure_at:
                raise self.Error('Serial reply timed out')
            values = {'sample':1000000+sequence_state[0]*20000,
                      'imu':1001000+sequence_state[0]*20000,
                      'online':1 if unsafe else 0, **(payload_changes or {})}
            return payload(ns,**values)
        def fake_emit(kind,expected,**values):
            if pipe_fails and kind=="state":raise BrokenPipeError("Mac disconnected")
            events.append({"kind":kind,"generation":expected["generation"],**values})
        holder_calls=[0]
        def holders(*args):
            holder_calls[0]+=1
            return [{"pid":42,"fd":"8"}] if holder_calls[0]==holder_stage else []
        ns.update(health_snapshot=fake_health,_holders=holders,
                  ControlChannel=FakeControl,
                  serial_proof=lambda *args:(types.SimpleNamespace(st_rdev=188),{}),
                  _raw_settings=lambda fd:{"hupcl":False},_exchange=fake_exchange,emit=fake_emit)
        modem_calls=[0]
        def modem(fd):
            modem_calls[0]+=1
            return {"dtr":False,"rts":modem_changes and modem_calls[0]>1}
        ns["_modem_bits"]=modem
        with mock.patch.object(ns["os"],"open",side_effect=fake_open), \
                mock.patch.object(ns["os"],"close",side_effect=lambda fd:closed.append(fd)), \
                mock.patch.object(ns["os"],"set_blocking"), \
                mock.patch.object(ns["os"],"getppid",side_effect=lambda:42 if not parent_gone or clock[0]<.02 else 1), \
                mock.patch.object(ns["fcntl"],"flock"),mock.patch.object(ns["fcntl"],"ioctl"), \
                mock.patch.object(ns["signal"],"signal"), \
                mock.patch.object(ns["time"],"monotonic",side_effect=lambda:clock[0]), \
                mock.patch.object(ns["time"],"sleep",side_effect=lambda seconds:clock.__setitem__(0,clock[0]+seconds)):
            result = ns["run_reader"](self.expected)
        return result, events, requests, opened, closed

    def test_native_health_worker_is_checked_each_cycle_and_joined_before_uart_close(self):
        checks=[]
        class Watch:
            def check(self):checks.append('check');return 0.
            def close(self):checks.append('close')
        factory=mock.Mock(return_value=Watch())
        result,events,requests,opened,closed=self.fake_run(health_factory=factory)
        self.assertEqual(result,0)
        factory.assert_called_once_with(self.expected,91)
        self.assertEqual(checks[-1],'close')
        self.assertEqual(checks.count('check'),len(requests)-1)
        self.assertTrue(events[-1]['normal']);self.assertEqual(closed,[91])
        class FailedWatch(Watch):
            def check(self):raise self_error('background USB identity changed')
        self_error=self.Error
        result,events,requests,opened,closed=self.fake_run(health_factory=lambda *args:FailedWatch())
        self.assertEqual(result,20);self.assertEqual(closed,[91])
        self.assertEqual(requests,[(1,1)])
        self.assertFalse(events[-1]['normal'])
        self.assertIn('background USB identity changed',events[-1]['reason'])

    def test_one_open_one_hello_explicit_end_close_and_monotonic_state_requests(self):
        result,events,requests,opened,closed=self.fake_run()
        self.assertEqual(result,0)
        self.assertEqual(len(opened),1)
        self.assertEqual(closed,[91])
        self.assertEqual(requests[0],(1,1))
        self.assertTrue(all(kind==2 for kind,_ in requests[1:]))
        self.assertEqual([sequence for _,sequence in requests],list(range(1,len(requests)+1)))
        self.assertEqual(events[0]["kind"],"ready")
        self.assertEqual(events[-1]["kind"],"closed")
        self.assertTrue(events[-1]["serial_closed"])
        self.assertTrue(events[-1]['normal'])
        self.assertEqual(events[-1]['reason'],'Feedback ended by Mac')
        self.assertFalse(any(event['kind']=='fault' for event in events))
        self.assertTrue(all(event["generation"]==self.expected["generation"] for event in events))
        self.assertGreater(events[-1]["query"]["state_records"],0)

    def test_first_unsafe_feedback_records_masks_before_gate_without_emitting_state(self):
        cases = (({'fault':1 << 4},'Unsafe armed/cookie/fault/torque'),
                 ({'torque':1 << 14},'Unsafe armed/cookie/fault/torque'),
                 ({'armed':1},'Firmware reports ARMED'),
                 ({'cookie':23},'nonzero arm cookie'))
        for changes,reason in cases:
            with self.subTest(changes=changes):
                self.ns=runtime()
                result,events,requests,opened,closed=self.fake_run(
                    payload_changes={'online':self.ns['ALL_MASK'],**changes})
                self.assertEqual(result,20)
                self.assertEqual(requests,[(1,1),(2,2)])
                self.assertEqual(len(opened),1)
                self.assertEqual(closed,[91])
                self.assertEqual([event['kind'] for event in events],['ready','fault','closed'])
                fault=events[1]
                self.assertIn(reason,fault['reason'])
                query=fault['query']
                self.assertEqual((query['state_requests'],query['state_records']),(1,0))
                self.assertEqual(query['last_feedback'],{
                    'sequence':2,'sample_us':1020000,'imu_us':1021000,
                    'online_mask':32767,'fault_mask':changes.get('fault',0),
                    'torque_mask':changes.get('torque',0),'armed':bool(changes.get('armed',0)),
                    'arm_cookie':changes.get('cookie',0),'imu_ok':True,'last_error':0})
                self.assertEqual(events[-1]['query']['last_feedback'],query['last_feedback'])
                self.assertTrue(events[-1]['serial_closed'])
                self.assertFalse(events[-1]['normal'])

    def test_scalar_diagnostics_survive_original_decoder_vector_rejection_without_nan(self):
        for changes in ({'accel':[float('nan'),0.,9.8]},
                        {'fault':32,'accel':[float('nan'),0.,9.8]},
                        {'torque':64,'accel':[float('inf'),0.,9.8]}):
            with self.subTest(changes=changes):
                self.ns=runtime()
                result,events,requests,_,closed=self.fake_run(payload_changes=changes)
                self.assertEqual(result,20)
                self.assertEqual(requests,[(1,1),(2,2)])
                self.assertEqual(closed,[91])
                self.assertFalse(any(event['kind']=='state' for event in events))
                diagnosis=events[1]['query']['last_feedback']
                self.assertIn('Invalid state field',events[1]['reason'])
                self.assertEqual(diagnosis['fault_mask'],changes.get('fault',0))
                self.assertEqual(diagnosis['torque_mask'],changes.get('torque',0))
                self.assertEqual(len(diagnosis),10)
                self.assertTrue(all(type(value) in (bool,int) for value in diagnosis.values()))
                json.dumps(diagnosis,allow_nan=False)

    def test_invalid_scalar_fields_are_labeled_without_valid_diagnostic_or_acceptance(self):
        for changes in ({'armed':2},{'imu_ok':2},{'fault':32768},{'last_error':6}):
            with self.subTest(changes=changes):
                self.ns=runtime()
                result,events,requests,_,closed=self.fake_run(payload_changes=changes)
                self.assertEqual(result,20)
                self.assertEqual(requests,[(1,1),(2,2)])
                self.assertEqual(closed,[91])
                self.assertNotIn('last_feedback',events[1]['query'])
                self.assertEqual(events[1]['query']['last_feedback_invalid'],'invalid_scalar_fields')
                self.assertFalse(any(event['kind']=='state' for event in events))
        self.assertEqual(self.ns['_feedback_summary'](b'not a state',2),
                         (None,'state_length_mismatch'))

    def test_timeout_diagnostic_cannot_reuse_previous_accepted_state(self):
        result,events,requests,_,closed=self.fake_run(reply_failure_at=2)
        self.assertEqual(result,20)
        self.assertEqual(requests,[(1,1),(2,2),(2,3)])
        self.assertEqual(closed,[91])
        states=[event for event in events if event['kind']=='state']
        self.assertEqual(len(states),1)
        fault=next(event for event in events if event['kind']=='fault')
        self.assertEqual((fault['query']['state_requests'],fault['query']['state_records']),(2,1))
        self.assertNotIn('last_feedback',fault['query'])
        self.assertNotIn('last_feedback_invalid',fault['query'])
        self.assertNotIn('last_feedback',events[-1]['query'])

    def test_last_feedback_summary_is_copied_and_only_latest_decoded_status_is_kept(self):
        last,invalid=self.ns['_feedback_summary'](payload(self.ns),2)
        self.assertIsNone(invalid)
        metadata={'request_types_attempted':self.ns['collections'].deque([1,2],maxlen=4),
                  'reply_types_received':self.ns['collections'].deque([0x81,0x82],maxlen=4),
                  'last_feedback':last}
        snapshot=self.ns['_query_summary'](metadata)
        metadata['last_feedback']['torque_mask']=8
        metadata['request_types_attempted'].append(2)
        self.assertEqual(snapshot['last_feedback']['fault_mask'],0)
        self.assertEqual(snapshot['last_feedback']['torque_mask'],0)
        self.assertEqual(snapshot['request_types_attempted'],[1,2])
        snapshot['last_feedback']['imu_ok']=False
        self.assertIs(metadata['last_feedback']['imu_ok'],True)
        self.ns=runtime()
        result,events,requests,_,_=self.fake_run()
        self.assertEqual(result,0)
        query=events[-1]['query']
        self.assertEqual(query['last_feedback']['sequence'],requests[-1][1])
        self.assertEqual(query['last_feedback']['sample_us'],1000000+query['state_requests']*20000)

    def test_fault_open_modem_health_and_stdout_fail_close_without_retry(self):
        for changes in ({"unsafe":True},{"open_fails":True},{"pipe_fails":True},
                        {"health_fails":True},{"modem_changes":True}):
            with self.subTest(changes=changes):
                self.ns=runtime()
                result,events,requests,opened,closed=self.fake_run(**changes)
                self.assertEqual(result,20)
                self.assertEqual(len(opened),1)
                self.assertEqual(closed,[] if changes.get("open_fails") else [91])
                self.assertEqual(events[-1]["kind"],"closed")
                self.assertTrue(events[-1]["serial_closed"])
                self.assertFalse(events[-1]["normal"])
                self.assertTrue(all(kind in (1,2) for kind,_ in requests))
                self.assertLessEqual(sum(kind==1 for kind,_ in requests),1)

    def test_end_before_open_is_normal_without_uart_and_no_control_health_failure_still_closes(self):
        result,events,requests,opened,closed=self.fake_run(end_at=0)
        self.assertEqual(result,0)
        self.assertEqual((requests,opened,closed),([],[],[]))
        self.assertEqual(events[-1]['reason'],'Feedback ended by Mac')
        self.assertTrue(events[-1]['serial_closed'])
        self.ns=runtime()
        result,events,requests,opened,closed=self.fake_run(end_at=None,health_fails=True)
        self.assertEqual(result,20)
        self.assertEqual(len(opened),1)
        self.assertEqual(closed,[91])
        self.assertIn('new USB fault',events[-1]['reason'])

    def test_control_faults_or_parent_disappearance_close_without_reconnect(self):
        for change in ({'control_problem':'EOF'}, {'control_problem':'partial timeout'},
                       {'control_problem':'malformed'}, {'parent_gone':True}):
            with self.subTest(change=change):
                self.ns=runtime()
                result,events,requests,opened,closed=self.fake_run(**change)
                self.assertEqual(result,20)
                self.assertEqual(len(opened),1)
                self.assertEqual(closed,[91])
                self.assertFalse(events[-1]['normal'])
                self.assertTrue(events[-1]['serial_closed'])
                self.assertLessEqual(sum(kind==1 for kind,_ in requests),1)

    def test_existing_or_racing_uart_holder_never_receives_a_request(self):
        for stage in (1,2):
            with self.subTest(stage=stage):
                self.ns=runtime()
                result,events,requests,opened,closed=self.fake_run(holder_stage=stage)
                self.assertEqual(result,20)
                self.assertEqual(requests,[])
                self.assertEqual(len(opened),0 if stage==1 else 1)
                self.assertEqual(closed,[] if stage==1 else [91])
                self.assertTrue(events[-1]["serial_closed"])

    def fake_rx_capture(self, arrivals=(), *, holder_stage=0, health_fail_at=None,
                        parent_gone_at=None, pipe_gone_at=None, uart_eof=False):
        ns = self.ns
        clock, holder_calls = [0.], [0]
        pending = list(arrivals)
        events, opened, closed, reads, waits = [], [], [], [], []
        def health(expected, fd=None):
            if health_fail_at is not None and clock[0] >= health_fail_at:
                raise self.Error('new USB/WiFi/storage fault')
            return {'kernel_log':expected['kernel_prefix'],'historical_faults_cleared':False}
        def holders(*args):
            holder_calls[0] += 1
            return [{'pid':99,'fd':'4'}] if holder_calls[0]==holder_stage else []
        def fake_select(readable,writable,exceptional,timeout):
            self.assertEqual((readable,writable,exceptional),([91],[],[91]))
            self.assertGreater(timeout,0)
            self.assertLessEqual(timeout,.05)
            waits.append(timeout)
            if pending and pending[0][0] <= clock[0]+timeout:
                clock[0] = max(clock[0],pending[0][0])+1e-8
                return ([91],[],[])
            clock[0] += timeout+1e-8
            return ([],[],[])
        def read(fd,count):
            reads.append((fd,count))
            self.assertEqual(fd,91)
            self.assertGreater(count,0)
            self.assertLessEqual(count,4096)
            if uart_eof:return b''
            return pending.pop(0)[1][:count]
        class FakeOutputPoll:
            def register(poll,fd,mask):pass
            def poll(poll,timeout):
                bad=pipe_gone_at is not None and clock[0]>=pipe_gone_at
                return [(1,ns['select'].POLLERR if bad else ns['select'].POLLOUT)]
        def forbidden(*args,**kwargs):
            raise AssertionError('RX capture attempted protocol framing/write/control')
        ns.update(health_snapshot=health, _holders=holders,
                  serial_proof=lambda *args:(types.SimpleNamespace(st_rdev=188),{}),
                  _raw_settings=lambda fd:{'hupcl':False},
                  _modem_bits=lambda fd:{'dtr':True,'rts':True},
                  emit=lambda kind,expected,**values:events.append(
                      {'kind':kind,'generation':expected['generation'],**values}),
                  _frame=forbidden,_exchange=forbidden,_write_once=forbidden,
                  ControlChannel=forbidden)
        with mock.patch.object(ns['os'],'open',side_effect=lambda *args:opened.append(args) or 91), \
                mock.patch.object(ns['os'],'close',side_effect=lambda fd:closed.append(fd)), \
                mock.patch.object(ns['os'],'read',side_effect=read), \
                mock.patch.object(ns['os'],'write',side_effect=forbidden), \
                mock.patch.object(ns['os'],'set_blocking'), \
                mock.patch.object(ns['os'],'getppid',side_effect=lambda:42 if parent_gone_at is None or clock[0]<parent_gone_at else 1), \
                mock.patch.object(ns['select'],'select',side_effect=fake_select), \
                mock.patch.object(ns['select'],'poll',side_effect=FakeOutputPoll), \
                mock.patch.object(ns['fcntl'],'flock'),mock.patch.object(ns['fcntl'],'ioctl'), \
                mock.patch.object(ns['signal'],'signal'), \
                mock.patch.object(ns['time'],'monotonic',side_effect=lambda:clock[0]):
            result=ns['run_rx_capture'](self.expected)
        return result,events,opened,closed,reads,waits,clock[0]

    def test_rx_factory_validates_profile_but_selects_only_diagnostic_entry(self):
        path, profile=self.reviewed_profile()
        source=feedback_stream.rx_capture_source(self.profile_expected(profile),
            reviewed_profile_path=path,reviewed_profile_sha256=feedback_stream.RECONNECT_PROFILE_SHA256)
        self.assertTrue(source.endswith('raise SystemExit(run_rx_capture(EXPECTED))\n'))
        compile(source,'<rx-diagnostic>','exec')
        with self.assertRaises(ValueError):feedback_stream.rx_capture_source(self.profile_expected(profile))

    def test_rx_capture_readonly_one_open_no_requests_raw_preserved_and_quiet_closes(self):
        data=b'ESP-ROM:esp32s3\r\n'+bytes((0,255,1))
        result,events,opened,closed,reads,waits,elapsed=self.fake_rx_capture([(0.,data)])
        self.assertEqual(result,0)
        self.assertEqual(len(opened),1)
        self.assertEqual(opened[0][1] & os.O_ACCMODE,os.O_RDONLY)
        self.assertEqual(closed,[91])
        self.assertEqual([event['kind'] for event in events],['rx_ready','rx_capture','closed'])
        capture=events[1]
        self.assertEqual(base64.b64decode(capture['raw_base64']),data)
        self.assertEqual(capture['sha256'],hashlib.sha256(data).hexdigest())
        self.assertEqual(capture['count'],len(data))
        self.assertFalse(capture['truncated'])
        self.assertTrue(events[-1]['serial_closed'])
        self.assertTrue(events[-1]['normal'])
        self.assertEqual(events[-1]['query']['request_bytes_written'],0)
        self.assertEqual(events[-1]['query']['request_types_attempted'],[])
        self.assertLess(elapsed,.151)
        self.assertTrue(all(event['generation']==self.expected['generation'] for event in events))

    def test_rx_capture_empty_byte_limit_and_continuous_time_limit_are_bounded(self):
        result,events,*rest=self.fake_rx_capture()
        self.assertEqual(result,0)
        self.assertTrue(events[-2]['empty'])
        self.ns=runtime()
        result,events,opened,closed,reads,waits,elapsed=self.fake_rx_capture([(0.,b'x'*4096),(.001,b'UNCONSUMED')])
        self.assertEqual(result,20)
        self.assertEqual(reads,[(91,4096)])
        self.assertEqual(events[-2]['count'],4096)
        self.assertTrue(events[-2]['truncated'])
        self.assertEqual(closed,[91])
        self.ns=runtime()
        result,events,opened,closed,reads,waits,elapsed=self.fake_rx_capture([(i*.04,b'x') for i in range(40)])
        self.assertEqual(result,0)
        self.assertLessEqual(elapsed,1.0001)
        self.assertLessEqual(len(reads),26)
        self.assertEqual(events[-1]['reason'],'RX capture time limit reached')
        self.assertEqual(closed,[91])

    def test_rx_capture_new_health_fault_lost_parent_pipe_eof_or_holder_close_without_retry(self):
        cases=({'health_fail_at':.25},{'health_fail_at':0.},
               {'parent_gone_at':.04},{'pipe_gone_at':.04},
               {'uart_eof':True},{'holder_stage':1},{'holder_stage':2})
        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                self.ns=runtime()
                result,events,opened,closed,reads,waits,elapsed=self.fake_rx_capture(
                    [(i*.04,b'x') for i in range(40)],**kwargs)
                self.assertEqual(result,20)
                before_open=kwargs.get('holder_stage')==1 or kwargs.get('health_fail_at')==0.
                self.assertEqual(len(opened),0 if before_open else 1)
                self.assertEqual(closed,[] if before_open else [91])
                self.assertTrue(events[-1]['serial_closed'])
                self.assertFalse(events[-1]['normal'])
                self.assertTrue(any(event['kind']=='fault' for event in events))
                self.assertEqual(events[-1]['query']['request_bytes_written'],0)


if __name__ == "__main__":
    unittest.main()
