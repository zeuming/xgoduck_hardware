"""Local guard/process tests.  No SSH, UART, motor command or remote access."""
import ast
import copy
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

import native_transports as native
import maintenance_transport as maintenance


class NativeTransportTests(unittest.TestCase):
    def partial_reader_run(self, *, changes=None, diagnostic_error=None):
        from test_feedback_stream import ReaderTests
        from test_feedback_fault_probe import payload
        harness = ReaderTests()
        harness.ns = {}
        exec(native.partial_servo_diagnostic_source(self.runtime_source), harness.ns)
        harness.Error = harness.ns['ProbeError']
        harness.expected = self.expected
        calls = []
        def exchange(fd, sequence, deadline, metadata):
            calls.append((fd, sequence, deadline - harness.ns['time'].monotonic()))
            metadata['request_types_attempted'].append(7)
            if diagnostic_error:
                raise harness.Error(diagnostic_error)
            metadata['reply_types_received'].append(0x87)
            return payload(harness.ns)
        harness.ns['_fault_exchange'] = exchange
        result = harness.fake_run(payload_changes=changes or {'online': 32766})
        return result, calls

    def test_partial_servo_snapshot_preserves_fault_and_actual_close_without_state_acceptance(self):
        (code, events, requests, opened, closed), calls = self.partial_reader_run()
        self.assertEqual(code, 20)
        self.assertEqual(requests, [(1, 1), (2, 2)])
        self.assertEqual(closed, [91]); self.assertEqual(len(opened), 1)
        self.assertEqual([event['kind'] for event in events], ['ready', 'fault', 'closed'])
        self.assertEqual(calls, [(91, 3, .10)])
        for event in events[1:]:
            self.assertEqual(event['reason'], 'Partial servo communication; reader will close')
            record = event['query']['partial_servo_diagnostic']
            self.assertEqual(record['rejected_online_mask'], 32766)
            self.assertFalse(record['snapshot_is_latched'])
            self.assertFalse(record['motion_authorized'])
            self.assertFalse(record['details']['fault_cause_determined'])
            self.assertEqual(event['query']['state_records'], 0)
        self.assertTrue(events[-1]['serial_closed']); self.assertFalse(events[-1]['normal'])

    def test_terminal_diagnostic_timeout_never_retries_or_replaces_original_fault(self):
        (code, events, requests, opened, closed), calls = self.partial_reader_run(
            diagnostic_error='Response deadline expired')
        self.assertEqual(code, 20); self.assertEqual(len(calls), 1)
        self.assertEqual(requests, [(1, 1), (2, 2)]); self.assertEqual(closed, [91])
        self.assertEqual(events[-1]['reason'], 'Partial servo communication; reader will close')
        record = events[-1]['query']['partial_servo_diagnostic']
        self.assertEqual(record['error'], 'Response deadline expired')
        self.assertNotIn('details', record)

    def test_other_faults_and_healthy_streams_do_not_request_terminal_diagnostics(self):
        for changes in ({'online': 32767}, {'online': 0},
                        {'online': 32766, 'torque': 1}, {'online': 32766, 'fault': 1},
                        {'online': 32766, 'last_error': 2}, {'online': 32767, 'armed': 1}):
            with self.subTest(changes=changes):
                (code, events, requests, opened, closed), calls = self.partial_reader_run(changes=changes)
                self.assertFalse(calls)
                self.assertNotIn('partial_servo_diagnostic', events[-1]['query'])
                self.assertEqual(closed, [91])

    def test_terminal_diagnostic_refuses_changed_open_uart_and_exhausted_sequence(self):
        from test_feedback_stream import state
        ns = {}; exec(native.partial_servo_diagnostic_source(self.runtime_source), ns)
        exchange = mock.Mock()
        ns['_fault_exchange'] = exchange
        ns['serial_proof'] = mock.Mock(side_effect=ns['ProbeError']('Connected UART changed'))
        for sequence in (2, 0xffffffff):
            metadata = {}
            ns['_native_partial_servo_diagnostic'](91, sequence, state(ns, online=32766),
                                                    self.expected, metadata)
            self.assertIn('error', metadata['partial_servo_diagnostic'])
        exchange.assert_not_called()

    def test_terminal_diagnostic_builder_rejects_unknown_acceptance_boundary(self):
        with self.assertRaisesRegex(ValueError, 'acceptance boundary'):
            native.partial_servo_diagnostic_source('def unknown_reader(): pass\n')

    def test_slow_full_health_does_not_block_sampler_and_closes_worker(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        def health(expected, fd):
            calls.append((expected, fd)); entered.set()
            if not release.wait(.4):
                raise ValueError('test did not release health check')
        ns = dict(time=time, HEALTH_INTERVAL=.005, health_snapshot=health, ProbeError=ValueError)
        exec(native.NATIVE_READER_HEALTH, ns)
        watch = ns['NativeReaderHealth']({'board':'same'}, 42)
        try:
            self.assertTrue(entered.wait(.2))
            for _ in range(100):
                self.assertIsInstance(watch.check(), float)
            self.assertEqual(calls, [({'board':'same'}, 42)])
            self.assertTrue(watch.thread.is_alive())
        finally:
            release.set(); watch.close()
        self.assertFalse(watch.thread.is_alive())

    def test_background_health_error_and_original_half_second_deadline_fail_closed(self):
        entered, release = threading.Event(), threading.Event()
        now = [0.]
        def health(expected, fd):
            entered.set(); release.wait(.4)
        ns = dict(time=types.SimpleNamespace(monotonic=lambda:now[0]),
                  HEALTH_INTERVAL=.005, health_snapshot=health, ProbeError=ValueError)
        exec(native.NATIVE_READER_HEALTH, ns)
        watch = ns['NativeReaderHealth']({}, 42)
        try:
            self.assertTrue(entered.wait(.2)); now[0] = .501
            with self.assertRaisesRegex(ValueError, 'timing guarantee'):
                watch.check()
        finally:
            release.set()
            with self.assertRaisesRegex(ValueError, 'took too long'):
                watch.close()
        self.assertFalse(watch.thread.is_alive())
        failed = threading.Event()
        def bad_health(expected, fd):
            failed.set(); raise ValueError('actual identity changed')
        ns.update(time=time, health_snapshot=bad_health)
        watch = ns['NativeReaderHealth']({}, 42)
        self.assertTrue(failed.wait(.2)); watch.thread.join(.2)
        with self.assertRaisesRegex(ValueError, 'actual identity changed'):watch.check()
        with self.assertRaisesRegex(ValueError, 'actual identity changed'):watch.close()

    def test_pinned_request_guard_checks_both_nodes_without_reopening_or_usb_walk(self):
        node=types.SimpleNamespace(st_dev=5,st_ino=77,st_rdev=188<<8,st_mode=stat.S_IFCHR|0o600)
        resolve=mock.Mock(return_value='/dev/ttyUSB0')
        check=native.pinned_uart_checker('/dev/ttyUSB0',(node.st_dev,node.st_ino,node.st_rdev),
            resolve_path=resolve,error_type=ValueError)
        with mock.patch.object(native.os,'stat',return_value=node) as path_stat, \
                mock.patch.object(native.os,'fstat',return_value=node) as descriptor_stat, \
                mock.patch('builtins.open',side_effect=AssertionError('No sysfs walk or reopen')):
            actual,receipt=check(self.expected,42)
        self.assertIs(actual,node)
        self.assertEqual(receipt['proof'],'same_character_node_and_open_descriptor')
        path_stat.assert_called_once_with('/dev/ttyUSB0',follow_symlinks=False)
        descriptor_stat.assert_called_once_with(42)
        resolve.assert_called_once_with(self.expected)

    def test_pinned_request_guard_rejects_unplug_reenumeration_descriptor_swap_and_symlink(self):
        node=types.SimpleNamespace(st_dev=5,st_ino=77,st_rdev=188<<8,st_mode=stat.S_IFCHR|0o600)
        resolve=mock.Mock(return_value='/dev/ttyUSB0')
        check=native.pinned_uart_checker('/dev/ttyUSB0',(node.st_dev,node.st_ino,node.st_rdev),
            resolve_path=resolve,error_type=ValueError)
        for side in ('path','descriptor'):
            for change in (dict(st_ino=78),dict(st_rdev=(188<<8)|1),dict(st_mode=stat.S_IFLNK|0o777)):
                other=copy.copy(node)
                for key,value in change.items():setattr(other,key,value)
                with self.subTest(side=side,change=change), \
                        mock.patch.object(native.os,'stat',return_value=other if side=='path' else node), \
                        mock.patch.object(native.os,'fstat',return_value=other if side=='descriptor' else node), \
                        self.assertRaisesRegex(ValueError,'node changed'):
                    check(self.expected,42)
        with mock.patch.object(native.os,'stat',side_effect=FileNotFoundError('unplugged')),self.assertRaises(FileNotFoundError):
            check(self.expected,42)
        resolve.return_value='/dev/ttyUSB1'
        with self.assertRaisesRegex(ValueError,'path or descriptor'):
            check(self.expected,42)

    @classmethod
    def setUpClass(cls):
        profile=maintenance.reader.EVIDENCE_DIR/'feedback-returned-esp32-reviewed-20261004-01.json'
        value=maintenance.reader.load_reviewed_profile(profile,maintenance.reader.RETURNED_ESP32_PROFILE_SHA256)
        expected={key:copy.deepcopy(value[key]) for key in
                  ('boot_id','cid','root_mount_line','model','serial','kernel_prefix','historical_faults')}
        expected['generation']=42
        cls.expected,_=maintenance.prepare_source(expected,profile,maintenance.reader.RETURNED_ESP32_PROFILE_SHA256)
        cls.runtime_source=maintenance.reader.runtime_source()

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name).resolve()
        self.root.chmod(0o700)
        info=self.root.stat()
        self.spec=dict(root_path=str(self.root),root_node=[info.st_dev,info.st_ino],
                       config_mount=['config exact'],shm_mount=['shm exact'])
        self.serial=types.SimpleNamespace(st_dev=5,st_ino=77,st_rdev=188<<8)
        self.guard=dict(EXPECTED=copy.deepcopy(self.expected),ram_health=mock.Mock(),
                       serial_proof=lambda expected:(self.serial,expected['serial']),
                       serial_path=lambda expected:'/dev/ttyUSB0',_holders=lambda *args:[],
                       NATIVE_RAM_GUARD=self.spec,NATIVE_RUNTIME_SOURCE=self.runtime_source)

    def tearDown(self):
        self.temp.cleanup()

    def usb_tail(self, start=10.):
        port,tty=self.expected['serial']['sysfs_name'],Path(self.expected['serial']['path']).name
        messages=[f'usb {port}: USB disconnect, device number 3',
            f'ch341-uart {tty}: ch341-uart converter now disconnected from {tty}',
            f'ch341 {port}:1.0: device disconnected',
            f'usb {port}: new full-speed USB device number 4 using xhci-hcd',
            f'usb {port}: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64',
            f'usb {port}: New USB device strings: Mfr=0, Product=2, SerialNumber=0',
            f'usb {port}: Product: USB Serial',
            f'ch341 {port}:1.0: ch341-uart converter detected',
            f'usb {port}: ch341-uart converter now attached to {tty}']
        return ''.join(f'[ {start+i*.001:.6f}] {message}\n' for i,message in enumerate(messages))

    def test_explicit_usb_recovery_preserves_history_and_rejects_the_next_fault(self):
        namespace={};exec(self.runtime_source,namespace)
        kernel=self.expected['kernel_prefix']+self.usb_tail()
        proof=native.usb_recovery_proof(kernel,self.expected,self.expected['serial'],namespace['validate_kernel'])
        self.assertEqual(proof['reconnect_count'],1)
        self.assertFalse(proof['historical_faults_cleared']);self.assertFalse(proof['motion_resumed'])
        recovered=native.install_usb_recovery(namespace,self.runtime_source,proof)
        for source in (recovered,maintenance._identity_runtime(recovered),native.connection.runtime_source(recovered)):
            parsed={};exec(source,parsed)
            result=parsed['validate_kernel'](kernel,self.expected)
            self.assertEqual(result['kernel_log'],kernel)
            self.assertEqual(result['historical_faults'],self.expected['historical_faults']+proof['faults'])
            for tail in (self.usb_tail(20), '[ 20.0] mmc1: error -5\n',
                         '[ 20.0] usb 1-1.7: USB disconnect, device number 5\n'):
                with self.assertRaises(parsed['ProbeError']):parsed['validate_kernel'](kernel+tail,self.expected)
        # A second explicit click can reconcile another complete return;
        # an existing connected guard above cannot silently do that itself.
        original={};exec(self.runtime_source,original)
        repeated=native.usb_recovery_proof(kernel+self.usb_tail(20),self.expected,
            self.expected['serial'],original['validate_kernel'])
        self.assertEqual(repeated['reconnect_count'],2);self.assertEqual(len(repeated['faults']),2)

    def test_usb_recovery_rejects_incomplete_foreign_corrupt_or_unrelated_history(self):
        namespace={};exec(self.runtime_source,namespace)
        prefix=self.expected['kernel_prefix'];tail=self.usb_tail()
        invalid=[prefix+tail.rsplit('\n',2)[0]+'\n',prefix+tail+'[ 20.0] mmc1: error -5\n',
            prefix+tail.replace('idVendor=1a86','idVendor=ffff'),
            prefix+tail.replace('bcdDevice= 2.64','bcdDevice= 9.99'),
            prefix+tail.replace('ttyUSB0','ttyUSB1'),prefix+tail.replace('using xhci-hcd','using foreign-driver'),
            prefix+tail+'[ 20.0] usb 1-1.7: USB disconnect, device number 5\n',
            prefix.split('\n',1)[1]+tail,prefix+tail.replace('[ 10.004000]','[ 9.004000]')]
        for kernel in invalid:
            with self.subTest(kernel=kernel[-130:]),self.assertRaises(ValueError):
                native.usb_recovery_proof(kernel,self.expected,self.expected['serial'],namespace['validate_kernel'])
        wrong=copy.deepcopy(self.expected);wrong['boot_id']='00000000-0000-0000-0000-000000000000'
        with self.assertRaises(namespace['ProbeError']):
            native.usb_recovery_proof(prefix+tail,wrong,wrong['serial'],namespace['validate_kernel'])

    def test_network_ready_logs_interleave_without_skipping_usb_stages_or_faults(self):
        namespace={};exec(self.runtime_source,namespace)
        tail=self.usb_tail().splitlines(True)
        interleaved=('[ 10.002200] IPv6: ADDRCONF(NETDEV_CHANGE): usb0: link becomes ready\n'
                     '[ 10.002400] IPv6: ADDRCONF(NETDEV_CHANGE): usb1: link becomes ready\n')
        after=('[ 20.000000] rwnx_send_sm_connect_req drv_vif_index:0 connect to example(5) channel:5200 auth_type:0\n'
               '[ 20.001000] get_txpwr_max:txpwr_max:18 \n'
               '[ 20.002000] IPv6: ADDRCONF(NETDEV_CHANGE): wlan0: link becomes ready\n'
               '[ 20.003000] deinit:macaddr:18,ce,df,79,6e,34\n'
               '[ 20.004000] reord_mac:18,ce,df,79,6e,34\n'
               '[ 20.005000] device wlan0 entered promiscuous mode\n'
               '[ 20.006000] device wlan0 left promiscuous mode\n')
        suffix=''.join(tail[:3])+interleaved+''.join(tail[3:])+after
        kernel=self.expected['kernel_prefix']+suffix
        proof=native.usb_recovery_proof(kernel,self.expected,self.expected['serial'],namespace['validate_kernel'])
        self.assertEqual(proof['reconnect_count'],1)
        self.assertEqual(proof['informational_lines'],(interleaved+after).splitlines())
        self.assertEqual(proof['kernel_prefix'],kernel)
        self.assertEqual(len(proof['faults']),1)
        self.assertFalse(proof['motion_resumed'])
        recovered=native.install_usb_recovery(namespace,self.runtime_source,proof)
        for source in (recovered,maintenance._identity_runtime(recovered),native.connection.runtime_source(recovered)):
            parsed={};exec(source,parsed)
            self.assertEqual(parsed['validate_kernel'](kernel,self.expected)['kernel_log'],kernel)
            with self.assertRaises(parsed['ProbeError']):
                parsed['validate_kernel'](kernel+'[ 30.0] mmc1: error -5\n',self.expected)
        original={};exec(self.runtime_source,original)
        invalid=[suffix.replace(tail[-1],''),suffix.replace(tail[3],''),
                 suffix.replace('usb0: link becomes ready','usb0: link becomes error'),
                 suffix.replace('txpwr_max:18','txpwr_max:-1'),
                 suffix.replace('[ 10.002200]','[ 9.002200]'),
                 suffix+'[ 30.0] arbitrary informational text\n',
                 suffix+'[ 30.0] sdio_err:<aicwf_sdio_hal_irqhandler,2855>: ret=-110, intstatus=ff\n']
        for value in invalid:
            with self.subTest(value=value[-180:]),self.assertRaises(ValueError):
                native.usb_recovery_proof(self.expected['kernel_prefix']+value,
                    self.expected,self.expected['serial'],original['validate_kernel'])

    def test_descriptor_error_requires_bounded_same_port_complete_enumeration(self):
        namespace={};exec(self.runtime_source,namespace)
        tail=self.usb_tail().splitlines(True)
        error='[ 10.003500] usb '+self.expected['serial']['sysfs_name']+': device descriptor read/64, error -32\n'
        suffix=''.join(tail[:4])+error+''.join(tail[4:])
        proof=native.usb_recovery_proof(self.expected['kernel_prefix']+suffix,
            self.expected,self.expected['serial'],namespace['validate_kernel'])
        self.assertEqual(proof['reconnect_count'],1)
        self.assertEqual(len(proof['faults']),2)
        for invalid in (suffix.replace(tail[-1],''),suffix.replace('error -32','error -71'),
                        suffix.replace(error,error*3),suffix.replace(error,error.replace('10.003500','13.003500')),
                        ''.join(tail)+error):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):
                native.usb_recovery_proof(self.expected['kernel_prefix']+invalid,
                    self.expected,self.expected['serial'],namespace['validate_kernel'])

    def test_network_info_still_requires_the_original_kernel_validator(self):
        line='[ 10.002200] IPv6: ADDRCONF(NETDEV_CHANGE): usb0: link becomes ready\n'
        tail=self.usb_tail().splitlines(True)
        kernel=self.expected['kernel_prefix']+''.join(tail[:3])+line+''.join(tail[3:])
        namespace={};exec(self.runtime_source,namespace)
        original=namespace['validate_kernel']
        def validate(value,expected):
            if line in value:raise ValueError('original classifier refuses this network line')
            return original(value,expected)
        with self.assertRaisesRegex(ValueError,'original classifier refuses'):
            native.usb_recovery_proof(kernel,self.expected,self.expected['serial'],validate)

    def test_same_port_hub_warning_requires_a_complete_return_and_retains_both_faults(self):
        namespace={};exec(self.runtime_source,namespace)
        serial=self.expected['serial'];hub,_,number=serial['sysfs_name'].rpartition('.')
        warning=f'[ 9.999000] usb {hub}-port{number}: disabled by hub (EMI?), re-enabling...\n'
        kernel=self.expected['kernel_prefix']+warning+self.usb_tail()
        proof=native.usb_recovery_proof(kernel,self.expected,serial,namespace['validate_kernel'])
        self.assertEqual(proof['reconnect_count'],1);self.assertEqual(len(proof['faults']),2)
        self.assertEqual(proof['faults'][0]['line'],warning.rstrip('\n'))
        self.assertFalse(proof['historical_faults_cleared']);self.assertFalse(proof['motion_resumed'])
        recovered=native.install_usb_recovery(namespace,self.runtime_source,proof)
        for source in (recovered,maintenance._identity_runtime(recovered),native.connection.runtime_source(recovered)):
            parsed={};exec(source,parsed)
            result=parsed['validate_kernel'](kernel,self.expected)
            self.assertEqual(result['historical_faults'],self.expected['historical_faults']+proof['faults'])
            with self.assertRaises(parsed['ProbeError']):parsed['validate_kernel'](kernel+warning,self.expected)
        original={};exec(self.runtime_source,original)
        repeated=native.usb_recovery_proof(kernel+self.usb_tail(20),self.expected,serial,original['validate_kernel'])
        self.assertEqual(repeated['reconnect_count'],2);self.assertEqual(len(repeated['faults']),3)

    def test_hub_warning_cannot_hide_incomplete_foreign_repeated_or_unrelated_faults(self):
        namespace={};exec(self.runtime_source,namespace)
        serial=self.expected['serial'];hub,_,number=serial['sysfs_name'].rpartition('.')
        warning=f'[ 9.999000] usb {hub}-port{number}: disabled by hub (EMI?), re-enabling...\n'
        tail=self.usb_tail();prefix=self.expected['kernel_prefix']
        invalid=[warning,warning+tail.rsplit('\n',2)[0]+'\n',warning+warning+tail,
            warning.replace(f'-port{number}:','-port99:')+tail,
            warning.replace(f'usb {hub}-', 'usb 9-9-')+tail,
            warning+tail+'[ 20.0] mmc1: error -5\n',
            tail+'[ 20.0] '+warning.split('] ',1)[1],
            tail.splitlines(True)[0]+warning+''.join(tail.splitlines(True)[1:])]
        for suffix in invalid:
            with self.subTest(tail=suffix[-120:]),self.assertRaises(ValueError):
                native.usb_recovery_proof(prefix+suffix,self.expected,serial,namespace['validate_kernel'])

    def test_setup_address_history_does_not_block_current_connection(self):
        namespace = {}; exec(self.runtime_source, namespace)
        port = self.expected['serial']['sysfs_name']
        kernel = self.expected['kernel_prefix'] + f'[ 29.421225] usb {port}: Device not responding to setup address.\n'
        namespace['bounded_dmesg'] = lambda: kernel
        namespace['serial_proof'] = lambda expected: (self.serial, expected['serial'])
        proof = native.prepare_usb_recovery(namespace, self.expected)
        native.install_usb_recovery(namespace, self.runtime_source, proof)
        namespace['validate_kernel'](kernel, self.expected)
        self.assertFalse(proof['motion_resumed'])
        with self.assertRaises(namespace['ProbeError']):
            namespace['validate_kernel'](kernel + '[ 30.0] mmc1: error -5\n', self.expected)
        original = {}; exec(self.runtime_source, original)
        with self.assertRaises(original['ProbeError']):
            native.current_usb_recovery_proof(kernel + '[ 30.0] mmc1: error -5\n',
                self.expected, self.expected['serial'], original['validate_kernel'])

    def test_warning_alone_requires_recovery_even_when_legacy_classifier_ignores_it(self):
        namespace={};exec(self.runtime_source,namespace)
        serial=self.expected['serial'];hub,_,number=serial['sysfs_name'].rpartition('.')
        warning=f'[ 9.999000] usb {hub}-port{number}: disabled by hub (EMI?), re-enabling...\n'
        kernel=self.expected['kernel_prefix']+warning
        namespace['bounded_dmesg']=lambda:kernel
        namespace['serial_proof']=lambda expected:(self.serial,serial)
        namespace['validate_kernel'](kernel,self.expected)  # Legacy classifier.
        self.assertIsNotNone(native.prepare_usb_recovery(namespace,self.expected))
        kernel+=self.usb_tail()
        self.assertEqual(native.prepare_usb_recovery(namespace,self.expected)['reconnect_count'],1)

    def test_ch341_device_removed_callback_requires_complete_same_port_return(self):
        namespace={};exec(self.runtime_source,namespace)
        serial=self.expected['serial'];port=serial['sysfs_name']
        callback=f'[ 10.000500] usb {port}: ch341_read_int_callback - usb_submit_urb failed: -19\n'
        tail=self.usb_tail().splitlines(True)
        complete=tail[0]+callback+''.join(tail[1:])
        kernel=self.expected['kernel_prefix']+complete
        proof=native.usb_recovery_proof(kernel,self.expected,serial,namespace['validate_kernel'])
        self.assertEqual(proof['reconnect_count'],1)
        self.assertEqual([f['line'] for f in proof['faults']],
                         [tail[0].rstrip('\n'),callback.rstrip('\n')])
        self.assertFalse(proof['historical_faults_cleared'])
        recovered=native.install_usb_recovery(namespace,self.runtime_source,proof)
        for source in (recovered,maintenance._identity_runtime(recovered),native.connection.runtime_source(recovered)):
            parsed={};exec(source,parsed)
            parsed['validate_kernel'](kernel,self.expected)
            for fault in (callback.replace('10.000500','20.000000'),
                          callback.replace('10.000500','20.000000').replace('-19','-71')):
                with self.assertRaises(parsed['ProbeError']):
                    parsed['validate_kernel'](kernel+fault,self.expected)
        invalid=[callback,tail[0]+callback,callback+self.usb_tail(),
                 tail[0]+callback+callback+''.join(tail[1:]),
                 complete.replace('-19','-71'),complete.replace('-19','-5'),
                 complete.replace(f'usb {port}: ch341_read', 'usb 9-9.9: ch341_read'),
                 tail[0]+tail[1]+callback+''.join(tail[2:]),
                 complete.replace('10.000500','10.101000')]
        for suffix in invalid:
            with self.subTest(suffix=suffix),self.assertRaises(ValueError):
                native.usb_recovery_proof(self.expected['kernel_prefix']+suffix,
                    self.expected,serial,namespace['_native_strict_validate_kernel'])
        original={};exec(self.runtime_source,original)
        original['bounded_dmesg']=lambda:self.expected['kernel_prefix']+callback
        original['serial_proof']=lambda expected:(self.serial,serial)
        self.assertIsNotNone(native.prepare_usb_recovery(original,self.expected))

    def test_source_has_only_explicit_scope_entry_and_pinned_serial(self):
        for scope,entry in (('imu','run_reader'),('joints','run_reader'),
                            ('metadata','check_connection_metadata'),('maintenance','run_maintenance')):
            expected,source=native.child_source(self.guard,self.root,79,scope=scope)
            self.assertEqual(expected['generation'],79)
            self.assertLess(len(source),native.MAX_SOURCE)
            tree=ast.parse(source)
            final=tree.body[-1]
            self.assertEqual(final.exc.args[0].func.id,entry)
            namespace={}
            assignments=[node for node in tree.body if isinstance(node,ast.Assign) and
                         any(isinstance(target,ast.Name) and target.id=='NATIVE_PIN' for target in node.targets)]
            exec(compile(ast.Module(body=assignments,type_ignores=[]),'<pin>','exec'),namespace)
            self.assertEqual(namespace['NATIVE_PIN']['serial_node'],[5,77,188<<8])
            if scope=='imu':
                self.assertIn(b'StateTracker = ImuCollectionStateTracker',source)
                self.assertIn(b'collection_scope',source)
            elif scope=='joints':
                self.assertNotIn(b'StateTracker = ImuCollectionStateTracker',source)
            self.assertEqual(b'reader_health_watcher = NativeReaderHealth' in source,
                             scope in ('imu','joints'))
            self.assertEqual(b'def _native_partial_servo_diagnostic(' in source, scope == 'joints')

    def test_source_refuses_owner_and_mismatched_ram_before_child(self):
        self.guard['_holders']=lambda *args:[{'pid':123}]
        with self.assertRaisesRegex(ValueError,'owner'):
            native.child_source(self.guard,self.root,79,scope='metadata')
        self.guard['_holders']=lambda *args:[]
        self.spec['root_node'][1]+=1
        with self.assertRaisesRegex(ValueError,'identity'):
            native.child_source(self.guard,self.root,79,scope='metadata')

    def test_exact_child_audit_rejects_unrelated_python_or_generation_args(self):
        source=b'print("local test")\n'
        command=native.child_command(source)
        self.assertTrue(native.allowed_child_command(command[0],command))
        for index,value in ((1,'-c'),(5,'print("unsafe")'),(6,'0'),(7,'x'*64),(8,'1')):
            changed=list(command);changed[index]=value
            self.assertFalse(native.allowed_child_command(changed[0],changed))
        self.assertFalse(native.allowed_child_command('/other/python',command))

    def tracker(self):
        namespace=dict(math=math,ALL_MASK=32767,ProbeError=ValueError)
        exec(native.IMU_TRACKER,namespace)
        return namespace['ImuCollectionStateTracker']()

    def state(self,**changes):
        value=dict(armed=False,arm_cookie=0,torque_mask=0,online_mask=0,fault_mask=32767,
                   last_error=0,imu_ok=True,accel_ms2=[0.,0.,9.81],gyro_dps=[0.,0.,0.],
                   sample_us=100000,imu_us=100000)
        value.update(changes)
        return value

    def test_imu_preserves_offline_partial_and_online_servo_masks(self):
        for online,fault in ((0,32767),(0,0),(2,32765),(32767,0),(32767,1)):
            value=self.state(online_mask=online,fault_mask=fault)
            before=copy.deepcopy(value)
            self.assertTrue(self.tracker().accept(value,1.))
            self.assertEqual(value,before)

    def test_imu_rejects_invalid_masks_or_motor_and_firmware_faults(self):
        for changes in (dict(online_mask=False),dict(online_mask='1'),dict(online_mask=-1),
                        dict(online_mask=32768),dict(fault_mask=True),dict(fault_mask=None),
                        dict(fault_mask=-1),dict(fault_mask=32768),
                        dict(armed=True),dict(arm_cookie=1),dict(torque_mask=1),
                        dict(last_error=1),dict(imu_ok=False)):
            with self.assertRaises(ValueError):self.tracker().accept(self.state(**changes),1.)

    def test_imu_repeats_are_not_fresh_and_frozen_clocks_close(self):
        tracker=self.tracker();state=self.state()
        self.assertTrue(tracker.accept(state,1.))
        self.assertFalse(tracker.accept(copy.deepcopy(state),1.1))
        with self.assertRaisesRegex(ValueError,'stopped advancing'):
            tracker.accept(copy.deepcopy(state),1.6)
        tracker=self.tracker();tracker.accept(state,1.)
        with self.assertRaisesRegex(ValueError,'reversed/reset'):
            tracker.accept(self.state(sample_us=10,imu_us=10),1.1)
        with self.assertRaisesRegex(ValueError,'stale'):
            self.tracker().accept(self.state(sample_us=700001),1.)

    def child_guard(self,*,authority=False):
        policy=self.root/'policy';policy.mkdir()
        calibration=policy/'calibration.json';calibration.write_text('{}\n');calibration.chmod(0o600)
        pin=dict(self.spec,serial_path='/dev/ttyUSB0',serial_node=[5,77,188<<8],
                 calibration_path=str(calibration),calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest())
        if authority:
            pin.update(calibration_authority_path=str(policy/'authority.json'),
                       calibration_lock_path=str(policy/'authority.lock'),
                       calibration_authority_key='ab'*64,calibration_revision=0)
            (policy/'authority.lock').write_text('');(policy/'authority.lock').chmod(0o600)
            self.authorize(pin,0)
        def bounded(path,limit):
            if path=='/proc/self/mountinfo':return 'x y 179:1 a b\nx y z a /dev/shm\n'
            return Path(path).read_text()
        pin['config_mount']=['x y 179:1 a b'];pin['shm_mount']=['x y z a /dev/shm']
        namespace=dict(os=os,stat=stat,time=time,json=json,hashlib=hashlib,ProbeError=ValueError,
                       fcntl=__import__('fcntl'),health_snapshot=lambda expected,fd=None:{'verified':True},
                       serial_proof=lambda expected,fd=None:(self.serial,{}),
                       serial_path=lambda expected:'/dev/ttyUSB0',bounded_read=bounded,
                       NATIVE_PIN=pin,NATIVE_PARENT=os.getppid())
        exec(native.CHILD_GUARD,namespace)
        return namespace,pin

    def authorize(self,pin,revision):
        digest=hashlib.sha256(Path(pin['calibration_path']).read_bytes()).hexdigest()
        value=dict(sha256=digest,revision=revision)
        message=json.dumps(value,sort_keys=True,separators=(',',':')).encode()
        value['mac']=hmac.new(bytes.fromhex(pin['calibration_authority_key']),message,hashlib.sha256).hexdigest()
        Path(pin['calibration_authority_path']).write_text(json.dumps(value))
        Path(pin['calibration_authority_path']).chmod(0o600)

    def test_guard_retains_node_and_fixed_ram_mounts(self):
        namespace,pin=self.child_guard()
        self.assertEqual(namespace['health_snapshot']({}),{'verified':True})
        self.serial.st_ino+=1
        with self.assertRaisesRegex(ValueError,'UART differs'):namespace['serial_proof']({})
        self.serial.st_ino-=1
        pin['config_mount']=['wrong']
        with self.assertRaisesRegex(ValueError,'mount changed'):namespace['health_snapshot']({})

    def test_authorized_calibration_commit_is_accepted_without_changing_identity(self):
        namespace,pin=self.child_guard(authority=True)
        namespace['health_snapshot']({})
        Path(pin['calibration_path']).write_text('{"saved":true}\n')
        self.authorize(pin,1)
        self.assertEqual(namespace['health_snapshot']({}),{'verified':True})
        record=json.loads(Path(pin['calibration_authority_path']).read_text());record['revision']=2
        Path(pin['calibration_authority_path']).write_text(json.dumps(record))
        with self.assertRaisesRegex(ValueError,'MAC differs'):namespace['health_snapshot']({})

    def test_unsigned_calibration_mutation_and_authority_replay_rejected(self):
        namespace,pin=self.child_guard(authority=True)
        Path(pin['calibration_path']).write_text('{"tampered":true}\n')
        with self.assertRaisesRegex(ValueError,'authorized commit'):namespace['health_snapshot']({})
        self.authorize(pin,0);pin['calibration_revision']=1
        with self.assertRaisesRegex(ValueError,'record differs'):namespace['health_snapshot']({})

    def test_actual_local_child_exchange_and_exit_are_truthful(self):
        source=("import json,os,sys\n"
                "print(json.dumps({'kind':'ready'}),flush=True)\n"
                "raw=os.read(sys.stdin.fileno(),4096)\n"
                "print(json.dumps({'kind':'closed','control':json.loads(raw)}),flush=True)\n").encode()
        channel=native.NativeLocalChannel(source,self.root)
        try:
            self.assertEqual(channel.receive(time.monotonic()+2.)['kind'],'ready')
            channel.send({'kind':'close'},.2)
            value=channel.receive(time.monotonic()+2.)
            self.assertEqual(value['control'],{'kind':'close'})
            with self.assertRaises(EOFError):channel.receive(time.monotonic()+2.)
            proof=channel.finish()
            self.assertTrue(proof['process_exited']);self.assertEqual(proof['process_exit_code'],0)
            self.assertEqual(proof['transport'],'local');self.assertFalse(proof['ssh_started'])
            self.assertNotIn('ssh_exit_code',proof)
        finally:channel.finish()

    def test_actual_child_exit_without_closed_does_not_fabricate_uart_proof(self):
        channel=native.NativeLocalChannel(b'raise SystemExit(20)\n',self.root)
        try:
            with self.assertRaises(EOFError):channel.receive(time.monotonic()+2.)
            proof=channel.finish()
            self.assertEqual(proof['process_exit_code'],20)
            self.assertNotIn('serial_closed',proof)
        finally:channel.finish()

    def test_exact_source_boundary_preserves_pipelined_end_control(self):
        source=b'import os,sys\nsys.stdout.buffer.write(os.read(0,4096))\n'
        child=subprocess.Popen(native.child_command(source),stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        output,error=child.communicate(source+b'END_FEEDBACK\n',timeout=2.)
        self.assertEqual(child.returncode,0);self.assertEqual(error,b'')
        self.assertEqual(output,b'END_FEEDBACK\n')

    def test_maintenance_local_wire_contract_and_closed_remain_distinct(self):
        expected=copy.deepcopy(self.expected)
        ready=dict(kind='ready',generation=42,scope='maintenance',
                   allowed_requests=list(maintenance.ALLOWED_REPLIES),
                   identity=dict(expected,kernel_log=expected['kernel_prefix']),
                   query=dict(serial_open_attempts=1,exclusive=True,automatic_retry=False,
                              board_files_written_by_transport=False))
        closed=dict(kind='closed',generation=42,serial_closed=True,normal=True,
                    query=dict(serial_open_attempts=1,automatic_retry=False,
                               board_files_written_by_transport=False))
        class Channel:
            def __init__(self,*args):self.records=[ready];self.sent=[]
            def receive(self,deadline):
                if not self.records:raise TimeoutError('bounded test')
                return self.records.pop(0)
            def send(self,value,timeout):
                self.sent.append(value)
                if value.get('kind')=='close':self.records.append(closed)
                else:self.records.append(dict(kind='response',generation=42,sequence=value['sequence'],
                    type=136,payload=__import__('base64').b64encode(b'XGODUCK_MAINT_1').decode()))
            def finish(self):return dict(transport='local',ssh_started=False,process_exited=True,
                                        process_exit_code=0,stderr='')
            observe_exit=finish
        transport=native.NativeMaintenanceTransport(expected,b'# test no hardware\n',self.root,
            validation_source=self.runtime_source,channel_factory=Channel)
        self.assertEqual(transport.request(8,b'',136,.3),b'XGODUCK_MAINT_1')
        proof=transport.close()
        self.assertTrue(proof['serial_closed']);self.assertTrue(proof['process_exited'])
        self.assertEqual(proof['remote_closed_report'],closed)
        self.assertEqual(proof['transport'],'local');self.assertNotIn('ssh_exit_code',proof)

    def test_child_rejects_replayed_older_signed_calibration_revision(self):
        namespace,pin=self.child_guard(authority=True)
        namespace['health_snapshot']({})
        self.authorize(pin,2);namespace['health_snapshot']({})
        self.authorize(pin,1)
        with self.assertRaisesRegex(ValueError,'record differs'):namespace['health_snapshot']({})

    def test_maintenance_preflight_refusal_never_launches_child(self):
        def refuse():raise ValueError('fixed kernel guard refused')
        factory=native.NativeLocalTransports(refuse,self.root,channel_factory=mock.Mock())
        with self.assertRaises(maintenance.MaintenancePreflightError) as failure:factory.maintenance_factory()
        self.assertTrue(failure.exception.close_report['never_opened'])
        self.assertEqual(failure.exception.close_report['serial_open_attempts'],0)
        self.assertEqual(failure.exception.close_report['transport'],'local')
        factory.channel_factory.assert_not_called()

    def test_reader_preflight_failure_marks_only_current_unstarted_attempt(self):
        def refuse():raise ValueError('fixed kernel refused')
        factory=native.NativeLocalTransports(refuse,self.root)
        session=types.SimpleNamespace(feedback_scope='imu',generation=42,process=None,
                                     lock=__import__('threading').RLock(),
                                     _reader_preflight_failure_generation=None)
        with self.assertRaisesRegex(ValueError,'fixed kernel'):
            factory.launch_reader(session,42,self.root)
        self.assertEqual(session._reader_preflight_failure_generation,42)
        record=json.loads((self.root/'local-preflight-failure.json').read_text())
        self.assertFalse(record['process_started']);self.assertEqual(record['serial_open_attempts'],0)
        self.assertEqual(record['stage'],'native_reader_preflight')
        self.assertGreaterEqual(record['preflight_elapsed_seconds'],0.)
        self.assertEqual(record['failure_stack'][-1]['function'],'refuse')
        self.assertTrue(all(set(frame)=={'file','function','line'} for frame in record['failure_stack']))
        session._reader_preflight_failure_generation=None
        with self.assertRaises(ValueError):factory.launch_reader(session,43,self.root)
        self.assertIsNone(session._reader_preflight_failure_generation)

    def test_reader_launch_inherits_audit_environment_without_explicit_env(self):
        import feedback_calibration_server as host
        factory=native.NativeLocalTransports(lambda:self.guard,self.root)
        session=types.SimpleNamespace(feedback_scope='imu')
        with mock.patch.object(host,'run_reader') as launch:
            factory.launch_reader(session,42,self.root)
        args=launch.call_args.args
        callback=launch.call_args.kwargs['launch_preflight']
        command,environment,askpass=callback(args[0],args[1],args[2],args[3],42)
        self.assertIsNone(environment)
        self.assertTrue(native.allowed_child_command(command[0],command))
        self.assertEqual(askpass.parent,self.root)

    def test_preflight_stack_bounds_callsites_and_excludes_source_locals_secrets(self):
        private_hmac_key='df'*64
        def nested(depth):
            secret_in_local=private_hmac_key
            if depth:
                return nested(depth-1)
            raise ValueError('fixed guard deadline expired')
        try:
            nested(20)
        except ValueError as error:
            stack=native.preflight_stack(error)
        self.assertEqual(len(stack),8)
        self.assertTrue(all(set(frame)=={'file','function','line'} for frame in stack))
        self.assertTrue(all(frame['file']=='test_native_transports.py' and frame['function']=='nested'
                            and type(frame['line']) is int and frame['line']>0 for frame in stack))
        raw=json.dumps(stack)
        self.assertNotIn(private_hmac_key,raw)
        self.assertNotIn('secret_in_local',raw)
        self.assertNotIn(str(self.root),raw)

    def holder_fixture(self,body="return [{'pid':123,'fd':'4'}]"):
        return ("class ProbeError(RuntimeError): pass\n"
                "def _check_deadline(deadline):\n"
                "    if time.monotonic()>=deadline: raise ProbeError('Overall probe deadline expired')\n"
                "def _holders(rdev,deadline):\n    "+body+"\n")

    def prepared_holder_fixture(self,body="return [{'pid':123,'fd':'4'}]"):
        source=self.holder_fixture(body)
        native.prepare_holder_runtime(source)
        return source

    def test_holder_preparation_is_cpu_only_exact_code_and_hash_evidence(self):
        source=self.holder_fixture("raise RuntimeError('must not execute while preparing')")
        source+="\nraise RuntimeError('module entry must not execute while preparing')\n"
        with mock.patch.object(native.os,'open',side_effect=AssertionError('no device opens')) as opens,\
                mock.patch.object(native,'NativeLocalChannel') as child,\
                mock.patch.object(native.time,'monotonic',side_effect=AssertionError('no deadline exists')) as clock:
            proof=native.prepare_holder_runtime(source)
        opens.assert_not_called();child.assert_not_called();clock.assert_not_called()
        helper=native._holder_runtime(source)
        self.assertEqual(proof['kind'],'native_holder_runtime_prepared')
        self.assertEqual(proof['base_source_sha256'],hashlib.sha256(source.encode()).hexdigest())
        self.assertEqual(proof['helper_source_sha256'],hashlib.sha256(helper.encode()).hexdigest())
        self.assertEqual(proof['helper_source_bytes'],len(helper.encode()))
        self.assertEqual(proof['helper_names'],['ProbeError','_check_deadline','_holders'])
        self.assertEqual(proof['serial_open_attempts'],0)
        self.assertFalse(proof['owners_cached'])
        self.assertFalse(proof['board_files_written'])
        self.assertFalse(proof['automatic_retry'])
        self.assertNotIn('module entry',helper)

    def test_unprepared_holder_scan_fails_without_ast_work_child_or_deadline_extension(self):
        source=self.holder_fixture()+"\n# distinct unprepared digest\n"
        channel=mock.Mock()
        with mock.patch.object(native.ast,'parse',side_effect=AssertionError('AST outside deadline only')) as parse,\
                mock.patch.object(native.time,'monotonic',return_value=10.):
            with self.assertRaisesRegex(ValueError,'prepared before its absolute scan deadline'):
                native.isolated_holders(source,188<<8,10.2,self.root,channel_factory=channel)
        parse.assert_not_called();channel.assert_not_called()

    def test_prepared_holder_scans_reuse_only_code_and_collect_new_owner_inventories(self):
        source=self.prepared_holder_fixture()
        channel,instances=self.holder_channel([self.holder_record()])
        with mock.patch.object(native.ast,'parse',side_effect=AssertionError('no AST in scan')) as parse,\
                mock.patch.object(native.time,'monotonic',return_value=10.):
            first=native.isolated_holders(source,188<<8,10.2,self.root,channel_factory=channel)
            other,other_instances=self.holder_channel([self.holder_record(owners=[{'pid':456,'fd':'7'}])])
            second=native.isolated_holders(source,188<<8,10.2,self.root,channel_factory=other)
            repeated=native.prepare_holder_runtime(source)
        parse.assert_not_called()
        self.assertEqual(first,[{'pid':123,'fd':'4'}])
        self.assertEqual(second,[{'pid':456,'fd':'7'}])
        self.assertEqual(len(instances)+len(other_instances),2)
        self.assertEqual(instances[0].deadlines,[10.2,10.2])
        self.assertEqual(other_instances[0].deadlines,[10.2,10.2])
        self.assertFalse(repeated['owners_cached'])

    def holder_record(self,deadline=10.2,**changes):
        result=dict(kind='holders',parent_pid=os.getpid(),rdev=188<<8,deadline=deadline,
                    serial_open_attempts=0,serial_bytes_written=0,board_files_written=False,
                    automatic_retry=False,owners=[{'pid':123,'fd':'4'}],observed_monotonic=10.1)
        result.update(changes)
        return result

    def holder_channel(self,records,exit_code=0):
        instances=[]
        class Channel:
            def __init__(current,source,directory,**kwargs):
                current.source=source;current.kwargs=kwargs;current.records=list(records)
                current.process=types.SimpleNamespace(poll=lambda:exit_code)
                current.deadlines=[];current.finished=0;instances.append(current)
            def receive(current,deadline):
                current.deadlines.append(deadline)
                if not current.records:raise EOFError('actual test EOF')
                value=current.records.pop(0)
                if isinstance(value,BaseException):raise value
                return value
            def observe_exit(current):
                return dict(transport='local',ssh_started=False,process_exited=True,
                            process_exit_code=exit_code,stderr='')
            def finish(current):
                current.finished+=1;return current.observe_exit()
        return Channel,instances

    def test_isolated_holders_preserves_original_absolute_deadline_and_owner_list(self):
        channel,instances=self.holder_channel([self.holder_record()])
        directory=self.root/'holder-evidence'
        source=self.prepared_holder_fixture()
        with mock.patch.object(native.time,'monotonic',return_value=10.):
            owners=native.isolated_holders(source,188<<8,10.2,directory,channel_factory=channel)
        self.assertEqual(owners,[{'pid':123,'fd':'4'}])
        self.assertEqual(directory.stat().st_mode&0o777,0o700)
        self.assertEqual(instances[0].kwargs['absolute_deadline'],10.2)
        self.assertEqual(instances[0].deadlines,[10.2,10.2])
        self.assertIn(b'NATIVE_HOLDER_DEADLINE=10.2',instances[0].source)
        record=json.loads(next(directory.glob('holder-check-*/result.json')).read_text())
        self.assertEqual(record['process']['process_exit_code'],0)
        self.assertEqual(record['serial_open_attempts'],0)

    def test_isolated_holders_expired_deadline_never_launches_or_resets_budget(self):
        channel=mock.Mock()
        with mock.patch.object(native.time,'monotonic',return_value=10.3):
            with self.assertRaisesRegex(TimeoutError,'before launch'):
                native.isolated_holders(self.holder_fixture(),188<<8,10.2,self.root,channel_factory=channel)
        channel.assert_not_called()
        with mock.patch.object(native.time,'monotonic',return_value=10.):
            with self.assertRaisesRegex(ValueError,'unchanged 0.2'):
                native.isolated_holders(self.holder_fixture(),188<<8,11.,self.root,channel_factory=channel)
        channel.assert_not_called()

    def test_isolated_holders_rejects_trailing_missing_eof_bad_exit_and_schema(self):
        cases=([self.holder_record(),{'extra':True}],0),([self.holder_record(),TimeoutError('no EOF')],0),(
            [self.holder_record()],20),([self.holder_record(serial_open_attempts=1)],0),(
            [self.holder_record(deadline=10.21)],0),([self.holder_record(owners=[{'pid':1,'fd':'04'}])],0)
        source=self.prepared_holder_fixture()
        for records,code in cases:
            channel,instances=self.holder_channel(records,code)
            with self.subTest(records=records,code=code),mock.patch.object(native.time,'monotonic',return_value=10.):
                with self.assertRaises((ValueError,TimeoutError)):
                    native.isolated_holders(source,188<<8,10.2,self.root,channel_factory=channel)
            self.assertEqual(len(instances),1)
            self.assertEqual(instances[0].finished,1)

    def test_isolated_holders_rejects_receipt_observed_after_original_deadline(self):
        channel,_=self.holder_channel([self.holder_record()])
        source=self.prepared_holder_fixture()
        with mock.patch.object(native.time,'monotonic',side_effect=[10.,10.1,10.21,10.22]):
            with self.assertRaisesRegex(TimeoutError,'completed receipt'):
                native.isolated_holders(source,188<<8,10.2,self.root,channel_factory=channel)

    def test_isolated_holders_copies_exact_reviewed_helpers_without_uart_openers(self):
        native.prepare_holder_runtime(self.runtime_source)
        source=native._isolated_holder_source(self.runtime_source,188<<8,10.2,os.getpid()).decode()
        original=ast.parse(self.runtime_source);actual=ast.parse(source)
        for name in ('ProbeError','_check_deadline','_holders'):
            left=next(node for node in original.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name==name)
            right=next(node for node in actual.body if isinstance(node,(ast.FunctionDef,ast.ClassDef)) and node.name==name)
            self.assertEqual(ast.dump(left),ast.dump(right))
        functions={node.name for node in actual.body if isinstance(node,ast.FunctionDef)}
        self.assertEqual(functions,{'_check_deadline','_holders','isolated_holder_main'})
        self.assertFalse(any(isinstance(node,ast.Attribute) and node.attr in ('open','ioctl','flock')
                             for node in ast.walk(actual)))
        self.assertNotIn('calibration_authority_key',source)

    def test_actual_isolated_holder_child_returns_real_clean_exit_without_uart(self):
        source=self.prepared_holder_fixture()
        deadline=time.monotonic()+.2
        owners=native.isolated_holders(source,188<<8,deadline,self.root/'holder-evidence')
        self.assertEqual(owners,[{'pid':123,'fd':'4'}])
        record=json.loads(next(self.root.glob('holder-evidence/holder-check-*/result.json')).read_text())
        self.assertEqual(record['record']['deadline'],deadline)
        self.assertLess(record['record']['observed_monotonic'],deadline)
        self.assertTrue(record['process']['process_exited'])
        self.assertEqual(record['process']['process_exit_code'],0)
        self.assertFalse(record['process']['ssh_started'])

    def test_actual_isolated_holder_child_cannot_return_owners_after_original_deadline(self):
        source=self.prepared_holder_fixture("time.sleep(.25); return []")
        deadline=time.monotonic()+.2
        with self.assertRaisesRegex(TimeoutError,'original absolute deadline'):
            native.isolated_holders(source,188<<8,deadline,self.root/'holder-evidence')
        record=json.loads(next(self.root.glob('holder-evidence/holder-check-*/result.json')).read_text())
        self.assertIsNotNone(record['error'])
        self.assertTrue(record['process']['process_exited'])
        self.assertEqual(record['serial_open_attempts'],0)


if __name__=='__main__':unittest.main()
