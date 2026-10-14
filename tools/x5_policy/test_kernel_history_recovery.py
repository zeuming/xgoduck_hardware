"""Kernel-ring retention tests: no network, serial opens or motor authority."""
import copy
import base64
import gzip
import hashlib
import json
import os
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

import persistent_boot as boot
import native_service as service
import native_transports as native
import feedback_stream
import feedback_connection_check as connection
import maintenance_transport as maintenance


def history(count=90):
    return '[    0.000000] Booting Linux on physical CPU 0x00\n'+''.join(
        f'[ {i:.6f}] ordinary boot record {i}\n' for i in range(1,count))


def metadata(kernel):
    return dict(boot_id='71d1b708-a0cd-4a02-8356-eddbfb39dc49',cid=boot.CARD,model=boot.MODEL,
        root_mount_line='21 1 179:2 / / rw,relatime - ext4 /dev/root rw',
        swaps='Filename Type Size Used Priority\n',kernel_log=kernel)


def usb_tail(start=100):
    port='1-1.4';tty='ttyUSB0'
    messages=[f'usb 1-1-port4: disabled by hub (EMI?), re-enabling...',
        f'usb {port}: USB disconnect, device number 4',
        f'ch341-uart {tty}: ch341-uart converter now disconnected from {tty}',
        f'ch341 {port}:1.0: device disconnected',
        f'usb {port}: new full-speed USB device number 5 using xhci-hcd',
        f'usb {port}: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64',
        f'usb {port}: New USB device strings: Mfr=0, Product=2, SerialNumber=0',
        f'usb {port}: Product: USB Serial',
        f'ch341 {port}:1.0: ch341-uart converter detected',
        f'usb {port}: ch341-uart converter now attached to {tty}']
    return ''.join(f'[ {start+i*.01:.6f}] {message}\n' for i,message in enumerate(messages))


class KernelHistoryTests(unittest.TestCase):
    def test_rotated_prefix_restored_byte_for_byte_without_losing_tail(self):
        old=history();full=old+'[ 95.000000] later harmless event\n'
        for dropped in (1,3,20,26):
            raw='\n'.join(full.splitlines()[dropped:])+'\n'
            self.assertEqual(boot.restore_kernel_history(raw,old),full)
        self.assertEqual(boot.restore_kernel_history(full,old),full)

    def test_gap_short_overlap_edit_reorder_or_truncation_rejected(self):
        old=history();raw='\n'.join(old.splitlines()[3:])+'\n'
        invalid=[history(3),raw.replace('record 4','changed 4'),
            raw.replace('[ 5.000000]','[ 2.000000]'),raw.rstrip(),
            '\n'.join(old.splitlines()[-10:])+'\n',
            raw+'[ 1.000000] time regressed\n',raw+'untimestamped\n',
            '[ 200.000000] gap after entire retained history\n']
        for value in invalid:
            with self.subTest(value=value[:40]),self.assertRaises(ValueError):
                boot.restore_kernel_history(value,old)

    def test_retained_multiline_continuations_are_matched_exactly(self):
        old=history().replace('ordinary boot record 12\n','ordinary boot record 12\nretained continuation\n\n')
        raw='\n'.join(old.splitlines()[3:])+'\n'+usb_tail()
        self.assertEqual(boot.restore_kernel_history(raw,old),old+usb_tail())
        with self.assertRaises(ValueError):
            boot.restore_kernel_history(raw.replace('retained continuation','changed continuation'),old)

    def test_new_cfg80211_multiline_notice_is_preserved_with_exact_anchor(self):
        old = history()
        raw = '\n'.join(old.splitlines()[3:]) + '\n'
        notice = ('[ 100.000000] ieee80211 phy0: \n'
                  '               *******************************************************\n'
                  '               ** CAUTION: USING PERMISSIVE CUSTOM REGULATORY RULES **\n'
                  '               *******************************************************\n')
        self.assertEqual(boot.restore_kernel_history(raw + notice, old), old + notice)
        for invalid in (notice.replace('ieee80211 phy0:', 'unrelated:'),
                        notice.replace('CUSTOM', 'ALTERED'),
                        notice + 'unframed error continuation\n',
                        notice + '[ 1.000000] time regressed\n'):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                boot.restore_kernel_history(raw + invalid, old)
        source = feedback_stream.runtime_source() + '\n' + Path(boot.__file__).read_text()
        ns = {}; exec(source, ns)
        expected = ns['boot_expected']({}, metadata(old), ns['classify_feedback_log'])
        self.assertEqual(ns['validate_kernel'](raw + notice, expected)['kernel_log'], old + notice)
        with self.assertRaises(ns['ProbeError']):
            ns['validate_kernel'](raw + notice + '[ 101.000000] mmc1: error -5\n', expected)

    def test_restoring_prefix_does_not_approve_new_faults(self):
        source=feedback_stream.runtime_source()+'\n'+Path(boot.__file__).read_text()
        ns={};exec(source,ns)
        old=history();expected=ns['boot_expected']({},metadata(old),ns['classify_feedback_log'])
        raw='\n'.join(old.splitlines()[3:])+'\n'
        self.assertEqual(ns['validate_kernel'](raw,expected)['kernel_log'],old)
        for suffix in (usb_tail(),'[ 100.000000] mmc1: error -5\n'):
            with self.assertRaises(ns['ProbeError']):ns['validate_kernel'](raw+suffix,expected)
        bad=copy.deepcopy(expected);bad['native_persistent_boot']['boot_id']='another'
        with self.assertRaises(ns['ProbeError']):ns['validate_kernel'](raw,bad)

    def test_complete_usb_recovery_across_rotation_and_child_source_filters(self):
        source=feedback_stream.runtime_source()+'\n'+Path(boot.__file__).read_text()
        ns={};exec(source,ns)
        old=history();expected=ns['boot_expected']({},metadata(old),ns['classify_feedback_log'])
        expected['serial']=dict(sysfs_name='1-1.4',path='/dev/ttyUSB0',vid='1a86',pid='7523',serial_number='')
        full=old+usb_tail()+usb_tail(200);raw='\n'.join(full.splitlines()[3:])+'\n'
        ns['bounded_dmesg']=lambda:raw
        ns['serial_proof']=lambda e:(None,expected['serial'])
        proof=native.prepare_usb_recovery(ns,expected)
        self.assertEqual(proof['reconnect_count'],2)
        self.assertEqual(proof['kernel_prefix'],full)
        self.assertFalse(proof['historical_faults_cleared'])
        recovered=native.install_usb_recovery(ns,source,proof)
        self.assertEqual(ns['bounded_dmesg'](),full)
        for filtered in (recovered,connection.runtime_source(recovered),maintenance._identity_runtime(recovered)):
            parsed={};exec(filtered,parsed)
            result=parsed['validate_kernel'](raw,expected)
            self.assertEqual(result['kernel_log'],full)
            self.assertEqual(len(result['historical_faults']),4)
            for suffix in (usb_tail(300),'[ 300.000000] mmc1: error -5\n'):
                with self.assertRaises(parsed['ProbeError']):parsed['validate_kernel'](raw+suffix,expected)

    def test_incomplete_or_foreign_usb_stays_blocked_after_restoration(self):
        source=feedback_stream.runtime_source()+'\n'+Path(boot.__file__).read_text()
        for suffix in (usb_tail().rsplit('\n',2)[0]+'\n',usb_tail().replace('ttyUSB0','ttyUSB1'),
                       usb_tail()+'[ 210.000000] mmc1: error -5\n'):
            ns={};exec(source,ns);old=history();e=ns['boot_expected']({},metadata(old),ns['classify_feedback_log'])
            e['serial']=dict(sysfs_name='1-1.4',path='/dev/ttyUSB0',vid='1a86',pid='7523',serial_number='')
            ns['bounded_dmesg']=lambda:'\n'.join(old.splitlines()[3:])+'\n'+suffix
            ns['serial_proof']=lambda expected:(None,e['serial'])
            with self.assertRaises(ValueError):native.prepare_usb_recovery(ns,e)

    def test_private_same_boot_archive_is_reused_at_restart(self):
        with tempfile.TemporaryDirectory() as d:
            state=Path(d);directory=state/'boot-records';directory.mkdir(mode=0o700)
            old=history();m=metadata('\n'.join(old.splitlines()[3:])+'\n'+usb_tail())
            p=directory/(m['boot_id']+'-1.json')
            record=dict(startup=dict(core_metadata={k:m[k] for k in ('boot_id','cid','model','root_mount_line')},
                kernel_log=old,kernel_sha256=hashlib.sha256(old.encode()).hexdigest()))
            p.write_text(json.dumps(record));p.chmod(0o600)
            original_lstat=Path.lstat;original_fstat=os.fstat
            def owned(node):
                return types.SimpleNamespace(**{key:(0 if key=='st_uid' else getattr(node,key))
                    for key in dir(node) if key.startswith('st_')})
            with (mock.patch.object(service,'STATE',state),
                  mock.patch.object(Path,'lstat',lambda path:owned(original_lstat(path))),
                  mock.patch.object(service.os,'fstat',lambda fd:owned(original_fstat(fd)))):
                self.assertEqual(service.restore_archived_kernel(m,boot.restore_kernel_history),old+usb_tail())
                second=directory/(m['boot_id']+'-2.json')
                normalized_equivalent=old.replace('\n','\r\n')+usb_tail()
                second_record=copy.deepcopy(record)
                second_record['startup'].update(kernel_log=normalized_equivalent,
                    kernel_sha256=hashlib.sha256(normalized_equivalent.encode()).hexdigest())
                second.write_text(json.dumps(second_record));second.chmod(0o600)
                self.assertEqual(service.restore_archived_kernel(m,boot.restore_kernel_history),old+usb_tail())
                edited=normalized_equivalent.replace('record 12','edited record 12')
                second_record['startup'].update(kernel_log=edited,kernel_sha256=hashlib.sha256(edited.encode()).hexdigest())
                second.write_text(json.dumps(second_record))
                with self.assertRaises(ValueError):service.restore_archived_kernel(m,boot.restore_kernel_history)
                second.unlink()
                record['startup']['core_metadata']['cid']='another'
                p.write_text(json.dumps(record))
                with self.assertRaises(ValueError):service.restore_archived_kernel(m,boot.restore_kernel_history)
                record['startup']['core_metadata']['cid']=m['cid'];record['startup']['kernel_sha256']='bad'
                p.write_text(json.dumps(record))
                with self.assertRaises(ValueError):service.restore_archived_kernel(m,boot.restore_kernel_history)
                p.unlink();p.symlink_to(state/'missing')
                with self.assertRaises(OSError):service.restore_archived_kernel(m,boot.restore_kernel_history)

    def test_no_archive_cannot_turn_partial_zero_timestamp_log_into_new_baseline(self):
        with (tempfile.TemporaryDirectory() as d,mock.patch.object(service,'STATE',Path(d)),
              mock.patch.object(service,'read_boot_kernel_journal',side_effect=ValueError('Early boot history unavailable'))):
            with self.assertRaises(ValueError):
                service.restore_archived_kernel(metadata('[    0.000000] some later boot event\n'),boot.restore_kernel_history)
            full=history()
            self.assertEqual(service.restore_archived_kernel(metadata(full),boot.restore_kernel_history),full)

    def test_no_archive_accepts_adjacent_zero_time_linux_version_and_matching_model(self):
        full = ('[    0.000000] Linux version 6.1.83 (root@x5) (gcc 11.2.0) #1 SMP PREEMPT\n'
                '[    0.000000] Machine model: D-Robotics RDK X5 V1.0\n'
                '[    0.000000] Reserved memory initialized\n'
                '[    1.000000] later boot record\n')
        restore = mock.Mock(side_effect=AssertionError('No archive may be invented'))
        with tempfile.TemporaryDirectory() as d,mock.patch.object(service,'STATE',Path(d)):
            self.assertEqual(service.restore_archived_kernel(metadata(full),restore),full)
            self.assertEqual(list(Path(d).iterdir()), [])
        restore.assert_not_called()

    def test_no_archive_linux_version_anchor_rejects_missing_late_or_foreign_model(self):
        version = '[    0.000000] Linux version 6.1.83 (root@x5) (gcc 11.2.0) #1 SMP PREEMPT\n'
        model = '[    0.000000] Machine model: D-Robotics RDK X5 V1.0\n'
        invalid = [version, model, version + model.replace('RDK X5', 'RDK X3'),
                   version.replace('0.000000', '0.100000') + model,
                   version + model.replace('0.000000', '0.100000'),
                   version + '[    0.000000] unrelated early record\n' + model,
                   version.replace('6.1.83 (root@x5) (gcc 11.2.0) #1 SMP PREEMPT', '') + model,
                   (version + model).rstrip('\n')]
        with (tempfile.TemporaryDirectory() as d,mock.patch.object(service,'STATE',Path(d)),
              mock.patch.object(service,'read_boot_kernel_journal',side_effect=ValueError('Early boot history unavailable'))):
            for value in invalid:
                with self.subTest(kernel=value),self.assertRaisesRegex(ValueError,'Early boot history'):
                    service.restore_archived_kernel(metadata(value),boot.restore_kernel_history)

    def test_linux_version_boot_anchor_does_not_approve_later_hardware_fault(self):
        full = ('[    0.000000] Linux version 6.1.83 (root@x5) (gcc 11.2.0) #1 SMP PREEMPT\n'
                '[    0.000000] Machine model: D-Robotics RDK X5 V1.0\n'
                '[    1.000000] later boot record\n'
                '[    2.000000] mmc1: error -5\n')
        with tempfile.TemporaryDirectory() as d,mock.patch.object(service,'STATE',Path(d)):
            retained = service.restore_archived_kernel(metadata(full),boot.restore_kernel_history)
        self.assertEqual(retained, full)
        source = feedback_stream.runtime_source() + '\n' + Path(boot.__file__).read_text()
        ns = {};exec(source,ns)
        expected = ns['boot_expected']({},metadata(retained),ns['classify_feedback_log'])
        with self.assertRaisesRegex(ns['ProbeError'],'New kernel fault'):
            ns['validate_kernel'](retained,expected)


class KernelJournalRecoveryTests(unittest.TestCase):
    def setUp(self):
        import re
        self.full=re.sub(r'^\[ ([0-9]+)\.000000\]',lambda match:f'[{int(match[1]):5d}.000000]',history(100),flags=re.M)
        self.current=''.join(self.full.splitlines(keepends=True)[3:])
        self.metadata=metadata(self.current)
        self.records=[]
        for line in self.full.splitlines():
            clock,message=line.split('] ',1)
            seconds,micros=clock.lstrip('[ ').split('.')
            self.records.append(dict(MESSAGE=message,_SOURCE_MONOTONIC_TIMESTAMP=str(int(seconds)*1000000+int(micros)),
                _BOOT_ID=self.metadata['boot_id'].replace('-',''),_TRANSPORT='kernel'))

    def raw(self,records=None):
        return ''.join(json.dumps(record)+'\n' for record in (self.records if records is None else records)).encode()

    def test_exact_source_overlap_recovers_only_prefix_and_retains_faults_and_raw_evidence(self):
        fault='[ 120.000000] mmc1: error -5\n'
        self.metadata['kernel_log']+=fault
        raw=self.raw()
        restored,proof=service.restore_journal_kernel(self.metadata,raw,boot.restore_kernel_history)
        self.assertEqual(restored,self.full+fault)
        self.assertEqual(proof['recovered_prefix_lines'],3)
        self.assertEqual(proof['overlap_timestamped_records'],97)
        self.assertFalse(proof['hardware_gate_passed'])
        self.assertEqual(gzip.decompress(base64.b64decode(proof['journal_raw_gzip_base64'])),raw)
        self.assertEqual(gzip.decompress(base64.b64decode(proof['dmesg_raw_gzip_base64'])),self.metadata['kernel_log'].encode())
        ns={};exec(feedback_stream.runtime_source()+'\n'+Path(boot.__file__).read_text(),ns)
        expected=ns['boot_expected']({},metadata(restored),ns['classify_feedback_log'])
        with self.assertRaisesRegex(ns['ProbeError'],'New kernel fault'):
            ns['validate_kernel'](restored,expected)

    def test_missing_journal_self_messages_after_verified_overlap_do_not_remove_dmesg_records(self):
        records=self.records[:80]+self.records[81:]
        restored,proof=service.restore_journal_kernel(self.metadata,self.raw(records),boot.restore_kernel_history)
        self.assertEqual(restored,self.full)
        self.assertEqual(proof['overlap_timestamped_records'],77)

    def test_wrong_boot_transport_missing_source_time_bad_encoding_and_short_overlap_refused(self):
        for change in (dict(_BOOT_ID='0'*32),dict(_TRANSPORT='syslog'),
                       dict(_SOURCE_MONOTONIC_TIMESTAMP=None,__MONOTONIC_TIMESTAMP='0'),
                       dict(MESSAGE=[256]),dict(_SOURCE_MONOTONIC_TIMESTAMP='-1')):
            records=copy.deepcopy(self.records);records[0].update(change)
            with self.subTest(change=change),self.assertRaises(ValueError):
                service.restore_journal_kernel(self.metadata,self.raw(records),boot.restore_kernel_history)
        records=copy.deepcopy(self.records);records[70]['_SOURCE_MONOTONIC_TIMESTAMP']='1'
        with self.assertRaisesRegex(ValueError,'regressed'):
            service.restore_journal_kernel(self.metadata,self.raw(records),boot.restore_kernel_history)
        short=dict(self.metadata,kernel_log=''.join(self.full.splitlines(keepends=True)[-63:]))
        with self.assertRaisesRegex(ValueError,'64-record'):
            service.restore_journal_kernel(short,self.raw(),boot.restore_kernel_history)
        edited=dict(self.metadata,kernel_log=self.current.replace('record 4','edited record 4'))
        with self.assertRaisesRegex(ValueError,'64-record'):
            service.restore_journal_kernel(edited,self.raw(),boot.restore_kernel_history)

    def test_dmesg_empty_multiline_binary_and_syslog_prefix_formats_preserve_tail(self):
        records=copy.deepcopy(self.records)
        records[1].update(MESSAGE='service ready',SYSLOG_IDENTIFIER='systemd',SYSLOG_PID='1')
        records[2]['MESSAGE']='banner\ncontinuation'
        records[10]['MESSAGE']=list(b'control:\x02\r')
        records[11]['MESSAGE']=''
        full=self.full.replace('[    1.000000] ordinary boot record 1\n','[    1.000000] systemd[1]: service ready\n')
        full=full.replace('[    2.000000] ordinary boot record 2\n','[    2.000000] banner\n               continuation\n')
        full=full.replace('[   10.000000] ordinary boot record 10\n','[   10.000000] control:\\x02\r\n')
        full=full.replace('[   11.000000] ordinary boot record 11\n','\n')
        # Use dmesg's fixed-width timestamps throughout this fixture.
        import re
        full=re.sub(r'^\[ ([0-9]+)\.000000\]',lambda match:f'[{int(match[1]):5d}.000000]',full,flags=re.M)
        current=''.join(full.splitlines(keepends=True)[5:])
        restored,proof=service.restore_journal_kernel(dict(self.metadata,kernel_log=current),self.raw(records),boot.restore_kernel_history)
        self.assertEqual(restored,full)
        self.assertTrue(restored.endswith(current))

    def test_journal_is_used_only_for_missing_anchor_and_never_after_archive_refusal(self):
        # Synthetic dmesg uses fixed-width formatting, as the real tool does.
        import re
        self.full=re.sub(r'^\[ ([0-9]+)\.000000\]',lambda match:f'[{int(match[1]):5d}.000000]',self.full,flags=re.M)
        self.metadata['kernel_log']=''.join(self.full.splitlines(keepends=True)[3:])
        with tempfile.TemporaryDirectory() as d,mock.patch.object(service,'STATE',Path(d)),mock.patch.object(service,'read_boot_kernel_journal',return_value=self.raw()) as read:
            self.assertEqual(service.restore_archived_kernel(self.metadata,boot.restore_kernel_history),self.full)
            read.assert_called_once_with(self.metadata['boot_id'])
            self.assertEqual(self.metadata['kernel_journal_recovery']['recovered_prefix_lines'],3)
            read.reset_mock()
            self.assertEqual(service.restore_archived_kernel(metadata(self.full),boot.restore_kernel_history),self.full)
            read.assert_not_called()
            archive=Path(d)/'boot-records';archive.mkdir(mode=0o777);archive.chmod(0o777)
            with self.assertRaisesRegex(ValueError,'not private'):
                service.restore_archived_kernel(self.metadata,boot.restore_kernel_history)
            read.assert_not_called()

    def test_journal_command_failure_does_not_turn_partial_history_into_boot_evidence(self):
        with tempfile.TemporaryDirectory() as d,mock.patch.object(service,'STATE',Path(d)),mock.patch.object(service.subprocess,'Popen',side_effect=OSError('journal temporarily unavailable')):
            with self.assertRaisesRegex(OSError,'journal temporarily unavailable'):
                service.restore_archived_kernel(self.metadata,boot.restore_kernel_history)
        self.assertNotIn('kernel_journal_recovery',self.metadata)

    def test_journal_reader_uses_compact_boot_selector_and_source_identity_fields(self):
        with mock.patch.object(service.subprocess,'Popen',side_effect=OSError('capture invocation')) as launch:
            with self.assertRaisesRegex(OSError,'capture invocation'):
                service.read_boot_kernel_journal(self.metadata['boot_id'])
        command=launch.call_args.args[0]
        self.assertIn('--boot='+self.metadata['boot_id'].replace('-',''),command)
        self.assertIn('_TRANSPORT=kernel',command)
        fields=next(value.split('=',1)[1].split(',') for value in command if value.startswith('--output-fields='))
        self.assertTrue({'MESSAGE','_SOURCE_MONOTONIC_TIMESTAMP','_BOOT_ID','_TRANSPORT','SYSLOG_IDENTIFIER','SYSLOG_PID'}<=set(fields))
        self.assertNotIn('__MONOTONIC_TIMESTAMP',fields)
        self.assertIn('--lines=4096',command)


if __name__=='__main__':unittest.main()
