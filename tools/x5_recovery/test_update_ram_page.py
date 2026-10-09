#!/usr/bin/env python3
"""Hardware-free negative tests for the finite RAM HTML transaction."""
import copy
import hashlib
import io
import json
import os
from pathlib import Path
import stat
import tempfile
import types
import unittest
from unittest import mock
import update_ram_page as page


class Tests(unittest.TestCase):
    old = b'<html>old reviewed page</html>'
    new = b'<html>new reviewed page</html>'

    def fixture(self, directory):
        os.chmod(directory, 0o700)
        policy = Path(directory) / 'policy'; policy.mkdir(mode=0o700)
        index = policy / 'index.html'; index.write_bytes(self.old); index.chmod(0o600)
        config = dict(directory=str(directory), uid=os.getuid(), old_sha256=hashlib.sha256(self.old).hexdigest(),
                      new_sha256=hashlib.sha256(self.new).hexdigest(), new_size=len(self.new))
        return config, index

    def execute(self, config, data=None, commit=b'COMMIT\n', gate=None):
        stream = io.BytesIO(self.new if data is None else data)
        events = []
        page.page_transaction(config, stream.read, lambda: commit, events.append, gate or (lambda: None))
        return events

    def test_success_is_atomic_and_only_replaces_index(self):
        with tempfile.TemporaryDirectory() as directory:
            config, index = self.fixture(directory)
            original = index.stat().st_ino
            calibration = index.parent / 'calibration.json'; calibration.write_bytes(b'keep')
            events = self.execute(config)
            self.assertEqual(['READY', 'PREPARED', 'DONE'], [e['phase'] for e in events])
            self.assertEqual(self.new, index.read_bytes())
            self.assertNotEqual(original, index.stat().st_ino)
            self.assertEqual(0o600, stat.S_IMODE(index.stat().st_mode))
            self.assertEqual(b'keep', calibration.read_bytes())
            self.assertEqual(['calibration.json', 'index.html'], sorted(p.name for p in index.parent.iterdir()))

    def test_old_hash_bad_transfer_commit_abort_or_gate_failure_never_replaces(self):
        for fault in ('old-sha', 'short-transfer', 'new-sha', 'host-abort', 'prepare-gate', 'commit-gate'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                config, index = self.fixture(directory)
                data, commit = self.new, b'COMMIT\n'
                if fault == 'old-sha': config['old_sha256'] = '0' * 64
                elif fault == 'short-transfer': data = self.new[:-1]
                elif fault == 'new-sha': data = b'x' * len(self.new)
                elif fault == 'host-abort': commit = b'ABORT\n'
                calls = 0
                def gate():
                    nonlocal calls
                    calls += 1
                    if (fault == 'prepare-gate' and calls == 2) or (fault == 'commit-gate' and calls == 3):
                        raise RuntimeError('mock guard/kernel/identity changed')
                with self.assertRaises(RuntimeError): self.execute(config, data, commit, gate)
                self.assertEqual(self.old, index.read_bytes())
                self.assertEqual(['index.html'], [p.name for p in index.parent.iterdir()])

    def test_symlink_hardlink_public_file_or_directory_is_rejected(self):
        for fault in ('index-symlink', 'policy-symlink', 'hardlink', 'index-public', 'policy-public', 'ram-public'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                config, index = self.fixture(directory)
                if fault == 'index-symlink':
                    saved=index.parent/'other'; index.rename(saved); index.symlink_to(saved)
                elif fault == 'policy-symlink':
                    saved=Path(directory)/'other'; index.parent.rename(saved); (Path(directory)/'policy').symlink_to(saved, target_is_directory=True)
                elif fault == 'hardlink': os.link(index, index.parent/'other')
                elif fault == 'index-public': index.chmod(0o644)
                elif fault == 'policy-public': index.parent.chmod(0o755)
                else: Path(directory).chmod(0o755)
                with self.assertRaises((RuntimeError, OSError)): self.execute(config)

    def test_index_inode_change_after_prepare_rejects_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            config, index = self.fixture(directory)
            calls = 0
            def gate():
                nonlocal calls
                calls += 1
                if calls == 3:
                    replacement = index.parent/'replacement'; replacement.write_bytes(self.old); replacement.chmod(0o600)
                    os.replace(replacement,index)
            with self.assertRaisesRegex(RuntimeError, 'inode changed'):
                self.execute(config, gate=gate)
            self.assertEqual(self.old,index.read_bytes())
            self.assertEqual(['index.html'],[p.name for p in index.parent.iterdir()])

    def test_deadline_cleanup_unlinks_uncommitted_temp(self):
        with tempfile.TemporaryDirectory() as directory:
            config, index = self.fixture(directory)
            calls=0
            def gate():
                nonlocal calls
                calls+=1
                if calls==3: raise TimeoutError('deadline')
            with self.assertRaises(TimeoutError): self.execute(config,gate=gate)
            self.assertEqual(self.old,index.read_bytes())
            self.assertEqual(['index.html'],[p.name for p in index.parent.iterdir()])

    def live_fixture(self):
        return dict(generation=7,mode='off',armed=False,motor_output=False,connected=True,paused=False,
                    pending_stop=False,fault_ids=[],torque_ids=[],online_ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],
                    imu_feedback_ok=True,age_ms=3.,imu_age_ms=3.,imu_wizard={'busy':False,'stage':'idle'},imu={'calibrating':False})

    def test_http_gate_rejects_active_wizard_stale_masks_paused_or_generation_change(self):
        changes=({'generation':8},{'mode':'shadow'},{'armed':True},{'motor_output':True},{'connected':False},
                 {'paused':True},{'pending_stop':True},{'fault_ids':[32]},{'torque_ids':[10]},
                 {'online_ids':[10]},{'imu_feedback_ok':False},{'age_ms':100.},{'imu_age_ms':100.},
                 {'imu_wizard':{'busy':True,'stage':'idle'}},{'imu_wizard':{'busy':False,'stage':'front-up'}},
                 {'imu_wizard':{'busy':False,'stage':'failed'}},{'imu':{'calibrating':True}})
        for change in ({},)+changes:
            with self.subTest(change=change):
                state={**self.live_fixture(),**change}
                conn=mock.Mock(); response=mock.Mock(status=200); response.read.return_value=json.dumps(state).encode()
                conn.getresponse.return_value=response
                with mock.patch('http.client.HTTPConnection',return_value=conn):
                    if change:
                        with self.assertRaises(RuntimeError): page.live_page_status({'remote_port':8769,'generation':7})
                    else: self.assertEqual(state,page.live_page_status({'remote_port':8769,'generation':7}))
                conn.close.assert_called_once_with()
                self.assertEqual('GET',conn.request.call_args.args[0])

    def test_snapshot_rejects_kernel_root_boot_cid_swap_mount_or_usb_change(self):
        kernel=b'[1] clean\n'
        root='21 1 179:2 / / rw,relatime - ext4 /dev/root rw\n'
        shm='34 1 0:32 / /dev/shm rw,nosuid,nodev - tmpfs shm rw\n'
        review={'expected_root':{'device':'179:2','source':'/dev/root','filesystem':'ext4'},'required_unmounted_device':'179:1',
                'expected_boot_id':'boot','expected_cid':'cid','kernel_log_sha256':page.normalized_kernel_sha256(kernel.decode()),
                'serial_device':{'mock':True}}
        for fault in (None,'kernel','root-ro','swap','config','boot','cid','ram-ro','usb'):
            with self.subTest(fault=fault):
                files={'/proc/self/mountinfo':root+shm,'/proc/swaps':'Filename Type Size Used Priority\n',
                       '/proc/sys/kernel/random/boot_id':'boot','/sys/block/mmcblk1/device/cid':'cid'}
                if fault=='root-ro': files['/proc/self/mountinfo']=root.replace('rw,relatime','ro,relatime')+shm
                elif fault=='ram-ro': files['/proc/self/mountinfo']=root+shm.replace('rw,nosuid','ro,nosuid')
                elif fault=='swap': files['/proc/swaps']+='mock partition 1 1 1\n'
                elif fault=='config': files['/proc/self/mountinfo']+='35 1 179:1 / /boot/config rw - vfat /dev/mmcblk1p1 rw\n'
                elif fault=='boot': files['/proc/sys/kernel/random/boot_id']='different'
                elif fault=='cid': files['/sys/block/mmcblk1/device/cid']='different'
                block=types.SimpleNamespace(st_mode=stat.S_IFBLK,st_rdev=os.makedev(179,2))
                ram=types.SimpleNamespace(st_mode=stat.S_IFDIR,st_dev=22,st_ino=33)
                with mock.patch.object(Path,'read_text',lambda obj,*a,**k:files[str(obj)]), \
                     mock.patch('update_ram_page.os.stat',return_value=block),mock.patch('update_ram_page.os.lstat',return_value=ram), \
                     mock.patch('update_ram_page.subprocess.run',return_value=types.SimpleNamespace(returncode=0,stderr=b'',stdout=kernel if fault!='kernel' else b'changed\n')), \
                     mock.patch('update_ram_page.serial_device_proof',side_effect=RuntimeError('USB changed') if fault=='usb' else None):
                    if fault:
                        with self.assertRaises(RuntimeError): page.snapshot(review)
                    else: self.assertEqual(('34','0:32','shm',22,33),page.snapshot(review))

    def test_local_input_private_hash_bound_and_remote_source_compiles(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'new.html'; p.write_bytes(self.new);p.chmod(0o600)
            self.assertEqual(self.new,page.private_bytes(p))
            p.chmod(0o644)
            with self.assertRaises(ValueError):page.private_bytes(p)
            p.chmod(0o600); link=Path(directory)/'link';link.symlink_to(p)
            with self.assertRaises(ValueError):page.private_bytes(link)
        for path in ('/tmp/xgoduck-model-12345678','/dev/shm/xgoduck-model-../../x','/dev/shm/other'):
            with self.assertRaises(ValueError):page.validate_ram_path(path)
        source=page.remote_source({'directory':'/dev/shm/xgoduck-model-qz32depp','review':{},'remote_port':8769})
        compile(source,'<finite-ram-page>','exec')
        self.assertNotIn('SerialTransport',source)
        self.assertNotIn('calibration.json',source)
        self.assertIn('signal.setitimer(signal.ITIMER_REAL,1.5)',source)
        self.assertIn("signal.signal(signal.SIGHUP,timed_out)",source)

    def test_host_guard_or_runner_process_change_rejects_transaction(self):
        with tempfile.TemporaryDirectory() as directory:
            runner=Path(directory)/'status.json'
            state={'state':'running','capability_scope':page.calibration.SCOPE,'motor_output':False,
                   'servo_arm_authorized':False,'remote_stop_confirmed':False,'local_pid':4444,
                   'ram_directory':'/dev/shm/xgoduck-model-qz32depp'}
            runner.write_text(json.dumps(state));runner.chmod(0o600)
            args=types.SimpleNamespace(guard_status='mock',runner_status=runner)
            with mock.patch.object(page.calibration,'check_guard',return_value={'local_pid':3333}),mock.patch('update_ram_page.os.kill'):
                self.assertEqual((3333,(4444,state['ram_directory'])),page.host_check(args,{}))
                with self.assertRaisesRegex(RuntimeError,'guard process changed'):
                    page.host_check(args,{},guard_pid=5555)
                with self.assertRaisesRegex(RuntimeError,'supervisor changed'):
                    page.host_check(args,{},runner_expected=(5555,state['ram_directory']))
                runner.write_text(json.dumps({**state,'feedback_paused':True}))
                with self.assertRaises(RuntimeError):page.host_check(args,{})
            with mock.patch.object(page.calibration,'check_guard',side_effect=RuntimeError('guard stopped')):
                with self.assertRaisesRegex(RuntimeError,'guard stopped'):page.host_check(args,{})


if __name__=='__main__':unittest.main(verbosity=2)
