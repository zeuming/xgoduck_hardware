"""Launcher lifecycle regressions using local files and fake children only."""
import contextlib
import errno
import io
import json
import os
from pathlib import Path
import signal
import tempfile
import time
import types
import unittest
from unittest import mock

import native_service as service


class FakeChild:
    def __init__(self, returncode=None):
        self.returncode=returncode
        self.terminated=0

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated+=1
        self.returncode=0


def root_metadata(node, **changes):
    """Represent root-owned fixtures without requiring tests to run as root."""
    value={key:getattr(node,key) for key in dir(node) if key.startswith('st_')}
    value['st_uid']=0;value.update(changes)
    return types.SimpleNamespace(**value)


class LauncherTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory=Path(self.temporary.name).resolve()
        self.root=self.directory/'ram';self.root.mkdir(mode=0o700)
        self.install=self.directory/'install';self.install.mkdir()
        self.state=self.directory/'state';self.state.mkdir(mode=0o700)
        (self.install/'installation.json').write_text('{"files":{}}')
        (self.install/'deployment.json').write_text('{}')
        (self.install/'native-bundle.tar.gz').write_bytes(b'fake archive')
        (self.install/'persistent_boot.py').write_bytes(b'')
        self.lock=self.directory/'launcher.lock'
        node=self.root.lstat();self.identity=(node.st_dev,node.st_ino)
        self.lifecycle=dict(child=None,stopping=False,stop_requested_ns=None,dependency_nodes={})
        self.real_lstat=Path.lstat
        self.real_fstat=os.fstat
        self.real_open=os.open
        # Only ownership is simulated; inode, canonical paths, permissions,
        # link counts, timestamps and all file operations remain real.
        self.stack=contextlib.ExitStack();self.addCleanup(self.stack.close)
        def lstat(path):
            node=self.real_lstat(path)
            return root_metadata(node) if self.directory in (path,*path.parents) else node
        self.stack.enter_context(mock.patch.object(Path,'lstat',lstat))
        self.stack.enter_context(mock.patch.object(service.os,'fstat',lambda fd:root_metadata(self.real_fstat(fd))))

    def dependencies(self):
        for name in ('site','wheels'):
            path=self.root/name;path.mkdir(mode=0o700)
            node=path.lstat();self.lifecycle['dependency_nodes'][name]=(node.st_dev,node.st_ino)
            (path/'rebuildable').write_bytes(b'large dependency')

    def evidence(self):
        values={'native-evidence/action.json':b'{"real":"evidence"}',
                'policy/walk.onnx':b'policy','service/expected.json':b'{}',
                'native-pin.json':b'{"pin":true}'}
        for name,raw in values.items():
            path=self.root/name;path.parent.mkdir(mode=0o700,parents=True,exist_ok=True);path.write_bytes(raw)
        (self.state/'calibration-state.json').write_bytes(b'private calibration')
        (self.state/'recovery-holds.json').write_bytes(b'private recovery holds')
        return values

    def assert_evidence(self,values):
        for name,raw in values.items():self.assertEqual((self.root/name).read_bytes(),raw)
        self.assertEqual((self.state/'calibration-state.json').read_bytes(),b'private calibration')
        self.assertEqual((self.state/'recovery-holds.json').read_bytes(),b'private recovery holds')

    def proof(self,**changes):
        proof=dict(serial_closed=True,release_confirmed=True,serial_open_attempts=1,no_uart_opened=False)
        value=dict(phase='closed',owns_connection=False,exit_reconciled=True,closed_proof=proof)
        value.update(changes)
        path=self.root/'native-evidence/live-state.json';path.parent.mkdir(mode=0o700,exist_ok=True)
        service.private_json(path,value)
        return path

    def cleanup(self):
        return service.cleanup_dependencies(self.root,self.identity,self.lifecycle)

    def call_main(self,run):
        def open_lock(path,*args,**kwargs):
            descriptor=self.real_open(self.lock if str(path)=='/run/xgoduck-workbench.lock' else path,*args,**kwargs)
            if str(path)=='/run/xgoduck-workbench.lock':self.addCleanup(os.close,descriptor)
            return descriptor
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(service,'INSTALL',self.install))
            stack.enter_context(mock.patch.object(service.os,'geteuid',return_value=0))
            stack.enter_context(mock.patch.object(service.os,'open',side_effect=open_lock))
            stack.enter_context(mock.patch.object(service.fcntl,'flock'))
            stack.enter_context(mock.patch.object(service.tempfile,'mkdtemp',return_value=str(self.root)))
            stack.enter_context(mock.patch.object(service,'run_owned',side_effect=run))
            stack.enter_context(contextlib.redirect_stderr(io.StringIO()))
            return service.main()

    def test_main_failed_unpack_reclaims_partial_dependencies_and_keeps_evidence(self):
        evidence=self.evidence()
        other=self.directory/'other-runtime';other.mkdir();(other/'site').mkdir()
        failure=OSError(errno.ENOSPC,'No space left on device')
        def fail(root,manifest,lifecycle):
            (root/'site/partial.so').write_bytes(b'partial wheel')
            (root/'wheels/partial.whl').write_bytes(b'partial archive')
            raise failure
        with self.assertRaises(OSError) as caught:self.call_main(fail)
        self.assertIs(caught.exception,failure)
        self.assertFalse((self.root/'site').exists());self.assertFalse((self.root/'wheels').exists())
        self.assertTrue((other/'site').is_dir());self.assertTrue(self.root.is_dir())
        self.assert_evidence(evidence)

    def test_main_systemexit_zero_after_controlled_child_exit_also_cleans(self):
        evidence=self.evidence()
        def stop(root,manifest,lifecycle):
            lifecycle.update(child=FakeChild(0),stopping=True,stop_requested_ns=time.time_ns())
            raise SystemExit(0)
        with self.assertRaises(SystemExit) as caught:self.call_main(stop)
        self.assertEqual(caught.exception.code,0)
        self.assertFalse((self.root/'site').exists());self.assertFalse((self.root/'wheels').exists())
        self.assert_evidence(evidence)

    def test_child_still_alive_retains_dependencies_without_signalling_it(self):
        self.dependencies();child=FakeChild();self.lifecycle.update(child=child,stopping=True)
        self.assertFalse(self.cleanup()['cleaned'])
        self.assertTrue((self.root/'site/rebuildable').is_file());self.assertEqual(child.terminated,0)

    def test_unexpected_exit_retains_even_with_a_closed_packet(self):
        self.dependencies();self.proof();self.lifecycle.update(child=FakeChild(1))
        self.assertFalse(self.cleanup()['cleaned']);self.assertTrue((self.root/'wheels').exists())

    def test_controlled_abnormal_exit_retains_even_with_fresh_motion_closed_proof(self):
        # Motion close can persist before a later maintenance close fails.
        # Its packet alone cannot certify that every child owner closed.
        self.dependencies();self.lifecycle.update(child=FakeChild(1),stopping=True,stop_requested_ns=1)
        self.proof()
        self.assertFalse(self.cleanup()['cleaned']);self.assertTrue((self.root/'site').exists())

    def test_uncontrolled_zero_exit_also_requires_current_shutdown_proof(self):
        self.dependencies();self.lifecycle.update(child=FakeChild(0))
        self.assertFalse(self.cleanup()['cleaned'])

    def test_signal_exit_without_shutdown_proof_retains(self):
        self.dependencies()
        for code in (-15,-2,130):
            with self.subTest(code=code):
                self.lifecycle.update(child=FakeChild(code),stopping=True,stop_requested_ns=time.time_ns())
                self.assertFalse(self.cleanup()['cleaned']);self.assertTrue((self.root/'site').exists())

    def test_signal_exit_with_new_private_closed_released_proof_cleans_only_dependencies(self):
        self.dependencies();evidence=self.evidence()
        self.lifecycle.update(child=FakeChild(-15),stopping=True,stop_requested_ns=time.time_ns()-1_000_000_000)
        path=self.proof();raw=path.read_bytes()
        self.assertTrue(self.cleanup()['cleaned'])
        self.assertEqual(path.read_bytes(),raw);self.assert_evidence(evidence)

    def test_signal_exit_with_explicit_no_uart_opened_proof_cleans(self):
        self.dependencies();self.lifecycle.update(child=FakeChild(-2),stopping=True,stop_requested_ns=1)
        self.proof(closed_proof=dict(serial_closed=True,release_confirmed=False,no_uart_opened=True,serial_open_attempts=0))
        self.assertTrue(self.cleanup()['cleaned'])

    def test_old_or_incomplete_shutdown_packets_retain(self):
        self.dependencies();self.lifecycle.update(child=FakeChild(-15),stopping=True,stop_requested_ns=time.time_ns())
        path=self.proof();old=self.lifecycle['stop_requested_ns']-1_000_000_000
        os.utime(path,ns=(old,old))
        self.assertFalse(self.cleanup()['cleaned'])
        path.unlink();self.proof(closed_proof=dict(serial_closed=True,release_confirmed=False))
        self.lifecycle['stop_requested_ns']=1
        self.assertFalse(self.cleanup()['cleaned'])

    def test_private_proof_rejects_permissions_links_owner_oversize_and_noncanonical_path(self):
        self.dependencies();self.lifecycle.update(child=FakeChild(-15),stopping=True,stop_requested_ns=1)
        path=self.proof()
        path.chmod(0o644);self.assertFalse(self.cleanup()['cleaned']);path.chmod(0o600)
        link=self.root/'proof-link';os.link(path,link);self.assertFalse(self.cleanup()['cleaned']);link.unlink()
        with mock.patch.object(service.os,'fstat',lambda fd:root_metadata(self.real_fstat(fd),st_uid=1000)):
            self.assertFalse(self.cleanup()['cleaned'])
        path.write_bytes(b' '*(2*1024*1024+1));self.assertFalse(self.cleanup()['cleaned'])
        path.unlink();outside=self.directory/'outside-proof';outside.write_text('{}');path.symlink_to(outside)
        self.assertFalse(self.cleanup()['cleaned']);self.assertEqual(outside.read_text(),'{}')
        self.assertTrue((self.root/'site').is_dir())

    def test_invalid_proof_does_not_replace_the_original_main_exception(self):
        original=RuntimeError('Original startup failure')
        def fail(root,manifest,lifecycle):
            path=root/'native-evidence';path.mkdir();(path/'live-state.json').write_text('invalid JSON')
            (path/'live-state.json').chmod(0o600)
            lifecycle.update(child=FakeChild(-15),stopping=True,stop_requested_ns=1)
            raise original
        with self.assertRaises(RuntimeError) as caught:self.call_main(fail)
        self.assertIs(caught.exception,original);self.assertTrue((self.root/'site').exists())

    def test_replaced_root_inode_is_not_deleted(self):
        self.dependencies();original=self.directory/'original';self.root.rename(original)
        self.root.mkdir(mode=0o700);(self.root/'site').mkdir();(self.root/'site/unknown').write_bytes(b'keep')
        self.assertFalse(self.cleanup()['cleaned'])
        self.assertTrue((self.root/'site/unknown').exists());self.assertTrue((original/'site/rebuildable').exists())

    def test_root_symlink_is_not_followed(self):
        self.dependencies();original=self.directory/'original';self.root.rename(original);self.root.symlink_to(original,target_is_directory=True)
        self.assertFalse(self.cleanup()['cleaned']);self.assertTrue((original/'site/rebuildable').exists())

    def test_dependency_symlink_or_inode_exchange_prevents_any_deletion(self):
        self.dependencies();original=self.root/'original-wheels';(self.root/'wheels').rename(original)
        (self.root/'wheels').symlink_to(original,target_is_directory=True)
        self.assertFalse(self.cleanup()['cleaned']);self.assertTrue((self.root/'site/rebuildable').exists())
        (self.root/'wheels').unlink();(self.root/'wheels').mkdir(mode=0o700)
        (self.root/'wheels/unknown').write_bytes(b'keep')
        self.assertFalse(self.cleanup()['cleaned'])
        self.assertTrue((self.root/'wheels/unknown').exists());self.assertTrue((original/'rebuildable').exists())

    def run_fake_parent(self,packet):
        """Run the actual parent polling/hold loop with fake boot/network/process."""
        directory=self.root/'service';directory.mkdir()
        for name,value in (('guard-runtime.py',''),('system_health.py',''),('expected.json','{}'),
                           ('deployment.json','{"bind_host":"192.168.1.11","port":8767}'),
                           ('startup-snapshot.json','{}')):(directory/name).write_text(value)
        child=FakeChild();handlers={};urls=[]
        def open_url(url,timeout):
            urls.append((url,timeout))
            if url.endswith('/'):
                return io.BytesIO(b'page with cookie')
            if isinstance(packet,Exception):raise packet
            return io.BytesIO(json.dumps(packet).encode())
        def namespace_exec(code,namespace):
            if namespace.get('__name__') == 'x5_health_launcher':
                namespace['HealthMonitor'] = mock.Mock()
                return
            def no_serial(expected):raise ValueError('fake offline startup')
            namespace.update(boot_expected=lambda *args:{},classify_feedback_log=lambda *args:None,
                validate_os=lambda *args:None,validate_kernel=lambda *args:None,
                bind_serial=lambda *args:None,serial_proof=no_serial)
        metadata=dict(boot_id='fake-boot',cid='fake-card',model='fake-model',root_mount_line='root',
            swaps='',kernel_log='',mounts=['1 0 179:1 / /boot/config rw - vfat boot rw',
                                         '2 0 0:1 / /dev/shm rw - tmpfs shm rw'])
        def sleep(seconds):handlers[signal.SIGTERM]()
        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(service,'INSTALL',self.install))
            stack.enter_context(mock.patch.object(service,'STATE',self.state))
            stack.enter_context(mock.patch.object(service,'unpack'))
            stack.enter_context(mock.patch.object(service,'exec',side_effect=namespace_exec,create=True))
            stack.enter_context(mock.patch.object(service,'capture',return_value=metadata))
            stack.enter_context(mock.patch.object(service.socket,'socket',return_value=mock.Mock()))
            stack.enter_context(mock.patch.object(service.subprocess,'Popen',return_value=child))
            stack.enter_context(mock.patch.object(service.signal,'signal',side_effect=lambda signum,handler:handlers.update({signum:handler})))
            stack.enter_context(mock.patch.object(service.urllib.request,'build_opener',return_value=types.SimpleNamespace(open=open_url)))
            stack.enter_context(mock.patch.object(service.time,'sleep',side_effect=sleep))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            with self.assertRaises(SystemExit) as caught:service.run_owned(self.root,dict(files={}),self.lifecycle)
            self.assertEqual(caught.exception.code,0)
        self.assertIs(self.lifecycle['child'],child);self.assertTrue(self.lifecycle['stopping'])
        self.assertIsInstance(self.lifecycle['stop_requested_ns'],int);self.assertEqual(child.terminated,1)
        return urls

    def test_actual_parent_uses_compact_get_and_preserves_both_recovery_holds_without_tokens(self):
        urls=self.run_fake_parent(dict(maintenance=dict(fault_hold=True,identity_changed=True,error='review actual fault'),
                                       token='must not persist',motion=dict(hardware=None)))
        self.assertEqual(urls,[('http://127.0.0.1:8767/',1),('http://127.0.0.1:8767/api/live-status',1)])
        path=self.state/'recovery-holds.json';value=json.loads(path.read_text())
        holds=value['native_recovery_holds'];self.assertEqual(set(holds),{'maintenance-fault-hold.json','maintenance-identity-hold.json'})
        self.assertEqual(holds['maintenance-fault-hold.json']['reason'],'review actual fault')
        self.assertTrue(holds['maintenance-identity-hold.json']['requires_physical_mapping_review'])
        self.assertNotIn('token',path.read_text());self.assertEqual(path.stat().st_mode&0o777,0o600)

    def test_poll_error_keeps_existing_private_recovery_holds_and_restores_them_on_boot(self):
        path=self.state/'recovery-holds.json'
        service.remember_holds(dict(maintenance=dict(fault_hold=True,error='prior hold')),path)
        original=path.read_bytes();self.run_fake_parent(OSError('fake HTTP timeout'))
        self.assertEqual(path.read_bytes(),original)
        restored=json.loads((self.root/'service/startup-snapshot.json').read_text())
        self.assertEqual(restored['native_recovery_holds'],json.loads(original)['native_recovery_holds'])


if __name__=='__main__':unittest.main()
