"""Pure boot guards and archive validation; no network, UART or services."""
import hashlib
import io
import tarfile
import tempfile
import unittest
from unittest import mock
from pathlib import Path

import persistent_boot as boot
import native_service as service
import feedback_connection_check as connection
import feedback_stream
import maintenance_transport as maintenance


class BootTests(unittest.TestCase):
    def metadata(self):
        return dict(boot_id='69fdecfb-7b85-4fe9-abb5-25172f1ce952',cid=boot.CARD,model=boot.MODEL,
            root_mount_line='21 1 179:2 / / rw,relatime shared:1 - ext4 /dev/root rw',
            swaps='Filename Type Size Used Priority\n',kernel_log='[    0.000000] Linux boot\n')

    def classify(self,line):
        return 'fault' if any(word in line for word in ('USB disconnect','FAT-fs','I/O error')) else None

    def test_new_boot_uses_current_identity_without_motor_or_old_fault_approval(self):
        m=self.metadata();e=boot.boot_expected({},m,self.classify)
        self.assertEqual(e['boot_id'],m['boot_id'])
        self.assertFalse(boot.persistent_kernel(m['kernel_log'],e,self.classify)['historical_faults_cleared'])
        m['boot_id']='7d23364b-a37c-4ca9-a52f-56bb90547d88'
        next_e=boot.boot_expected({},m,self.classify)
        self.assertNotEqual(next_e['native_persistent_boot'],e['native_persistent_boot'])

    def test_only_exact_previously_known_boot_warning_is_retained(self):
        m=self.metadata();m['kernel_log']+='[   19.500001] FAT-fs (mmcblk1p1): Volume was not properly unmounted. Some data may be corrupt. Please run fsck.\n'
        e=boot.boot_expected({},m,self.classify)
        self.assertEqual(len(boot.persistent_kernel(m['kernel_log'],e,self.classify)['historical_faults']),1)
        for suffix in ('[ 20.0] I/O error on mmcblk1\n','[ 20.0] usb 1-1.4: USB disconnect, device number 3\n'):
            bad={**m,'kernel_log':m['kernel_log']+suffix};bad_e=boot.boot_expected({},bad,self.classify)
            self.assertNotIn(suffix.strip(),bad_e['kernel_prefix'])
            with self.assertRaisesRegex(ValueError,'New kernel fault'):boot.persistent_kernel(bad['kernel_log'],bad_e,self.classify)

    def test_changed_card_mount_lost_prefix_or_forged_allowances_refused(self):
        for change in (dict(cid='another'),dict(model='another'),dict(root_mount_line='21 1 8:2 / / rw - ext4 /dev/root rw'),dict(swaps='Filename Type Size Used Priority\n/swap file 1 1 1\n')):
            with self.assertRaises(ValueError):boot.boot_expected({},dict(self.metadata(),**change),self.classify)
        m=self.metadata();e=boot.boot_expected({},m,self.classify)
        with self.assertRaises(ValueError):boot.persistent_kernel('[ 10.0] prefix lost\n',e,self.classify)
        e['historical_faults']=[dict(line='unapproved',reason='fault')]
        with self.assertRaises(ValueError):boot.persistent_kernel(m['kernel_log'],e,self.classify)

    def test_archive_checksums_and_paths_are_checked_before_use(self):
        data=b'print("safe")\n';buffer=io.BytesIO()
        with tarfile.open(fileobj=buffer,mode='w:gz') as archive:
            item=tarfile.TarInfo('service/main.py');item.size=len(data);archive.addfile(item,io.BytesIO(data))
        raw=buffer.getvalue();manifest=dict(archive_sha256=hashlib.sha256(raw).hexdigest(),files={'service/main.py':dict(size_bytes=len(data),sha256=hashlib.sha256(data).hexdigest())})
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);service.unpack(raw,manifest,root)
            self.assertEqual((root/'service/main.py').read_bytes(),data)
            with self.assertRaises(ValueError):service.unpack(raw+b'corrupt',manifest,root)
        for name in ('../escape','/absolute','service/../escape'):
            with self.assertRaises(ValueError):service.safe_name(name)

    def test_guard_survives_metadata_and_maintenance_source_filters(self):
        source=feedback_stream.runtime_source()+'\n'+Path(boot.__file__).read_text()
        namespace={};exec(source,namespace)
        m=self.metadata();e=namespace['boot_expected']({},m,namespace['classify_feedback_log'])
        for filtered in (source,connection.runtime_source(source),maintenance._identity_runtime(source)):
            parsed={};exec(filtered,parsed)
            self.assertEqual(parsed['validate_kernel'](m['kernel_log'],e)['kernel_lines'],1)
            with self.assertRaises(parsed['ProbeError']):
                parsed['validate_kernel'](m['kernel_log']+'[ 20.0] usb 1-1.4: USB disconnect, device number 3\n',e)

    def test_line_cache_reuses_classification_without_caching_history_acceptance(self):
        source=feedback_stream.runtime_source()+'\n'+Path(boot.__file__).read_text()
        for filtered in (source,connection.runtime_source(source),maintenance._identity_runtime(source)):
            parsed={};exec(filtered,parsed)
            classify=parsed['classify_feedback_log']
            m=self.metadata();e=boot.boot_expected({},m,classify)
            first=parsed['validate_kernel'](m['kernel_log'],e)
            misses=classify.cache_info().misses
            self.assertEqual(parsed['validate_kernel'](m['kernel_log'],e),first)
            self.assertEqual(classify.cache_info().misses,misses)
            fault='[ 20.0] usb 1-1.4: USB disconnect, device number 3'
            for _ in range(2):
                with self.assertRaises(parsed['ProbeError']):
                    parsed['validate_kernel'](m['kernel_log']+fault+'\n',e)
            with self.assertRaises(parsed['ProbeError']):
                parsed['validate_kernel']('[ 0.0] changed boot prefix\n',e)
            forged={**e,'historical_faults':[dict(line=fault,reason=classify(fault))]}
            with self.assertRaises(parsed['ProbeError']):
                parsed['validate_kernel'](m['kernel_log'],forged)
            for index in range(2100):classify(f'[ {index}.0] normal line {index}')
            self.assertLessEqual(classify.cache_info().currsize,2048)
            self.assertEqual(parsed['validate_kernel'](m['kernel_log'],e),first)
            with self.assertRaises(parsed['ProbeError']):
                parsed['validate_kernel'](m['kernel_log']+fault+'\n',e)

    def test_reviewed_return_retains_fault_and_rejects_new_or_changed_history(self):
        m=self.metadata()
        disconnect='[ 20.0] usb 1-1.4: USB disconnect, device number 3'
        returned=m['kernel_log']+disconnect+'\n[ 20.5] ch341-uart converter now attached to ttyUSB0\n'
        receipt=dict(boot_id=m['boot_id'],kernel_sha256=hashlib.sha256(returned.encode()).hexdigest(),
                     kernel_lines=3,fault_line=disconnect)
        with mock.patch.object(boot,'REVIEWED_USB_RETURN',receipt):
            e=boot.boot_expected({},dict(m,kernel_log=returned),self.classify)
            self.assertNotIn(disconnect,e['kernel_prefix'])
            proof=boot.persistent_kernel(returned,e,self.classify)
            self.assertEqual(proof['historical_faults'],[dict(line=disconnect,reason='fault')])
            self.assertFalse(proof['historical_faults_cleared'])
            self.assertFalse(proof['reviewed_usb_return']['cause_known'])
            self.assertFalse(proof['reviewed_usb_return']['automatic_retry'])
            boot.persistent_kernel(returned+'[ 21.0] ordinary event\n',e,self.classify)
            for changed in (returned+disconnect+'\n',returned+'[ 21.0] I/O error\n',
                            returned.replace('20.5','20.6'),returned.rsplit('\n',2)[0]+'\n'):
                with self.assertRaisesRegex(ValueError,'New kernel fault'):
                    boot.persistent_kernel(changed,e,self.classify)
            other=boot.boot_expected({},dict(m,boot_id='7d23364b-a37c-4ca9-a52f-56bb90547d88',kernel_log=returned),self.classify)
            other['reviewed_usb_return']=receipt
            with self.assertRaisesRegex(ValueError,'New kernel fault'):
                boot.persistent_kernel(returned,other,self.classify)


if __name__=='__main__':unittest.main()
