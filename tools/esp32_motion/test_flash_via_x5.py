"""CPU-only checks for the bounded application writer; no device imports."""
import ast
import hashlib
import io
import json
from pathlib import Path
import struct
import tarfile
import unittest
import flash_via_x5 as app

class FlashPreparationTests(unittest.TestCase):
    def helpers(self):
        payload,files=app.bundle();source=app.prepare_source(payload,files).decode()
        tree=ast.parse(source)
        values={node.targets[0].id:ast.literal_eval(node.value) for node in tree.body
                if isinstance(node,ast.Assign) and isinstance(node.targets[0],ast.Name)
                and node.targets[0].id in ('HELPERS','APP_SIZE','REGION_SIZE','APP_OFFSET','SECTOR_SIZE','PARTITION_SHA256')}
        namespace=dict(values,struct=struct,json=json,_sha=lambda data:hashlib.sha256(data).hexdigest())
        exec(values['HELPERS'],namespace)
        return payload,files,source,namespace

    def test_bundle_has_exact_verified_image_and_linux_compatible_pure_python_tools(self):
        payload,files,source,_=self.helpers()
        self.assertLess(len(source.encode()),262144)
        with tarfile.open(fileobj=io.BytesIO(payload),mode='r:gz') as archive:
            self.assertEqual({item.name for item in archive},set(files))
            self.assertEqual(hashlib.sha256(archive.extractfile('firmware.bin').read()).hexdigest(),app.EXPECTED_APP)
        self.assertTrue(any(name.endswith('esp32s3.json') for name in files))
        self.assertFalse(any(name.endswith(('.so','.pyc')) or '__pycache__' in name for name in files))

    def test_partition_identity_is_required_before_an_application_write(self):
        _,_,_,namespace=self.helpers()
        partition=(app.PACKAGE/'partitions.bin').read_bytes()+b'\xff'*1024
        namespace['_check_partition'](partition)
        for value in (partition[:-1],bytes([partition[0]^1])+partition[1:]):
            with self.assertRaises(RuntimeError):namespace['_check_partition'](value)

    def test_write_failure_is_never_replayed_and_stays_in_backed_up_sectors(self):
        _,_,_,namespace=self.helpers();extent=namespace['REGION_SIZE'];attempts=[]
        class Stub:
            ESP_FLASH_DATA=3;CHIP_DETECT_MAGIC_REG_ADDR=0x40001000
            def get_erase_size(self,offset,size):
                self.offset,self.size=offset,size;return size
            def flash_begin(self,size,offset):
                self.started=(size,offset);return size//4096
            def checksum(self,data):return 0
            def check_command(self,label,kind,data,checksum,timeout):
                length,sequence,zero1,zero2=struct.unpack_from('<IIII',data)
                attempts.append(sequence)
                assert length==4096 and len(data)==4112 and (zero1,zero2)==(0,0)
                if sequence==3:raise OSError('injected uncertain block write')
            def read_reg(self,*args,**kwargs):raise AssertionError('No success barrier after a failed block')
        stub=Stub()
        with self.assertRaisesRegex(OSError,'uncertain'):
            namespace['_write_app_once'](stub,b'x'*extent)
        self.assertEqual(attempts,[0,1,2,3])
        self.assertEqual(stub.started,(extent,0x10000))
        with self.assertRaises(RuntimeError):namespace['_write_app_once'](Stub(),b'x'*(extent-1))

if __name__=='__main__':unittest.main()
