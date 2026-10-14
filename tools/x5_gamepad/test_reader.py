import copy
import json
import os
from pathlib import Path
import struct
import tempfile
import unittest

import reader
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'x5_policy'))
import gamepad_web


class InputTests(unittest.TestCase):
    def test_atomic_publication_between_call_and_open_is_fresh(self):
        """A scheduler switch before os.open must not make new data 'future'."""
        from unittest.mock import patch
        state=dict(schema=1,receiver='0079:181c',boot_id='test',published_at=10.,
                   connected=True,sequence=0,session='a'*32,axes=[0]*8,buttons=[0]*15,events=[])
        clock=[10.]
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'input.json';path.write_text(json.dumps(state))
            real_open=os.open;real_read=Path.read_text
            def switched_open(file, flags, *a, **kw):
                if Path(file)==path:
                    # Exactly the reader service's publish-then-rename order.
                    state['published_at']=10.01;clock[0]=10.02
                    replacement=path.with_suffix('.new');replacement.write_text(json.dumps(state))
                    os.replace(replacement,path)
                return real_open(file,flags,*a,**kw)
            def read_text(file,*a,**kw):
                return 'test' if str(file)=='/proc/sys/kernel/random/boot_id' else real_read(file,*a,**kw)
            with patch.object(gamepad_web.os,'open',switched_open), \
                 patch.object(gamepad_web.time,'monotonic',lambda:clock[0]), \
                 patch.object(Path,'read_text',read_text):
                value=gamepad_web.snapshot(path)
            self.assertTrue(value['connected'],value)
            self.assertEqual(value['age_ms'],10.)

    def test_stale_error_retains_measured_age(self):
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'input.json';path.write_text(json.dumps(dict(published_at=10.)))
            value=gamepad_web.snapshot(path,now=10.6)
            self.assertFalse(value['connected'])
            self.assertEqual(value['error'],'Input reader stale')
            self.assertEqual(value['age_ms'],600.)

    def test_events_reject_truncated_unknown_or_out_of_range_input(self):
        self.assertEqual(reader.decode(struct.pack('<IhBB',1,-32767,130,4)), (2,4,-32767,True))
        self.assertEqual(reader.decode(struct.pack('<IhBB',1,1,1,7)), (1,7,1,False))
        for raw in (b'', b'x'*7, struct.pack('<IhBB',1,1,1,15),
                    struct.pack('<IhBB',1,1,2,8), struct.pack('<IhBB',1,2,1,1),
                    struct.pack('<IhBB',1,1,3,1)):
            with self.subTest(raw=raw), self.assertRaises(ValueError): reader.decode(raw)

    def test_snapshot_rejects_stale_foreign_boot_and_bad_events(self):
        good=dict(schema=1,receiver='0079:181c',boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                  if Path('/proc/sys/kernel/random/boot_id').exists() else 'test',
                  published_at=10.,connected=True,sequence=1,session='a'*32,
                  axes=[0]*8,buttons=[0]*15,events=[dict(sequence=1,at=9.9,kind=1,number=7,value=1)])
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'state.json'
            # macOS test host has no Linux boot_id file.
            actual_read=Path.read_text
            def read_text(p,*a,**kw):
                return good['boot_id'] if str(p)=='/proc/sys/kernel/random/boot_id' else actual_read(p,*a,**kw)
            with patch.object(Path,'read_text',read_text):
                path.write_text(json.dumps(good))
                out=gamepad_web.snapshot(path,now=10.1)
                self.assertTrue(out['connected']);self.assertEqual(out['events'][0]['age_ms'],200.)
                self.assertFalse(gamepad_web.snapshot(path,now=10.6)['connected'])
                self.assertFalse(gamepad_web.snapshot(path,now=9.9)['connected'])
                changes=[dict(boot_id='foreign'),dict(receiver='other'),dict(axes=[0]*7),
                         dict(events=[dict(sequence=2,at=9.,kind=1,number=7,value=1)]),
                         dict(events=[dict(sequence=1,at=11.,kind=1,number=7,value=1)]),
                         dict(events=[dict(sequence=1,at=9.,kind=1,number=7,value=2)])]
                for change in changes:
                    bad=copy.deepcopy(good);bad.update(change);path.write_text(json.dumps(bad))
                    self.assertFalse(gamepad_web.snapshot(path,now=10.1)['connected'],change)
                path.write_text('x'*(gamepad_web.MAX_BYTES+1))
                self.assertFalse(gamepad_web.snapshot(path,now=10.1)['connected'])


if __name__=='__main__': unittest.main()
