"""ARM transition reproducer, fake UART only; never starts hardware."""
import argparse,ast,json,struct,sys,tempfile,time,unittest
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--expect-old-bug',action='store_true');a=p.parse_args()
sys.path[:0]=[str(a.candidate/'policy'),str(Path(__file__).parents[1]/'x5_policy')]
import motion
from test_motion import FakeTransport,FakePolicy,FakeClock,ready_config
tree=ast.parse(Path(__file__).with_name('test_source_host.py').read_text())
node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Transport')
ns=dict(FakeTransport=FakeTransport,motion=motion,struct=struct)
exec(compile(ast.Module(body=[node],type_ignores=[]),'<memory transport>','exec'),ns)
tree=ast.parse((a.candidate/'service/worker-classes.py').read_text())
node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='OneShotController')
controller_namespace=dict(motion=motion,time=time,request_counts={})
exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual native controller>','exec'),controller_namespace)
Controller=controller_namespace['OneShotController']
class Transport(ns['Transport']):
 missing=False;invalid_imu=False
 def request(self,kind,payload=b''):
  result=super().request(kind,payload)
  if kind==motion.SOURCE_STATE and self.armed:
   packet=bytearray(result)
   if self.missing:struct.pack_into('<H',packet,8,motion.ALL_MASK&~1)
   if self.invalid_imu:
    fields=list(motion.STATE_PACKET.unpack(packet[:motion.STATE_PACKET.size]))
    fields[6]=0
    packet[:motion.STATE_PACKET.size]=motion.STATE_PACKET.pack(*fields)
   return bytes(packet)
  return result

class TransitionTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
  path=Path(self.tmp.name)/'config.json';path.write_text(json.dumps(ready_config()))
  self.clock=FakeClock();self.t=Transport('memory-only');self.t.gyro=[0.,0.,0.]
  self.c=Controller({n:FakePolicy(n) for n in motion.MODEL_MODES},path,transport_factory=lambda _:self.t,clock=self.clock)
  self.c._connect_once()
 def test_partial_first_post_arm_feedback_transition(self):
  c=self.c;generation=c._generation;self.t.missing=True
  if a.expect_old_bug:
   with self.assertRaisesRegex(AttributeError,'angles'):
    c.start_mode('hold',expected_generation=generation)
   self.assertTrue(self.t.armed);self.assertEqual(c._mode,'hold')
   return
  result=c.start_mode('hold',expected_generation=generation)
  self.assertEqual(c._mode,'hold');self.assertTrue(self.t.armed)
  self.assertIsNotNone(c._filtered);self.assertEqual(result['generation'],generation+1)
  self.assertEqual(sum(k==motion.ARM for k,_ in self.t.requests),1)
 def test_invalid_imu_after_arm_releases_before_mode_commit(self):
  if a.expect_old_bug:self.skipTest('baseline characterization is partial STATE bug only')
  c=self.c;generation=c._generation;self.t.invalid_imu=True
  with self.assertRaises(motion.MotionError):c.start_mode('hold',expected_generation=generation)
  self.assertFalse(self.t.armed);self.assertEqual(c._mode,'off')
  self.assertIsNone(c._cookie if c._cookie else None)
  self.assertFalse(c._state['armed']);self.assertEqual(c._state['torque_mask'],0)
  self.assertTrue(any(k==motion.STOP for k,_ in self.t.requests))
  self.assertTrue(c.status()['release_confirmed'])
 def test_off_prearm_partial_feedback_still_rejected_without_arm(self):
  if a.expect_old_bug:self.skipTest('candidate regression')
  c=self.c;c._state['online_mask']&=~1
  with self.assertRaises(motion.MotionError):c.start_mode('hold',expected_generation=c._generation)
  self.assertFalse(self.t.armed);self.assertFalse(any(k==motion.ARM for k,_ in self.t.requests))

result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(TransitionTests))
a.output.write_text(json.dumps(dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skipped=len(result.skipped),passed=result.wasSuccessful(),hardware_access=False,baseline_reproduced=a.expect_old_bug),indent=2)+'\n')
raise SystemExit(not result.wasSuccessful())
