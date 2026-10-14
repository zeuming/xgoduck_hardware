"""Source voltage decoder/freshness regressions against packaged controller, no hardware."""
import argparse,runpy,sys,struct,unittest,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.argv=['test_source_host','--candidate',str(a.candidate),'--output',str(a.output)]
n=runpy.run_path(str(Path(__file__).with_name('test_source_host.py')),run_name='source_test_helpers')
motion=n['motion'];Base=n['RuntimeTests']
class VoltageTests(Base):
 def packet(self,raw=74,seen=32767,valid=32767,age=1,stop=False):
  s=n['Transport'].request(self.transport,motion.SOURCE_STATE)
  s=bytearray(s);s[109]=4 if stop else 3
  if stop:
   s.extend(motion.TARGET_STOP_PACKET.pack(motion.TARGET,2,1,1,5,1,32767,0,0,10,0,0,250001,0,255,0,0,0,0)+struct.pack('<I',123))
  return bytes(s)+struct.pack('<BHH15B15B',1,seen,valid,*([raw]*15),*([age]*15))
 def inject(self,packet):
  original=self.transport.request
  self.transport.request=lambda kind,payload=b'':packet if kind==motion.SOURCE_STATE else original(kind,payload)
  self.c._accept_state_locked(self.c._read_state())
 def test_live_voltage(self):
  self.inject(self.packet());s=self.c.status()
  self.assertTrue(s['voltage_supported']);self.assertTrue(all(r['volts']==7.4 and r['fresh'] for r in s['motor_voltages']))
  self.assertEqual(s['servo_voltage_polling']['additional_servo_requests'],0)
  self.clock.advance(1.1);self.assertFalse(any(r['fresh'] for r in self.c.status()['motor_voltages']))
 def test_missing_and_zero_unknown(self):
  self.inject(self.packet(seen=32766,valid=32766));s=self.c.status();self.assertIsNone(s['motor_voltages'][0]['volts'])
  with self.assertRaises(motion.MotionError):self.inject(self.packet(raw=0))
 def test_low_high_and_stale(self):
  for raw,condition in [(39,'low'),(85,'high')]:
   self.inject(self.packet(raw));self.assertEqual(self.c.status()['motor_voltages'][0]['condition'],condition)
  self.inject(self.packet(age=11));self.assertFalse(self.c.status()['motor_voltages'][0]['fresh'])
 def test_invalid_masks_and_shapes(self):
  for packet in (self.packet(seen=0,valid=1),self.packet(seen=65535),self.packet()[:-1]):
   with self.assertRaises(motion.MotionError):self.inject(packet)
 def test_stop_preserved(self):
  self.inject(self.packet(stop=True));self.assertEqual(self.c._state['firmware_first_stop']['arm_cookie'],123)
  self.assertTrue(self.c.status()['voltage_supported'])
# Run only new cases, inherited cases have their own recorded suite.
names=[x for x in VoltageTests.__dict__ if x.startswith('test_')]
r=unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(VoltageTests(x) for x in names))
a.output.write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,hardware_access=False),indent=2)+'\n');raise SystemExit(not r.wasSuccessful())
