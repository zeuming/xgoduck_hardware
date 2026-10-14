"""Captured zero values remain saveable; synthetic feedback and temporary files."""
from pathlib import Path
import os,sys,types,unittest
from unittest import mock
ROOT=Path(__file__).resolve().parents[2]
stage=os.environ.get('XGO_ZERO_SAVE_STAGE')
if stage:sys.path[:0]=[str(Path(stage)/'native/service'),str(Path(stage).parent/'before/service')]
sys.path.extend([str(ROOT/'tools/x5_recovery'),str(ROOT/'tools/source_parity')])
import native_calibration,native_transports
import test_native_calibration as fixtures

class ZeroSaveRegression(unittest.TestCase):
 def setUp(self):
  self.h=fixtures.NativeCalibrationTests('runTest');self.h.setUp();self.h.start('joints');self.s=self.h.session
 def tearDown(self):self.h.tearDown()
 def body(self,action,**changes):
  v=dict(generation=7,id=22,action=action,support_confirmed=True,zero_reference_confirmed=True);v.update(changes);return v
 def capture(self,raw=1738):
  self.h.positions[7]=raw;self.h.feed(torque_known_mask=32767)
  self.s.joint_zero_action(self.body('capture'))
  self.assertEqual(self.s.joint_zero['stage'],'ready');self.assertEqual(self.s.joint_zero['candidate_zero'],raw)
 def test_capture_fixes_current_reading_without_waiting_for_more_frames(self):
  self.capture();self.assertEqual(self.s.joint_zero['sample_count'],1);self.assertFalse(self.s.joint_zero['busy']);self.assertEqual(self.h.commits,[])
 def test_save_uses_captured_value_after_torque_loss_movement_and_staleness(self):
  self.capture();self.h.feed(torque_known_mask=32639);self.h.positions[7]=1900;self.h.feed(torque_known_mask=32767);self.h.clock.now+=.8
  self.assertEqual(self.s.joint_zero['candidate_zero'],1738)
  self.h.save_after_reader_close('/api/joint-zero',self.body('save',support_confirmed=False,zero_reference_confirmed=False))
  self.assertEqual(self.s.config['joints'][7]['zero'],1738);self.assertEqual(self.s.joint_zero['stage'],'saved')
  for i in range(15):
   if i!=7:self.assertEqual(self.s.config['joints'][i],self.h.original['joints'][i])
 def test_recorded_value_survives_reader_close_and_saves_without_feedback(self):
  self.capture();self.s.end_action(dict(generation=7));self.s.accept(dict(kind='closed',generation=7,normal=True,serial_closed=True));self.s.process.returncode=0;self.s.confirm_closed(0);self.s.connection_done.set()
  self.assertEqual(self.s.joint_zero['candidate_zero'],1738);self.assertIsNone(self.s.feedback)
  self.s.joint_zero_action(self.body('save',support_confirmed=False,zero_reference_confirmed=False))
  self.assertEqual(self.s.config['joints'][7]['zero'],1738)
 def test_save_without_a_captured_value_cannot_modify_calibration(self):
  with self.assertRaises(ValueError):self.s.joint_zero_action(self.body('save'))
  self.assertEqual(self.s.config,self.h.original);self.assertEqual(self.h.commits,[])
 def test_explicit_reader_acquisition_uses_usb_recovery_guard(self):
  old=mock.Mock(side_effect=AssertionError('stale startup guard used'))
  fresh=mock.Mock(return_value={'NATIVE_RUNTIME_SOURCE':'definitions'})
  t=native_transports.NativeLocalTransports(old,self.h.directory,recovery_guard_factory=fresh)
  with mock.patch.object(native_transports,'child_source',return_value=({},b'fresh reader')):
   for scope in ('joints','imu','metadata','maintenance'):self.assertEqual(t._source(7,scope),({},b'fresh reader','definitions'))
  self.assertEqual(fresh.call_count,4);old.assert_not_called()

if __name__=='__main__':unittest.main()
