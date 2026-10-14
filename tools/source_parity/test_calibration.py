"""Passive calibration, pinned ONNX reference and stored-data preservation."""
import argparse,copy,hashlib,importlib.util,json,math,sys,unittest
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
sys.path.insert(0,str(a.candidate/'service'))
import joint_calibration as j
from model_reference import WALK_HOME_RAD,WALK_MODEL_SHA256,WALK_DEFAULT_DEG
ROOT=Path(__file__).resolve().parents[2]
class Tests(unittest.TestCase):
 def config(self):
  return dict(joints=[dict(id=sid,zero=2000,sign=1,min=100,max=3900,direction_confirmed=True) for sid in j.IDS],imu={'unchanged':True})
 def state(self,sid,raw,t):
  values=[2000]*15;values[j.IDS.index(sid)]=raw
  return dict(positions=values,sample_us=t)
 def test_reference_is_generated_from_real_onnx_and_runtime_agrees(self):
  import onnxruntime,numpy as np
  model=ROOT/'output/xgoduck_runtime_upstream/python/xgoduck_walk.onnx'
  self.assertEqual(hashlib.sha256(model.read_bytes()).hexdigest(),WALK_MODEL_SHA256)
  metadata=onnxruntime.InferenceSession(str(model),providers=['CPUExecutionProvider']).get_modelmeta().custom_metadata_map
  self.assertEqual(tuple(map(float,metadata['default_joint_pos'].split(','))),WALK_HOME_RAD)
  spec=importlib.util.spec_from_file_location('paired_core',a.candidate/'policy/rl_core.py');core=importlib.util.module_from_spec(spec);sys.modules[spec.name]=core;spec.loader.exec_module(core)
  np.testing.assert_allclose(core.stance_from_home(core.policy_profile_from_metadata(metadata)['home_rad']),j.DEFAULT_DEG,rtol=0,atol=1e-12)
  self.assertEqual(core.FIXED_STANCE_OVERRIDES,{})
 def test_no_hidden_margin_or_unsupported_displacement_threshold(self):
  self.assertEqual(j.safe_limits(1000,3000,2000,1,0,10),(1000,3000))
  self.assertEqual(j.infer_sign(2000,2001),1);self.assertEqual(j.infer_sign(2000,1999),-1)
  for end in (2000,4048):
   with self.assertRaises(ValueError):j.infer_sign(2000,end)
 def test_default_pose_still_must_fit_without_widening(self):
  with self.assertRaises(j.RangeValidationError):j.safe_limits(1980,2010,2000,1,20,14)
  self.assertEqual(j.safe_limits(2000,2100,2000,1,0,10),(2000,2100))
 def test_one_post_click_fresh_read_preview_requires_explicit_save(self):
  cfg=self.config();before=copy.deepcopy(cfg);w=j.JointWizard();t=1000000
  for action,raw in [('direction-start',2000),('direction-end',2001),('range-a',1500),('range-b',2500)]:
   w.start(action,10,self.state(10,raw,t),100.)
   w.tick(self.state(10,raw,t),100.,cfg);self.assertTrue(w.value['busy'])
   t+=10000;w.tick(self.state(10,raw,t),100.01,cfg)
   self.assertFalse(w.value['busy']);self.assertEqual(w.value['sample_count'],1)
  self.assertEqual(cfg,before);proposed=w.proposed_config(cfg,10)
  self.assertEqual((proposed['joints'][0]['min'],proposed['joints'][0]['max']),(1500,2500))
  self.assertEqual(j.zero_signature(proposed),j.zero_signature(before));self.assertEqual(proposed['imu'],before['imu'])
  self.assertEqual(proposed['joints'][1:],before['joints'][1:])
 def test_saved_ranges_are_never_retroactively_untrimmed(self):
  cfg=self.config();cfg['joints'][0].update(min=1512,max=2488)
  w=j.JointWizard();w.value.update(id=10,stage='await-range-a',busy=False,start_position=2000,end_position=2001,inferred_sign=1)
  new=w.proposed_direction_config(cfg,10)
  self.assertEqual(new,cfg)
  self.assertEqual(j.completed_count(cfg),15)
 def test_stale_backward_wrap_or_duplicate_clock_change_rejected(self):
  for sample,raw in ((999999,2000),(1200000,2000),(1000000,2001),(1010000,4090)):
   w=j.JointWizard();w.start('direction-start',10,self.state(10,2000,1000000),100.)
   with self.assertRaises(ValueError):w.tick(self.state(10,raw,sample),100.01,self.config())
 def test_mouth_does_not_invent_closed_zero_or_opening_margin(self):
  self.assertEqual(j.safe_limits(2000,2001,2000,1,0,34),(2000,2001))
  with self.assertRaises(ValueError):j.safe_limits(1990,1999,2000,1,0,34)
 def test_existing_saved_calibration_bytes_unchanged(self):
  original=ROOT/'output/duck_sim/source-parity-20261012/baseline/saved-calibration.json'
  self.assertEqual((a.candidate/'saved-calibration.json').read_bytes(),original.read_bytes())
r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
a.output.write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,errors=[str(x) for x in r.errors+r.failures],hardware_access=False),indent=2)+'\n');raise SystemExit(not r.wasSuccessful())
