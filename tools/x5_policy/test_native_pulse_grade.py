import copy,unittest
from native_pulse_grade import grade_native

def trace(hz=50):
 rows=[]
 for k in range(round(9.2*hz)):
  t=(k+1)/hz;feet={s:dict(mesh_min_height_m=0.,vertical_load_n=4.) for s in ('left','right')}
  if 5.10-1e-9<=t<=5.20+1e-9:feet['left']=dict(mesh_min_height_m=.004,vertical_load_n=0.)
  if 6.05-1e-9<=t<=6.20+1e-9:feet['right']=dict(mesh_min_height_m=.004,vertical_load_n=0.)
  rows.append(dict(time_s=t,command_m_s=.15 if 5<t<6.2 else 0.,feet=feet,tilt_deg=2.,failed_terms=[],task_complete=k==round(9.2*hz)-1,root_position=[0. if t<=5 else .03,0.,.14],heading_rad=0.))
 return rows
class GateTest(unittest.TestCase):
 def test_landing_after_pulse_counts(self):
  for hz in (40,50):
   g=grade_native(trace(hz),hz);self.assertTrue(g['accepted']);self.assertEqual(g['completed_lift_land_cycles'],[1,1])
 def test_forty_milliseconds_is_not_sixty(self):
  rows=trace()
  for r in rows:
   if r['time_s']>5.14 and r['feet']['left']['vertical_load_n']==0:r['feet']['left']=dict(mesh_min_height_m=0.,vertical_load_n=4.)
  self.assertFalse(grade_native(rows,50)['accepted'])
 def test_sample_gap_breaks_run(self):
  rows=[r for r in trace() if abs(r['time_s']-5.16)>1e-8]
  self.assertFalse(grade_native(rows,50)['accepted'])
 def test_support_and_clearance_and_recovery_are_required(self):
  for kind in ('support','clearance','recovery','tilt','range'):
   rows=trace()
   for r in rows:
    if kind=='support' and r['feet']['left']['vertical_load_n']==0:r['feet']['right']['vertical_load_n']=0
    if kind=='clearance' and r['feet']['left']['vertical_load_n']==0:r['feet']['left']['mesh_min_height_m']=.0029
    if kind=='recovery' and r['time_s']>=8.2:r['feet']['left']['vertical_load_n']=0
    if kind=='tilt' and r['time_s']==6:r['tilt_deg']=15
    if kind=='range' and r['time_s']==6:r['failed_terms']=['range']
   self.assertFalse(grade_native(rows,50)['accepted'],kind)
if __name__=='__main__':unittest.main()
