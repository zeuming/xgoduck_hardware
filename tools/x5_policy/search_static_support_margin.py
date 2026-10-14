#!/usr/bin/env python3
"""Bounded static support-margin maximization, not a gait or a plant mutation.

The previous local solver required an extra 2mm interior COM margin. Here the
margin itself is the objective, under the same CAD/saved joint bounds, fixed
head, actual saved mass/COM, 15-degree torso boundary and 3mm whole-sole lift.
Support-foot pose is fixed only inside this geometric model. No result may be
sent to a robot or counted as a dynamically reachable state.
"""
import argparse,hashlib,json,time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import differential_evolution,minimize,NonlinearConstraint
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from simulation_evidence import world_com

ROOT=Path(__file__).resolve().parents[2]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--output',type=Path,required=True)
 p.add_argument('--seconds-per-side',type=float,default=45)
 p.add_argument('--generations',type=int,default=200)
 p.add_argument('--free-head-counterfactual',action='store_true',help='Geometry only: declared incompatibility with the fixed-head plant')
 a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
 sp=ROOT/'output/duck_sim/overnight-20261011/weight-transfer-multistart-v5/summary.json'
 c=json.loads(cp.read_text())['candidate'];source=json.loads(sp.read_text())
 xml=Path(c['reset_placement']['xml_path']);assert sha(xml)==c['reset_placement']['xml_sha256']
 m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
 joints=np.array([m.joint(n).id for n in c['joint_names']]);adr=m.jnt_qposadr[joints]
 lo=np.maximum(c['lower_rad'],m.jnt_range[joints,0])+1e-6
 hi=np.minimum(c['upper_rad'],m.jnt_range[joints,1])-1e-6
 idx=np.array([i for i,sid in enumerate(c['policy_joint_ids']) if sid<30 or a.free_head_counterfactual])
 report=dict(scope=__doc__,hardware_access=False,physics_modified=False,limits_modified=False,
  static_only=True,dynamic_path_validated=False,walking_accepted=False,
  fixed_head_controller_compatible=not a.free_head_counterfactual,
  free_head_counterfactual=a.free_head_counterfactual,
  sources={str(x):sha(x) for x in (Path(__file__),cp,sp,xml)},cases=[],complete=False)
 def save():(a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
 (a.output/'source.py').write_bytes(Path(__file__).read_bytes());save()
 for src in source['cases']:
  started=time.monotonic();qpos=np.array(src['settled_qpos']);q0=qpos[adr].copy()
  mass=np.array(src['effective_body_properties']['body_mass_kg']);ipos=np.array(src['effective_body_properties']['body_ipos_m'])
  assert len(mass)==m.nbody+1 and mass[0]==mass[1]==0
  mass=np.r_[mass[:1],mass[2:]];ipos=np.concatenate((ipos[:1],ipos[2:]))
  check=next(s for s in src['stages'] if s['geometric_witness'])
  d.qpos[:]=check['qpos'];mujoco.mj_forward(m,d)
  np.testing.assert_allclose(world_com(d,mass,ipos),check['com_m'],atol=2e-7,rtol=0)
  swing=src['swing'];stance='right' if swing=='left' else 'left'
  gids={s:m.geom(s+'_foot_collision').id for s in (swing,stance)};vertices={}
  for s,g in gids.items():
   mid=m.geom_dataid[g];start=m.mesh_vertadr[mid];n=m.mesh_vertnum[mid]
   vertices[s]=m.mesh_vert[start:start+n].astype(float)
  d.qpos[:]=qpos;mujoco.mj_forward(m,d)
  poses={s:(d.geom_xpos[g].copy(),d.geom_xmat[g].reshape(3,3).copy()) for s,g in gids.items()}
  planes=ConvexHull(np.array(src['support_polygon_xy_m'])).equations
  rootrot=Rotation.from_quat(qpos[3:7],scalar_first=True).as_matrix()
  cache={};evaluations=0;best=None
  def geometry(v):
   nonlocal evaluations,best
   key=v.tobytes()
   if cache.get('key')==key:return cache['value']
   evaluations+=1;q=q0.copy();q[idx]=v
   d.qpos[:]=qpos;d.qpos[adr]=q;mujoco.mj_forward(m,d)
   g=gids[stance];correction=poses[stance][1]@d.geom_xmat[g].reshape(3,3).T
   d.qpos[:3]=poses[stance][0]+correction@(qpos[:3]-d.geom_xpos[g])
   rot=correction@rootrot;d.qpos[3:7]=Rotation.from_matrix(rot).as_quat(scalar_first=True)
   mujoco.mj_forward(m,d)
   np.testing.assert_allclose(d.geom_xpos[g],poses[stance][0],atol=1e-10,rtol=0)
   np.testing.assert_allclose(d.geom_xmat[g].reshape(3,3),poses[stance][1],atol=1e-10,rtol=0)
   com=world_com(d,mass,ipos);wg=gids[swing]
   sole=vertices[swing]@d.geom_xmat[wg].reshape(3,3).T+d.geom_xpos[wg]
   margin=float(-(planes[:,:2]@com[:2]+planes[:,2]).max())
   height=float(sole[:,2].min());con=np.array([rot[2,2]-np.cos(np.deg2rad(14.9999)),(height-.003)/.1])
   value=dict(margin=margin,constraints=con,q=q.copy(),qpos=d.qpos.copy(),com=com.copy(),height=height,
    tilt=float(np.rad2deg(np.arccos(np.clip(rot[2,2],-1,1)))))
   if con.min()>=-1e-8 and (best is None or margin>best['margin']):best=value
   cache.update(key=key,value=value);return value
  bounds=list(zip(lo[idx],hi[idx]))
  def callback(*args,**kwargs):return time.monotonic()-started>a.seconds_per_side
  fit=differential_evolution(lambda v:-geometry(v)['margin'],bounds,
   constraints=NonlinearConstraint(lambda v:geometry(v)['constraints'],0,np.inf),
   seed=20261011,popsize=12,maxiter=a.generations,polish=False,callback=callback,tol=1e-7)
  records=[]
  starts=[('global',fit.x),('settled',np.clip(q0[idx],lo[idx],hi[idx]))]
  if best is not None:starts.append(('best_feasible',best['q'][idx]))
  for name,x in starts:
   if time.monotonic()-started>a.seconds_per_side+10:break
   polished=minimize(lambda v:-geometry(v)['margin'],x,method='SLSQP',bounds=bounds,
    constraints=[dict(type='ineq',fun=lambda v:geometry(v)['constraints'])],options=dict(maxiter=250,ftol=1e-11))
   v=geometry(polished.x)
   records.append(dict(start=name,success=bool(polished.success),margin_m=v['margin'],tilt_deg=v['tilt'],
    whole_sole_min_m=v['height'],constraint_min=float(v['constraints'].min())))
  chosen=best if best is not None else geometry(fit.x)
  # Geometric endpoint collision is checked independently; reachability and
  # actuator torque are still untested even if this endpoint passes.
  d.qpos[:]=chosen['qpos'];mujoco.mj_forward(m,d)
  collisions=[]
  for contact in d.contact:
   g1,g2=map(int,contact.geom)
   if g1>=0 and g2>=0 and m.geom_bodyid[g1]!=0 and m.geom_bodyid[g2]!=0 and contact.dist<0:
    collisions.append(dict(geom1=m.geom(g1).name,geom2=m.geom(g2).name,penetration_m=float(-contact.dist)))
  rec=dict(side=swing,actual_saved_mass_kg=float(mass.sum()),global_generations=int(fit.nit),
   evaluations=evaluations,wall_seconds=time.monotonic()-started,local_polish=records,
   best_margin_m=chosen['margin'],tilt_deg=chosen['tilt'],whole_sole_min_m=chosen['height'],
   constraints=chosen['constraints'].tolist(),joint_rad=chosen['q'].tolist(),qpos=chosen['qpos'].tolist(),
   com_m=chosen['com'].tolist(),self_penetrations=collisions,
   endpoint_witness=bool(chosen['constraints'].min()>=-1e-8 and chosen['margin']>0 and not collisions),
   not_an_infeasibility_proof=True,not_a_gait=True)
  report['cases'].append(rec);save();print(json.dumps({k:v for k,v in rec.items() if k not in ('joint_rad','qpos','local_polish')}),flush=True)
 report['complete']=True;save()

if __name__=='__main__':main()
