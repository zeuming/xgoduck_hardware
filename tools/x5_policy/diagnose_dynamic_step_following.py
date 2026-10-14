"""Offline counterfactual kinematics for a recorded unsuccessful step.

Reconstruct actual contact geometry, then replace only joint angles by logged
EMA/slew goals or requested targets at the SAME recorded root. These alternate
geometries are not realizable dynamics or successful steps.
"""
import argparse,hashlib,json
from pathlib import Path


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
 p=argparse.ArgumentParser(description=__doc__)
 for n in ('candidate','search','output'):p.add_argument('--'+n,type=Path,required=True)
 a=p.parse_args()
 from train_constrained_policy import install_offline_guards
 install_offline_guards()
 import numpy as np
 import mujoco
 from constrained_training_candidate import load_candidate,digest
 c=load_candidate(a.candidate);r=json.loads(a.search.read_text());b=r['best']
 if not r['completed'] or not r['source_hashes_unchanged'] or r['candidate_sha256']!=digest(c):raise ValueError('Source incomplete/mismatch')
 a.output.mkdir(exist_ok=False,parents=True)
 trace=a.search.parent/f"generation-{b['generation']:02d}.npz";t=np.load(trace);idx=b['index'];side=0 if r['side']=='left' else 1
 xml=Path(c['reset_placement']['xml_path'])
 if sha(xml)!=c['reset_placement']['xml_sha256']:raise ValueError('Geometry binding changed')
 model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
 adr=np.array([model.jnt_qposadr[model.joint(name).id] for name in c['joint_names']]);foot=[]
 for name in ('left','right'):
  gid=model.geom(name+'_foot_collision').id;mid=int(model.geom_dataid[gid]);v=int(model.mesh_vertadr[mid]);n=int(model.mesh_vertnum[mid]);foot.append((gid,np.asarray(model.mesh_vert[v:v+n],dtype=float)))
 def geometry(root,q):
  data.qpos[:7]=root;data.qpos[adr]=q;mujoco.mj_forward(model,data)
  return [float((vertices@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid])[:,2].min()) for gid,vertices in foot]
 rows=[];error=0.
 for step in range(len(t['tilt'])):
  if not t['active_before'][step,idx] or t['range_violation'][step,idx] or t['tilt'][step,idx]>=15:break
  actual=geometry(t['root'][step,idx],t['actual'][step,idx]);error=max(error,float(np.max(abs(np.array(actual)-t['height'][step,idx]))))
  row=dict(time_s=(step+1)*.02,actual_height_m=actual,requested_at_actual_root_height_m=geometry(t['root'][step,idx],t['requested'][step,idx]),goal_at_actual_root_height_m=geometry(t['root'][step,idx],t['goal'][step,idx]),actual_at_initial_root_height_m=geometry(t['root'][0,idx],t['actual'][step,idx]),load_n=t['load'][step,idx].tolist(),tilt_deg=float(t['tilt'][step,idx]),root_drop_m=float(t['root'][0,idx,2]-t['root'][step,idx,2]),request_actual_error_deg=np.degrees(t['requested'][step,idx]-t['actual'][step,idx]).tolist(),request_goal_error_deg=np.degrees(t['requested'][step,idx]-t['goal'][step,idx]).tolist(),goal_actual_error_deg=np.degrees(t['goal'][step,idx]-t['actual'][step,idx]).tolist())
  rows.append(row)
 if error>3e-7:raise ValueError(f'Reconstruction mismatch {error}')
 relevant=[x for x in rows if x['time_s']<=b['reference_end_s']]
 air=[x for x in relevant if x['requested_at_actual_root_height_m'][side]>=.003 and x['actual_height_m'][side]<.003]
 peak=max(relevant,key=lambda x:x['requested_at_actual_root_height_m'][side])
 stats=dict(reference_samples=len(relevant),request_geometry_above3mm_without_actual_lift_samples=len(air),peak_requested_geometry=peak,
 maximum_actual_height_m=max(x['actual_height_m'][side] for x in relevant),maximum_request_geometry_height_m=max(x['requested_at_actual_root_height_m'][side] for x in relevant),maximum_goal_geometry_height_m=max(x['goal_at_actual_root_height_m'][side] for x in relevant),maximum_actual_with_fixed_initial_root_height_m=max(x['actual_at_initial_root_height_m'][side] for x in relevant),maximum_root_drop_m=max(x['root_drop_m'] for x in relevant),maximum_request_to_goal_lag_deg=max(max(abs(v) for v in x['request_goal_error_deg']) for x in relevant),maximum_goal_to_actual_lag_deg=max(max(abs(v) for v in x['goal_actual_error_deg']) for x in relevant))
 report=dict(scope=__doc__,hardware_access=False,source_search=str(a.search),candidate_sha256=digest(c),selected_attempt=b,kinematic_reconstruction_max_error_m=error,summary=stats,sources={str(p):sha(p) for p in (Path(__file__),a.candidate,a.search,trace,xml)},rows=rows,walking_qualified=False)
 (a.output/'source.py').write_bytes(Path(__file__).read_bytes());(a.output/'diagnosis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(stats))

if __name__=='__main__':main()
