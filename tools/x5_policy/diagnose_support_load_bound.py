"""Necessary static load bounds from saved, source-bound dynamic trajectories.

No forward simulation, hardware, actuator change or optimization of a control
policy. Whole-foot XY projections on the flat ground are optimistic contact
sets: this intentionally computes a LOWER bound on required swing-foot load.
A positive lower bound excludes quasi-static single support at that recorded
configuration, not dynamic stepping or other configurations.
"""
import hashlib,json,time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import linprog
from scipy.spatial import ConvexHull
from simulation_evidence import world_com

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'


def hull_xy(points):
    return points[ConvexHull(points[:,:2]).vertices,:2]


def minimum_swing_load(left,right,com,swing=0):
    points=np.concatenate((left,right));c=np.r_[np.ones(len(left)),np.zeros(len(right))]
    if swing:c=1-c
    constraints=np.vstack((np.ones(len(points)),points.T))
    result=linprog(c,A_eq=constraints,b_eq=np.r_[1.,com[:2]],bounds=(0.,None),method='highs')
    if not result.success:return dict(feasible=False,message=result.message)
    residual=float(abs(constraints@result.x-np.r_[1.,com[:2]]).max())
    assert residual<1e-7 and result.x.min()>-1e-8
    return dict(feasible=True,minimum_swing_load_fraction=float(result.fun),equilibrium_residual=residual)


def main():
    started=time.monotonic();cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    c=json.loads(cp.read_text())['candidate'];xml=Path(c['reset_placement']['xml_path'])
    assert hashlib.sha256(xml.read_bytes()).hexdigest()==c['reset_placement']['xml_sha256']
    model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model)
    adr=np.asarray([model.jnt_qposadr[model.joint(n).id] for n in c['joint_names']])
    feet={};vertices={}
    for side in ('left','right'):
        gid=model.geom(side+'_foot_collision').id;feet[side]=gid;mesh=model.geom_dataid[gid]
        a,n=model.mesh_vertadr[mesh],model.mesh_vertnum[mesh];vertices[side]=model.mesh_vert[a:a+n].astype(float)
    report=dict(completed=False,hardware_access=False,forward_simulation=False,maximum_wall_seconds=90,
        semantics=__doc__,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),cp,xml,Path(__file__).with_name('simulation_evidence.py'))},runs=[])
    out=BASE/'support-load-bound-v1';out.mkdir(exist_ok=False)
    (out/'source.py').write_bytes(Path(__file__).read_bytes())
    def save():(out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    save()
    paths=[BASE/f'nonlinear-swing-clearance-v4/seed-{s}' for s in range(4)]
    paths+=[BASE/f'transfer-preservation-ablation-v1/preserve-seed-{s}' for s in range(4)]
    try:
        for directory in paths:
            if time.monotonic()-started>90:raise TimeoutError('Bounded load analysis')
            src=directory/'search.json';s=json.loads(src.read_text());g=s['generations'][0]
            trace=directory/'generation-00.npz';assert hashlib.sha256(trace.read_bytes()).hexdigest()==g['trace_sha256']
            z=np.load(trace);bp=g['env0_effective_body_properties'];mass=np.asarray(bp['body_mass_kg']);ipos=np.asarray(bp['body_ipos_m'])
            assert len(mass)==model.nbody+1 and mass[0]==mass[1]==0
            mass=np.r_[mass[:1],mass[2:]];ipos=np.concatenate((ipos[:1],ipos[2:]))
            records=[];errors=[]
            for i in range(len(z['root'])):
                if not z['active_before'][i,0]:break
                data.qpos[:7]=z['root'][i,0];data.qpos[adr]=z['actual'][i,0];mujoco.mj_forward(model,data)
                com=world_com(data,mass,ipos);hulls={};minima=[]
                for side,gid in feet.items():
                    pts=vertices[side]@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
                    hulls[side]=hull_xy(pts);minima.append(float(pts[:,2].min()))
                error=float(abs(np.asarray(minima)-z['height'][i,0]).max());errors.append(error)
                assert error<2e-7
                bound=minimum_swing_load(hulls['left'],hulls['right'],com)
                planes=ConvexHull(hulls['right']).equations
                stance_margin=float(-(planes[:,:2]@com[:2]+planes[:,2]).max())
                rec=dict(step=i,time_s=(i+1)*.02,tilt_deg=float(z['tilt'][i,0]),com_m=com.tolist(),
                    stance_projected_hull_margin_m=stance_margin,
                    actual_swing_load_fraction=float(z['load'][i,0,0]/max(1e-12,z['load'][i,0].sum())),
                    static_load_bound=bound,actual_clearance_m=float(z['height'][i,0,0]))
                records.append(rec)
            finite=[r['static_load_bound']['minimum_swing_load_fraction'] for r in records if r['static_load_bound']['feasible']]
            run=dict(source=str(directory.relative_to(BASE)),source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),trace_sha256=g['trace_sha256'],
                initial_state_sha256=g['initial_state_sha256'],effective_mass_kg=float(mass.sum()),maximum_geometry_error_m=max(errors),
                recorded_frames=len(records),statically_infeasible_with_even_both_projected_feet=sum(not r['static_load_bound']['feasible'] for r in records),
                frames_admitting_static_single_support=sum(v<=1e-6 for v in finite),minimum_static_swing_load_fraction=min(finite,default=None),
                prepare_end=records[min(len(records)-1,34)],lift_peak=records[min(len(records)-1,55)],
                least_measured_swing_load=min(records,key=lambda r:r['actual_swing_load_fraction']),records=records)
            report['runs'].append(run);save()
            print(json.dumps({k:v for k,v in run.items() if k not in ('records','prepare_end','lift_peak','least_measured_swing_load')}),flush=True)
        report['completed']=True
    except Exception as e:report['error']=str(e);raise
    finally:report['wall_seconds']=time.monotonic()-started;save()


if __name__=='__main__':main()
