"""Local ideal-contact inverse-dynamics authority diagnostic, never executed.

This LP freezes current mass, bias, friction and motor law. It asks for a 3mm
sole endpoint by the existing lift-peak time, fixes stance acceleration and
constrains a local linearization of body tilt to the unchanged 15deg boundary.
Nonlinear endpoint geometry is independently evaluated. Ground
contacts use an optimistic full sole projection and a conservative friction
pyramid. Actuator target delays and EMA are omitted, explicitly optimistically.
Solutions are instantaneous conditional demands, NOT feasible paths or a teacher.
Failure is only failure of this local constrained task, not physical impossibility.
"""
import hashlib,json,time
from pathlib import Path
import numpy as np
import mujoco
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

ROOT=Path(__file__).resolve().parents[2];BASE=ROOT/'output/duck_sim/overnight-20261011'


def main():
    started=time.monotonic();out=BASE/'inverse-contact-authority-v2';out.mkdir(exist_ok=False)
    cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json';c=json.loads(cp.read_text())['candidate']
    folder=BASE/'full-dynamics-replay-v3';summary=json.loads((folder/'dynamics-summary.json').read_text())
    assert summary['completed'] and all(summary['uninstrumented_array_parity'].values())
    source=json.loads((folder/'search.json').read_text());z=np.load(folder/'full-dynamics.npz');tap=np.load(folder/'generation-00.npz')
    xml=Path(c['reset_placement']['xml_path']);assert hashlib.sha256(xml.read_bytes()).hexdigest()==c['reset_placement']['xml_sha256']
    model=mujoco.MjModel.from_xml_path(str(xml));data=mujoco.MjData(model);minus=mujoco.MjData(model);plus=mujoco.MjData(model)
    joints=np.asarray([model.joint(n).id for n in c['joint_names']]);adr=model.jnt_qposadr[joints];vadr=model.jnt_dofadr[joints]
    order=np.r_[np.arange(6),vadr];assert len(set(order))==20
    low=np.maximum(c['lower_rad'],model.jnt_range[joints,0]);high=np.minimum(c['upper_rad'],model.jnt_range[joints,1])
    head=np.array([5,6,7,8]);properties=summary['effective_model'];bam=properties['bam']
    # Reconstruct the effective inertia independently in native float64 MuJoCo.
    for name in ('body_mass','body_ipos','body_iquat','body_inertia'):
        values=np.asarray(properties[name]);assert len(values)==model.nbody+1
        getattr(model,name)[:]=np.concatenate((values[:1],values[2:]))
    model.dof_armature[order]=properties['dof_armature']
    mujoco.mj_setConst(model,data)
    feet=[]
    for side in ('left','right'):
        gid=model.geom(side+'_foot_collision').id;mid=model.geom_dataid[gid];a,n=model.mesh_vertadr[mid],model.mesh_vertnum[mid]
        feet.append((gid,int(model.geom_bodyid[gid]),model.mesh_vert[a:a+n].astype(float)))
    friction=[v for name,v in zip(properties['geom_names'],properties['geom_friction']) if name and name.split('/')[-1]=='right_foot_collision'][0][0]
    report=dict(completed=False,scope=__doc__,hardware_access=False,controller_created=False,walking_qualified=False,
        maximum_wall_seconds=90,mu_effective_foot=float(friction),rows=[],sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (Path(__file__),cp,xml,folder/'dynamics-summary.json',folder/'full-dynamics.npz',folder/'generation-00.npz',folder/'search.json')})
    (out/'source.py').write_bytes(Path(__file__).read_bytes())
    def save():(out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    def jac(point,body,d):
        jp=np.zeros((3,20));jr=np.zeros_like(jp);mujoco.mj_jac(model,d,jp,jr,point,body)
        return jp[:,order],jr[:,order]
    def tilt_coordinates(q):
        minus.qpos[:]=q;mujoco.mj_forward(model,minus)
        R=minus.xmat[int(model.jnt_bodyid[0])].reshape(3,3)
        return np.array([np.arctan2(R[2,1],R[2,2]),np.arcsin(np.clip(-R[2,0],-1,1))])
    peak=sum(source['generations'][0]['retiming'][k][0] for k in ('lift_start_s','swing_rise_s'))
    try:
        for frame in (47,49,51):
            if time.monotonic()-started>90:raise TimeoutError('Bounded inverse-contact diagnosis')
            M=z['mass_matrix'][frame].astype(float);qvel=z['qvel'][frame].astype(float)
            balance=M@z['qacc'][frame]-(z['qfrc_smooth'][frame]+z['qfrc_constraint'][frame])
            if abs(balance).max()>2e-4:raise ValueError('Selected state fails native force-balance check')
            data.qpos[:7]=z['qpos'][frame,:7];data.qpos[adr]=tap['actual'][frame,0];data.qvel[order]=qvel;mujoco.mj_forward(model,data)
            native_M=np.zeros((20,20));mujoco.mj_fullM(model,data,native_M)
            mass_error=float(abs(native_M[np.ix_(order,order)]-M).max())
            if mass_error>2e-7:raise ValueError(f'Independent mass matrix mismatch: {mass_error}')
            horizon=peak-(frame+1)*.02;assert .05<horizon<.2
            initial_tilt=tilt_coordinates(data.qpos);tilt_jac=np.zeros((2,20))
            for j in range(3,6):
                dv=np.zeros(20);dv[j]=1.;parts=[]
                for sign in (-1,1):
                    q=data.qpos.copy();mujoco.mj_integratePos(model,q,dv,sign*1e-5);parts.append(tilt_coordinates(q))
                tilt_jac[:,j]=(parts[1]-parts[0])/2e-5
            tilt_v=tilt_jac@qvel
            for d,sgn in ((minus,-1),(plus,1)):
                d.qpos[:]=data.qpos;mujoco.mj_integratePos(model,d.qpos,data.qvel,sgn*1e-5);mujoco.mj_forward(model,d)
            def body_jac(gid,body,local):
                def at(d):
                    point=local@d.geom_xmat[gid].reshape(3,3).T+d.geom_xpos[gid]
                    return point,jac(point,body,d)
                point,(jp,jr)=at(data);_,(jm,rm)=at(minus);_,(jplus,rplus)=at(plus)
                drift=np.r_[(jplus-jm)@qvel,(rplus-rm)@qvel]/2e-5
                return point,jp,jr,drift
            sg,sb,sv=feet[1];sp=sv@data.geom_xmat[sg].reshape(3,3).T+data.geom_xpos[sg]
            vertices=sp[ConvexHull(sp[:,:2]).vertices].copy();vertices[:,2]=0
            Jforces=np.concatenate([jac(p,sb,data)[0].T for p in vertices],axis=1)
            _,Jp,Jr,stance_drift=body_jac(sg,sb,np.zeros(3))
            swing=[]
            for local in feet[0][2]:
                point,jp,_,drift=body_jac(feet[0][0],feet[0][1],local)
                requirement=2*(.003-point[2]-(jp@qvel)[2]*horizon)/(horizon*horizon)-drift[2]
                swing.append((jp[2],float(requirement)))
            nforce=Jforces.shape[1];size=20+14+nforce+14
            Aeq=[];beq=[];Aub=[];bub=[]
            dyn=np.zeros((20,size));dyn[:,:20]=M;dyn[6:20,20:34]=-np.eye(14);dyn[:,34:34+nforce]=-Jforces
            # Native joint friction is held at its recorded value; contact force
            # distribution is replaced. This is not a friction identification.
            rhs=z['qfrc_passive'][frame]+tap['bam_friction_force'][frame,0]-z['qfrc_bias'][frame]
            Aeq.extend(dyn);beq.extend(rhs)
            for row,b in zip(np.vstack((Jp,Jr)),-stance_drift):
                v=np.zeros(size);v[:20]=row;Aeq.append(v);beq.append(float(b))
            # Inner polygon of the 15-degree local roll/pitch disk; it neither
            # changes the original gate nor guarantees nonlinear path safety.
            for angle in np.arange(16)*2*np.pi/16:
                normal=np.array([np.cos(angle),np.sin(angle)])
                v=np.zeros(size);v[:20]=.5*horizon*horizon*(normal@tilt_jac)
                Aub.append(v);bub.append(float(np.deg2rad(15)*np.cos(np.pi/16)-normal@(initial_tilt+tilt_v*horizon)))
            for row,b in swing:
                v=np.zeros(size);v[:20]=-row;Aub.append(v);bub.append(-b)
            for k in range(len(vertices)):
                at=34+3*k
                for x,y in ((1,1),(1,-1),(-1,1),(-1,-1)):
                    v=np.zeros(size);v[at:at+3]=[x,y,-friction];Aub.append(v);bub.append(0.)
            current=tap['qfrc_actuator'][frame,0,6:].astype(float)
            for j in range(14):
                for sign in (-1,1):
                    v=np.zeros(size);v[20+j]=sign;v[34+nforce+j]=-1;Aub.append(v);bub.append(float(sign*current[j]))
            # Quantify one-frame target authority separately from the entire
            # saved interval. Neither result includes EMA/delay reachability.
            actual=tap['actual'][frame,0].astype(float);vin=tap['bam_vin'][frame,0];kp=tap['bam_kp'][frame,0]
            gain=vin*kp*bam['error_gain']*bam['error_gain_ratio'];kd=np.asarray(bam['kd_scale']).reshape(-1)
            def torque(target):
                voltage=np.clip(gain*(target-actual),-vin*bam['max_pwm'],vin*bam['max_pwm'])
                return bam['kt']/bam['resistance']*(voltage-bam['kt']*kd*qvel[6:])
            for mode in ('full_saved_interval','one_20ms_goal_interval'):
                lo=low.copy();hi=high.copy()
                if mode=='one_20ms_goal_interval':
                    delta=c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn']*.02
                    lo=np.maximum(lo,tap['goal'][frame,0]-delta);hi=np.minimum(hi,tap['goal'][frame,0]+delta)
                lo[head]=c['held_head_rad'];hi[head]=c['held_head_rad']
                tlo,thi=torque(lo),torque(hi)
                bounds=[(None,None)]*20+list(zip(tlo,thi))+[(None,None),(None,None),(0,None)]*len(vertices)+[(0,None)]*14
                for j in range(14):
                    q=actual[j];v=qvel[6+j];bounds[6+j]=((low[j]-q-v*horizon)*2/horizon**2,(high[j]-q-v*horizon)*2/horizon**2)
                cost=np.zeros(size);cost[34+nforce:]=1
                result=linprog(cost,A_ub=np.asarray(Aub),b_ub=bub,A_eq=np.asarray(Aeq),b_eq=beq,bounds=bounds,method='highs',options={'time_limit':10})
                rec=dict(step=frame,time_s=(frame+1)*.02,horizon_s=horizon,mode=mode,feasible=bool(result.success),
                    native_balance_max_error=float(abs(balance).max()),independent_mass_matrix_error=mass_error,message=result.message)
                if result.success:
                    x=result.x;rec.update(maximum_equation_residual=float(abs(np.asarray(Aeq)@x-beq).max()),
                        maximum_inequality_violation=float(max(0,(np.asarray(Aub)@x-bub).max())),
                        qacc_rad_or_m_s2=x[:20].tolist(),motor_torque_nm=x[20:34].tolist(),contact_forces_n=x[34:34+nforce].reshape(-1,3).tolist(),
                        objective_total_torque_change_nm=float(result.fun))
                    assert rec['maximum_equation_residual']<1e-6 and rec['maximum_inequality_violation']<1e-6
                    req=actual+(bam['resistance']/bam['kt']*x[20:34]+bam['kt']*kd*qvel[6:])/gain
                    rec['algebraic_motor_goal_rad']=req.tolist();rec['maximum_goal_change_deg']=float(np.rad2deg(abs(req-tap['goal'][frame,0])).max())
                    predicted=data.qpos.copy();mujoco.mj_integratePos(model,predicted,(qvel+.5*x[:20]*horizon)[np.argsort(order)],horizon)
                    minus.qpos[:]=predicted;mujoco.mj_forward(model,minus)
                    rotation=minus.xmat[int(model.jnt_bodyid[0])].reshape(3,3);tilt=float(np.rad2deg(np.arccos(np.clip(rotation[2,2],-1,1))))
                    rec['constant_acceleration_endpoint_tilt_deg']=tilt
                    gid=feet[0][0];pts=feet[0][2]@minus.geom_xmat[gid].reshape(3,3).T+minus.geom_xpos[gid]
                    rec['nonlinear_endpoint_swing_clearance_m']=float(pts[:,2].min())
                    rec['nonlinear_endpoint_within_original_tilt']=tilt<15
                    rec['not_validated_dynamic_path']=True
                report['rows'].append(rec);save();print(json.dumps(rec),flush=True)
        report['completed']=True
    except Exception as e:report['error']=str(e);raise
    finally:report['wall_seconds']=time.monotonic()-started;save()


if __name__=='__main__':main()
