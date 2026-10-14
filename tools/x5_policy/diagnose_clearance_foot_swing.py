"""Clearance-only IK teacher with free foot orientation; offline only.

This is a new offline reference-controller experiment, not PPO or a walking
policy. The original plant, EMA, 600tick/s limit, saved bounds and 15deg gate
remain unchanged. Search uses zero interval pushes with unchanged draw shapes;
successful candidates still require independent validation with original pushes.
"""
import argparse
import hashlib
import importlib.util
import inspect
import json
import os
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parents[2]
EVALUATOR=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py'
NAMES=['transfer_scale','lift_scale','bilateral_crouch_scale','stance_lift_scale',
       'prepare_s','lift_rise_s','lift_hold_s','return_s']
LOW=[-.5,.2,0.,-1.,.15,.12,0.,.15]
HIGH=[1.4,3.,1.5,1.,1.1,.6,.15,1.1]
INITIAL=[.8,1.2,.15,0.,.6,.25,.04,.5]


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def build_trajectory(parameters,transfer,lift,stance,slew,lift_delay=0.,swing_extra=None,return_fraction=None):
    import numpy as np
    from diagnose_dynamic_step_preload import build_trajectory as baseline
    from cubic_reference import retime,evaluate
    if return_fraction is None:
        return baseline(parameters,transfer,lift,stance,slew,lift_delay,swing_extra)
    x=np.asarray(parameters,dtype=float)
    prep=x[:,0,None]*transfer+x[:,2,None]*(lift+stance)
    swing=x[:,1,None]*lift+x[:,3,None]*stance
    if swing_extra is not None:swing=swing+np.asarray(swing_extra,dtype=float)
    tp=np.maximum(x[:,4],1.5*np.max(np.abs(prep),axis=1)/slew)
    tr=np.maximum(x[:,5],1.5*np.max(np.abs(swing),axis=1)/slew)
    back=np.maximum(x[:,7],1.5*np.max(np.abs(prep),axis=1)/slew)
    begin=np.maximum(.02,tp+lift_delay)
    paths=[];end=[];audits=[]
    for i in range(len(x)):
        ret=begin[i]+return_fraction*tr[i]
        pulses=[(0.,tp[i],prep[i]),(ret,back[i],-prep[i]),
                (begin[i],tr[i],swing[i]),(begin[i]+tr[i]+x[i,6],tr[i],-swing[i])]
        path,audit=retime(pulses,slew);paths.append(path);end.append(max(st+duration for st,duration,_ in path));audits.append(audit)
    end=np.asarray(end)
    def at(t):return np.stack([evaluate(path,t) for path in paths]),end
    return at,end,dict(composite_rate_proof='velocity quadratic on each knot interval; exact endpoints and stationary points, global time dilation',
        return_fraction_of_lift_rise=return_fraction,paths=[[(float(st),float(dur),delta.tolist()) for st,dur,delta in p] for p in paths],rate_audits=audits)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('candidate','run','basis','output'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--side',choices=('left','right'),default='left')
    p.add_argument('--batch',type=int,default=32)
    p.add_argument('--generations',type=int,default=12)
    p.add_argument('--wall-seconds',type=float,default=420.)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--parameters',type=Path)
    p.add_argument('--nonlinear-swing-ik',action='store_true')
    p.add_argument('--return-at-lift-fraction',type=float,default=None)
    p.add_argument('--swing-preload-scale',type=float,choices=(0.,1.),default=0.)
    p.add_argument('--lift-delay-seconds',type=float,default=0.)
    p.add_argument('--push-scale',type=float,choices=(0.,1.),default=0.)
    a=p.parse_args()
    if a.nonlinear_swing_ik and (a.batch!=1 or a.return_at_lift_fraction is not None):raise ValueError('IK teacher initially only one world and original phase')
    if a.return_at_lift_fraction is not None and not 0.<=a.return_at_lift_fraction<=2.:raise ValueError('Bounded return fraction')
    if not a.parameters or a.generations!=1:raise ValueError('Only frozen-parameter paired diagnoses supported')
    if not -.5<=a.lift_delay_seconds<=1.:raise ValueError('Bounded phase diagnosis')
    if not 1<=a.batch<=64 or not 1<=a.generations<=16:raise ValueError('Bounded search size')
    if a.parameters and a.generations!=1:raise ValueError('Frozen evaluation must be one generation')
    a.output.mkdir(parents=True,exist_ok=False);os.environ['MPLCONFIGDIR']=str(a.output/'mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np
    import torch
    import mujoco
    import onnxruntime as ort
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    from stand_button_training_profile import apply_task_profile
    from simulation_evidence import audit_interval_push,effective_body_properties
    from mjlab.envs import ManagerBasedRlEnv
    c=load_candidate(a.candidate);r=json.loads((a.run/'run.json').read_text());basis=json.loads(a.basis.read_text())
    policy=a.run/'unvalidated_actor.onnx'
    if not basis['completed'] or basis['candidate_sha256']!=digest(c) or r['candidate_sha256']!=digest(c) or sha(policy)!=r['export']['sha256']:
        raise ValueError('Candidate/basis/source actor binding differs')
    sources={str(x):sha(x) for x in (Path(__file__),EVALUATOR,a.candidate,a.run/'run.json',a.basis,policy,Path(inspect.getfile(audit_interval_push)))}
    for helper in ('cubic_reference.py','diagnose_dynamic_step_preload.py','nonlinear_swing_reference.py','nonlinear_swing_reference_current.py','nonlinear_swing_reference_precise.py','clearance_swing_reference.py'):
        path=Path(__file__).with_name(helper);sources[str(path)]=sha(path)
    if a.parameters:sources[str(a.parameters)]=sha(a.parameters)
    report=dict(scope=__doc__,sources=sources,hardware_access=False,policy_weights_changed=False,optimizer_updates=0,
        mode='frozen_reference_evaluation' if a.parameters else 'bounded_CEM_trajectory_search',side=a.side,
        search_objective='same hard qualification, add continuous swing-foot unloading while opposite foot supports at least half initial total weight',
        lift_delay_seconds=a.lift_delay_seconds,
        nonlinear_swing_ik=a.nonlinear_swing_ik,teacher_full_simulated_root_pose=a.nonlinear_swing_ik,
        teacher_request_slew_within_original_limit=a.nonlinear_swing_ik,
        swing_preload_scale=a.swing_preload_scale,return_at_lift_fraction=a.return_at_lift_fraction,
        preload_semantics='scale1 fades baseline requested-minus-actual bias on swing leg during geometric lift; stance bias unchanged; trajectory retimed within original rate. Hypothesis, not validated servo identification.',
        candidate_sha256=digest(c),original_acceptance_gate_unchanged=True,original_physical_limits_unchanged=True,
        reference_controller_root_state_writes=0,push_scale=a.push_scale,batch=a.batch,seed=a.seed,
        parameter_names=NAMES,parameter_low=LOW,parameter_high=HIGH,maximum_generations=a.generations,
        maximum_wall_seconds=a.wall_seconds,completed=False,generations=[],walking_qualified=False,
        single_cycle_gate=dict(foot_clearance_m=.003,air_load_below_n=.001,consecutive_air_frames=2,
            recovery_last_frames=25,recovery_tilt_below_deg=3.,both_feet_contact=True,stance_contact_during_air=True,
            recovery_root_height_tolerance_m=.005,recovery_joint_tolerance_deg=5.,
            maximum_tilt_below_deg=15.,no_range_violation=True,no_self_collision=True,
            semantics='new necessary single-cycle diagnostic, not original walking acceptance'))
    def save():(a.output/'search.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    (a.output/'source.py').write_bytes(Path(__file__).read_bytes());save()
    torch.set_num_threads(1);torch.set_num_interop_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed)
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(policy),opts,providers=['CPUExecutionProvider'])
    spec=importlib.util.spec_from_file_location('trajectory_measurements',EVALUATOR)
    independent=importlib.util.module_from_spec(spec);spec.loader.exec_module(independent)
    selected={b['side']:b for b in basis['cases']};sideidx=0 if a.side=='left' else 1
    transfer=np.asarray(selected[a.side]['transfer_basis_rad']);lift=np.asarray(selected[a.side]['lift_basis_rad'])
    stance=np.asarray(selected['right' if a.side=='left' else 'left']['lift_basis_rad'])
    home=np.asarray(c['home_rad'],dtype=np.float32)
    low=np.maximum(c['lower_rad'],basis['lower_intersection_rad'])
    high=np.minimum(c['upper_rad'],basis['upper_intersection_rad'])
    leg=np.asarray([i for i,j in enumerate(c['policy_joint_ids']) if j<30])
    rng=np.random.default_rng(20261011);mean=np.asarray(INITIAL);std=(np.asarray(HIGH)-LOW)*.22
    best=None;env=None;started=time.monotonic()
    def check_time():
        if time.monotonic()-started>a.wall_seconds:raise TimeoutError('Bounded dynamic reference search')
    try:
        cfg,_=make_mjlab_configs(c);apply_task_profile(cfg,'stand-button')
        cfg.seed=a.seed;cfg.scene.num_envs=a.batch;cfg.auto_reset=False
        ledger=[];report['push_audit']=audit_interval_push(cfg,ledger,a.push_scale)
        env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);term=env.action_manager.get_term('joint_pos')
        model=env.sim.mj_model;foot=[]
        for side in ('left','right'):
            gid=[g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').split('/')[-1]==side+'_foot_collision'][0]
            mid=int(model.geom_dataid[gid]);start=int(model.mesh_vertadr[mid]);n=int(model.mesh_vertnum[mid])
            foot.append((gid,torch.tensor(np.asarray(model.mesh_vert[start:start+n]),dtype=torch.float32)))
        def measure():
            gravity=env.scene['robot'].data.projected_gravity_b
            tilt=torch.rad2deg(torch.acos((-gravity[:,2]/gravity.norm(dim=1)).clamp(-1,1))).numpy()
            height=[]
            for gid,vertices in foot:
                rotation=env.sim.data.geom_xmat[:,gid].reshape(a.batch,3,3)
                points=torch.einsum('bij,vj->bvi',rotation,vertices)+env.sim.data.geom_xpos[:,gid,None,:]
                height.append(points[:,:,2].min(dim=1).values.numpy())
            sensor=env.scene['feet_ground_contact']._extract_sensor_data()
            contact=sensor.found.numpy().copy();loads=sensor.force[:,:,2].abs().numpy().copy()
            collision=env.scene['self_collision']._extract_sensor_data().found.reshape(a.batch,-1).sum(dim=1).numpy()>0
            actual=term._entity.data.joint_pos[:,term._target_ids].numpy().copy()
            requested=term.requested_targets.numpy().copy();goal=term._goal.numpy().copy()
            bad=np.zeros(a.batch,dtype=bool)
            for v in (actual,requested,goal):bad|=(~np.isfinite(v)).any(axis=1)|(v<low).any(axis=1)|(v>high).any(axis=1)
            return dict(tilt=tilt,height=np.stack(height,axis=1),load=loads,contact=contact,
                self_collision=collision,range_violation=bad,actual=actual,requested=requested,goal=goal,
                root=env.sim.wp_data.qpos.numpy()[:,:7].copy(),qvel=env.sim.wp_data.qvel.numpy().copy(),qfrc_actuator=env.sim.data.qfrc_actuator.numpy().copy(),qfrc_bias=env.sim.data.qfrc_bias.numpy().copy(),qfrc_constraint=env.sim.data.qfrc_constraint.numpy().copy())
        def zero_actor(observation):
            actor=observation['actor'].numpy().copy();actor[:,48:51]=0.
            # Source export binds batch=1. Keep its exact graph/weights and
            # evaluate rows separately instead of silently changing the graph.
            return np.concatenate([session.run(None,{session.get_inputs()[0].name:row[None,:]})[0] for row in actor],axis=0)
        for generation in range(a.generations):
            check_time()
            if generation and a.wall_seconds-(time.monotonic()-started)<25:
                report['stopped_at_generation_boundary_for_budget']=True;break
            if a.parameters:
                value=json.loads(a.parameters.read_text());v=value.get('parameters',value.get('best',{}).get('parameters'))
                parameters=np.tile(np.asarray(v,dtype=float),(a.batch,1))
            else:
                parameters=np.clip(rng.normal(mean,std,(a.batch,len(mean))),LOW,HIGH)
                parameters[0]=mean
                if best is not None and a.batch>1:parameters[1]=best['parameters']
            trajectory,end,retiming=build_trajectory(parameters,transfer,lift,stance,
                c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn'],a.lift_delay_seconds,return_fraction=a.return_at_lift_fraction)
            if 5+float(end.max())+1>=cfg.episode_length_s:raise ValueError('Reference exceeds original episode duration')
            torch.manual_seed(a.seed);np.random.seed(a.seed);obs,_=env.reset(seed=a.seed)
            warm_failed=np.zeros(a.batch,dtype=bool);warm_max=np.zeros(a.batch)
            for step in range(250):
                check_time();obs,_,terminated,truncated,_=env.step(torch.from_numpy(zero_actor(obs)))
                m=measure();warm_max=np.maximum(warm_max,m['tilt'])
                fail=terminated.numpy()|truncated.numpy()|m['range_violation']|(m['tilt']>=15)|m['self_collision']
                warm_failed|=fail
                if fail.any():
                    ids=torch.from_numpy(np.flatnonzero(fail));obs,_=env.reset(env_ids=ids)
            base=term.requested_targets.numpy().copy()
            initial_actual=term._entity.data.joint_pos[:,term._target_ids].numpy().copy()
            swing_mask=np.asarray([int(sid in range(10 if a.side=='left' else 20,15 if a.side=='left' else 25)) for sid in c['policy_joint_ids']])
            swing_extra=-a.swing_preload_scale*(base-initial_actual)*swing_mask
            trajectory,end,retiming=build_trajectory(parameters,transfer,lift,stance,
                c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn'],a.lift_delay_seconds,swing_extra,a.return_at_lift_fraction)
            if 5+float(end.max())+1>=cfg.episode_length_s:raise ValueError('Retimed preload path exceeds original episode duration')
            ik_teacher=None
            if a.nonlinear_swing_ik:
                from clearance_swing_reference import SwingReference
                xml=Path(c['reset_placement']['xml_path'])
                if sha(xml)!=c['reset_placement']['xml_sha256']:raise ValueError('IK model changed')
                ik_teacher=SwingReference(xml,c['joint_names'],c['policy_joint_ids'],a.side,
                    env.sim.wp_data.qpos.numpy()[0,:7],initial_actual[0],low,high)
            teacher_records=[];last_teacher_request=base.copy()
            initial_root_height=env.sim.wp_data.qpos.numpy()[:,2].copy()
            initial_measurement=measure()
            initial_weight=initial_measurement['load'].sum(axis=1)
            if np.any(initial_weight[~warm_failed]<1.):raise ValueError('Insufficient initial support load')
            initial_hash=hashlib.sha256(env.sim.wp_data.qpos.numpy().tobytes()+env.sim.wp_data.qvel.numpy().tobytes()).hexdigest()
            _,_,properties=effective_body_properties(env)
            active=~warm_failed;failed=warm_failed.copy();history=[]
            max_height=np.full(a.batch,-1.);max_tilt=warm_max.copy();max_air=np.zeros(a.batch,dtype=int);air_run=np.zeros_like(max_air)
            minimum_swing_load_share=np.ones(a.batch)
            ever_air=np.zeros(a.batch,dtype=bool);landed=np.zeros(a.batch,dtype=bool);recovery=np.zeros(a.batch,dtype=int)
            stop_steps=np.full(a.batch,-1,dtype=int);stop_reasons=[[] for _ in range(a.batch)]
            horizon=int(np.ceil((end.max()+1.)/env.step_dt))
            for step in range(horizon):
                check_time();t=(step+1)*env.step_dt;offset,_=trajectory(t)
                teacher_request=base+offset
                if ik_teacher is not None and t<=end[0]:
                    begin=retiming['lift_start_s'][0];rise=retiming['swing_rise_s'][0]
                    hold=parameters[0,6]*retiming['phase_stretch'][0]
                    def sm(z):z=np.clip(z,0.,1.);return z*z*(3-2*z)
                    phase=sm((t-begin)/rise)-sm((t-begin-rise-hold)/rise)
                    height=selected[a.side]['foot_lift_target_m']*parameters[0,1]*phase
                    preload=(base[0]-initial_actual[0])*(1-phase)
                    now_root=env.sim.wp_data.qpos.numpy()[0,:7]
                    now_actual=term._entity.data.joint_pos[:,term._target_ids].numpy()[0]
                    teacher_target,proof=ik_teacher.solve(now_root,now_actual,height,preload) if phase>1e-9 else (teacher_request[0,ik_teacher.index],dict(success=True,nfev=0,reference='original preparation before/after lift'))
                    proof['time_s']=t;teacher_records.append(proof)
                    if teacher_target is None:
                        failed[0]=True;active[0]=False;stop_steps[0]=step;stop_reasons[0]=['ik_planning_failure']
                        if not history:
                            sample=measure();sample['active_before']=np.zeros(a.batch,dtype=bool);history.append(sample)
                        break
                    teacher_request[0,ik_teacher.index]=teacher_target
                raw=(teacher_request-home)/c['action_scale']
                recovery_raw=zero_actor(obs)
                blend=np.clip((t-end)/.5,0,1)[:,None]
                raw=raw*(1-blend)+recovery_raw*blend
                if ik_teacher is not None:
                    request=home+raw*c['action_scale']
                    maximum=c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn']*env.step_dt
                    request=last_teacher_request+np.clip(request-last_teacher_request,-maximum,maximum)
                    last_teacher_request=request.copy();raw=(request-home)/c['action_scale']
                raw[~active]=recovery_raw[~active]
                obs,_,terminated,truncated,_=env.step(torch.from_numpy(raw.astype(np.float32)))
                m=measure();m['active_before']=active.copy();history.append(m)
                if step==0:
                    extra=independent.actual_plant_measurements(env)
                    for s,side in enumerate(('left','right')):
                        np.testing.assert_allclose(m['height'][0,s],extra['feet'][side]['mesh_min_height_m'],atol=2e-7,rtol=0)
                        np.testing.assert_allclose(m['load'][0,s],extra['feet'][side]['vertical_load_n'],atol=1e-4,rtol=1e-4)
                done=terminated.numpy()|truncated.numpy()
                fail=active&(done|m['range_violation']|(m['tilt']>=15)|m['self_collision']|(~np.isfinite(m['tilt'])))
                valid=active&~fail
                max_tilt[active]=np.maximum(max_tilt[active],m['tilt'][active])
                supported=valid&(m['contact'][:,1-sideidx]>0)
                loaded=supported&(m['load'][:,1-sideidx]>=.5*initial_weight)&(t<=end)
                load_share=m['load'][:,sideidx]/np.maximum(m['load'].sum(axis=1),1e-9)
                minimum_swing_load_share[loaded]=np.minimum(minimum_swing_load_share[loaded],load_share[loaded])
                max_height[supported]=np.maximum(max_height[supported],m['height'][supported,sideidx])
                air=valid&(m['height'][:,sideidx]>=.003)&(m['load'][:,sideidx]<.001)&(m['contact'][:,1-sideidx]>0)
                air_run=np.where(air,air_run+1,0);max_air=np.maximum(max_air,air_run);ever_air|=air
                ground=(m['contact']>0).all(axis=1)&(m['height']<.001).all(axis=1)
                landed|=valid&ever_air&ground&(t>end-.1)
                joint_return_error=np.degrees(np.abs(m['actual'][:,leg]-initial_actual[:,leg])).max(axis=1)
                returning=valid&(t>=end+.5)&ground&(m['tilt']<3.)&(np.abs(m['root'][:,2]-initial_root_height)<.005)&(joint_return_error<5.)
                recovery=np.where(returning,recovery+1,0)
                for idx in np.flatnonzero(fail):
                    stop_steps[idx]=step
                    stop_reasons[idx]=[name for name,value in [('environment_done',done),('range',m['range_violation']),('tilt',m['tilt']>=15),('self_collision',m['self_collision'])] if value[idx]]
                failed|=fail;active&=~fail
                # Failed search worlds are retired. Reset only to keep simulator
                # numerics valid; never credit their subsequent reset trajectory.
                reset=fail|done
                if reset.any():obs,_=env.reset(env_ids=torch.from_numpy(np.flatnonzero(reset)))
                if not active.any():break
            last=history[-1]
            qualified=(~failed)&(max_air>=2)&landed&(recovery>=25)
            return_error=np.degrees(np.abs(last['actual'][:,leg]-initial_actual[:,leg])).max(axis=1)
            scores=50*np.clip(max_height/.003,0,1)+20*np.clip(max_air/3,0,1)+20*landed+10*np.clip(recovery/25,0,1)-max_tilt-.2*return_error
            scores+=100*np.clip((.5-minimum_swing_load_share)/.5,0,1)
            scores=np.where(failed,-1000+scores,scores)
            order=np.argsort(scores)[::-1];case_records=[]
            for i in range(a.batch):
                record=dict(index=i,parameters=parameters[i].tolist(),score=float(scores[i]),single_cycle_qualified=bool(qualified[i]),
                    failed=bool(failed[i]),warmup_failed=bool(warm_failed[i]),max_tilt_deg=float(max_tilt[i]),
                    maximum_swing_clearance_m=float(max_height[i]),max_consecutive_air_frames=int(max_air[i]),landed=bool(landed[i]),
                    minimum_swing_load_share=float(minimum_swing_load_share[i]),
                    recovery_frames=int(recovery[i]),return_joint_error_deg=float(return_error[i]),stop_step=int(stop_steps[i]),stop_reasons=stop_reasons[i],
                    reference_end_s=float(end[i]))
                case_records.append(record)
                if best is None or (record['single_cycle_qualified'],record['score'])>(best['single_cycle_qualified'],best['score']):best=dict(record,generation=generation)
            arrays={key:np.stack([h[key] for h in history]) for key in history[0]}
            trace=a.output/f'generation-{generation:02d}.npz';np.savez_compressed(trace,parameters=parameters,**arrays)
            info=dict(generation=generation,initial_state_sha256=initial_hash,env0_effective_body_properties=properties,
                retiming=retiming,teacher_ik=teacher_records,swing_extra_rad=swing_extra.tolist(),candidates=case_records,trace_sha256=sha(trace),steps_recorded=len(history),
                wall_seconds=time.monotonic()-started,qualified_count=int(qualified.sum()),best_this_generation=case_records[int(order[0])])
            report['generations'].append(info);report['best']=best;report['interval_pushes']=ledger;save()
            print(json.dumps(dict(generation=generation,qualified=int(qualified.sum()),best_score=float(scores[order[0]]),
                best_clearance_m=case_records[int(order[0])]['maximum_swing_clearance_m'],best_tilt_deg=case_records[int(order[0])]['max_tilt_deg'],
                failed=int(failed.sum()),wall_seconds=info['wall_seconds'])),flush=True)
            if not a.parameters:
                elite=parameters[order[:max(2,a.batch//5)]]
                mean=.5*mean+.5*elite.mean(axis=0)
                std=np.maximum(.6*std+.4*elite.std(axis=0),(np.asarray(HIGH)-LOW)*.04)
        report['completed']=True
    except TimeoutError as error:
        report['budget_stop']=str(error)
    except Exception as error:
        report['error']=type(error).__name__+': '+str(error)
        raise
    finally:
        if env is not None:env.close()
        report['wall_seconds']=time.monotonic()-started
        report['source_hashes_unchanged']=all(sha(p)==s for p,s in sources.items());save()
    if not report['source_hashes_unchanged']:raise ValueError('Search sources changed')


if __name__=='__main__':main()
