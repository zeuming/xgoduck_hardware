"""Independent four-task plant evaluation; rewards are not acceptance evidence."""
from __future__ import annotations
import argparse,copy,json,math,random,time
from dataclasses import dataclass,fields
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--mode',choices=('walk','getup','pick','roulade'),required=True)
    p.add_argument('--body',default='rear-mid')
    p.add_argument('--walk-speed',type=float,default=.3)
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu')
    p.add_argument('--control-hz',type=int,choices=(40,50),default=50)
    p.add_argument('--sensor-lag-steps',type=int,choices=(0,1),default=0)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wall-seconds',type=float,default=240)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from train_constrained_policy import install_offline_guards
    install_offline_guards(accelerator=a.device=='cuda:0')
    import torch,numpy as np,onnxruntime as ort,mujoco
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand,UniformVelocityCommandCfg
    from current_body_training import load_contract,configure,sha
    from current_body_measurements import actual_plant_measurements
    from evaluate_slow_gait import grade
    from simulation_evidence import effective_body_properties
    from current_body_fingerprint import initial_fingerprint
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
    contract=load_contract(a.contract);cfg,_,declaration=configure(contract,a.mode,a.body)
    cfg.decimation=round(1/a.control_hz/cfg.sim.mujoco.timestep)
    cfg.scene.num_envs=1;cfg.seed=a.seed;cfg.auto_reset=False;cfg.curriculum.clear()
    # Keep the source randomization and contact model, but disable exogenous
    # pushes for an explicit no-push baseline. Train/evaluate disturbed cases
    # separately; this is never described as an original untouched rollout.
    cfg.events.pop('push_robot',None)
    seconds=16. if a.mode=='walk' else 8. if a.mode=='getup' else 4. if a.mode=='pick' else 6.
    cfg.episode_length_s=seconds+.02
    if a.mode=='roulade':
        cfg.events['set_roulade_state'].params.update(standing_prob=1.,midroll_prob=0.)
    if a.mode=='walk':
        class Scheduled(UniformVelocityCommand):
            def _resample(self,ids):
                self.time_left[ids]=float('inf');self.vel_command_b[ids]=0;self.vel_command_w[ids]=0
                self.is_standing_env[ids]=True;self.is_heading_env[ids]=False;self.is_world_env[ids]=False;self.is_forward_env[ids]=False
                self.command_counter[ids]+=1
            def compute(self,dt):
                t=self._env.episode_length_buf*self._env.step_dt
                speed=torch.where((t>=5)&(t<13),a.walk_speed,0.)
                self.vel_command_b.zero_();self.vel_command_b[:,0]=speed
                self.vel_command_w.copy_(self.vel_command_b);self.is_standing_env[:]=speed==0
        @dataclass(kw_only=True)
        class ScheduledCfg(UniformVelocityCommandCfg):
            def build(self,env):return Scheduled(self,env)
        old=cfg.commands['twist']
        kw={f.name:copy.deepcopy(getattr(old,f.name)) for f in fields(UniformVelocityCommandCfg)}
        kw.update(init_velocity_prob=0.,heading_command=False,rel_heading_envs=0.,rel_world_envs=0.)
        kw['ranges'].heading=None
        cfg.commands['twist']=ScheduledCfg(**kw)
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(a.model),opts,providers=['CPUExecutionProvider'])
    if session.get_inputs()[0].shape!=[1,61] or session.get_outputs()[0].shape!=[1,14]:
        raise ValueError('Expected deployment-compatible61-to14 ONNX')
    env=None;rows=[];started=time.monotonic()
    report=dict(mode=a.mode,seed=a.seed,body_scenario=a.body,model_sha256=sha(a.model),
        contract_sha256=sha(a.contract),declaration=declaration,hardware_verified=False,
        body_identified=False,pushes_enabled=False,simulation_accepted=False)
    report.update(control_hz=a.control_hz,additional_actor_sensor_lag_steps=a.sensor_lag_steps,
        sensor_lag_scope='first34 sensor slots only; raw action history and command remain current')
    try:
        env=ManagerBasedRlEnv(cfg,device=a.device,render_mode=None)
        obs,_=env.reset(seed=a.seed)
        report['initial_fingerprint']=initial_fingerprint(env)
        _,_,report['effective_body']=effective_body_properties(env)
        report['initial_qpos_sha256']=__import__('hashlib').sha256(env.sim.data.qpos.cpu().numpy().tobytes()).hexdigest()
        report['initial_qvel_sha256']=__import__('hashlib').sha256(env.sim.data.qvel.cpu().numpy().tobytes()).hexdigest()
        mouth=[i for i in range(env.sim.mj_model.nsite) if (mujoco.mj_id2name(env.sim.mj_model,mujoco.mjtObj.mjOBJ_SITE,i) or '').split('/')[-1]=='mouth_tip']
        if len(mouth)!=1:raise ValueError('Expected unique CAD mouth tip')
        term=env.action_manager.get_term('joint_pos')
        steps=round(seconds/env.step_dt)
        sensor_history=[]
        with (a.output/'trace.jsonl').open('x') as trace:
            for step in range(steps):
                if time.monotonic()-started>a.wall_seconds:raise TimeoutError('Independent evaluator wall budget')
                raw=obs['actor'].cpu().numpy().copy()
                sensor_history.append(raw[0,:34].copy())
                if len(sensor_history)>a.sensor_lag_steps+1:sensor_history.pop(0)
                raw[0,:34]=sensor_history[0]
                action=session.run(None,{session.get_inputs()[0].name:raw})[0]
                obs,_,terminated,truncated,_=env.step(torch.tensor(action,device=a.device))
                row=actual_plant_measurements(env)
                gravity=env.scene['robot'].data.projected_gravity_b[0].cpu().numpy()
                row.update(time_s=(step+1)*env.step_dt,command_m_s=float(raw[0,48]),
                    actor_observation=raw[0].tolist(),raw_action=action[0].tolist(),
                    processed_joint_rad=term._processed_actions[0].tolist(),transmitted_ticks=term.decoder.ticks[0].tolist(),
                    projected_joint_goals=int(term.decoder.projected.sum()),
                    mouth_tip_height_m=float(env.sim.data.site_xpos[0,mouth[0],2]),
                    signed_pitch_rad=float(np.arctan2(gravity[0],-gravity[2])),
                    failed_terms=[k for k in env.termination_manager.active_terms if k!='time_out' and bool(env.termination_manager.get_term(k)[0])],
                    task_complete=step==steps-1)
                rows.append(row);trace.write(json.dumps(row,allow_nan=False)+'\n')
                if bool(terminated[0]) or bool(truncated[0]):break
        if a.mode=='walk':
            gate=grade(rows,'zero' if a.walk_speed==0 else 'forward',sample_period=env.step_dt)
        else:
            tail=rows[-round(1/env.step_dt):]
            complete=bool(rows) and rows[-1]['task_complete']
            stable=len(tail)==round(1/env.step_dt) and all(r['tilt_deg']<15 and not r['failed_terms'] for r in tail)
            supported=sum(all(r['feet'][s]['vertical_load_n']>=.1 and r['feet'][s]['mesh_min_height_m']<.0015 for s in ('left','right')) for r in tail)
            drift=math.dist(tail[-1]['root_position'][:2],tail[0]['root_position'][:2]) if tail else float('inf')
            recovered=stable and supported>=.9*len(tail) and drift<=.005
            gate=dict(complete=complete,final_stable=recovered,final_supported_frames=supported,final_drift_m=drift)
            if a.mode=='getup':
                gate.update(accepted=complete and recovered,
                    initial_root_height_m=rows[0]['root_position'][2] if rows else None,
                    minimum_root_height_m=min((r['root_position'][2] for r in rows),default=None))
            elif a.mode=='pick':
                descent=[r for r in rows if .8<=r['time_s']<=1.8]
                min_height=min((r['mouth_tip_height_m'] for r in descent),default=float('inf'))
                gate.update(accepted=complete and recovered and min_height<=.04,
                    minimum_descent_mouth_height_m=min_height,phase_period_s=4.,jaw_actuation_unmodeled=True,
                    criterion='CAD mouth tip within4cm during descent plus final1s upright/support/drift; mouth grip itself unverified')
            else:
                pitch=np.unwrap([r['signed_pitch_rad'] for r in rows])
                turn=float(np.max(np.abs(pitch-pitch[0]))) if len(pitch) else 0.
                gate.update(accepted=complete and recovered and turn>=1.5*math.pi,
                    maximum_unwrapped_pitch_turn_rad=turn,
                    criterion='standing initial state; trunk pitch rotates>=270degrees and final1s stable two-foot support')
        report.update(gate=gate,simulation_accepted=bool(gate['accepted']),frames=len(rows),wall_seconds=time.monotonic()-started)
    except Exception as error:
        report['error']=repr(error);raise
    finally:
        if env is not None:env.close()
        (a.output/'gate.json').write_text(json.dumps(report,indent=2,default=repr)+'\n')
    print(json.dumps({k:report.get(k) for k in ('mode','seed','body_scenario','simulation_accepted','frames','gate','error')}))


if __name__=='__main__':main()
