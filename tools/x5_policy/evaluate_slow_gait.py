#!/usr/bin/env python3
"""Independent deterministic plant/sole/contact evaluation; rewards ignored."""
import argparse
import importlib.util
import json
from pathlib import Path
import math


def grade(rows,mode,*,sample_period=.02):
    if not math.isfinite(sample_period) or sample_period<=0:
        raise ValueError('Positive finite observation period required')
    # Require observed samples spanning >=60ms (four samples at50Hz).
    # Three20ms samples span40ms, not60ms. A gap must break the run.
    # Neither reward nor displacement produced after a fall is accepted.
    lift_run=[0,0];land_run=[0,0];air=[False,False];cycles=[0,0];sequence=[]
    lift_start=[None,None];land_start=[None,None];previous_time=None
    peak=[0.,0.];qualified_frames=[0,0]
    active=[r for r in rows if r['command_m_s']>0]
    for r in active:
        contiguous=previous_time is not None and 0<r['time_s']-previous_time<=sample_period+1e-6
        for i,side in enumerate(('left','right')):
            foot=r['feet'][side];other=r['feet'][('right','left')[i]]
            peak[i]=max(peak[i],foot['mesh_min_height_m'])
            valid=(r['tilt_deg']<15 and foot['mesh_min_height_m']>=.003 and foot['vertical_load_n']<.001 and other['vertical_load_n']>=1.)
            lift_run[i]=lift_run[i]+1 if valid and contiguous else int(valid)
            lift_start[i]=(lift_start[i] if contiguous and lift_start[i] is not None else r['time_s']) if valid else None
            qualified_frames[i]+=int(valid)
            if valid and r['time_s']-lift_start[i]>=.06-1e-9 and not air[i]:air[i]=True;sequence.append(side)
            landed=(foot['mesh_min_height_m']<.001 and foot['vertical_load_n']>=1.)
            land_run[i]=land_run[i]+1 if landed and contiguous else int(landed)
            land_start[i]=(land_start[i] if contiguous and land_start[i] is not None else r['time_s']) if landed else None
            if air[i] and landed and r['time_s']-land_start[i]>=.06-1e-9:cycles[i]+=1;air[i]=False
        previous_time=r['time_s']
    complete=len(rows)>0 and rows[-1].get('task_complete',False)
    safe=bool(rows) and max(r['tilt_deg'] for r in rows)<15 and all(not r['failed_terms'] for r in rows)
    zero=[r for r in rows if r['time_s']<=5.]
    start=rows[0]['root_position'] if rows else [0,0,0]
    zero_drift=max((math.hypot(r['root_position'][0]-start[0],r['root_position'][1]-start[1]) for r in zero),default=float('inf'))
    heading=rows[0].get('heading_rad',0.) if rows else 0.
    net=((rows[-1]['root_position'][0]-start[0])*math.cos(heading)
        +(rows[-1]['root_position'][1]-start[1])*math.sin(heading)) if rows else 0.
    alternating=len(sequence)>=4 and all(a!=b for a,b in zip(sequence,sequence[1:]))
    recovery_frames=round(1/sample_period)
    tail=rows[-recovery_frames:]
    recovery_drift=(math.hypot(tail[-1]['root_position'][0]-tail[0]['root_position'][0],
        tail[-1]['root_position'][1]-tail[0]['root_position'][1]) if len(tail)==recovery_frames else float('inf'))
    supported=sum(all(r['feet'][s]['vertical_load_n']>=.1 and r['feet'][s]['mesh_min_height_m']<.0015 for s in ('left','right')) for r in tail)
    recovered=(complete and len(tail)==recovery_frames and supported>=math.ceil(.9*recovery_frames) and recovery_drift<=.005
        and all(r['command_m_s']==0 and r['tilt_deg']<15 for r in tail))
    accepted=safe and complete and zero_drift<=.015 and recovered
    if mode=='forward':accepted=accepted and min(cycles)>=2 and alternating and net>=.02 and recovered
    return dict(accepted=accepted,mode=mode,complete=complete,safe=safe,frames=len(rows),
        max_tilt_deg=max((r['tilt_deg'] for r in rows),default=None),zero_drift_m=zero_drift,
        max_whole_sole_clearance_m=peak,qualified_lift_frames=qualified_frames,
        completed_lift_land_cycles=cycles,lift_sequence=sequence,alternating=alternating,
        net_forward_m=net,recovered=recovered,recovery_drift_m=recovery_drift,final_supported_frames=supported,hardware_verified=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--model',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seed',type=int,default=0)
    p.add_argument('--mode',choices=('zero','forward'),default='forward')
    p.add_argument('--measurement-source',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from train_constrained_policy import install_offline_guards,file_sha
    install_offline_guards()
    import torch
    import numpy as np
    import onnxruntime as ort
    from constrained_training_candidate import load_candidate,make_mjlab_configs,digest
    import slow_gait_experiment as experiment
    from mjlab.envs import ManagerBasedRlEnv
    module_spec=importlib.util.spec_from_file_location('independent_original_measurement',a.measurement_source)
    measurements=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(measurements)
    torch.set_num_threads(1);torch.manual_seed(a.seed);np.random.seed(a.seed)
    candidate=load_candidate(a.candidate);cfg,_=make_mjlab_configs(candidate)
    # Independent longer warmup/recovery than training. No simulation root writes.
    experiment.SPEC.update(warmup_s=5.,walk_s=16. if a.mode=='forward' else 0.,recovery_s=3. if a.mode=='forward' else 0.)
    task=experiment.configure(cfg);cfg.scene.num_envs=1;cfg.seed=a.seed;cfg.auto_reset=False
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    session=ort.InferenceSession(str(a.model),options,providers=['CPUExecutionProvider'])
    inputs=session.get_inputs();size=inputs[0].shape[1]
    hybrid=[x.name for x in inputs]==['obs61','phase2','held10','update1']
    if hybrid:
        if [x.shape for x in inputs]!=[[1,61],[1,2],[1,10],[1,1]]:raise ValueError('Invalid hybrid interface')
        held=np.zeros((1,10),dtype=np.float32)
    elif len(inputs)!=1 or size not in (61,63):raise ValueError('Unsupported actor interface')
    rows=[];env=None
    try:
        env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None)
        obs,_=env.reset(seed=a.seed)
        initial=measurements.actual_plant_measurements(env)
        (a.output/'initial.json').write_text(json.dumps(initial,indent=2)+'\n')
        nsteps=round(cfg.episode_length_s/env.step_dt)
        with (a.output/'trace.jsonl').open('x') as trace:
            for step in range(nsteps):
                command=float(env.command_manager.get_command('twist')[0,0])
                raw=obs['actor']
                if size==63:raw=torch.cat((raw,experiment.phase_features(env)),dim=-1)
                if hybrid:
                    # Explicit residual state/clock, while lower feedback still reads
                    # the newest original61 observation on every20ms control period.
                    action,held=session.run(None,dict(obs61=raw.numpy(),phase2=experiment.phase_features(env).numpy(),
                        held10=held,update1=np.array([[float(step%10==0)]],dtype=np.float32)))
                    if held.shape!=(1,10) or not np.isfinite(held).all():raise ValueError('Invalid held residual state')
                else:action=session.run(None,{session.get_inputs()[0].name:raw.numpy()})[0]
                if action.shape!=(1,14) or not np.isfinite(action).all():raise ValueError('Invalid policy action')
                obs,reward,terminated,truncated,_=env.step(torch.from_numpy(action))
                measurement=measurements.actual_plant_measurements(env)
                failed=[k for k in env.termination_manager.active_terms if k!='time_out' and bool(env.termination_manager.get_term(k)[0])]
                if measurement['tilt_deg']>=15 and 'post_step_15deg' not in failed:failed.append('post_step_15deg')
                row=dict(step=step,time_s=(step+1)*env.step_dt,command_m_s=command,actions=action[0].tolist(),
                    failed_terms=failed,task_complete=step+1==nsteps,**measurement)
                if hybrid:row.update(held_residual=held[0].tolist(),residual_updated=step%10==0)
                rows.append(row);trace.write(json.dumps(row,allow_nan=False)+'\n')
                if failed or bool(terminated[0]) or bool(truncated[0]):break
        result=grade(rows,a.mode)
        result.update(candidate_sha256=digest(candidate),model_sha256=file_sha(a.model),seed=a.seed,task=task,
            measurement_source_sha256=file_sha(a.measurement_source),initial_state=initial,
        acceptance='samples spanning60ms whole-sole3mm / swing load<1mN / other support>=1N, 2 complete cycles per side alternating, >=2cm net forward, no15deg/range failure, final1s zero support')
        (a.output/'gate.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ('initial_state','task')}),flush=True)
    finally:
        if env is not None:env.close()


if __name__=='__main__':main()
