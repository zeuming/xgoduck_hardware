"""One frozen inverse-contact position lead, replayed only in the simulator.

The earliest feasible saved-state inverse solution supplies a coupled bilateral
goal increment. Start it early enough for its cubic rise at the ORIGINAL rate;
all combined requests are then limited by that same rate before original EMA,
BAM/delay and physical gates. This is a conditional feed-forward experiment,
not a feedback inverse-dynamics controller, gait policy or hardware command.
"""
import hashlib,json,os,sys
from pathlib import Path
import numpy as np


def main():
    mode=sys.argv[sys.argv.index('--lead-mode')+1];i=sys.argv.index('--lead-mode');del sys.argv[i:i+2]
    if mode not in ('none','inverse'):raise ValueError('Unsupported lead mode')
    root=Path(__file__).resolve().parents[2];base=root/'output/duck_sim/overnight-20261011'
    out=Path(sys.argv[sys.argv.index('--output')+1]);os.environ.setdefault('MPLCONFIGDIR','/tmp/xgoduck-dynamics-mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    import diagnose_bam_control_tap as original_diagnostic
    cp=Path(sys.argv[sys.argv.index('--candidate')+1]);c=json.loads(cp.read_text())['candidate']
    solution_file=base/'inverse-contact-authority-v2/summary.json';solution=json.loads(solution_file.read_text())
    rec=next(r for r in solution['rows'] if r['feasible'] and r['mode']=='full_saved_interval')
    ref=base/'full-dynamics-replay-v3/generation-00.npz';z=np.load(ref)
    delta=np.asarray(rec['algebraic_motor_goal_rad'])-z['goal'][rec['step'],0]
    delta[[5,6,7,8]]=0
    slew=c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn']
    duration=1.5*float(abs(delta).max())/slew
    begin=rec['time_s']-duration;peak=rec['time_s']+rec['horizon_s']
    assert begin>0 and peak+duration<2
    count=0;last=None;telemetry=[];original_step=ManagerBasedRlEnv.step
    input_files=(Path(__file__),Path(original_diagnostic.__file__),solution_file,ref,cp)
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in input_files}
    def sm(x):x=np.clip(x,0,1);return x*x*(3-2*x)
    def observed(env,action):
        nonlocal count,last
        count+=1
        if count<=250:
            result=original_step(env,action)
            if count==250:last=env.action_manager.get_term('joint_pos').requested_targets.numpy().copy()
            return result
        t=(count-250)*env.step_dt;weight=sm((t-begin)/duration)-sm((t-peak)/duration)
        before=action.numpy().copy();home=np.asarray(c['home_rad'])
        original_request=home+c['action_scale']*before
        if mode=='inverse':
            desired=original_request+weight*delta
            combined=last+np.clip(desired-last,-slew*env.step_dt,slew*env.step_dt)
            action=torch.from_numpy(((combined-home)/c['action_scale']).astype(np.float32))
        result=original_step(env,action)
        final=env.action_manager.get_term('joint_pos').requested_targets.numpy().copy()
        rate=float(abs(final-last).max()/env.step_dt)
        telemetry.append(dict(time_s=t,weight=float(weight),maximum_request_rate_rad_s=rate,
            final_requested_rad=final[0].tolist(),baseline_requested_rad=original_request[0].tolist()))
        last=final;return result
    ManagerBasedRlEnv.step=observed
    try:original_diagnostic.main()
    finally:ManagerBasedRlEnv.step=original_step
    run=json.loads((out/'search.json').read_text());source=json.loads((base/'bam-tracking-control-tap-v2/search.json').read_text())
    assert run['generations'][0]['initial_state_sha256']==source['generations'][0]['initial_state_sha256']
    assert run['generations'][0]['env0_effective_body_properties']==source['generations'][0]['env0_effective_body_properties']
    result=dict(completed=True,scope=__doc__,hardware_access=False,mode=mode,sources=hashes,
        original_physical_limits_unchanged=True,original_reference_snapshot=rec,
        lead_delta_rad=delta.tolist(),rise_s=duration,begin_s=begin,hold_until_s=peak,end_s=peak+duration,
        maximum_final_request_rate_rad_s=max(t['maximum_request_rate_rad_s'] for t in telemetry),original_rate_rad_s=slew,
        initial_state_and_effective_properties_match=True,trace=telemetry,
        outcome=run['generations'][0]['candidates'][0],walking_qualified=False)
    if mode=='none':
        actual=np.load(out/'generation-00.npz');result['control_array_parity']={k:bool(np.array_equal(z[k],actual[k])) for k in z.files}
        assert all(result['control_array_parity'].values())
    else:assert result['maximum_final_request_rate_rad_s']<slew+2e-5
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    (out/'lead-summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    (out/'lead-source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v for k,v in result.items() if k not in ('trace','sources','scope','original_reference_snapshot','control_array_parity')}),flush=True)


if __name__=='__main__':main()
