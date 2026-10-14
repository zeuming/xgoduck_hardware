"""Paired derivative-scale diagnosis of the new predictive teacher objective.

No diagnostic command enters the source plant. Same full initial state, delay
history and random stream for every finite perturbation; compare central
differences at .001/.004rad. Neither scale changes physical target/rate limits.
"""
import hashlib
import json
import os
import sys
import time
from pathlib import Path
import numpy as np


def main():
    repeat_only = '--repeat-only' in sys.argv
    if repeat_only:sys.argv.remove('--repeat-only')
    cap100 = '--solver-cap100' in sys.argv
    if cap100:sys.argv.remove('--solver-cap100')
    native = '--native-crosscheck' in sys.argv
    if native:sys.argv.remove('--native-crosscheck')
    if native and cap100:raise ValueError('Select one numerical cross-check')
    probe_seconds = 240 if cap100 or native else 120
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/xgoduck-dynamics-mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch
    from mjlab.envs import ManagerBasedRlEnv
    from bam_prediction_branch import BamPredictionBranch
    if native:
        from native_bam_prediction_branch import NativeBamPredictionBranch
    import diagnose_bam_control_tap as baseline
    out = Path(sys.argv[sys.argv.index('--output')+1])
    root = Path(__file__).resolve().parents[2]
    base = root/'output/duck_sim/overnight-20261011'
    cp = Path(sys.argv[sys.argv.index('--candidate')+1])
    c = json.loads(cp.read_text())['candidate']
    source = base/'bam-tracking-control-tap-v2/generation-00.npz'
    z = np.load(source)
    nominal = z['requested'][:, 0].astype(float)
    home = np.asarray(c['home_rad'], dtype=np.float32)
    leg = np.asarray([i for i,j in enumerate(c['policy_joint_ids']) if j < 30])
    bounds = json.loads((base/'dynamic-step-basis-v1/basis.json').read_text())
    low = np.maximum(c['lower_rad'], bounds['lower_intersection_rad'])
    high = np.minimum(c['upper_rad'], bounds['upper_intersection_rad'])
    slew = c['action']['goal_slew_ticks_s']*2*np.pi/c['action']['ticks_per_turn']
    max_delta = slew*.02
    # One read-only probe of eighty bounded perturbations; no optimizer run.
    replans = (32,)
    end = 82
    plans = []; command_trace = []; count = 0; last = None; active_plan = None
    sources = [Path(__file__), Path(baseline.__file__), Path(__file__).with_name('bam_prediction_branch.py'), cp, source]
    if native:sources.append(Path(__file__).with_name('native_bam_prediction_branch.py'))
    hashes = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    def prediction_branch(env):
        branch = NativeBamPredictionBranch(env) if native else BamPredictionBranch(env)
        if cap100:
            branch.model.opt.iterations = 100
        return branch
    def save():
        (out/'predictive-progress.json').write_text(json.dumps(dict(plans=plans,commands=command_trace),indent=2)+'\n')
    def plan(env, index):
        started = time.monotonic(); evaluations = []; best = None; best_x = None
        horizon = end-index
        knot = np.array([0., horizon*.28, horizon*.62, horizon-1.])
        initial_delta = last[0]-nominal[index-1]
        n = len(leg)
        x0 = np.zeros(2*n)
        if active_plan is not None:
            for k, when in enumerate(knot[1:3]):
                at = min(end-1, index+int(round(when)))
                x0[k*n:(k+1)*n] = (active_plan[at-index_prev]-nominal[at])[leg]
        lo=[];hi=[]
        for when in knot[1:3]:
            at=min(end-1,index+int(round(when)))
            lo.extend((low-nominal[at])[leg]);hi.extend((high-nominal[at])[leg])
        lo=np.asarray(lo);hi=np.asarray(hi);x0=np.clip(x0,lo+1e-6,hi-1e-6)
        initial = prediction_branch(env).measure()
        weight = max(float(initial['load'].sum()), 1.)
        cache = {}
        def objective(x):
            nonlocal best,best_x
            key = np.asarray(x,dtype=np.float64).tobytes()
            if key in cache:return cache[key][0]
            if time.monotonic()-started>probe_seconds:raise TimeoutError(f'{probe_seconds}s sensitivity budget')
            branch = prediction_branch(env)
            offsets = np.zeros((4,14));offsets[0]=initial_delta
            offsets[1,leg]=x[:n];offsets[2,leg]=x[n:]
            previous=last[0].copy(); commands=[]; history=[]; residual=[]
            for h in range(horizon):
                offset=np.array([np.interp(h,knot,offsets[:,j]) for j in range(14)])
                desired=nominal[index+h]+offset
                # New teacher reference stays in the original CAD/saved domain
                # and at the original rate before the unchanged action layer.
                desired=np.clip(desired,low,high)
                requested=previous+np.clip(desired-previous,-max_delta,max_delta)
                previous=requested.copy();commands.append(requested)
                raw=torch.from_numpy(((requested[None,:]-home)/c['action_scale']).astype(np.float32))
                branch.step(raw);m=branch.measure();history.append(m)
                t=(index+h+1)*.02
                def smooth(v):v=np.clip(v,0.,1.);return v*v*(3-2*v)
                phase=smooth((t-.80)/.22)-smooth((t-1.18)/.26)
                desired_height=.0035*phase
                residual.extend([(m['height'][0]-desired_height)/.003,
                    max(m['height'][1]-.0005,0.)/.001,
                    2*phase*m['load'][0]/weight,
                    max(.5*weight-m['load'][1],0.)/weight,
                    .15*m['tilt']/15., 20*max(m['tilt']-15.,0.)/15.])
                residual.extend(.15*(m['positions'][1,:2]-initial['positions'][1,:2])/.005)
                residual.extend(20*np.maximum(low-m['actual'],0.)/.05)
                residual.extend(20*np.maximum(m['actual']-high,0.)/.05)
                residual.append(10.*float(m['self_collision']))
            residual.extend(.05*x/.05)
            residual=np.asarray(residual)
            cost=float(np.dot(residual,residual))
            valid=all(m['tilt']<15 and not m['range_failed'] and not m['self_collision']
                      and np.all(m['actual']>=low) and np.all(m['actual']<=high) for m in history)
            record=dict(evaluation=len(evaluations),cost=cost,admissible=valid,
                max_tilt_deg=max(m['tilt'] for m in history),
                max_clearance_m=float(max(m['height'][0] for m in history)),
                min_swing_load_n=float(min(m['load'][0] for m in history)),
                air_frames=int(sum(m['height'][0]>=.003 and m['load'][0]<.001 and m['contact'][1]>0 for m in history)))
            evaluations.append(record)
            with (out/'evaluations.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(replan_index=index,wall_seconds=time.monotonic()-started,**record))+'\n')
            cache[key]=(residual,np.asarray(commands),record)
            if valid and (best is None or cost<best['cost']):best=record.copy();best_x=np.asarray(x).copy()
            return residual
        gradients=[];baseline_residual=objective(x0).copy()
        if repeat_only:
            repeats=[]
            for _ in range(3):
                cache.clear()
                repeated=objective(x0).copy()
                repeats.append(dict(bitwise_equal=bool(np.array_equal(repeated,baseline_residual)),
                    maximum_residual_difference=float(abs(repeated-baseline_residual).max())))
            info=dict(index=index,identical_query_repeats=repeats,
                all_identical=all(v['bitwise_equal'] for v in repeats),
                wall_seconds=time.monotonic()-started,evaluations=evaluations)
            plans.append(info);save();return None
        for epsilon in (.001,.004):
            J=np.zeros((len(baseline_residual),len(x0)))
            for j in range(len(x0)):
                plus=x0.copy();minus=x0.copy();plus[j]+=epsilon;minus[j]-=epsilon
                if plus[j]>hi[j] or minus[j]<lo[j]:raise ValueError('Probe crosses waypoint bound')
                J[:,j]=(objective(plus)-objective(minus))/(2*epsilon)
            gradients.append(J)
        first,second=gradients
        columns=[]
        groups={'clearance':[0,1],'load':[2,3],'tilt':[4,5],'stance_xy':[6,7]}
        for j in range(len(x0)):
            a,b=first[:,j],second[:,j]
            columns.append(dict(joint_id=c['policy_joint_ids'][leg[j%n]],waypoint=j//n,
                relative_change=float(np.linalg.norm(a-b)/max(np.linalg.norm(a),1e-12)),
                cosine=float(a@b/max(np.linalg.norm(a)*np.linalg.norm(b),1e-12)),
                cost_derivative_small=float(baseline_residual@a),cost_derivative_large=float(baseline_residual@b)))
        group_report={}
        for label,slots in groups.items():
            ids=np.asarray([37*h+i for h in range(horizon) for i in slots])
            a,b=first[ids],second[ids]
            group_report[label]=dict(relative_change=float(np.linalg.norm(a-b)/max(np.linalg.norm(a),1e-12)),
                norm_small=float(np.linalg.norm(a)),norm_large=float(np.linalg.norm(b)))
        np.savez_compressed(out/'sensitivity.npz',baseline=baseline_residual,jac_small=first,jac_large=second)
        info=dict(index=index,horizon_s=horizon*.02,epsilon_rad=[.001,.004],
            full_initial_state_and_future_random_stream_paired=True,columns=columns,groups=group_report,
            solver_iterations_in_prediction=100 if cap100 or native else 10,
            native_float64_engine_crosscheck=native,
            evaluations=evaluations,wall_seconds=time.monotonic()-started)
        plans.append(info);save()
        print(json.dumps({k:v for k,v in info.items() if k!='evaluations'}),flush=True)
        return None
    original=ManagerBasedRlEnv.step; index_prev=0
    def controlled(env, action):
        nonlocal count,last,active_plan,index_prev
        if count<250:
            result=original(env,action);count+=1
            if count==250:last=env.action_manager.get_term('joint_pos').requested_targets.numpy().copy()
            return result
        index=count-250
        if index in replans:
            active_plan=plan(env,index);index_prev=index
        # Probe is read-only relative to the plant: original action is untouched.
        result=original(env,action)
        last=env.action_manager.get_term('joint_pos').requested_targets.numpy().copy()
        count+=1
        return result
    ManagerBasedRlEnv.step=controlled
    try:baseline.main()
    finally:ManagerBasedRlEnv.step=original
    run=json.loads((out/'search.json').read_text())
    (out/'predictive-summary.json').write_text(json.dumps(dict(completed=True,scope=__doc__,hardware_access=False,
        original_physical_limits_unchanged=True,source_sha256=hashes,plans=plans,
        known_future_delay=False,sensitivity_only=True,maximum_probe_seconds=probe_seconds,
        solver_iterations_in_prediction=100 if cap100 or native else 10,
        native_float64_engine_crosscheck=native,
        original_plant_solver_unchanged=True,
        original_rate_rad_s=slew,
        outcome=run['generations'][0]['candidates'][0],walking_qualified=False),indent=2)+'\n')
    original_arrays=np.load(source);actual_arrays=np.load(out/'generation-00.npz')
    parity={k:bool(np.array_equal(original_arrays[k],actual_arrays[k])) for k in original_arrays.files}
    (out/'source-isolation.json').write_text(json.dumps(parity,indent=2)+'\n')
    assert all(parity.values())
    save()
    for p in sources:
        if p.suffix=='.py':(out/p.name).write_bytes(p.read_bytes())
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())


if __name__=='__main__':main()
