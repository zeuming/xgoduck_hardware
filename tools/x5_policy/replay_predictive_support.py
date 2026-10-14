"""Bounded receding-horizon support/swing teacher, simulation only.

Uses the complete cloned Warp/BAM/EMA/delay state and original plant. A recorded
failed teacher supplies a reference, not future physical states. Two bilateral
joint waypoints are optimized by finite-difference shooting at each replan.
Predicted future delay draws are an assumption, never copied from future plant
observations. Only requested joint targets enter the source environment.
"""
import hashlib
import json
import os
import sys
import time
from pathlib import Path
import numpy as np


def main():
    solver = 'least_squares'
    if '--solver' in sys.argv:
        i=sys.argv.index('--solver');solver=sys.argv[i+1];del sys.argv[i:i+2]
    if solver not in ('least_squares','powell','powell_mixed'):raise ValueError('Unknown optimizer')
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/xgoduck-dynamics-mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch
    from scipy.optimize import least_squares, minimize
    from mjlab.envs import ManagerBasedRlEnv
    from bam_prediction_branch import BamPredictionBranch
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
    # One bounded episode, four replans, maximum 40 seconds each. No automatic
    # budget extension or model/guard tuning after observing a failed result.
    replans = (32, 40, 48, 56)
    end = 82
    plans = []; command_trace = []; count = 0; last = None; active_plan = None
    sources = [Path(__file__), Path(baseline.__file__), Path(__file__).with_name('bam_prediction_branch.py'), cp, source]
    hashes = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
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
        initial = BamPredictionBranch(env).measure()
        weight = max(float(initial['load'].sum()), 1.)
        cache = {}
        def objective(x):
            nonlocal best,best_x
            key = np.asarray(x,dtype=np.float64).tobytes()
            if key in cache:return cache[key][0]
            if time.monotonic()-started>40:raise TimeoutError('40s replan budget')
            branch = BamPredictionBranch(env)
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
        def jac(x):
            # Absolute 1e-3rad perturbation, far above float32 ULP. No default
            # ~1e-8 finite difference that previously invalidated the IK helper.
            f=objective(x);J=np.zeros((len(f),len(x)))
            for j in range(len(x)):
                y=x.copy();d=.001 if x[j]+.001<=hi[j] else -.001
                if y[j]+d<lo[j]:d=min(.001,(hi[j]-lo[j])/2)
                y[j]+=d;J[:,j]=(objective(y)-f)/d
            return J
        status='not_started'
        try:
            objective(x0)
            if solver=='least_squares':
                result=least_squares(objective,x0,jac=jac,bounds=(lo,hi),max_nfev=8,
                    x_scale=.025,ftol=1e-5,xtol=1e-5,gtol=1e-5)
            else:
                # Derivative-free comparison after the paired .001/.004rad
                # Jacobians disagreed. Same objective, bounds, state and budget.
                def scalar(x):
                    r=objective(x);return float(r@r)
                options=dict(maxfev=160,maxiter=4,xtol=1e-4,ftol=1e-5)
                if solver=='powell_mixed':
                    # Orthogonal cosine directions cover BOTH legs and BOTH
                    # waypoints from the first line search. Original axis-order
                    # Powell exhausted 40s on the first few left-leg variables.
                    size=len(x0); k=np.arange(size)[:,None]; j=np.arange(size)[None,:]
                    directions=np.sqrt(2/size)*np.cos(np.pi*(j+.5)*k/size)
                    directions[0]=1/np.sqrt(size)
                    np.testing.assert_allclose(directions@directions.T,np.eye(size),atol=1e-13,rtol=0)
                    options['direc']=directions
                result=minimize(scalar,x0,method='Powell',bounds=list(zip(lo,hi)),options=options)
            status=result.message
        except TimeoutError as exc:status=str(exc)
        if best_x is None:
            # No predicted-safe plan admitted. Preserve original reference;
            # never silently execute an optimizer's failed/rejected iterate.
            selected=None
        else:selected=cache[best_x.tobytes()][1]
        info=dict(index=index,time_s=index*.02,horizon_s=horizon*.02,
            solver=solver,
            knots_steps=knot.tolist(),max_budget_seconds=40,wall_seconds=time.monotonic()-started,
            status=status,evaluations=evaluations,best=best,
            chosen_waypoint_delta_rad=None if best_x is None else best_x.tolist(),
            chosen_commands_rad=None if selected is None else selected.tolist())
        plans.append(info);save();print(json.dumps({k:v for k,v in info.items() if k not in ('evaluations','chosen_commands_rad')}),flush=True)
        return selected
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
        original_request=(home+c['action_scale']*action.numpy()).astype(float)
        if active_plan is not None and index<end:
            request=active_plan[index-index_prev][None,:]
        else:request=original_request
        request=last+np.clip(request-last,-max_delta,max_delta)
        chosen=torch.from_numpy(((request-home)/c['action_scale']).astype(np.float32))
        result=original(env,chosen)
        final=env.action_manager.get_term('joint_pos').requested_targets.numpy().copy()
        command_trace.append(dict(index=index,time_s=(index+1)*.02,
            request_rate_rad_s=float(abs(final-last).max()/.02),requested_rad=final[0].tolist()))
        last=final;count+=1;return result
    ManagerBasedRlEnv.step=controlled
    try:baseline.main()
    finally:ManagerBasedRlEnv.step=original
    run=json.loads((out/'search.json').read_text())
    (out/'predictive-summary.json').write_text(json.dumps(dict(completed=True,scope=__doc__,hardware_access=False,
        original_physical_limits_unchanged=True,source_sha256=hashes,plans=plans,
        known_future_delay=False,optimizer_budget_seconds=160,full_state_teacher_not_deployable_policy=True,
        solver=solver,
        original_rate_rad_s=slew,max_request_rate_rad_s=max(r['request_rate_rad_s'] for r in command_trace),
        outcome=run['generations'][0]['candidates'][0],walking_qualified=False),indent=2)+'\n')
    save()
    for p in sources:
        if p.suffix=='.py':(out/p.name).write_bytes(p.read_bytes())
    assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())


if __name__=='__main__':main()
