"""File-only head-coordinate map for two DECLARED load hypotheses.

No fitted real mass or deployment. Keep root and ALL leg coordinates identical
and solve the head-only minimum-change posture whose mean fore-aft COM equals
the reference. For these same-total-mass inertial endpoints the residual span
is fixed, so centering it minimizes the worst HOME fore-aft COM error. This is
an explicit minimax geometric objective, not a runtime lean/gain trim. Actual
stability, combined mechanical/cable clearance and uncertainty coverage must
be independently checked. Individual saved/CAD bounds are not expanded.
"""
import argparse,copy,json,time
from pathlib import Path
import mujoco as mj
import numpy as np
from scipy.optimize import minimize
from scipy.spatial import ConvexHull
from audit_pose import set_pose,forward_kinematics
from offline_mass_distribution import scenario_spec
from offline_calibration_overlay import range_snapshot
from solve_loaded_stance import ROOT,foot_vertices,sha


def solve(initial,candidate,xml,reference,loaded,objective='source-com'):
    build=lambda:mj.MjSpec.from_file(str(Path(xml).resolve()))
    models=[];evidence=[]
    for path in (reference,loaded):
        fn,proof=scenario_spec(build,path);models.append(fn().compile());evidence.append(proof)
    ds=[mj.MjData(m) for m in models];home=np.asarray(candidate['home_rad']);names=candidate['joint_names']
    ids=[candidate['policy_joint_ids'].index(i) for i in (30,31)]
    root=models[0].body('trunk_base').id
    for m,d in zip(models,ds):set_pose(m,d,dict(zip(names,home)),np.eye(3))
    target=float(ds[0].subtree_com[root,0]);baseline_foot=[d.xpos[[m.body('ankle_'+s).id for s in ('left','right')]].copy() for m,d in zip(models,ds)]
    initial_span=abs(ds[0].subtree_com[root,0]-ds[1].subtree_com[root,0])
    joints=[models[0].joint(names[i]).id for i in ids]
    low=np.maximum(np.asarray(candidate['lower_rad'])[ids],models[0].jnt_range[joints,0])-home[ids]
    high=np.minimum(np.asarray(candidate['upper_rad'])[ids],models[0].jnt_range[joints,1])-home[ids]
    # The pinned official command API already bounds each head delta to1rad.
    low=np.maximum(low,-1.);high=np.minimum(high,1.)
    hulls=[ConvexHull(np.concatenate([foot_vertices(m,d,s)[:,:2] for s in ('left','right')])) for m,d in zip(models,ds)]
    started=time.monotonic()
    def values(x):
        if time.monotonic()-started>30:raise TimeoutError('Bounded30s geometry solve')
        q=home.copy();q[ids]+=x
        for m,d in zip(models,ds):set_pose(m,d,dict(zip(names,q)),np.eye(3))
        return np.asarray([d.subtree_com[root,0]-target for d in ds])
    x0=np.asarray(initial['stance_delta_rad'])[ids]/2
    primary=None
    if objective=='source-com':
        constraint={'type':'eq','fun':lambda x:values(x).mean()}
    elif objective=='support-margin':
        def supports(x):
            values(x)
            return np.concatenate([-h.equations[:,:2]@d.subtree_com[root,:2]-h.equations[:,2] for h,d in zip(hulls,ds)])
        primary=minimize(lambda x:-x[2],np.r_[x0,supports(x0).min()],method='SLSQP',
            bounds=[*zip(low,high),(None,None)],
            constraints={'type':'ineq','fun':lambda x:supports(x[:2])-x[2]},
            options={'ftol':1e-12,'maxiter':100})
        if not primary.success:raise ValueError('Support-margin geometric optimum failed: '+str(primary.message))
        optimum=float(supports(primary.x[:2]).min());x0=primary.x[:2]
        #1nm is solver comparison tolerance, not a robot protection parameter.
        constraint={'type':'ineq','fun':lambda x:supports(x)-(optimum-1e-9)}
    else:raise ValueError('Unknown declared geometric objective')
    fit=minimize(lambda x:.5*float(x@x),x0,method='SLSQP',bounds=list(zip(low,high)),
                 constraints=constraint,options={'ftol':1e-12,'maxiter':100})
    error=values(fit.x);delta=np.zeros(14);delta[ids]=fit.x
    footerror=0.;fkerror=0.;margins=[]
    for m,d,feet0 in zip(models,ds,baseline_foot):
        feet=[m.body('ankle_'+s).id for s in ('left','right')]
        footerror=max(footerror,float(abs(d.xpos[feet]-feet0).max()))
        ro,po=forward_kinematics(m,d.qpos);fkerror=max(fkerror,float(abs(po-d.xpos).max()),float(abs(ro-d.xmat.reshape(-1,3,3)).max()))
        vertices=np.concatenate([foot_vertices(m,d,s)[:,:2] for s in ('left','right')]);hull=ConvexHull(vertices)
        xy=d.subtree_com[root,:2];margins.append(float((-hull.equations[:,:2]@xy-hull.equations[:,2]).min()))
    objective_valid=(abs(error.mean())<1e-7 if objective=='source-com' else min(margins)>=optimum-1e-8)
    valid=bool(fit.success and objective_valid and abs(error[0]-error[1])-initial_span<1e-10 and footerror<1e-6 and fkerror<1e-10 and min(margins)>0)
    out=copy.deepcopy(initial)
    out.update(kind='two_hypothesis_minimax_head_frame_ONLY_OFFLINE',geometry_verified=valid,
        solver_success=bool(fit.success),optimizer_message=str(fit.message),evaluations=fit.nfev,
        body_pitch_rad=0.,body_pitch_deg=0.,body_frame_rotation=np.eye(3).tolist(),
        trunk_forward_translation_m=0.,trunk_lateral_translation_m=0.,trunk_height_change_m=0.,
        stance_delta_rad=delta.tolist(),stance_delta_deg=np.degrees(delta).tolist(),stance_target_rad=(home+delta).tolist(),
        reference=evidence[0],loaded=evidence[1],whole_foot_position_error_m=footerror,whole_foot_orientation_error_rad=0.,max_independent_fk_error=fkerror,
        restored_whole_com_projection_error_m=None,mean_home_com_error_m=float(error.mean()),
        endpoint_home_com_error_m=error.tolist(),worst_home_com_error_m=float(abs(error).max()),endpoint_convex_support_margin_m=margins,
        initial_endpoint_com_span_m=float(initial_span),source_sha256=sha(__file__),wall_seconds=time.monotonic()-started,
        uncertainty_scope='ONLY the two unmeasured declared inertial hypotheses; not identified real bounds or actual support/contact proof',
        geometric_check_scope='Identity root/leg map; complete feet preserved; same-mass endpoint COM span centered exactly, minimizing worst HOME fore-aft error. Positive convex-hull margins do not prove dynamic/contact stability.',
        objective_scope='Minimum squared head-angle offset among the minimax HOME solutions; no runtime gain, threshold, reward or neural weight changes',
        reference_gait_sha256=None,reference_gait_samples=0,hardware_eligible=False,dynamic_stability_verified=False)
    out['declared_objective']=objective
    if primary is not None:
        out.update(primary_support_margin_m=optimum,
            geometric_check_scope='Identity root/legs; maximize worst two-hypothesis convex support margin, then minimize head-angle change. Static geometry only; no dynamic/contact certificate.',
            objective_scope='Lexicographic worst support margin then minimum squared head delta; fixed original head command bounds, no parameter sweep')
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('initial','calibration','reference','loaded','output'):p.add_argument('--'+k,type=Path,required=True)
    p.add_argument('--objective',choices=('source-com','support-margin'),default='source-com')
    a=p.parse_args();assert not a.output.exists()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    c,overlay=range_snapshot(json.loads(cp.read_text())['candidate'],a.calibration)
    initial=json.loads(a.initial.read_text());assert initial['calibration_overlay']['canonical_sha256']==c['calibration']['sha256']
    xml=ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml'
    out=solve(initial,c,xml,a.reference,a.loaded,a.objective);out['calibration_overlay']=overlay
    a.output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps({k:out[k] for k in ('geometry_verified','stance_delta_deg','endpoint_home_com_error_m','endpoint_convex_support_margin_m')}))

if __name__=='__main__':main()
