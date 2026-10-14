"""Passive full-dynamics snapshots of the existing frozen seed-3 teacher.

Runs the original diagnostic once, never modifies its actions or plant. Records
the effective Warp inertia/forces for a later inverse-dynamics analysis. It is
not an inverse-dynamics controller, training, or hardware validation.
"""
import copy,dataclasses,hashlib,json,os,sys
from pathlib import Path
import numpy as np


def main():
    os.environ.setdefault('MPLCONFIGDIR','/tmp/xgoduck-dynamics-mpl-cache')
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    from mjlab.envs import ManagerBasedRlEnv
    import mujoco
    import mujoco_warp as mjwarp
    import warp as wp
    import diagnose_bam_control_tap as baseline
    output=Path(sys.argv[sys.argv.index('--output')+1])
    original=ManagerBasedRlEnv.step;records=[];properties={};count=0;probes=[]
    def clone(value):
        if isinstance(value,wp.array):return wp.clone(value)
        if dataclasses.is_dataclass(value):
            result=copy.copy(value)
            for field in dataclasses.fields(value):setattr(result,field.name,clone(getattr(value,field.name)))
            return result
        if isinstance(value,np.ndarray):return value.copy()
        if isinstance(value,(tuple,list)):return type(value)(clone(v) for v in value)
        if isinstance(value,dict):return {k:clone(v) for k,v in value.items()}
        return value
    def observed(env,action):
        nonlocal count
        result=original(env,action);count+=1
        if count<=250:return result
        if env.num_envs!=1:raise ValueError('Only frozen single-world replay')
        sim=env.sim;nv=sim.mj_model.nv
        qm=sim.wp_data.qM.numpy()[0]
        if qm.shape[0]==1:
            # Do not guess version-dependent packed-tree storage.
            raise ValueError('Expected native dense mass matrix; sparse requires explicit conversion')
        matrix=qm[:nv,:nv].copy()
        if not np.allclose(matrix,matrix.T,atol=2e-7):
            raise ValueError('Mass matrix storage is not full symmetric')
        names=('qpos','qvel','qacc','qfrc_bias','qfrc_passive','qfrc_actuator','qfrc_constraint','qfrc_applied','qfrc_smooth','solver_niter')
        row={n:getattr(sim.wp_data,n).numpy()[0].copy() for n in names};row['mass_matrix']=matrix
        records.append(row)
        if not properties:
            for name in ('body_mass','body_ipos','body_iquat','body_inertia','dof_armature','dof_damping','dof_frictionloss','geom_friction'):
                properties[name]=getattr(sim.wp_model,name).numpy()[0].tolist()
            properties['body_names']=[mujoco.mj_id2name(sim.mj_model,mujoco.mjtObj.mjOBJ_BODY,i) for i in range(sim.mj_model.nbody)]
            properties['joint_names']=[mujoco.mj_id2name(sim.mj_model,mujoco.mjtObj.mjOBJ_JOINT,i) for i in range(sim.mj_model.njnt)]
            properties['joint_qpos_addresses']=sim.mj_model.jnt_qposadr.tolist()
            properties['joint_dof_addresses']=sim.mj_model.jnt_dofadr.tolist()
            properties['geom_names']=[mujoco.mj_id2name(sim.mj_model,mujoco.mjtObj.mjOBJ_GEOM,i) for i in range(sim.mj_model.ngeom)]
            bam=env.scene['robot'].actuators[0];act=bam._bam_model.actuator
            properties['bam']=dict(kt=float(bam._bam_model.kt.value),resistance=float(bam._bam_model.R.value),
                kd_scale=bam._as_tensor(bam.kd_scale).numpy().tolist(),error_gain=float(act.error_gain),
                error_gain_ratio=float(bam._bam_model.error_gain_ratio.value),max_pwm=float(act.max_pwm))
            properties['solver']=dict(iterations=int(sim.wp_model.opt.iterations),ls_iterations=int(sim.wp_model.opt.ls_iterations),
                tolerance=sim.wp_model.opt.tolerance.numpy().tolist(),ls_tolerance=sim.wp_model.opt.ls_tolerance.numpy().tolist())
        if count-251 in (34,47,51,52,55):
            # Counterfactual numeric convergence probes use separate state/model
            # objects. Neither their result nor options enter the original run.
            entry=dict(step=count-251,variants=[])
            for iterations in (sim.wp_model.opt.iterations,100):
                wm=copy.copy(sim.wp_model);wm.opt=copy.copy(wm.opt);wm.opt.iterations=int(iterations)
                wd=clone(sim.wp_data)
                assert wd.qpos.ptr!=sim.wp_data.qpos.ptr and wd.qacc.ptr!=sim.wp_data.qacc.ptr
                mjwarp.forward(wm,wd)
                m=wd.qM.numpy()[0,:nv,:nv];qacc=wd.qacc.numpy()[0]
                f=wd.qfrc_smooth.numpy()[0]+wd.qfrc_constraint.numpy()[0]
                entry['variants'].append(dict(iterations=int(iterations),used_iterations=int(wd.solver_niter.numpy()[0]),
                    maximum_balance_residual=float(abs(m@qacc-f).max()),
                    maximum_acceleration_difference=float(abs(qacc-row['qacc']).max()),
                    maximum_constraint_force_difference=float(abs(wd.qfrc_constraint.numpy()[0]-row['qfrc_constraint']).max())))
            probes.append(entry)
        return result
    ManagerBasedRlEnv.step=observed
    try:baseline.main()
    finally:ManagerBasedRlEnv.step=original
    arrays={key:np.stack([r[key] for r in records]) for key in records[0]}
    trace=output/'full-dynamics.npz';np.savez_compressed(trace,**arrays)
    residual=arrays['mass_matrix']@arrays['qacc'][...,None]
    residual=residual[...,0]-(arrays['qfrc_smooth']+arrays['qfrc_constraint'])
    force_sum=arrays['qfrc_actuator']+arrays['qfrc_passive']+arrays['qfrc_applied']-arrays['qfrc_bias']
    bias_residual=force_sum-arrays['qfrc_smooth']
    # Independent uninstrumented trace identity is required before admitting data.
    base=Path(__file__).resolve().parents[2]/'output/duck_sim/overnight-20261011/bam-tracking-control-tap-v2/generation-00.npz'
    before=np.load(base);after=np.load(output/'generation-00.npz')
    parity={k:bool(np.array_equal(before[k],after[k])) for k in before.files}
    if not all(parity.values()):raise ValueError('Passive recorder changed reference trajectory')
    balanced=bool(abs(residual).max()<=2e-4 and abs(bias_residual).max()<=2e-5)
    report=dict(completed=True,scope=__doc__,hardware_access=False,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (Path(__file__),Path(baseline.__file__),base,trace)},effective_model=properties,
        uninstrumented_array_parity=parity,frames=len(records),maximum_forward_dynamics_residual=float(abs(residual).max()),
        maximum_force_sum_residual=float(abs(bias_residual).max()),
        forward_force_balance_within_requested_tolerance=balanced,requested_balance_tolerance=2e-4,
        solver_probes=probes,not_an_exact_inverse_dynamics_reference=not balanced,
        minimum_mass_matrix_eigenvalue=float(min(np.linalg.eigvalsh(m).min() for m in arrays['mass_matrix'])))
    (output/'dynamics-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    (output/'recorder-source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v for k,v in report.items() if k not in ('effective_model','source_sha256','scope','uninstrumented_array_parity')}),flush=True)


if __name__=='__main__':main()
