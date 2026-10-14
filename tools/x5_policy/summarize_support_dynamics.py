"""Summarize bounded offline support diagnostics without admitting a gait."""
import hashlib,json,os
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2];BASE=ROOT/'output/duck_sim/overnight-20261011'


def main():
    paths=['support-load-bound-v1/summary.json','capture-balance-v1/summary.json','centroidal-wrench-v1/summary.json',
        'full-dynamics-replay-v3/dynamics-summary.json','inverse-contact-authority-v2/summary.json',
        'inverse-contact-replay-control-v1/lead-summary.json','inverse-contact-replay-lead-v2/lead-summary.json']
    docs={p:json.loads((BASE/p).read_text()) for p in paths}
    assert all(d['completed'] for d in docs.values())
    load,cap,wrench,dynamics,inverse,control,treatment=[docs[p] for p in paths]
    assert all(control['control_array_parity'].values()) and treatment['initial_state_and_effective_properties_match']
    assert not control['outcome']['single_cycle_qualified'] and not treatment['outcome']['single_cycle_qualified']
    assert all(dynamics['uninstrumented_array_parity'].values())
    paths.extend(['inverse-contact-replay-control-v1/generation-00.npz','inverse-contact-replay-lead-v2/generation-00.npz'])
    report=dict(completed=True,hardware_access=False,production_changed=False,walking_qualified=False,
        proof_of_impossibility=False,source_sha256={str(BASE/p):hashlib.sha256((BASE/p).read_bytes()).hexdigest() for p in paths},
        source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        static_support=dict(cases=len(load['runs']),minimum_swing_load_fraction=[r['minimum_static_swing_load_fraction'] for r in load['runs']],
            single_support_frames=sum(r['frames_admitting_static_single_support'] for r in load['runs']),
            assumptions='Gravity-only equilibrium, optimistic entire foot XY hull, recorded poses only; not dynamic impossibility.'),
        capture_point=dict(cases=len(cap['runs']),inside_stance_frames=sum(r['capture_inside_stance_frames'] for r in cap['runs']),
            maximum_velocity_jacobian_fd_error_m_s=max(r['maximum_jacobian_velocity_error_m_s'] for r in cap['runs']),
            assumptions='Constant-height zero-centroidal-momentum-rate reduced model only; conditional velocity demand never commanded.'),
        wrench=dict(cases=len(wrench['runs']),frame_checks=wrench['virtual_work_checks'],
            actual_zero_swing_load_stance_cop_frames=[dict(source=r['source'],frames=[x['step'] for x in r['records']
                if x['cop_distance_to_optimistic_stance_m']<1e-7 and x['actual_swing_load_fraction']<1e-7]) for r in wrench['runs']],
            warning='Recorded constraint wrench is source-aligned, but finite solver residual means it is not always exact M*qacc+body bias.'),
        dynamics=dict(frames=dynamics['frames'],maximum_force_balance_residual=dynamics['maximum_forward_dynamics_residual'],
            exact_balance_check_passed=dynamics['forward_force_balance_within_requested_tolerance'],
            solver_probes=dynamics['solver_probes'],passive_reference_array_equality=True,
            warning='Increasing solver iterations was tested on copies of five states only, not a changed trajectory or a walking fix.'),
        inverse_contact=dict(local_full_interval_feasible=sum(r['feasible'] for r in inverse['rows'] if r['mode']=='full_saved_interval'),
            local_one_frame_interval_feasible=sum(r['feasible'] for r in inverse['rows'] if r['mode']=='one_20ms_goal_interval'),
            independent_mass_matrix_max_error=max(r['independent_mass_matrix_error'] for r in inverse['rows']),
            assumptions=inverse['scope']),
        paired_replay=dict(control=control['outcome'],treatment=treatment['outcome'],rise_s=treatment['rise_s'],
            initial_state_match=True,control_array_equality=True,original_rate_rad_s=treatment['original_rate_rad_s'],
            measured_max_rate_rad_s=treatment['maximum_final_request_rate_rad_s'],
            conclusion='Frozen instantaneous inverse-contact lead does not produce a qualified lift; no amplitude sweep or hardware release justified.'),
        next_step='If continuing: actuator-state-aware multi-step contact prediction/controller, with state/target history, EMA, slew and BAM delay in the rollout; validate stand/lift/land/return and bilateral net forward independently. Do not reuse a static or instantaneous LP solution as a validated path.')
    (BASE/'support-dynamics-summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    os.environ.setdefault('MPLCONFIGDIR','/tmp/xgoduck-dynamics-mpl-cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(9,8),sharex=True,layout='constrained')
    for path,label,color in [('inverse-contact-replay-control-v1','Original teacher','#2878b5'),('inverse-contact-replay-lead-v2','Inverse-contact lead','#cf6432')]:
        z=np.load(BASE/path/'generation-00.npz');t=(np.arange(len(z['height']))+1)*.02
        axes[0].plot(t,1000*z['height'][:,0,0],label=label,color=color)
        axes[1].plot(t,z['load'][:,0,0]/z['load'][:,0].sum(axis=1),color=color)
        axes[2].plot(t,z['tilt'][:,0],color=color)
    axes[0].axhline(3,color='black',ls='--',lw=.9,label='Required clearance: 3 mm')
    axes[0].set_ylabel('Whole sole clearance (mm)');axes[0].legend(loc='upper right',fontsize=9)
    axes[1].set_ylabel('Swing-foot load fraction');axes[2].set_ylabel('Body tilt (degrees)')
    axes[2].axhline(15,color='black',ls='--',lw=.9);axes[2].set_xlabel('Time after 5-second zero-policy warmup (s)')
    for ax in axes:ax.grid(alpha=.2);ax.axvspan(treatment['begin_s'],treatment['end_s'],color='#cf6432',alpha=.08)
    fig.suptitle('Paired seed-3 simulation: neither controller passed the single-step gate')
    fig.savefig(BASE/'support-dynamics-comparison.png',dpi=160);plt.close(fig)
    print(json.dumps(dict(completed=True,static_cases=8,inverse_full_interval=report['inverse_contact']['local_full_interval_feasible'],
        inverse_one_frame=report['inverse_contact']['local_one_frame_interval_feasible'],
        control_max_clearance_mm=1000*control['outcome']['maximum_swing_clearance_m'],
        treatment_max_clearance_mm=1000*treatment['outcome']['maximum_swing_clearance_m'],walking_qualified=False)))


if __name__=='__main__':main()
