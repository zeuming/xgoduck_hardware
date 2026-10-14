"""Summarize isolated numerical checks without substituting their plant."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np

os.environ.setdefault('MPLCONFIGDIR', '/tmp/xgoduck-dynamics-mpl-cache')
ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'output/duck_sim/overnight-20261011'


def main():
    folder = BASE/'trajectory-convergence-v3'
    raw = json.loads((folder/'convergence-summary.json').read_text())
    assert raw['completed'] and all(raw['source_arrays_bitwise_equal'].values())
    arrays = {n: np.load(folder/(n+'.npz')) for n in raw['variants']}
    passive_parity = {}
    for name, z in arrays.items():
        old = np.load(BASE/'trajectory-convergence-v2'/(name+'.npz'))
        passive_parity[name] = {k: bool(np.array_equal(old[k],z[k])) for k in old.files}
    assert all(all(v.values()) for v in passive_parity.values())
    sensitivity = {}
    for label, subdir in [('native10','predictive-sensitivity-v1'),
                          ('cap100','predictive-sensitivity-cap100-v1'),
                          ('native_float64','predictive-sensitivity-native-v1')]:
        path = BASE/subdir/'predictive-summary.json'
        r = json.loads(path.read_text());plan=r['plans'][0]
        isolation=json.loads((BASE/subdir/'source-isolation.json').read_text())
        assert r['completed'] and all(isolation.values())
        columns = plan['columns']; groups = plan['groups']
        z=np.load(BASE/subdir/'sensitivity.npz')
        sensitivity[label] = dict(completed=True, queries=len(plan['evaluations']),
            negative_direction_cosines=sum(v['cosine']<0 for v in columns),
            sign_changed_cost_derivatives=sum(v['cost_derivative_small']*v['cost_derivative_large']<0 for v in columns),
            load_fraction_squared_jacobian=float(groups['load']['norm_small']**2/np.square(z['jac_small']).sum()),
            groups=groups, source_unchanged=all(isolation.values()),
            wall_seconds=plan['wall_seconds'],
            report_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    native_folder=BASE/'native-bam-crosscheck-v1'
    native=json.loads((native_folder/'native-physics-summary.json').read_text())
    repeated=json.loads((BASE/'predictive-repeat-native-v1/predictive-summary.json').read_text())
    assert native['completed'] and all(native['original_source_arrays_bitwise_equal'].values())
    assert repeated['completed'] and repeated['plans'][0]['all_identical']
    native_arrays=np.load(native_folder/'native-physics.npz')
    audited=json.loads((BASE/'native-bam-physics-audit-v1/physics-field-audit.json').read_text())
    audited_arrays=np.load(BASE/'native-bam-physics-audit-v1/native-physics.npz')
    audit_parity={k:bool(np.array_equal(native_arrays[k],audited_arrays[k])) for k in native_arrays.files}
    assert not audited['changed_physical_fields'] and not audited['changed_non_numerical_options']
    assert all(audit_parity.values())
    summary = dict(completed=True, scope=__doc__, hardware_access=False,
        solver_context_tap_passive_bitwise_checks=passive_parity,
        numerical_comparison=raw, sensitivity_comparison=sensitivity,
        native_float64_engine_crosscheck=native,
        native_repeat_identical=repeated['plans'][0],
        native_physics_field_audit=audited,native_physics_audit_array_parity=audit_parity,
        conclusion=dict(full_trajectory_numeric_convergence_established=False,
            increasing_iteration_cap_reduces_large_residuals_but_not_all=True,
            cost_improvement_termination_can_precede_small_force_gradient=True,
            native_engine_force_equation_residual_small_but_no_lift=True,
            microperturbation_direction_consistency_not_fixed_by_engine_crosscheck=True,
            original_reference_still_no_qualified_lift=True,
            no_numerical_change_selected_for_production=True,
            walking_qualified=False, physical_impossibility_proved=False),
        official_reference='https://mujoco.readthedocs.io/en/stable/computation.html',
        limitations=['2e-4 is an offline equation-check tolerance, not a changed robot protection limit.',
            'Known raw actions and delays; open-loop plant comparison, not closed-loop controller acceptance.',
            'Strict100 changes numerical tolerances and iteration budgets, not floating point dtype.',
            'Predictor omits zero-push coordinate roundoff; native copy/source difference is recorded explicitly.',
            'No physical device cause is established by these solver comparisons.'])
    vendor = ROOT/'output/duck_sim/repair-20261011/model/training-adapter/training-venv/lib/python3.12/site-packages/mujoco_warp/_src'
    sources=[Path(__file__),vendor/'solver.py',vendor/'forward.py',folder/'convergence-summary.json']
    summary['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (BASE/'trajectory-convergence-summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 9, 'axes.titlesize': 10})
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    fig.subplots_adjust(left=.085,right=.98,bottom=.08,top=.91,wspace=.28,hspace=.34)
    for name,z in arrays.items():
        t=(np.arange(len(z['height']))+1)*.02
        dt=raw['configurations'][name]['dt']; sub=(np.arange(len(z['substep_residual']))+1)*dt
        axes[0,0].plot(t,z['height'][:,0]*1000,label=name)
        axes[0,1].semilogy(sub,np.maximum(abs(z['substep_residual'][:,:3]).max(axis=1),1e-8),label=name,alpha=.8)
        axes[1,0].plot(t,z['tilt'],label=name)
        delta=np.cumsum(z['substep_load'].astype(float)-arrays['native10']['substep_load'],axis=0)*dt
        axes[1,1].plot(sub,delta[:,0],label=name+' left')
        axes[1,1].plot(sub,delta[:,1],linestyle='--',label=name+' right')
    t=(np.arange(len(native_arrays['height']))+1)*.02
    sub=(np.arange(len(native_arrays['substep_residual']))+1)*raw['configurations']['native10']['dt']
    axes[0,0].plot(t,native_arrays['height'][:,0]*1000,linestyle='--',label='float64 engine cross-check')
    axes[0,1].semilogy(sub,np.maximum(abs(native_arrays['substep_residual'][:,:3]).max(axis=1),1e-15),
                      linestyle='--',alpha=.7,label='float64 engine cross-check')
    axes[1,0].plot(t,native_arrays['tilt'],linestyle='--',label='float64 engine cross-check')
    axes[0,0].axhline(3,color='black',linestyle=':',label='required 3 mm')
    axes[0,0].set(title='Entire swing sole: none reaches the required height',ylabel='Minimum clearance (mm)')
    axes[0,1].axhline(2e-4,color='black',linestyle=':',label='equation-check threshold')
    axes[0,1].set(title='Root force-balance residual at every 5 ms substep',ylabel='Maximum root force residual (N)')
    axes[1,0].axhline(15,color='black',linestyle=':',label='unchanged observation boundary')
    axes[1,0].set(title='Body tilt',ylabel='Tilt (degrees)')
    axes[1,1].set(title='Contact impulse difference from native10',ylabel='Cumulative difference (N s)')
    for a in axes.ravel():
        a.set_xlabel('Time (s)');a.grid(alpha=.2);a.legend(fontsize=7)
    fig.suptitle('Isolated known-action / known-delay replay; no walking or hardware qualification',fontsize=12,y=.98)
    fig.savefig(BASE/'trajectory-convergence-comparison.png',dpi=150,bbox_inches='tight');plt.close(fig)
    print(json.dumps(dict(variants=raw['variants'],sensitivity=sensitivity),indent=2))


if __name__ == '__main__':
    main()
