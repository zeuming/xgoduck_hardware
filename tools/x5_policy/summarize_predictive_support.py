"""Reduce source-bound offline prediction/optimization evidence; no actuation."""
import hashlib
import json
import os
from pathlib import Path
import numpy as np


def main():
    root=Path(__file__).resolve().parents[2];base=root/'output/duck_sim/overnight-20261011'
    def read(path):return json.loads((base/path).read_text())
    replay=read('bam-prediction-replay-v5/prediction-parity.json')
    sampled=read('bam-prediction-sampled-v5/prediction-parity.json')
    assert replay['known_delay_replay_passed']
    assert all(replay['source_arrays_bitwise_equal'].values()) and all(sampled['source_arrays_bitwise_equal'].values())
    sens=read('predictive-sensitivity-v1/predictive-summary.json')['plans'][0]
    repeat=read('predictive-repeat-v1/predictive-summary.json')['plans'][0]
    assert repeat['all_identical']
    assert all(read('predictive-sensitivity-v1/source-isolation.json').values())
    assert all(read('predictive-repeat-v1/source-isolation.json').values())
    native=np.load(base/'bam-tracking-control-tap-v2/generation-00.npz')
    sd=np.asarray([r['predicted_measurement']['load'] for r in sampled['records']])
    true_load=native['load'][:len(sd),0];difference=sd-true_load
    force_windows={}
    for n in (1,3,5,9):
        d=np.stack([np.convolve(difference[:,i],np.ones(n)/n,'valid') for i in range(2)],1)
        force_windows[str(n*.02)]=dict(maximum_average_load_error_n=float(abs(d).max()),rmse_n=float(np.sqrt((d*d).mean())))
    jac=np.load(base/'predictive-sensitivity-v1/sensitivity.npz')
    load_indices=np.array([37*h+i for h in range(50) for i in (2,3)])
    sources=[Path(__file__),base/'bam-prediction-replay-v5/prediction-parity.json',
        base/'bam-prediction-sampled-v5/prediction-parity.json',
        base/'predictive-sensitivity-v1/predictive-summary.json',base/'predictive-sensitivity-v1/sensitivity.npz',
        base/'predictive-repeat-v1/predictive-summary.json',base/'bam-tracking-control-tap-v2/generation-00.npz']
    report=dict(completed=True,scope=__doc__,hardware_access=False,walking_qualified=False,
        root_state_writes_to_source_plant=0,original_physical_limits_unchanged=True,
        predictor=dict(known_delay_replay_passed=True,horizon_s=2.,
            bitwise_prefix_frames=next((r['step'] for r in replay['records'] if not all(r['bitwise_equal'].values())),len(replay['records'])),
            replay_maximum_error=replay['maximum_error'],replay_measurement_error=replay['measurement_maximum_error'],
            sampled_future_delay_measurement_error=sampled['measurement_maximum_error'],
            load_window_errors=force_windows,source_isolation_passed=True,
            caveat='Known delay replay verifies implementation; future delays in optimization are sampled. Neither is a real hardware observer or certified prediction tube.'),
        derivative_diagnosis=dict(perturbations_rad=sens['epsilon_rad'],
            directions_with_negative_cosine=sum(x['cosine']<0 for x in sens['columns']),
            cost_derivative_sign_flips=sum(x['cost_derivative_small']*x['cost_derivative_large']<0 for x in sens['columns']),
            directions=20,groups=sens['groups'],
            load_fraction_squared_jacobian_norm=float(np.sum(jac['jac_small'][load_indices]**2)/np.sum(jac['jac_small']**2)),
            three_identical_query_repeats_bitwise=True,
            interpretation='Local derivative sensitivity is poor under this objective/physics path; this is not a global no-walking proof or a diagnosis of the real motors.'),
        controllers=[])
    traces=[('reference',native)]
    for folder in ('predictive-support-v2','predictive-support-powell-v1','predictive-support-mixed-v1'):
        p=base/folder/'predictive-summary.json';r=json.loads(p.read_text());z=np.load(base/folder/'generation-00.npz')
        s=read(folder+'/search.json');ref=read('bam-tracking-control-tap-v2/search.json')
        initial_equal=s['generations'][0]['initial_state_sha256']==ref['generations'][0]['initial_state_sha256']
        properties_equal=s['generations'][0]['env0_effective_body_properties']==ref['generations'][0]['env0_effective_body_properties']
        assert initial_equal and properties_equal
        assert r['max_request_rate_rad_s'] <= r['original_rate_rad_s']+2e-5
        rows=[v for plan in r['plans'] for v in plan['evaluations']]
        record=dict(folder=folder,solver=r.get('solver','least_squares'),outcome=r['outcome'],
            initial_state_and_effective_properties_match=True,maximum_request_rate_rad_s=r['max_request_rate_rad_s'],
            nominal_total_optimizer_budget_seconds=160,actual_optimizer_wall_seconds=sum(x['wall_seconds'] for x in r['plans']),
            evaluation_counts=[len(x['evaluations']) for x in r['plans']],
            per_replan_status=[x['status'] for x in r['plans']],
            number_of_predicted_candidates_with_any_air=sum(v['air_frames']>0 for v in rows),
            admitted_waypoint_maximum_delta_deg=[float(np.rad2deg(np.max(abs(np.asarray(x['chosen_waypoint_delta_rad']))))) if x['chosen_waypoint_delta_rad'] is not None else None for x in r['plans']],
            is_offline_full_state_teacher=True,is_50hz_deployable_controller=False)
        report['controllers'].append(record);traces.append((record['solver'],z));sources.extend([p,base/folder/'search.json',base/folder/'generation-00.npz'])
    # The new control branches must share the same actual prefix before replanning.
    first=traces[1][1]
    prefix={}
    for label,z in traces[2:]:
        prefix[label]={k:bool(np.array_equal(first[k][:32],z[k][:32])) for k in first.files if k!='parameters'}
    assert all(all(v.values()) for v in prefix.values())
    report['paired_control_prefix_before_first_plan']=prefix
    report['sources']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    (base/'predictive-control-summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    os.environ.setdefault('MPLCONFIGDIR','/tmp/xgoduck-dynamics-mpl-cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(10,9),sharex=True)
    for label,z in traces:
        t=(np.arange(len(z['tilt']))+1)*.02
        ax[0].plot(t,z['height'][:,0,0]*1000,label=label)
        ax[1].plot(t,z['load'][:,0,0]/np.maximum(z['load'][:,0].sum(1),1e-8),label=label,alpha=.75)
        ax[2].plot(t,z['tilt'][:,0],label=label)
    ax[0].axhline(3,color='black',ls='--',label='Required 3 mm');ax[0].set_ylabel('Whole sole clearance (mm)')
    ax[1].set_ylabel('Left fraction of vertical load');ax[1].set_ylim(-.05,1.05)
    ax[2].axhline(15,color='black',ls='--');ax[2].set_ylabel('Body tilt (deg)');ax[2].set_xlabel('Seconds after preparation begins')
    ax[0].legend(ncol=2);fig.suptitle('Offline predictive teachers: actual simulated feet, unchanged physical limits')
    for a in ax:a.grid(alpha=.2)
    fig.tight_layout();fig.savefig(base/'predictive-control-comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k not in ('sources','paired_control_prefix_before_first_plan')},indent=2))


if __name__=='__main__':main()
