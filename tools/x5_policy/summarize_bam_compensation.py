"""Audit the completed paired motor-law compensation experiments."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'


def main():
    plan=json.loads((BASE/'bam-compensation-ablation-v1/plan.json').read_text())
    assert plan['completed']
    rows=[]
    for mode in ('none','back-emf','back-emf-friction'):
        for seed in range(4):
            directory=(BASE/'nonlinear-swing-clearance-v4'/f'seed-{seed}' if mode=='none'
                else BASE/'bam-compensation-ablation-v1'/f'{mode}-seed-{seed}')
            path=directory/'search.json';s=json.loads(path.read_text());g=s['generations'][0]
            trace=directory/'generation-00.npz'
            assert s['completed'] and s['source_hashes_unchanged']
            assert hashlib.sha256(trace.read_bytes()).hexdigest()==g['trace_sha256']
            z=np.load(trace);idx=int(z['tilt'][:,0].argmax())
            rpy=Rotation.from_quat(z['root'][idx,0,3:7],scalar_first=True).as_euler('xyz',degrees=True)
            c=g.get('compensation_records',[])
            row=dict(mode=mode,seed=seed,source=str(path.relative_to(BASE)),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                initial_state_sha256=g['initial_state_sha256'],best=s['best'],
                maximum_tilt_sample=dict(time_s=(idx+1)*.02,root_rpy_deg=rpy.tolist(),loads_n=z['load'][idx,0].tolist()),
                actual_goal_max_error_deg=float(np.rad2deg(abs(z['goal']-z['actual']).max())),
                reference_interpretation='goal error includes feedforward torque offset; not alone a tracking-quality comparison')
            if c:
                nominal=np.asarray([v['nominal_rad'] for v in c])
                assert len(nominal)==len(z['actual'])
                row.update(max_compensation_deg=float(np.rad2deg(np.abs([v['offset_rad'] for v in c]).max())),
                    range_projection_frames=sum(v['range_projected'] for v in c),
                    max_range_projection_deg=float(np.rad2deg(max(v['max_range_projection_rad'] for v in c))),
                    feedforward_pwm_projection_frames=sum(v['pwm_clipped'] for v in c),
                    sampled_max_total_duty=float(abs(z['bam_control_voltage']/z['bam_vin']).max()))
            speed=float(np.max(abs(np.diff(z['requested'],axis=0)))/.02)
            assert speed<=600*2*np.pi/4095+1e-5
            row['maximum_requested_speed_rad_s']=speed
            np.testing.assert_array_equal(z['requested'][:,:,5:9],np.broadcast_to(z['requested'][0:1,:,5:9],z['requested'][:,:,5:9].shape))
            rows.append(row)
    assert all(len({r['initial_state_sha256'] for r in rows if r['seed']==seed})==1 for seed in range(4))
    summary={mode:dict(cases=4,qualified=sum(r['best']['single_cycle_qualified'] for r in rows if r['mode']==mode),
        failed=sum(r['best']['failed'] for r in rows if r['mode']==mode),
        maximum_clearance_m=max(r['best']['maximum_swing_clearance_m'] for r in rows if r['mode']==mode))
        for mode in ('none','back-emf','back-emf-friction')}
    report=dict(completed=True,hardware_access=False,trained_policy=False,paired_initial_states=True,
        unchanged_actor=True,unchanged_gains_ranges_ema_slew=True,summary=summary,runs=rows,
        conclusion='Motor-law position lead alone did not produce a qualified single cycle; friction arm increased tilt failures. No gait or actuator-capacity proof.')
    (BASE/'bam-compensation-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(summary))


if __name__=='__main__':main()
