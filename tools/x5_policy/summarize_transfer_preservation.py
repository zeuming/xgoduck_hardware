"""Verify completed preservation control parity and fixed preparation axes."""
import hashlib,json
from pathlib import Path
import numpy as np

BASE=Path(__file__).resolve().parents[2]/'output/duck_sim/overnight-20261011'


def main():
    out=BASE/'transfer-preservation-ablation-v1'
    plan=json.loads((out/'plan.json').read_text());assert plan['completed']
    control=np.load(out/'control-seed-3/generation-00.npz')
    prior=np.load(BASE/'bam-compensation-control-v1/generation-00.npz')
    assert set(control.files)==set(prior.files)
    for name in control.files:np.testing.assert_array_equal(control[name],prior[name])
    rows=[]
    for seed in range(4):
        d=out/f'preserve-seed-{seed}';p=d/'search.json';s=json.loads(p.read_text());g=s['generations'][0]
        old=json.loads((BASE/f'nonlinear-swing-clearance-v4/seed-{seed}/search.json').read_text())
        assert s['completed'] and s['source_hashes_unchanged']
        assert g['initial_state_sha256']==old['generations'][0]['initial_state_sha256']
        trace=d/'generation-00.npz';assert hashlib.sha256(trace.read_bytes()).hexdigest()==g['trace_sha256']
        z=np.load(trace);speed=float(abs(np.diff(z['requested'],axis=0)).max()/.02)
        assert speed<=600*2*np.pi/4095+1e-5
        np.testing.assert_array_equal(z['requested'][:,:,5:9],np.broadcast_to(z['requested'][0:1,:,5:9],z['requested'][:,:,5:9].shape))
        rows.append(dict(seed=seed,source=str(p.relative_to(BASE)),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
            initial_state_sha256=g['initial_state_sha256'],maximum_requested_speed_rad_s=speed,best=s['best']))
    report=dict(completed=True,hardware_access=False,deployed=False,trained_policy=False,control_bitwise_parity_arrays=len(control.files),
        paired_initial_states=True,head_target_unchanged=True,original_rate_and_range_gates_unchanged=True,
        runs=rows,qualified=sum(r['best']['single_cycle_qualified'] for r in rows),
        failures=sum(r['best']['failed'] for r in rows),
        conclusion='Keeping the original hip yaw/roll transfer request during sagittal clearance IK is insufficient: all four cases have no qualified air frame. Not a sole-cause or impossibility proof.')
    (BASE/'transfer-preservation-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='runs'}))


if __name__=='__main__':main()
