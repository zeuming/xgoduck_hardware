"""Bind the offline teacher comparisons and separate geometry from dynamics."""
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'output/duck_sim/overnight-20261011'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    candidate_path = ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    c = json.loads(candidate_path.read_text())['candidate']
    model = mujoco.MjModel.from_xml_path(c['reset_placement']['xml_path'])
    data = mujoco.MjData(model)
    adr = np.asarray([model.jnt_qposadr[model.joint(n).id] for n in c['joint_names']])
    gid = model.geom('left_foot_collision').id
    mesh = model.geom_dataid[gid]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    vertices = model.mesh_vert[start:start+count].astype(float)
    def height(root, q):
        data.qpos[:7] = root
        data.qpos[adr] = q
        mujoco.mj_forward(model, data)
        return float((vertices@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid])[:,2].min())
    report = dict(hardware_access=False, walking_qualified=False,
        original_physical_limits_unchanged=True, original_actor_unchanged=True,
        old_float32_ik_failures_invalid_as_feasibility_evidence=True,
        counterfactual_note='Substitute joint configurations at the recorded post-step root. These geometries are not attainable dynamics or torque proofs.',
        runs=[])
    hashes = {}
    for arm in ('nonlinear-swing-precise-v3', 'nonlinear-swing-clearance-v4'):
        plan = json.loads((BASE/arm/'plan.json').read_text())
        assert plan['completed']
        for seed in range(4):
            path = BASE/arm/f'seed-{seed}'/'search.json'
            s = json.loads(path.read_text()); g = s['generations'][0]
            assert s['completed'] and s['source_hashes_unchanged']
            trace = path.with_name('generation-00.npz')
            assert sha(trace) == g['trace_sha256']
            z = np.load(trace)
            hashes.setdefault(seed, []).append(g['initial_state_sha256'])
            actual = z['actual'][:,0]; requested = z['requested'][:,0]; goal = z['goal'][:,0]
            reconstructed = np.asarray([height(root,q) for root,q in zip(z['root'][:,0],actual)])
            fk_error = float(abs(reconstructed-z['height'][:,0,0]).max())
            assert fk_error < 2e-7
            e = dict(arm=arm, seed=seed, source=str(path.relative_to(BASE)), source_sha256=sha(path),
                trace_sha256=sha(trace), initial_state_sha256=g['initial_state_sha256'],
                best=s['best'], actual_foot_reconstruction_max_error_m=fk_error,
                requested_to_goal_max_deg=float(np.rad2deg(abs(requested-goal).max())),
                goal_to_actual_max_deg=float(np.rad2deg(abs(goal-actual).max())))
            ik = [x for x in g['teacher_ik'] if x.get('success') and 'desired_minimum_sole_height_m' in x]
            if ik:
                peak = max(ik,key=lambda x:x['desired_minimum_sole_height_m'])
                index = round(peak['time_s']/.02)-1
                q = actual[index].astype(float).copy(); q[:5] = peak['configuration_rad']
                e['maximum_clearance_request_sample'] = dict(time_s=peak['time_s'],
                    desired_height_m=peak['desired_minimum_sole_height_m'],
                    raw_ik_at_recorded_post_step_root_m=height(z['root'][index,0],q),
                    requested_at_same_root_m=height(z['root'][index,0],requested[index]),
                    goal_at_same_root_m=height(z['root'][index,0],goal[index]),
                    actual_height_m=float(z['height'][index,0,0]),
                    left_load_n=float(z['load'][index,0,0]),right_load_n=float(z['load'][index,0,1]),
                    body_tilt_deg=float(z['tilt'][index,0]),
                    joint_goal_error_deg=np.rad2deg(goal[index]-actual[index]).tolist())
            report['runs'].append(e)
    report['paired_initial_states_equal'] = all(len(set(v))==1 for v in hashes.values())
    assert report['paired_initial_states_equal']
    report['summary'] = {arm:dict(qualified=sum(r['best']['single_cycle_qualified'] for r in report['runs'] if r['arm']==arm),
        failed=sum(r['best']['failed'] for r in report['runs'] if r['arm']==arm),
        maximum_actual_clearance_m=max(r['best']['maximum_swing_clearance_m'] for r in report['runs'] if r['arm']==arm))
        for arm in ('nonlinear-swing-precise-v3','nonlinear-swing-clearance-v4')}
    (BASE/'swing-teacher-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['summary']))
    for r in report['runs']:
        if r['arm'].endswith('v4'):
            print(r['seed'],json.dumps(r['maximum_clearance_request_sample']))


if __name__ == '__main__':
    main()
