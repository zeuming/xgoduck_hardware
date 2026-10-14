"""Offline feasibility of a whole-foot, fore-aft COM preserving pose map.

No hardware code or controller parameters are changed. Both mass distributions
are explicit, unmeasured hypotheses. A mapping is usable only if numerical
foot/COM and inverse consistency checks pass; a low least-squares cost alone
does not qualify it. Root translation is a geometric unknown, never written
into a running plant. The original ONNX is not modified or trained here.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from audit_pose import gravity_rotation, set_pose
from offline_calibration_overlay import range_snapshot
from offline_mass_distribution import scenario_spec

ROOT = Path(__file__).resolve().parents[2]
LEG_IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PoseMap:
    def __init__(self, candidate, source_scenario, destination_scenario):
        self.candidate = candidate
        xml = ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml'
        source = lambda: mj.MjSpec.from_file(str(xml))
        left, self.source_evidence = scenario_spec(source, source_scenario)
        right, self.destination_evidence = scenario_spec(source, destination_scenario)
        self.source, self.destination = left().compile(), right().compile()
        self.before, self.after = mj.MjData(self.source), mj.MjData(self.destination)
        self.names = candidate['joint_names']
        self.indices = [candidate['policy_joint_ids'].index(sid) for sid in LEG_IDS]
        self.joints = [self.destination.joint(self.names[i]).id for i in self.indices]
        self.qadr = self.destination.jnt_qposadr[self.joints]
        self.root = self.destination.body('trunk_base').id
        self.root_adr = self.destination.jnt_qposadr[self.destination.body_jntadr[self.root]]
        self.feet = [self.destination.body('ankle_'+s).id for s in ('left', 'right')]
        self.low = np.maximum(self.destination.jnt_range[self.joints, 0], np.array(candidate['lower_rad'])[self.indices])
        self.high = np.minimum(self.destination.jnt_range[self.joints, 1], np.array(candidate['upper_rad'])[self.indices])
        self.scale = float(self.destination.stat.extent)
        self.history = []

    def solve(self, q, rotation, initial_delta=None, *, virtual_destination=False):
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (14,) or not np.isfinite(q).all():
            raise ValueError('Expected14 finite sensor/target angles')
        set_pose(self.source, self.before, dict(zip(self.names, q)), rotation)
        set_pose(self.destination, self.after, dict(zip(self.names, q)), rotation)
        baseline = self.after.qpos.copy()
        feet_position = self.before.xpos[self.feet].copy()
        feet_rotation = self.before.xmat[self.feet].reshape(2, 3, 3).copy()
        com_x = float(self.before.subtree_com[self.root, 0])
        low = self.destination.jnt_range[self.joints, 0] if virtual_destination else self.low
        high = self.destination.jnt_range[self.joints, 1] if virtual_destination else self.high
        initial = np.r_[np.zeros(3), q[self.indices]]
        if initial_delta is not None:
            initial[3:] += np.asarray(initial_delta)[self.indices]
        # The optimizer starts inside its numerical domain. This is not output
        # clipping: any final failed range/geometric check rejects the map.
        initial[3:] = np.minimum(np.maximum(initial[3:], low+1e-12), high-1e-12)

        def residual(x):
            self.after.qpos[:] = baseline
            self.after.qpos[self.root_adr:self.root_adr+3] = baseline[self.root_adr:self.root_adr+3]+x[:3]
            self.after.qpos[self.qadr] = x[3:]
            mj.mj_forward(self.destination, self.after)
            position = (self.after.xpos[self.feet]-feet_position).ravel()
            angular = np.array([Rotation.from_matrix(old.T @ now).as_rotvec() for old, now in
                zip(feet_rotation, self.after.xmat[self.feet].reshape(2, 3, 3))]).ravel()
            return np.r_[position, self.scale*angular, self.after.subtree_com[self.root, 0]-com_x]

        began = time.perf_counter()
        result = least_squares(residual, initial, bounds=(np.r_[np.full(3, -self.scale), low],
            np.r_[np.full(3, self.scale), high]), xtol=1e-11, ftol=1e-11, gtol=1e-11,
            max_nfev=80, diff_step=1e-5)
        errors = residual(result.x)
        mapped = q.copy(); mapped[self.indices] = result.x[3:]
        angular_error = float(np.max(np.linalg.norm(errors[6:12].reshape(2, 3)/self.scale, axis=1)))
        position_error = float(np.max(np.abs(errors[:6])))
        com_error = float(abs(errors[-1]))
        singular = np.linalg.svd(result.jac, compute_uv=False)
        record = dict(solver_success=bool(result.success), evaluations=result.nfev,
            foot_position_error_m=position_error, foot_orientation_error_rad=angular_error,
            com_projection_error_m=com_error, root_translation_m=result.x[:3].tolist(),
            jacobian_singular_values=singular.tolist(), wall_ms=1000*(time.perf_counter()-began),
            angles_rad=mapped.tolist(),
            geometry_verified=bool(result.success and position_error < 1e-6 and angular_error < 1e-4 and com_error < 1e-7))
        self.history.append(record)
        return mapped, record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-scenario',type=Path,required=True)
    p.add_argument('--destination-scenario',type=Path,required=True)
    p.add_argument('--trace',type=Path,required=True)
    p.add_argument('--joint-field',choices=('processed_joint_rad','actual_joint_rad'),default='processed_joint_rad')
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    cp=ROOT/'output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json'
    candidate,overlay=range_snapshot(json.loads(cp.read_text())['candidate'],a.calibration)
    forward=PoseMap(candidate,a.source_scenario,a.destination_scenario)
    inverse=PoseMap(candidate,a.destination_scenario,a.source_scenario)
    rows=[json.loads(x) for x in a.trace.read_text().splitlines()]
    selected=np.unique(np.linspace(0,len(rows)-1,32).astype(int)); records=[]; began=time.perf_counter()
    for i in selected:
        if time.perf_counter()-began > 60:raise TimeoutError('File-only geometry budget')
        row=rows[i];q=np.asarray(row[a.joint_field])
        rotation=gravity_rotation(row['actor_observation'][3:6])
        mapped,f=forward.solve(q,rotation)
        restored,b=inverse.solve(mapped,rotation,virtual_destination=True)
        error=float(np.max(abs(restored-q)))
        records.append(dict(frame=int(i),time_s=row['time_s'],forward=f,inverse=b,
            roundtrip_joint_error_rad=error,roundtrip_verified=bool(error<1e-5)))
    report=dict(hardware_access=False,hardware_eligible=False,production_changed=False,
        source_sha256=sha(__file__),trace_sha256=sha(a.trace),calibration_overlay=overlay,
        source_scenario=forward.source_evidence,destination_scenario=forward.destination_evidence,
        scope='Full sole positions/orientations and sagittal COM; unknown root translation only in geometry, no running plant injection. Lateral COM and dynamics are not certified.',
        numerical_checks='Same foot1um,orientation0.0001rad,COM0.1um as previous geometry; roundtrip1e-5rad numerical consistency, no hardware limit change',
        joint_field=a.joint_field,samples=len(records),all_geometry_verified=all(x['forward']['geometry_verified'] and x['inverse']['geometry_verified'] for x in records),
        all_roundtrip_verified=all(x['roundtrip_verified'] for x in records),records=records,
        wall_seconds=time.perf_counter()-began)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:report[k] for k in ('samples','all_geometry_verified','all_roundtrip_verified','wall_seconds')}))


if __name__=='__main__':
    main()
