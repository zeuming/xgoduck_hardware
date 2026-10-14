"""Check physical execution, trace provenance and Three/MuJoCo kinematics."""
import json
import math
from pathlib import Path
import subprocess
import unittest

import numpy as np
from server import App, HERE, RECORD, ROBOT, POLICY_INDEX, gravity_quaternion


class SimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = App()

    def test_assets_and_joint_contract(self):
        self.assertEqual(len(self.app.meta['assets']), 16)
        self.assertEqual(len(self.app.meta['motor_ids']), 15)
        for asset in self.app.meta['assets'].values():
            self.assertTrue(((HERE / 'assets' / asset['file'].removeprefix('viewer/')) if asset['file'].startswith('viewer/') else (ROBOT / 'assets' / asset['file'])).is_file())

    def test_standing_and_forward_are_solved_not_animated(self):
        standing = self.app.sim.run(0, 6)
        forward = self.app.sim.run(.2, 6)
        self.assertEqual(len(forward['frames']), 301)
        self.assertLess(abs(standing['frames'][-1]['base'][0]), .01)
        self.assertGreater(forward['frames'][-1]['base'][0], .3)
        self.assertLess(max(f['tilt'] for f in forward['frames']), 15)
        for side in ('left', 'right'):
            contact = [f['contacts'][side] for f in forward['frames'][50:]]
            self.assertIn(True, contact)
            self.assertIn(False, contact)
        self.assertTrue(any(max(abs(a-b) for a,b in zip(f['q'],f['target'])) > 1
                            for f in forward['frames']))
        # Repeating a request resets all model/controller state.
        repeated = self.app.sim.run(.2, 6)
        np.testing.assert_allclose(forward['frames'][-1]['q'], repeated['frames'][-1]['q'], atol=1e-9)

    def test_three_body_transforms_match_mujoco(self):
        mj = self.app.sim.mj
        model = mj.MjModel.from_xml_path(str(ROBOT / 'robot_walk.xml'))
        data = mj.MjData(model)
        qi = [model.joint(name).qposadr[0] for name in self.app.meta['joint_names'][:14]]
        cases = []
        rng = np.random.default_rng(3410)
        for _ in range(20):
            root = rng.normal(size=4); root /= np.linalg.norm(root)
            angles = self.app.sim.profile['home_rad'] + rng.uniform(-.15, .15, 14)
            base = rng.uniform(-.2, .2, 3)
            data.qpos[:7] = np.r_[base, root]; data.qpos[qi] = angles
            mj.mj_forward(model, data)
            expected = {model.body(i).name: dict(pos=data.xpos[i].tolist(), quat=data.xquat[i].tolist())
                        for i in range(1, model.nbody)}
            anchors = {name:data.xanchor[model.joint(name).id].tolist() for name in self.app.meta['joint_names'][:14]}
            cases.append(dict(frame=dict(q=np.rad2deg(angles).tolist(), base=base.tolist(), quat=root.tolist()), expected=expected, anchors=anchors))
        script = """
import fs from 'node:fs';
import * as THREE from './vendor/three.module.js';
import {createRig, applyPose, quaternion} from './kinematics.js';
const {tree,cases}=JSON.parse(fs.readFileSync(0,'utf8'));
const rig=createRig(tree,()=>new THREE.Group());
let maxPosition=0,maxRotation=0,maxAnchor=0;
for(const c of cases){applyPose(rig,c.frame);rig.root.updateMatrixWorld(true);
  for(const joint of rig.joints.slice(0,14)){
    const point=joint.moving.localToWorld(joint.pos.clone());
    maxAnchor=Math.max(maxAnchor,point.distanceTo(new THREE.Vector3(...c.anchors[joint.info.name])));
  }
  for(const [name,expected] of Object.entries(c.expected)){
    const body=rig.bodies[name];
    maxPosition=Math.max(maxPosition,body.getWorldPosition(new THREE.Vector3()).distanceTo(new THREE.Vector3(...expected.pos)));
    maxRotation=Math.max(maxRotation,1-Math.abs(body.getWorldQuaternion(new THREE.Quaternion()).dot(quaternion(expected.quat))));
  }
}
console.log(JSON.stringify({maxPosition,maxRotation,maxAnchor}));
"""
        result = subprocess.run(['node', '--input-type=module', '-e', script], cwd=HERE,
                                input=json.dumps(dict(tree=self.app.meta['tree'],cases=cases)),
                                capture_output=True, text=True, check=True)
        errors = json.loads(result.stdout)
        self.assertLess(errors['maxPosition'], 1e-6)
        self.assertLess(errors['maxRotation'], 1e-8)
        self.assertLess(errors['maxAnchor'], 1e-6)
        print('Three/MuJoCo transform error:', errors)

    def test_record_angles_and_end_reason_are_preserved(self):
        record = self.app.replays[0]
        source = next(e['value'] for e in json.loads(RECORD.read_text())['entries'] if e['name'] == record['id'])
        np.testing.assert_allclose(record['frames'][0]['q'][:14], np.asarray(source['frames'][0]['actual_angle_deg'])[POLICY_INDEX], atol=1e-5)
        self.assertIsNone(record['frames'][0]['target'])
        self.assertEqual(record['reason'], source['reason'])
        self.assertEqual(len(record['frames']), len(source['frames']) + 1)
        self.assertTrue(any(m['label'] == '前进指令' for m in record['markers']))
        # A record does not fabricate global motion from command velocity.
        self.assertTrue(all(f['base'] == [0,0,.139439] for f in record['frames']))

    def test_saved_right_hip_inward_sample_moves_toward_body_center(self):
        # Operator-confirmed inward movement, 2026-10-10. Revision 24 stores
        # sign=-1, so the viewer must use these angles directly, without a flip.
        mj = self.app.sim.mj
        model = mj.MjModel.from_xml_path(str(ROBOT / 'robot_walk.xml'))
        frames = []
        index = self.app.meta['motor_ids'].index(21)
        for tick in (2541, 2472):
            q = list(self.app.meta['home'])
            q[index] = (tick - 2209) * 360 / 4095 * -1
            frames.append(dict(q=q, base=[0, 0, .22], quat=[1, 0, 0, 0]))
        self.assertGreater(frames[1]['q'][index], frames[0]['q'][index])
        script = """
import fs from 'node:fs';
import assert from 'node:assert/strict';
import * as THREE from './vendor/three.module.js';
import {createRig, applyPose} from './kinematics.js';
const {tree,frames,feet}=JSON.parse(fs.readFileSync(0,'utf8'));
const rig=createRig(tree,()=>new THREE.Group());
const original=JSON.stringify(frames);
function positions(frame){applyPose(rig,frame);rig.root.updateMatrixWorld(true);return {
  left:new THREE.Vector3(...feet.left).applyMatrix4(rig.bodies.ankle_left.matrixWorld).toArray(),
  right:new THREE.Vector3(...feet.right).applyMatrix4(rig.bodies.ankle_right.matrixWorld).toArray()};}
const before=positions(frames[0]),after=positions(frames[1]);
assert(after.right[1]>before.right[1]); // Right leg inward is body +Y.
assert.deepEqual(before.left,after.left);
assert.equal(JSON.stringify(frames),original);
console.log(JSON.stringify({beforeY:before.right[1],afterY:after.right[1]}));
"""
        result = subprocess.run(['node', '--input-type=module', '-e', script], cwd=HERE,
                                input=json.dumps(dict(tree=self.app.meta['tree'],frames=frames,
                                    feet={side:model.site(side+'_foot').pos.tolist()
                                          for side in ('left','right')})),
                                capture_output=True, text=True, check=True)
        print('Saved right hip inward sample (m):', json.loads(result.stdout))

    def test_gravity_orientation(self):
        mj=self.app.sim.mj
        for g in ([0,0,-1],[.3,-.2,-.9327379],[0,0,1]):
            q=gravity_quaternion(g); out=np.zeros(3)
            mj.mju_rotVecQuat(out,np.asarray(g)/np.linalg.norm(g),q)
            np.testing.assert_allclose(out,[0,0,-1],atol=1e-6)

    def test_invalid_and_overlapping_runs(self):
        for speed,duration in [(math.nan,6),(1,6),(.2,math.inf),(.2,100)]:
            with self.assertRaises(ValueError):self.app.sim.run(speed,duration)
        with self.app.sim.lock:
            with self.assertRaises(ValueError):self.app.sim.run(.2,6)


if __name__ == '__main__':
    unittest.main()
