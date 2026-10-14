"""Viewer-only jaw geometry and independence from the 14-DOF control model."""
import json
from pathlib import Path
import subprocess
import unittest
import numpy as np
from server import model_tree, ROBOT, HERE
from visual_jaw import with_visual_jaw


def triangles(path):
    raw=path.read_bytes();n=int.from_bytes(raw[80:84],'little')
    return np.frombuffer(raw,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)['v'].copy()


class JawTests(unittest.TestCase):
    def test_split_preserves_every_original_triangle_at_zero(self):
        source=triangles(ROBOT/'assets/head_roll.stl')
        fixed=triangles(HERE/'assets/head_fixed.stl');jaw=triangles(HERE/'assets/jaw_moving.stl')
        self.assertEqual(len(jaw),1272);self.assertEqual(len(fixed)+len(jaw),len(source))
        def canonical(v):return sorted(row.tobytes() for row in v)
        self.assertEqual(canonical(source),canonical(np.concatenate([fixed,jaw])))

    def test_jaw_moves_only_its_geometry_and_opens_downward(self):
        original,assets=model_tree();original_json=json.dumps(original)
        tree,_=with_visual_jaw(original,assets);self.assertEqual(json.dumps(original),original_json)
        script="""
import fs from 'node:fs';import assert from 'node:assert/strict';
import * as THREE from './vendor/three.module.js';import {createRig,applyPose} from './kinematics.js';
const {tree}=JSON.parse(fs.readFileSync(0,'utf8'));const rig=createRig(tree,()=>new THREE.Group());
const frame={q:Array(15).fill(0),base:[0,0,.22],quat:[1,0,0,0]};
function state(){rig.root.updateMatrixWorld(true);return Object.fromEntries(Object.entries(rig.bodies).map(([name,obj])=>[name,obj.matrixWorld.toArray()]));}
applyPose(rig,frame);const before=state();frame.q[14]=30;applyPose(rig,frame);const after=state();
for(const name of Object.keys(before)){if(name==='viewer_jaw')assert.notDeepEqual(before[name],after[name]);else assert.deepEqual(before[name],after[name]);}
// In the head frame +Y is up and -Z faces forward. Positive ID34 lowers the lower beak.
const jaw=rig.joints[14],tip=new THREE.Vector3(0,-.014,-.070),opened=tip.clone().applyMatrix4(jaw.moving.matrix);
assert(opened.y<tip.y);assert.equal(jaw.info.id,34);
const matrix=jaw.moving.matrix.toArray();frame.q[14]=null;applyPose(rig,frame);state();assert.deepEqual(jaw.moving.matrix.toArray(),matrix);
// Legacy 14-D physics output has no jaw angle and must not introduce NaNs.
frame.q=Array(14).fill(0);applyPose(rig,frame);state();assert(jaw.moving.matrix.elements.every(Number.isFinite));
console.log('ID34 opens downward; every parent/leg matrix unchanged; missing jaw remains finite');
"""
        result=subprocess.run(['node','--input-type=module','-e',script],cwd=HERE,input=json.dumps(dict(tree=tree)),text=True,capture_output=True,check=True)
        print(result.stdout.strip())
