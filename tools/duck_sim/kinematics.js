import * as THREE from './vendor/three.module.js';

export const quaternion = values => new THREE.Quaternion(values[1], values[2], values[3], values[0]).normalize();

export function createRig(tree, createMesh) {
  const joints=[], meshes=[], bodies={};
  function build(body) {
    const fixed=new THREE.Group();fixed.name=body.name;
    fixed.position.fromArray(body.pos);fixed.quaternion.copy(quaternion(body.quat));
    const moving=new THREE.Group();fixed.add(moving);bodies[body.name]=moving;
    if(body.joint){const j=body.joint;joints[j.index]={moving,axis:new THREE.Vector3(...j.axis).normalize(),pos:new THREE.Vector3(...j.pos),info:j};}
    for(const spec of body.meshes){
      const mesh=createMesh(spec,body.joint?.index);
      mesh.position.fromArray(spec.pos);mesh.quaternion.copy(quaternion(spec.quat));moving.add(mesh);meshes.push(mesh);
    }
    body.children.forEach(child=>moving.add(build(child)));return fixed;
  }
  return {root:build(tree),joints,meshes,bodies};
}

export function applyPose(model, frame, target=false) {
  model.root.position.fromArray(frame.base);model.root.quaternion.copy(quaternion(frame.quat));
  const angles=target?frame.target:frame.q;
  if(!angles)return;
  model.joints.forEach((j,i)=>{
    // Missing jaw data is not a measured closed mouth. Retain its last pose;
    // the UI shows unknown rather than inventing a sample or corrupting matrices.
    if(!Number.isFinite(angles[i]))return;
    j.moving.quaternion.setFromAxisAngle(j.axis,angles[i]*Math.PI/180);
    // MJCF hinge transform: T(anchor) R(q) T(-anchor).
    j.moving.position.copy(j.pos).sub(j.pos.clone().applyQuaternion(j.moving.quaternion));
  });
}
