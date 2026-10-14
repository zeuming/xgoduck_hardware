import assert from 'node:assert/strict';
import test from 'node:test';
import {MOTOR_IDS,JOINT_IDS,draftId,feedbackFrame,movementSummary,captureIntent} from './calibration_core.js';
function snapshot(){return {native_access:{full_workbench:true},session:{status:'active',generation:7,collection_scope:'joints',received_age_ms:20},
 calibration:{joints:MOTOR_IDS.map(id=>({id,zero:2048,sign:-1,min:1500,max:2800}))},
 feedback:{positions:MOTOR_IDS.map(()=>2048),joint_feedback_valid:true,sample_us:123,online_mask:32767,torque_mask:0,fault_mask:0,armed:false},
 joint_wizard:{id:null,stage:'idle',busy:false},joint_zero:{id:null,stage:'idle',busy:false}};}
const intent=(s,action,options={})=>captureIntent({snapshot:s,id:14,action,supported:true,confirmed:true,...options});
test('Encoder to rig mapping keeps policy order; copies rather than mutates raw positions',()=>{
 const s=snapshot();s.feedback.positions[4]=1900;s.feedback.positions[9]=2200;
 const f=feedbackFrame(s);assert.equal(f.q[4],148*360/4095);assert.equal(f.q[13],-152*360/4095);
 f.raw[4]=1;assert.equal(s.feedback.positions[4],1900);assert.deepEqual(JOINT_IDS,[10,11,12,13,14,30,31,32,33,20,21,22,23,24,34]);
});
test('Every bus ID changes only its named model joint, including jaw',async()=>{
 const fs=await import('node:fs/promises');
 const meta=JSON.parse(await fs.readFile(new URL('../../output/duck_sim/visual-calibration/model.json',import.meta.url),'utf8'));
 const expected={10:'left_hip_yaw',11:'left_hip_roll',12:'left_hip_pitch',13:'left_knee',14:'left_ankle',
  20:'right_hip_yaw',21:'right_hip_roll',22:'right_hip_pitch',23:'right_knee',24:'right_ankle',
  30:'neck_pitch',31:'head_pitch',32:'head_yaw',33:'head_roll',34:'jaw'};
 function visit(body){if(body.joint){const j=body.joint;assert.equal(expected[j.id],j.name);assert.equal(meta.motor_ids[j.index],j.id);assert.equal(meta.joint_names[j.index],j.name);}body.children.forEach(visit);}
 visit(meta.tree);
 for(const id of MOTOR_IDS){const s=snapshot();s.feedback.positions[MOTOR_IDS.indexOf(id)]+=100;const f=feedbackFrame(s);
  const changed=f.q.flatMap((q,i)=>q?[meta.joint_names[i]]:[]);
  assert.deepEqual(changed,[expected[id]],`Raw ID${id} must not animate a neighbouring joint`);
 }
});
test('Stale, unknown, missing, armed, fault and IMU-only feedback never grants capture',()=>{
 for(const change of [s=>s.session.received_age_ms=500,s=>s.session.received_age_ms=null,s=>s.feedback=null,
  s=>s.feedback.armed=true,s=>s.feedback.torque_mask=16,s=>s.feedback.fault_mask=16,s=>s.feedback.online_mask=32766,
  s=>s.session.collection_scope='imu',s=>s.feedback.sample_us=NaN,s=>s.feedback.positions[0]=NaN]){
  const s=snapshot();change(s);assert.equal(feedbackFrame(s),null);assert.throws(()=>intent(s,'direction-start'));
 }
 assert.equal(feedbackFrame(snapshot(),481),null);assert.ok(feedbackFrame(snapshot(),100));
});
test('Sampling requires explicit support and exact current step, preserving generation and confirmations',()=>{
 const s=snapshot();assert.throws(()=>intent(s,'direction-start',{supported:false}));assert.throws(()=>intent(s,'save'));
 const start=intent(s,'direction-start');assert.equal(start.path,'/api/joint-calibration');assert.equal(start.body.generation,7);
 s.joint_wizard={id:14,stage:'await-positive',busy:false};assert.throws(()=>intent(s,'direction-end',{confirmed:false}));assert.equal(intent(s,'direction-end').body.id,14);
});
test('Draft pins the selected ID, and zero capture stays separate from direction/range',()=>{
 const s=snapshot();s.joint_wizard={id:13,stage:'await-positive'};assert.equal(draftId(s),13);assert.throws(()=>intent(s,'direction-end'));
 s.joint_wizard={id:null,stage:'idle'};assert.throws(()=>intent(s,'save',{kind:'zero'}));
 assert.equal(intent(s,'capture',{kind:'zero'}).path,'/api/joint-zero');
 s.joint_zero={id:14,stage:'ready',candidate_zero:2040};assert.equal(draftId(s),14);assert.equal(intent(s,'save',{kind:'zero'}).body.zero_reference_confirmed,true);
});
test('Direction-only save must be explicitly permitted by the existing service',()=>{
 const s=snapshot();s.joint_wizard={id:14,stage:'await-range-a',direction_reuse_allowed:false};assert.throws(()=>intent(s,'save-direction'));
 s.joint_wizard.direction_reuse_allowed=true;assert.equal(intent(s,'save-direction').body.action,'save-direction');
});
test('Cancel works during sampling without fabricated torque proof or support confirmation',()=>{
 const s=snapshot();s.feedback=null;s.joint_wizard={id:14,stage:'direction-start',busy:true};
 assert.equal(intent(s,'cancel',{supported:false,confirmed:false}).body.action,'cancel');
});
test('All 15 joints are ranked; moving a knee is not reported as ankle motion',()=>{
 const a=MOTOR_IDS.map(()=>2048),b=[...a];b[3]+=479;b[4]+=1;
 const result=movementSummary([{raw:a},{raw:b}]);assert.equal(result[0].id,13);
 assert.equal(result.find(r=>r.id===14).span_ticks,1);assert.equal(result.length,15);
});
test('DOM source has no actuation route and no disabled calibration button',async()=>{
 const fs=await import('node:fs/promises');const code=await fs.readFile(new URL('calibration.js',import.meta.url),'utf8');
 assert.ok(!code.includes('/api/motion'));assert.ok(!code.includes('disabled='));
});
