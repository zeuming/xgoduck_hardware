'use strict';
const fs=require('node:fs'),path=require('node:path');
const harnessPath=path.join(__dirname,'../x5_recovery/test_feedback_calibration_ui.js');
let source=fs.readFileSync(harnessPath,'utf8').split('// Real pure JointWizard snapshots')[0];
source=source.replace("const html = fs.readFileSync(path.join(__dirname, 'feedback_calibration.html'), 'utf8');","const html = fs.readFileSync(process.argv[2], 'utf8');");
source=source.replace("const script = html.match(/<script>([\\s\\S]*?)<\\/script>/)[1];","const script = [...html.matchAll(/<script>([\\s\\S]*?)<\\/script>/g)].map(m=>m[1]).find(s=>s.includes('function render()'));");
const tests=String.raw`
(async()=>{
 const h=harness('http:','#motion-models');
 for(const id of ['motionHeadClearance','motionHeadCapture','motionHeadClearanceConfirmed','motionStancePreparation','motionSupportConfirmed'])assert(!h.nodes.has(id),id+' removed');
 const s=fixture('closed',19);s.feedback=null;
 s.native_access={full_workbench:true,authentication_required:false,page_generation:900};
 s.motion={phase:'ready',owns_connection:true,connection_sequence:2,models_loaded:['walk'],hardware:{
 connected:true,generation:7,mode:'off',age_ms:0,imu_age_ms:0,imu_ok:true,imu:{gravity:[0,0,-1]},release_confirmed:true,
 calibrated:true,online_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],fault_ids:[],torque_ids:[],armed:false,pending_stop:false,firmware_error:0,
 target_validation_protocol:1,policy_metadata_protocol:1,stand_target_deg:Array(15).fill(0),head_clearance_required:false,hold_head_deg:null}};
 h.set(s);h.eval('renderMotion()');
 assert.equal(h.eval('nativeHoldRequestReady()'),true,'no capture needed');
 for(let i=0;i<100;i++)h.eval('render()');
 assert.equal(h.posts().length,0,'render cannot capture or start');
 const hold=clone(s);Object.assign(hold.motion.hardware,{mode:'hold',hold_ready:true,armed:true,torque_ids:hold.motion.hardware.online_ids});
 h.set(hold);assert.equal(h.eval('nativeZeroBalanceRequestReady()'),true,'zero balance does not require capture');
 for(const change of [x=>x.motion.hardware.fault_ids=[14],x=>x.motion.hardware.hold_ready=false,x=>x.motion.hardware.imu.gravity=[1,0,0],x=>x.motion.hardware.start_blocked_reason='ID14 范围不通过']){
  const blocked=clone(hold);change(blocked);h.set(blocked);assert.equal(h.eval('nativeZeroBalanceRequestReady()'),false);
 }
 h.set(s);h.age(1600);assert.equal(h.eval('nativeHoldRequestReady()'),false,'stale state blocked');
 h.set(s);await h.nodes.get('motionStand').onclick();
 assert.equal(h.posts().length,1,'explicit click sends one start without checkbox');
 assert.equal(JSON.parse(h.posts()[0].options.body).support_confirmed,undefined,'do not invent a confirmation');
 console.log('PASS full native UI: removed capture/support panel, explicit start without invented confirmation, HOLD/walk readiness and retained guards');
})().catch(e=>{console.error(e);process.exitCode=1});
`;
new Function('require','process','__dirname',source+tests)(require,process,path.dirname(harnessPath));
