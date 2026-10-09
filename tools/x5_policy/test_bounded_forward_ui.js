'use strict';
// Actual full native HTML, fake DOM/fetch/timers; no hardware or network.
const fs = require('node:fs');
const path = require('node:path');
const sourcePath = path.join(__dirname, '../x5_recovery/test_feedback_calibration_ui.js');
let harnessSource = fs.readFileSync(sourcePath, 'utf8').split('// Real pure JointWizard snapshots')[0];
harnessSource = harnessSource.replace(
  "const html = fs.readFileSync(path.join(__dirname, 'feedback_calibration.html'), 'utf8');",
  "const html = fs.readFileSync(process.argv[2], 'utf8');");
harnessSource = harnessSource.replace('location: {protocol, hash},', "location: {protocol, hash, host:'192.168.1.11:8767'},");
harnessSource = harnessSource.replace('document: {getElementById', 'document: {querySelector() { return null; }, getElementById');
const checks = String.raw`
(async()=>{
  const h=harness('http:','#motion-models'),s=fixture('closed',19);
  s.native_access={full_workbench:true,authentication_required:false,page_generation:900};
  s.motion={phase:'ready',owns_connection:true,feedback_age_ms:0,connection_sequence:2,models_loaded:['walk'],
    hardware:{connected:true,generation:7,mode:'walk',age_ms:0,imu_age_ms:0,imu:{gravity:[0,0,-1]},release_confirmed:false,
      calibrated:true,armed:true,torque_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],online_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],fault_ids:[],firmware_error:0,
      forward_trial:{protocol:1,mode_generation:7,phase:'idle',used:false,allowed:true}}};
  h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.ownMotion();
  const b=h.nodes.get('walkModelForwardLocked');assert.equal(b.disabled,false);assert(h.nodes.get('walkModelForwardResult').textContent.includes('尚未触发'));
  for(const change of [{forward_trial:null},{forward_trial:{...s.motion.hardware.forward_trial,used:true}},{forward_trial:{...s.motion.hardware.forward_trial,allowed:false}},{armed:false},{imu_age_ms:1600},{forward_trial:{...s.motion.hardware.forward_trial,mode_generation:8}}]){
    const bad=clone(s);Object.assign(bad.motion.hardware,change);h.set(bad);h.eval('renderMotion()');assert.equal(b.disabled,true,JSON.stringify(change));
  }
  h.set(s);h.ownMotion();h.nodes.get('motionSupportConfirmed').checked=false;h.eval('renderMotion()');assert.equal(b.disabled,true);
  h.nodes.get('motionSupportConfirmed').checked=true;h.age(1500);assert.equal(b.disabled,true);
  h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.ownMotion();assert.equal(b.disabled,false,h.eval('nativeForwardBlockedReason()'));
  const pending=deferred();h.post((url,options)=>{const body=JSON.parse(options.body);
    if(body.action==='heartbeat')return response({heartbeat_accepted:true,generation:7,page_generation:900});
    assert.equal(url,'/api/motion');assert.equal(body.action,'forward-trial');assert.equal(body.support_confirmed,true);assert.equal(body.generation,7);
    assert.equal(body.forward,undefined);assert.equal(body.duration,undefined);return pending.promise;});
  const received=h.eval('receivedAt'),clicked=b.onclick();await tick();
  assert.equal(h.eval('motionBusy'),false,'forward receipt must not pause heartbeat');
  await b.onclick();assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='forward-trial').length,1,'pending cannot repeat');
  pending.resolve(response({action_receipt_protocol:1,action:'forward-trial',mode:'walk',generation:7,connected:true,release_confirmed:false,page_generation:900,connection_sequence:2,operation_epoch:2,control_owner:{id:h.eval('motionControlId'),generation:7}}));
  await clicked;assert.equal(h.eval('ownsMotionControl()'),true,'receipt preserves same-generation owner');
  assert.equal(h.eval('receivedAt'),received,'receipt cannot refresh sensor age');assert(h.nodes.get('walkModelForwardResult').textContent.includes('请求已接收'));
  const used=clone(s);used.motion.hardware.forward_trial={...s.motion.hardware.forward_trial,used:true,phase:'returned-to-zero',allowed:false,execution:{successful_nonzero_TARGETs:35,successful_zero_TARGETs_after_forward:3}};
  const before=h.posts().length;for(let i=0;i<5;i++)h.set(used);await b.onclick();assert.equal(h.posts().length,before,'polling and used pulse never trigger again');
  assert.equal(h.nodes.get('motionStop').disabled,false,'STOP remains available');assert(h.nodes.get('walkModelForwardResult').textContent.includes('35 帧'));assert(h.nodes.get('walkModelForwardResult').textContent.includes('恢复零速指令'));
  const stopped=clone(used);stopped.motion.hardware.mode='off';stopped.motion.hardware.forward_trial.phase='cancelled';h.set(stopped);assert(h.nodes.get('walkModelForwardResult').textContent.includes('当前已停止'));assert(!h.nodes.get('walkModelForwardReason').textContent.includes('当前保持零速'));
  console.log(JSON.stringify({forward_ui:true,one_request:true,no_auto_start:true,owner_preserved:true,heartbeat_not_paused:true}));
})().catch(error=>{console.error(error);process.exitCode=1;});
`;
new Function('require','process','__dirname',harnessSource+'\n'+checks)(require,process,path.dirname(sourcePath));
