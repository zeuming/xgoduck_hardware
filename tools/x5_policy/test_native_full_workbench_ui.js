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
  const h=harness('http:','#imu-steps');
  const native=fixture('closed',19);
  native.native_access={full_workbench:true,authentication_required:false,page_generation:900};
  native.motion={phase:'closed',owns_connection:false,hardware:null,connection_sequence:1,models_loaded:['walk','getup','pick','roulade']};
  native.connection_target={kind:'wifi',host:'192.168.1.11',native:true};
  native.session.recovery_blocked_reason=null;
  h.set(native);
  assert.equal(h.nodes.get('recoverFeedback').disabled,false,'full native must allow explicit IMU restart');
  assert(h.nodes.get('x5ConnectionTarget').textContent.includes('采集、维护和运控均在 X5'));
  h.post(async (url,options)=>{
    assert.equal(url,'/api/recover-feedback');
    assert.equal(JSON.parse(options.body).purpose,'imu');
    return response(native);
  });
  await h.eval('recoverFeedback()');
  assert.equal(h.posts().length,1);
  const active=clone(native);active.session.status='active';active.session.error=null;
  active.feedback.scope='imu';active.feedback.joint_feedback_valid=false;active.feedback.fault_mask=32767;
  h.set(active);h.nodes.get('mountedConfirmed').checked=true;h.nodes.get('supportConfirmed').checked=true;h.eval('render()');
  assert.equal(h.nodes.get('wizardLevel').disabled,false,'valid offline IMU scope can sample');
  assert.equal(h.eval('canJointAction("direction-start")'),false,'IMU scope cannot calibrate joints');
  assert.equal(h.eval('canZeroAction("capture")'),false,'IMU scope cannot save zeros');
  h.age(501);
  assert.equal(h.nodes.get('wizardLevel').disabled,true,'stale IMU must block sampling');
  const unscoped=clone(active);unscoped.feedback.scope='joints';unscoped.feedback.joint_feedback_valid=true;
  h.set(unscoped);h.nodes.get('mountedConfirmed').checked=true;h.nodes.get('supportConfirmed').checked=true;h.eval('render()');
  assert.equal(h.nodes.get('wizardLevel').disabled,true,'offline fault exemption must remain IMU-only');
  const motion=clone(native);motion.session.generation=20;
  motion.motion={phase:'ready',owns_connection:true,feedback_age_ms:0,connection_sequence:2,models_loaded:['walk','getup','pick','roulade'],
    hardware:{connected:true,generation:7,mode:'off',age_ms:0,imu:{gravity:[0,0,-1]},release_confirmed:true,
      calibrated:true,online_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],fault_ids:[]}};
  h.navigate('#motion-models');h.set(motion);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('render()');
  h.nodes.get('motionBehaviorConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').onchange();
  h.age(600);h.intervals[1]();
  assert.equal(h.nodes.get('motionSupportConfirmed').checked,true,'timer expiry must retain support confirmation');
  assert.equal(h.nodes.get('motionBehaviorConfirmed').checked,true,'timer expiry must retain behavior confirmation');
  assert.equal(h.eval('motionFresh()'),false,'browser delay cannot masquerade as 100ms control feedback');
  assert.equal(h.nodes.get('motionStand').disabled,false,'ordinary poll delay must not toggle an explicit hold request');
  const delayed=clone(motion);delayed.motion.last_hardware={...delayed.motion.hardware,age_ms:850};delayed.motion.hardware=null;
  h.set(delayed);assert.equal(h.nodes.get('motionStand').disabled,false,'display expiry must not discard a same-connection request generation');
  h.age(700);h.intervals[1]();
  assert.equal(h.nodes.get('motionStand').disabled,true,'long missing feedback must block requests');
  const postsBeforeGap=h.posts().length;
  await h.nodes.get('motionStand').onclick();assert.equal(h.posts().length,postsBeforeGap,'stale start callback cannot dispatch');
  for(const change of [{fault_ids:[10]},{online_ids:[10]},{error:'保护停止'},{start_blocked_reason:'电压过低'}]){
    const blocked=clone(motion);Object.assign(blocked.motion.hardware,change);h.set(blocked);
    assert.equal(h.nodes.get('motionStand').disabled,true,'known hardware fault must block the request');
  }
  h.set(motion);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionStand').disabled,false,'valid feedback allows one supported request after fault cases');
  h.post(async (url,options)=>{
    const body=JSON.parse(options.body);assert.equal(url,'/api/motion');assert.equal(body.page_generation,900);assert.equal(body.generation,7);
    const next=clone(motion);next.session.generation=21;next.motion.hardware.generation=8;next.motion.hardware.mode='hold';next.motion.hardware.hold_ready=true;
    return response(next);
  });
  h.set(delayed);
  await h.eval('runMotion("start",{mode:"hold",support_confirmed:true})');
  assert.equal(h.eval('ownsMotionControl()'),true,'acquisition generation change must preserve this page motion lease');
  assert.equal(h.nodes.get('motionSupportConfirmed').checked,true,'native acquisition changes do not replace the motion connection');
  const stopped=clone(motion);stopped.motion.hardware.generation=9;
  stopped.motion.hardware.last_stop_diagnostic={sequence:1,reason:'机身倾角超过 40°',before:{generation:8,mode:'hold'}};
  h.set(stopped);
  assert.equal(h.nodes.get('motionSupportConfirmed').checked,false,'new protective stop clears old confirmation');
  h.nodes.get('motionSupportConfirmed').checked=true;
  h.nodes.get('motionBehaviorConfirmed').checked=true;
  const stoppedGap=clone(stopped);stoppedGap.motion.last_hardware=clone(stopped.motion.hardware);stoppedGap.motion.hardware=null;
  const postsBeforeStoppedPolling=h.posts().length;
  for(let repeat=0;repeat<8;repeat++){
    h.set(stoppedGap);h.age(100);h.set(stopped);
    assert.equal(h.nodes.get('motionSupportConfirmed').checked,true,'same protective stop must not clear renewed support after a missing display frame');
    assert.equal(h.nodes.get('motionBehaviorConfirmed').checked,true);
  }
  assert.equal(h.posts().length,postsBeforeStoppedPolling,'polling must never start or retry an action');
  const stoppedAgain=clone(stopped);stoppedAgain.motion.hardware.generation=11;
  stoppedAgain.motion.hardware.last_stop_diagnostic={sequence:2,reason:'机身倾角超过 40°',before:{generation:10,mode:'hold'}};
  h.set(stoppedAgain);
  assert.equal(h.nodes.get('motionSupportConfirmed').checked,false,'a later stop with the same reason still requires a new confirmation');
  const restricted=clone(motion);Object.assign(restricted.motion.hardware,{generation:12,head_clearance_protocol:1,head_clearance_required:true,hold_head_deg:null,armed:false,torque_ids:[],pending_stop:false,firmware_error:0});
  h.set(restricted);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionHeadClearance').hidden,false);
  assert.equal(h.nodes.get('motionStand').disabled,true,'native hold must wait for a recorded physical head clearance');
  assert.equal(h.nodes.get('motionHeadCapture').disabled,true,'head recording needs explicit physical confirmation');
  h.nodes.get('motionHeadClearanceConfirmed').checked=true;h.nodes.get('motionHeadClearanceConfirmed').onchange();
  assert.equal(h.nodes.get('motionHeadCapture').disabled,false);
  const headGap=clone(restricted);headGap.motion.last_hardware=clone(headGap.motion.hardware);headGap.motion.hardware=null;
  h.set(headGap);h.age(600);h.intervals[1]();
  assert.equal(h.nodes.get('motionHeadCapture').disabled,false,'ordinary display delay must not flicker the explicit record button');
  const captured=clone(restricted);captured.motion.hardware.generation=13;captured.motion.hardware.hold_head_deg={'30':25,'31':-13,'32':3,'33':-2};
  h.post(async(url,options)=>{
    const body=JSON.parse(options.body);assert.equal(url,'/api/motion');assert.equal(body.action,'capture-head-clearance');assert.equal(body.clearance_confirmed,true);assert.equal(body.generation,12);
    assert.equal(body.angles,undefined,'head angles must come from current backend measurements');return response(captured);
  });
  const beforeCapture=h.posts().length;await h.nodes.get('motionHeadCapture').onclick();
  assert.equal(h.posts().length,beforeCapture+1,'one read-only capture request, no automatic start');
  assert.equal(h.nodes.get('motionStand').disabled,false);
  for(const id of ['motionWalk','motionGetup','motionPick','motionRoulade'])assert.equal(h.nodes.get(id).disabled,true,'unverified model head motion stays blocked');
  await h.eval('runMotion("start",{mode:"walk",support_confirmed:true})');
  assert.equal(h.posts().length,beforeCapture+1,'a bypassed model callback must not dispatch');
  const expiredCapture=clone(captured);expiredCapture.motion.hardware.age_ms=1600;h.set(expiredCapture);
  assert.equal(h.nodes.get('motionHeadCapture').disabled,true);
  const fault=clone(native);fault.motion.connection_sequence=3;fault.session.status='fault';fault.session.recheck_allowed=true;
  fault.session.recovery_recheck={category:'feedback',message:'舵机供电待核对'};
  h.set(fault);
  assert.equal(h.nodes.get('motionSupportConfirmed').checked,false,'actual connection termination clears confirmation');
  assert.equal(h.nodes.get('motionBehaviorConfirmed').checked,false);
  h.post(async (url,options)=>{
    assert.equal(url,'/api/recheck-feedback','full native recheck must actually review feedback holds');
    assert.equal(JSON.parse(options.body).purpose,'joints');return response(fault);
  });
  await h.nodes.get('motionRecheck').onclick();
  assert.equal(h.posts().at(-1).url,'/api/recheck-feedback');
  const offline=clone(native);offline.motion.connection_sequence=4;offline.session.status='fault';offline.session.recovery_allowed=false;
  offline.session.recheck_allowed=false;offline.session.recovery_recheck={category:'trust',message:'USB 连接未通过'};
  offline.session.connection_check={allowed:true,phase:'failed'};
  h.set(offline);
  assert.equal(h.nodes.get('motionRecheck').disabled,false,'a never-started hardware fault must remain recheckable');
  h.post(async(url,options)=>{
    assert.equal(url,'/api/check-connection','offline fault must use the strict metadata checker');
    assert.equal(JSON.parse(options.body).purpose,'joints');return response(offline);
  });
  await h.nodes.get('motionRecheck').onclick();
  assert.equal(h.posts().at(-1).url,'/api/check-connection');
  assert.equal(h.nodes.get('motionRecheck').disabled,false,'failed metadata checks must permit another explicit check');
  const preview=harness('http:','#motion-models'),p=clone(motion);
  Object.assign(p.motion.hardware,{armed:false,torque_ids:[],pending_stop:false,firmware_error:0,
    age_ms:224,head_clearance_required:true,hold_head_deg:{30:3.4,31:3.8,32:-1.6,33:5.9},
    model_start_blocked_reasons:{walk:'头颈联动尚未验证'}});
  preview.set(p);
  for(const age of [25,100,224,450,800]){
    const delayedPreview=clone(p);delayedPreview.motion.hardware.age_ms=age;
    preview.set(delayedPreview);preview.age(200);
    assert.equal(preview.nodes.get('motionShadow').disabled,false,'display lag must not make non-actuating request flicker');
    assert.equal(preview.nodes.get('motionPreviewModel').disabled,false);
    assert.equal(preview.nodes.get('motionWalk').disabled,true,'physical model gates remain locked');
  }
  const gap=clone(p);gap.motion.last_hardware=gap.motion.hardware;gap.motion.hardware=null;
  preview.set(gap);preview.nodes.get('motionPreviewModel').value='walk';
  const previewReceivedAt=preview.eval('receivedAt');
  preview.post(async(url,options)=>{
    const request=JSON.parse(options.body);
    if(request.action==='heartbeat'){assert.equal(request.generation,8);return response({heartbeat_accepted:true,generation:8,page_generation:900});}
    assert.equal(request.action,'start');assert.equal(request.mode,'shadow');assert.equal(request.generation,7);
    return response({action_receipt_protocol:1,action:'start',mode:'shadow',generation:8,
      connected:true,release_confirmed:false,page_generation:900,connection_sequence:2,operation_epoch:1,
      control_owner:{id:preview.eval('motionControlId'),generation:8}});
  });
  await preview.nodes.get('motionShadow').onclick();
  assert.equal(preview.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,1,'one click dispatches exactly one preview request');
  assert.equal(preview.posts().filter(p=>JSON.parse(p.options.body).action==='heartbeat').length,1,'native ACK immediately hands off the new lease');
  assert.equal(preview.eval('receivedAt'),previewReceivedAt,'native ACK never refreshes a sensor timestamp');
  const resumed=clone(p);resumed.motion.hardware.generation=9;preview.set(resumed);
  for(const change of [{torque_ids:[30]},{armed:true},{fault_ids:[23]},{online_ids:[30]},
    {pending_stop:true},{firmware_error:2},{error:'故障'},{mode:'hold'},{age_ms:1500}]){
    const invalid=clone(resumed);Object.assign(invalid.motion.hardware,change);preview.set(invalid);
    assert.equal(preview.nodes.get('motionShadow').disabled,true);
    await preview.nodes.get('motionShadow').onclick();
    assert.equal(preview.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,1,'unsafe or expired callback cannot dispatch');
  }
  preview.set(resumed);preview.age(1300);
  assert.equal(preview.nodes.get('motionShadow').disabled,true);
  assert.equal(preview.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,1,'polling and expiry never retry or start anything');
  const balance=harness('http:','#motion-models'),b=clone(motion);
  Object.assign(b.motion.hardware,{mode:'hold',generation:20,age_ms:224,imu_age_ms:224,imu_ok:true,
    armed:true,torque_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],
    pending_stop:false,firmware_error:0,hold_ready:true,head_clearance_required:true,
    head_clearance_protocol:1,hold_head_deg:{30:3.4,31:3.8,32:-1.6,33:5.9},
    fixed_head_walk_protocol:1,fixed_head_walk_ready:true,fixed_head_walk_zero_velocity_only:true,
    target_validation_protocol:1,policy_metadata_protocol:1,stand_target_deg:Array(15).fill(0),
    model_start_blocked_reasons:{walk:''}});
  const published=clone(b);
  published.process_isolation={protocol:1,status_source:'published_snapshot'};
  published.motion.feedback_age_ms=224;
  const display=harness('http:','#motion-models');display.set(published);
  assert.equal(display.eval('motionImuAngles()'),null,'display fallback never changes the 100ms control path');
  assert.equal(display.eval('motionImuDisplayAngles().age_ms'),224,'publication age is counted once');
  assert.equal(display.nodes.get('motionImuTilt').textContent,'0.0°','published IMU must display despite normal web lag');
  display.age(200);
  assert.equal(display.eval('motionImuDisplayFrame().age_ms'),424);
  display.age(80);
  assert.equal(display.eval('motionImuDisplayFrame()'),null,'500ms display expiry remains enforced');
  assert.equal(display.eval('motionImuDisplayAngles().retained'),true,'expired display is labeled as a retained reading');
  for(const mutate of [
    s=>{s.motion.hardware.imu_ok=false;},s=>{s.motion.phase='closed';s.motion.owns_connection=false;},
    s=>{s.motion.hardware.connected=false;},s=>{s.process_isolation.status_source='unknown';},
    s=>{s.motion.hardware.imu.gravity=[0,0,0];},s=>{s.motion.hardware.imu_age_ms=-1;}
  ]){
    const invalid=clone(published);mutate(invalid);display.set(invalid);
    assert.equal(display.eval('motionImuDisplayFrame()'),null,'invalid or disconnected data cannot enter native display fallback');
  }
  const headBlocked=clone(published);
  headBlocked.motion.hardware.hold_head_deg={30:-7.56,31:-3.43,32:-5.80,33:-2.989010989010989};
  headBlocked.motion.hardware.rawdata=[{id:30,angle_deg:-9.49},{id:31,angle_deg:-2.64},
    {id:32,angle_deg:-5.89},{id:33,angle_deg:-.8791208791208791}];
  headBlocked.motion.hardware.model_start_blocked_reasons.walk='固定头颈适配：实际头颈偏离记录位置达到 2°，已停止';
  display.set(headBlocked);display.nodes.get('motionSupportConfirmed').checked=true;display.eval('renderMotion()');
  assert.equal(display.nodes.get('motionWalk').disabled,true,'the actual 2 degree gate remains locked');
  const headReason=display.nodes.get('motionWalkGateStatus').textContent;
  assert(headReason.includes('ID 33')&&headReason.includes('2.11°')&&headReason.includes('小于 2°'));
  assert(headReason.includes('仍在保持固定站姿')&&!headReason.includes('已停止'),'HOLD failure must not falsely report a protective stop');
  assert.equal(display.nodes.get('motionWalk').title,headReason);
  await display.nodes.get('motionWalk').onclick();
  assert.equal(display.posts().length,0,'diagnostic rendering cannot unlock or start a blocked model');
  balance.set(b);balance.nodes.get('motionSupportConfirmed').checked=true;
  for(const age of [0,100,400,900]){
    balance.set(b);balance.age(age);balance.eval('renderMotion()');
    assert.equal(balance.eval('motionFresh()'),false,'display age is never relabeled as real 100ms feedback');
    assert.equal(balance.nodes.get('motionWalk').disabled,false,'ordinary display delay must not flicker an explicit zero balance request');
  }
  for(const change of [{hold_ready:false},{mode:'off'},{armed:false},{torque_ids:[]},
    {fault_ids:[11]},{online_ids:[10]},{error:'保护停止'},{pending_stop:true},{firmware_error:2},
    {hold_head_deg:null},{fixed_head_walk_ready:false},{fixed_head_walk_zero_velocity_only:false},
    {imu_ok:false},{imu_age_ms:1500},{age_ms:1500},{imu:{gravity:[.5,0,-.866]}},
    {model_start_blocked_reasons:{walk:'真实反馈超时'}}]){
    const invalid=clone(b);Object.assign(invalid.motion.hardware,change);balance.set(invalid);
    assert.equal(balance.nodes.get('motionWalk').disabled,true,'unsafe/expired zero balance must stay blocked');
    await balance.nodes.get('motionWalk').onclick();
    assert.equal(balance.posts().length,0,'blocked callback dispatches no model request');
  }
  const balanceGap=clone(b);balanceGap.motion.last_hardware=clone(balanceGap.motion.hardware);balanceGap.motion.hardware=null;
  balance.set(balanceGap);balance.nodes.get('motionSupportConfirmed').checked=true;balance.eval('renderMotion()');
  const balanceTimestamp=balance.eval('receivedAt');
  balance.post(async(url,options)=>{
    const body=JSON.parse(options.body);assert.equal(url,'/api/motion');
    if(body.action==='heartbeat'){assert.equal(body.generation,21);return response({heartbeat_accepted:true,generation:21,page_generation:900});}
    assert.equal(body.action,'start');assert.equal(body.mode,'walk');assert.equal(body.generation,20);
    return response({action_receipt_protocol:1,action:'start',mode:'walk',generation:21,connected:true,
      release_confirmed:false,page_generation:900,connection_sequence:2,operation_epoch:1,
      control_owner:{id:balance.eval('motionControlId'),generation:21}});
  });
  await balance.nodes.get('motionWalk').onclick();
  assert.equal(balance.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,1,'one supported click sends one zero model request with retained generation');
  assert.equal(balance.posts().filter(p=>JSON.parse(p.options.body).action==='heartbeat').length,1,'new generation gets immediate lease heartbeat');
  assert.equal(balance.eval('receivedAt'),balanceTimestamp,'action receipt never freshens the delayed sensor record');
  balance.set(b);balance.age(1300);balance.eval('renderMotion()');
  assert.equal(balance.nodes.get('motionWalk').disabled,true,'long missing feedback blocks zero balance');
  const ageGuard=harness('http:','#motion-models');ageGuard.set(motion);ageGuard.age(1000);
  const oldReceived=ageGuard.eval('receivedAt'),oldSensors=JSON.stringify(ageGuard.eval('snapshot'));
  assert.equal(ageGuard.eval('motionFresh()'),false);
  for(const change of [
    s=>{s.motion.hardware.generation=6;},
    s=>{s.motion.connection_sequence=1;s.motion.hardware.generation=100;},
    s=>{s.motion.snapshot_superseded=true;s.motion.hardware=null;s.motion.last_hardware=null;}
  ]){
    const obsolete=clone(motion);change(obsolete);obsolete.feedback.sample_us+=10000;ageGuard.set(obsolete);
    assert.equal(ageGuard.eval('receivedAt'),oldReceived,'obsolete native GET cannot freshen retained sensor age');
    assert.equal(JSON.stringify(ageGuard.eval('snapshot')),oldSensors,'obsolete native GET is rejected as a whole');
    assert.equal(ageGuard.eval('motionFresh()'),false);
  }
  console.log('Full native IMU restart, preview delay gating, stale gating and motion lease checks passed.');
})().catch(error=>{console.error(error);process.exitCode=1;});
`;
new Function('require', 'process', '__dirname', harnessSource + checks)(require, process, path.dirname(sourcePath));
