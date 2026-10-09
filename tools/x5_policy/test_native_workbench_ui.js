'use strict';
// Reuse the existing pure DOM harness with the actual serve-time native HTML.
// No browsers, HTTP listeners, network, SSH, or hardware are used.
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
  const native=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));
  const h=harness('http:','#motion-models');
  h.set(native);
  const caption=h.nodes.get('sessionTime').textContent;
  if(native.native_access.authentication_required===false){
    assert(caption.includes('浏览器控制会话'));
    assert(!caption.includes('口令登录'));
  }else assert(caption.includes('口令登录'));
  assert.equal(h.eval('workspaceView'),'motion-models');
  assert.equal(h.nodes.get('motionModelsCount').textContent,'4 / 4');
  assert.equal(h.nodes.get('motionConnect').disabled,true);
  assert.equal(h.nodes.get('motionStand').disabled,true);
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('render();');
  assert.equal(h.nodes.get('motionStand').disabled,false);
  for(const id of ['recoverFeedback','endFeedback','wizardLevel','wizardFront','wizardLeft','wizardFinish','jointDirectionStart','jointDirectionEnd','jointRangeA','jointRangeB','jointSave','zeroCapture','zeroSave','servoEnter']){
    assert.equal(h.nodes.get(id).disabled,true,id);
  }
  for(const hash of ['#imu-steps','#joint-steps','#joint-zeros','#servo-settings','#device-status']){
    h.navigate(hash);h.eval('render();');
    assert.equal(h.nodes.get('recoverFeedback').disabled,true,hash);
    assert.equal(h.nodes.get('endFeedback').disabled,true,hash);
    assert.equal(h.eval('canAction("level")'),false,hash);
    assert.equal(h.eval('canMaintenance("enter")'),false,hash);
  }
  h.navigate('#motion-models');h.nodes.get('motionSupportConfirmed').checked=true;h.eval('render();');
  h.post(async (_url, options)=>{
    const body=JSON.parse(options.body);
    if(body.action==='start'){
      const next=clone(native);next.motion.hardware.mode=body.mode;next.motion.hardware.generation++;next.motion.hardware.hold_ready=true;
      return response(next);
    }
    return response(native);
  });
  await h.eval('runMotion("start",{mode:"hold",support_confirmed:true})');
  const started=h.posts().find(call=>JSON.parse(call.options.body).action==='start');
  assert.equal(started.url,'/api/motion');
  assert.equal(h.eval('ownsMotionControl()'),true);
  const before=h.posts().length;h.emit('blur');await tick();
  assert.equal(h.posts().length,before,'blur must not stop fixed stance');
  const closed=clone(native);closed.motion.phase='closed';closed.motion.owns_connection=false;
  closed.motion.last_hardware={generation:native.motion.hardware.generation+2,mode:'off'};closed.motion.hardware=null;
  h.set(closed);
  assert.equal(h.eval('snapshot.motion.phase'),'closed','terminal hardware=null must replace old ready');
  assert.equal(h.nodes.get('motionRecheck').disabled,false,'recheck after proven closed');
  const connected=clone(native);connected.motion.connection_sequence=2;
  h.post(async (_url, options)=>{
    assert.equal(JSON.parse(options.body).action,'connect');
    return response(connected);
  });
  await h.nodes.get('motionRecheck').onclick();
  assert.equal(JSON.parse(h.posts().at(-1).options.body).action,'connect');
  assert.equal(h.eval('snapshot.motion.connection_sequence'),2,'new connection can replace closed');
  const unknown=clone(connected);unknown.motion.phase='fault';unknown.motion.owns_connection=true;
  unknown.motion.last_hardware=clone(connected.motion.hardware);unknown.motion.hardware=null;
  h.set(unknown);
  assert.equal(h.eval('snapshot.motion.phase'),'fault','hardware-null fault must be adopted');
  assert.equal(h.nodes.get('motionModelsCount').textContent,'4 / 4','fault must retain truthful loaded model count');
  assert.equal(h.nodes.get('motionHost').textContent,'X5 · CPU');
  assert.equal(h.nodes.get('motionCalibration').textContent,'已载入 · 连接待复查');
  assert.equal(h.nodes.get('motionStand').disabled,true);
  assert.equal(h.nodes.get('motionWalk').disabled,true);
  assert.equal(h.nodes.get('motionRecheck').disabled,true,'uncertain owner cannot reconnect');
  assert(h.nodes.get('motionRecoveryStatus').textContent.includes('确认串口已关闭'));
  assert(h.nodes.get('x5ConnectionTarget').textContent.includes('192.168.1.11:8767'));
  console.log('Native unified UI render/navigation/control/recheck checks passed.');
})().catch(error=>{console.error(error);process.exitCode=1;});
`;
new Function('require', 'process', '__dirname', harnessSource + checks)(require, process, path.dirname(sourcePath));
