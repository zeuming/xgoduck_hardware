'use strict';
// Serve-time native HTML with fake DOM/fetch/timers only. No UART or network.
// An optional first argument supplies an already rendered native HTML file.
const fs = require('node:fs');
const path = require('node:path');
const {execFileSync} = require('node:child_process');

const servedHTML = process.argv[2]
  ? fs.readFileSync(process.argv[2], 'utf8')
  : execFileSync(process.env.PYTHON || 'python3', ['-c', [
      'from pathlib import Path',
      'import native_workbench',
      'import command_view_page',
      "source = Path('../x5_recovery/feedback_calibration.html').read_text()",
      'page = native_workbench.native_page(source, full_workbench=True)',
      'print(command_view_page.command_view_page(page).decode())'
    ].join('\n')], {cwd: __dirname, encoding: 'utf8'});
// Served pages also contain health/log/gamepad scripts. Keep every DOM node,
// but execute the actual main workbench script, including command-view changes.
const scriptPattern = /<script\b[^>]*>([\s\S]*?)<\/script\s*>/g;
const mainScripts = [...servedHTML.matchAll(scriptPattern)]
  .filter(match => match[1].includes('function acceptSnapshot('));
if (mainScripts.length !== 1) throw new Error('Expected one main workbench script');
const nativeHTML = servedHTML.replace(scriptPattern, (whole, body) =>
  body === mainScripts[0][1] ? '<script>' + body + '</script>' : '');
const harnessPath = path.join(__dirname, '../x5_recovery/test_feedback_calibration_ui.js');
const harnessSource = fs.readFileSync(harnessPath, 'utf8')
  .split('// Real pure JointWizard snapshots')[0];

const checks = String.raw`
(async()=>{
  assert(nativeHTML.includes('Both passive calibration and explicit motion run on this X5.'),
    'exercise the actual full native page adaptation');
  const h=harness('http:','#motion-models',nativeHTML),s=fixture('closed',19);
  const buttonId='saveRangeBoundaries';
  assert(h.nodes.has(buttonId),'range save needs its own calibration button outside command-view motion admission');
  s.native_access={full_workbench:true,authentication_required:true,page_generation:900};
  for(const j of s.calibration.joints){j.min=1000;j.max=3000;}
  s.motion={phase:'ready',owns_connection:true,feedback_age_ms:0,connection_sequence:2,
    models_loaded:['walk'],deployment:{archive_sha256:'a',started_monotonic:12},
    hardware:{connected:true,generation:7,mode:'off',age_ms:0,armed:false,
      release_confirmed:true,torque_ids:[],online_ids:[10,11,12],
      rawdata:[{id:10,position:999},{id:11,position:3001},{id:12,position:2000}]}};
  h.set(s);
  assert.equal(h.nodes.get(buttonId).disabled,false);
  assert(h.nodes.get('motionRangeUpdateHint').textContent.includes('1000 → 999'));
  assert(h.nodes.get('motionRangeUpdateHint').textContent.includes('3000 → 3001'));
  h.age(900);
  assert.equal(h.nodes.get(buttonId).disabled,false,'one-second display polling must not block a stopped save');
  const delayed=harness('http:','#motion-models',nativeHTML),published=clone(s);
  published.motion.hardware.age_ms=320;published.motion.feedback_age_ms=320;delayed.set(published);
  assert.equal(delayed.nodes.get(buttonId).disabled,false,'display publication age is not the controller sample age');
  delayed.age(2200);
  assert.equal(delayed.nodes.get(buttonId).disabled,true,'an actually stale page still blocks saving');
  assert(delayed.eval("nativeButtonGateReason(el('saveRangeBoundaries'))").includes('页面反馈尚未更新'));

  const after=clone(s);
  after.calibration.joints[0].min=999;
  after.calibration.joints[1].max=3001;
  // reload_calibration clears live/historical feedback. The closed proof still
  // carries the generation needed to accept this newer response in native UI.
  Object.assign(after.motion,{hardware:null,last_hardware:null,phase:'closed',
    owns_connection:false,closed_proof:{serial_closed:true,
      after:{generation:8,mode:'off',release_confirmed:true}}});
  after.range_update_message='越界边界已保存；请手动重新连接。';
  const saveReply=deferred();
  h.post(async(url,options)=>{
    assert.equal(url,'/api/motion');
    const body=JSON.parse(options.body);
    assert.equal(body.action,'expand-feedback-ranges');
    assert.equal(body.generation,7);
    assert.equal(body.page_generation,900);
    assert.deepEqual(body.updates,[{id:10,position:999,min:1000,max:3000},
      {id:11,position:3001,min:1000,max:3000}]);
    return saveReply.promise;
  });
  const saving=h.nodes.get(buttonId).onclick();
  assert.equal(h.posts().length,1);
  assert.equal(h.nodes.get(buttonId).disabled,true,'save disables its own button while awaiting X5');
  await h.nodes.get(buttonId).onclick();
  assert.equal(h.posts().length,1,'a second pending click cannot send another save');
  saveReply.resolve(response(after));
  await saving;
  assert.equal(h.posts().length,1,'one click sends only the save request');
  assert.equal(h.eval('snapshot.motion.phase'),'closed');
  assert.equal(h.eval('snapshot.calibration.joints[0].min'),999);
  assert.equal(h.eval('snapshot.calibration.joints[1].max'),3001);
  assert.equal(h.eval('motionRows.get(10)[2].textContent'),'999–3000');
  assert.equal(h.eval('motionRows.get(11)[2].textContent'),'1000–3001');
  assert.equal(h.eval('motionRows.get(12)[2].textContent'),'1000–3000');
  assert.equal(h.nodes.get('motionRangeUpdateHint').textContent,after.range_update_message);
  assert.equal(h.nodes.get(buttonId).disabled,true);
  assert.equal(h.nodes.get('motionConnect').disabled,false,'reconnect remains manual');
  const untouched=JSON.parse(h.eval('JSON.stringify(snapshot.calibration)'));
  untouched.joints[0].min=1000;untouched.joints[1].max=3000;
  assert.deepEqual(untouched,s.calibration,'only selected boundaries change');
  await h.nodes.get(buttonId).onclick();
  assert.equal(h.posts().length,1,'disconnected duplicate click cannot save or reconnect');

  const failed=harness('http:','#motion-models',nativeHTML);
  failed.set(s);
  failed.post(async()=>response({error:'读数或已保存区间已变化，请刷新后重试'},409));
  await failed.nodes.get(buttonId).onclick();
  assert.equal(failed.posts().length,1,'a failed save is not retried');
  assert.equal(failed.eval('snapshot.calibration.joints[0].min'),1000);
  assert.equal(failed.eval('snapshot.motion.phase'),'ready');
  const failure=failed.eval("motionRangeUpdateMessage || motionError || (typeof commandViewResults!=='undefined'&&commandViewResults.at(-1)?.error) || ''");
  assert(failure.includes('请刷新后重试'));
  assert(!failure.includes('已保存为安全区间边界'));
  const warned=harness('http:','#motion-models',nativeHTML),warning=clone(after);
  warning.range_update_message='校准已持久保存，但 RAM 运控载入失败；运动已锁定，需重启工作台。';
  warned.set(s);warned.post(async()=>response(warning));
  await warned.nodes.get(buttonId).onclick();
  assert.equal(warned.posts().length,1);
  assert.equal(warned.nodes.get('motionRangeUpdateHint').textContent,warning.range_update_message,
    'backend durability or reload warnings must not become generic success');
  const disconnected=harness('http:','#motion-models',nativeHTML),closed=clone(s);
  Object.assign(closed.motion,{phase:'closed',owns_connection:false,hardware:null});disconnected.set(closed);
  assert(disconnected.nodes.get('motionRangeUpdateHint').textContent.includes('连接并载入模型'));
  assert(disconnected.eval("nativeButtonGateReason(el('saveRangeBoundaries'))").includes('连接并载入模型'));
  for(const change of [{mode:'hold'},{armed:true},{release_confirmed:false},{age_ms:2500},{torque_ids:[10]},
      {online_ids:[]},{rawdata:[{id:10,position:1000},{id:11,position:3000}]}]){
    const blocked=harness('http:','#motion-models',nativeHTML),invalid=clone(s);
    Object.assign(invalid.motion.hardware,change);blocked.set(invalid);
    assert.equal(blocked.nodes.get(buttonId).disabled,true,'range save retains its own live calibration gate');
    await blocked.nodes.get(buttonId).onclick();
    assert.equal(blocked.posts().length,0,'invalid readings never dispatch a save');
  }
  console.log('Native range save UI: display interval admitted, specific blocking reasons, one save request, both boundaries updated, manual reconnect, failed save preserved.');
})().catch(error=>{console.error(error);process.exitCode=1;});
`;
new Function('require','process','__dirname','nativeHTML',harnessSource+'\n'+checks)(
  require,process,path.dirname(harnessPath),nativeHTML);
