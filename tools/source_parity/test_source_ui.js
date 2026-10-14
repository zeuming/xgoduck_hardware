'use strict';
const fs=require('node:fs'),path=require('node:path');
const src=path.join(__dirname,'../x5_recovery/test_feedback_calibration_ui.js');
let hsrc=fs.readFileSync(src,'utf8').split('// Real pure JointWizard snapshots')[0]
 .replace("const html = fs.readFileSync(path.join(__dirname, 'feedback_calibration.html'), 'utf8');", "const html = fs.readFileSync(process.argv[2], 'utf8');")
 .replace(/^const script = .*;$/m, String.raw`const script = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m=>m[1]).find(s=>s.includes('function acceptSnapshot('));`)
 .replace('nodes.set(match[2],element);','element.id=match[2];nodes.set(match[2],element);');
const tests=String.raw`
(async()=>{
 const data=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));
 const h=harness('http:','#motion-models');h.set(data);
 data.motion.hardware.mode='walk';data.motion.hardware.armed=true;
 data.motion.hardware.release_confirmed=false;
 data.motion.hardware.torque_ids=[];
 data.motion.hardware.torque_observation={known_ids:[],commanded_enabled:true};
 h.set(data);h.eval('renderHardwareOverview()');
 assert.equal(h.nodes.get('statusTorque').textContent,'未确认');
 assert.equal(h.eval('deviceElements.get(10).torque.textContent'),'未确认');
 assert(h.nodes.get('motionLiveDetail').textContent.includes('扭矩实测未确认'));
 delete data.motion.hardware.torque_observation;

 data.motion.hardware.age_ms=5000;data.motion.hardware.start_blocked_reason='ID10 torque unknown';
 data.motion.hardware.hold_ready=false;data.motion.hardware.mode='off';
 data.motion.error='old fault';h.set(data);h.age(20000);
 h.post(async()=>response({error:'X5 says ID10 unavailable'},409));
 for(const id of ['motionStand','motionWalk','motionStop','walkModelForwardLocked']) {
  assert.equal(h.nodes.get(id).disabled,false,id);
  const before=h.posts().length;await h.nodes.get(id).onclick();assert.equal(h.posts().length,before+1,id);
 }
 assert(h.eval('commandViewResults.at(-1).error').includes('X5 says'));
 let commands=h.posts().map(x=>JSON.parse(x.options.body));
 assert.equal(commands[2].action,'stop');assert.equal(commands[2].stop_source,'button');
 assert.equal(commands[3].action,'command');assert.equal(commands[3].forward,.3);
 data.motion.owns_connection=false;data.motion.hardware=null;data.motion.closed_proof={serial_closed:true,release_confirmed:false};h.set(data);
 await h.nodes.get('motionStop').onclick();assert.equal(JSON.parse(h.posts().at(-1).options.body).action,'stop');
 await h.nodes.get('motionZeroCommand').onclick();assert.equal(JSON.parse(h.posts().at(-1).options.body).forward,0);
 // Read/connection controls also submit the chosen command, independent of cached session gates.
 h.post(async()=>response({error:'backend checked request',token:'private-csrf-value'},409));
 for(const [id,url] of [['motionImuRead','/api/check-connection'],['motionRecheck','/api/check-connection'],['motionImuEnd','/api/end-feedback']]) {
  await h.nodes.get(id).onclick();assert.equal(h.posts().at(-1).url,url);
 }
 assert(!h.nodes.get('motionCommandResults').textContent.includes('private-csrf-value'));
 // Even while one explicit request is pending, STOP reaches the independent backend lane immediately.
 const waiting=deferred();h.post(async(_url,opts)=>JSON.parse(opts.body).action==='start'?waiting.promise:response({error:'stop checked'},409));
 const start=h.nodes.get('motionWalk').onclick();await tick();const n=h.posts().length;
 await h.nodes.get('motionStop').onclick();assert.equal(h.posts().length,n+1);
 waiting.resolve(response({error:'start rejected'},409));await start;
 // Render, stale status, timers, navigation and visibility cannot send a command.
 const before=h.posts().length;h.ownMotion=()=>{};
 h.eval('document.hidden=true');h.emit('visibilitychange');h.emit('blur');h.emit('pagehide');
 h.navigate('#device-status');h.navigate('#motion-models');h.age(60000);
 for(const callback of h.intervals)callback();await tick();
 assert.equal(h.posts().length,before);
 assert(!script.includes("stop_source:'pagehide'"));
 assert(!script.includes("stop_source:'workspace-leave'"));
 assert(!script.includes("action:'heartbeat'"));
 assert(h.eval('commandViewResults.some(row=>row.state===\"failed\")'));
 console.log('PASS: stale/blocked snapshots still submit exact commands; STOP independent and server-routed; fixed forward/zero; concurrent STOP; backend errors shown; display/visibility/navigation/timers send no control');
})().catch(e=>{console.error(e);process.exitCode=1;});`;
new Function('require','process','__dirname',hsrc+tests)(require,process,path.dirname(src));
