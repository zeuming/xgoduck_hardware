'use strict';
// Run the actual rendered command page in the established fake-DOM harness.
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
 assert(html.includes('id="motionAllModels"'));
 const ids={walk:'motionWalk',getup:'motionGetup',pick:'motionPick',roulade:'motionRoulade'};
 h.nodes.get('motionBehaviorConfirmed').checked=true;
 h.post(async()=>response({error:'fake backend rejection; no hardware'},409));
 for(const [name,id] of Object.entries(ids)) {
  assert.equal((html.match(new RegExp('id="'+id+'"','g'))||[]).length,1);
  await h.nodes.get(id).onclick();
  const start=JSON.parse(h.posts().at(-1).options.body);
  assert.equal(start.action,'start');assert.equal(start.mode,name);
  if(name!=='walk')assert.equal(start.behavior_confirmed,true);
  await h.nodes.get('modelPreview-'+name).onclick();
  const preview=JSON.parse(h.posts().at(-1).options.body);
  assert.equal(preview.mode,'shadow');assert.equal(preview.preview_model,name);
 }
 assert.equal(h.posts().length,8,'one POST per explicit click, no retry');
 // Getup/roulade must carry the actual checkbox value, never invent confirmation.
 h.nodes.get('motionBehaviorConfirmed').checked=false;
 await h.nodes.get('motionGetup').onclick();
 assert.equal(JSON.parse(h.posts().at(-1).options.body).behavior_confirmed,false);
 for(const name of Object.keys(ids)) {
  data.motion.hardware.mode=name;data.motion.hardware.active_model=name;
  data.motion.hardware.generation++;data.motion.hardware.armed=true;
  data.motion.hardware.behavior_seconds_remaining=name==='walk'?null:1.2;
  h.set(data);
  assert.equal(h.nodes.get('modelCard-'+name).dataset.running,'true');
  assert(h.nodes.get('modelState-'+name).textContent.includes('最近运行反馈'));
 }
 data.motion.hardware.mode='off';data.motion.hardware.generation++;h.set(data);
 for(const name of Object.keys(ids))assert.equal(h.nodes.get('modelCard-'+name).dataset.running,'false');
 const before=h.posts().length;
 h.eval('document.hidden=true');h.emit('visibilitychange');h.emit('blur');h.emit('pagehide');
 h.navigate('#device-status');h.navigate('#motion-models');h.age(60000);
 for(const callback of h.intervals)callback();await tick();
 assert.equal(h.posts().length,before,'render/navigation/timers never start or renew models');
 console.log('PASS all four model buttons/previews, confirmation, active model, single POST and passive rendering');
})().catch(e=>{console.error(e);process.exitCode=1;});`;
new Function('require','process','__dirname',hsrc+tests)(require,process,path.dirname(src));
