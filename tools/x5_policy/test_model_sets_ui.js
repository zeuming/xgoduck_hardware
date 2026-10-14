'use strict';
// Actual rendered page, fake DOM/fetch; no robot connection.
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
 assert.equal(h.posts().length,0);
 assert.equal(h.nodes.get('motionConnect').textContent,'加载原版模型');
 assert.equal(h.nodes.get('motionConnectTrained').textContent,'加载自训练模型');
 h.post(async()=>response({error:'backend test; no physical connection'},409));
 for(const [id,group] of [['motionConnect','original'],['motionConnectTrained','trained']]){
  const before=h.posts().length;await h.nodes.get(id).onclick();
  assert.equal(h.posts().length,before+1);
  const body=JSON.parse(h.posts().at(-1).options.body);
  assert.equal(body.action,'connect');assert.equal(body.model_set,group);
  assert.equal(h.nodes.get(id).disabled,false);
 }
 data.motion.model_set='trained';data.motion.model_set_label='自训练模型';h.set(data);
 assert(h.nodes.get('motionModelSetStatus').textContent.includes('自训练模型'));
 assert.equal(h.nodes.get('motionConnectTrained').getAttribute('aria-pressed'),'true');
 data.motion.model_set='original';data.motion.model_set_label='原版模型';h.set(data);
 assert(h.nodes.get('motionModelSetStatus').textContent.includes('原版模型'));
 const count=h.posts().length;
 h.navigate('#device-status');h.navigate('#motion-models');h.age(60000);
 h.emit('visibilitychange');h.emit('blur');h.emit('pagehide');
 for(const callback of h.intervals)callback();await tick();
 assert.equal(h.posts().length,count,'render/navigation cannot load, connect or start');
 console.log('PASS: original/trained buttons issue one explicit connect each, correct model set, passive status and no auto actions');
})().catch(e=>{console.error(e);process.exitCode=1;});`;
new Function('require','process','__dirname',hsrc+tests)(require,process,path.dirname(src));
