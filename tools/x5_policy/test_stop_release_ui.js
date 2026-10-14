'use strict';
const fs=require('node:fs'),path=require('node:path');
const src=path.join(__dirname,'../x5_recovery/test_feedback_calibration_ui.js');
let harnessSource=fs.readFileSync(src,'utf8').split('// Real pure JointWizard snapshots')[0]
.replace("const html = fs.readFileSync(path.join(__dirname, 'feedback_calibration.html'), 'utf8');", "const html = fs.readFileSync(process.argv[2], 'utf8');");
const checks=String.raw`
(async()=>{
 const data=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));
 data.motion.phase='fault';data.motion.owns_connection=false;data.motion.hardware=null;
 data.motion.error='ARM ID10 register40 timeout';data.motion.closed_proof={serial_closed:true,release_confirmed:false};
 const h=harness('http:','#motion-models');h.set(data);
 assert.equal(h.nodes.get('motionStop').disabled,false);
 assert(h.nodes.get('motionStop').textContent.includes('重新连接并释放'));
 assert(h.nodes.get('motionWalkGateStatus').textContent.includes('ARM ID10'));
 h.post(async(_url,opts)=>{const body=JSON.parse(opts.body);assert.equal(body.action,'release-torque');
 data.motion.phase='closed';data.motion.connection_sequence++;data.motion.error=null;
 data.motion.closed_proof.release_confirmed=true;return response(data);});
 await h.nodes.get('motionStop').onclick();
 assert.equal(h.posts().length,1);assert.equal(JSON.parse(h.posts()[0].options.body).action,'release-torque');
 assert(!h.nodes.get('motionStop').textContent.includes('重新连接'));
 assert(h.eval("nativeButtonGateReason({id:'motionDisconnect'})").includes('已确认'));
 data.motion.closed_proof.release_confirmed=false;h.set(data);
 h.post(async()=>({ok:false,status:400,json:async()=>({error:'释放扭矩仍未确认：ID10'})}));
 await h.nodes.get('motionStop').onclick();
 assert(h.eval('motionError').includes('仍未确认'));
 console.log('PASS closed-fault release button, exact stop-only request, original fault, confirmed/unknown states, and failed retry visibility');
})().catch(e=>{console.error(e);process.exitCode=1;});`;
new Function('require','process','__dirname',harnessSource+checks)(require,process,path.dirname(src));
