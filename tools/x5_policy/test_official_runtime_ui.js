'use strict';
const fs=require('node:fs'),path=require('node:path');
const sourcePath=path.join(__dirname,'../x5_recovery/test_feedback_calibration_ui.js');
let harnessSource=fs.readFileSync(sourcePath,'utf8').split('// Real pure JointWizard snapshots')[0]
 .replace("const html = fs.readFileSync(path.join(__dirname, 'feedback_calibration.html'), 'utf8');", "const html = fs.readFileSync(process.argv[2], 'utf8');");
const checks=String.raw`
(async()=>{
 const data=JSON.parse(fs.readFileSync(process.argv[3],'utf8'));
 const h=harness('http:','#motion-models');h.set(data);
 assert.equal(h.eval('nativeZeroBalanceRequestReady()'),true);
 assert.equal(h.nodes.get('trackingProbePanel').hidden,true);
 h.post(async(_url,opts)=>{const b=JSON.parse(opts.body);if(b.action==='start') {data.motion.hardware.mode=b.mode;data.motion.hardware.generation++;data.motion.hardware.armed=true;}
 if(b.action==='command')data.motion.hardware.command={forward:b.forward,side:b.side,yaw:b.yaw};return response(data);});
 await h.nodes.get('motionWalk').onclick();
 assert(h.posts().some(x=>JSON.parse(x.options.body).action==='start'));
 assert.equal(h.eval('ownsMotionControl()'),true);
 assert(h.nodes.get('motionWalk').textContent.includes('运行中'));
 await h.nodes.get('walkModelForwardLocked').onclick();
 let commands=h.posts().map(x=>JSON.parse(x.options.body)).filter(x=>x.action==='command');
 assert.equal(commands.length,1);assert.equal(commands[0].forward,.3);
 assert(h.nodes.get('walkModelForwardLocked').textContent.includes('回零速'));
 await h.nodes.get('walkModelForwardLocked').onclick();
 commands=h.posts().map(x=>JSON.parse(x.options.body)).filter(x=>x.action==='command');
 assert.equal(commands.length,2);assert.equal(commands[1].forward,0);
 assert(h.nodes.get('walkModelForwardLocked').textContent.includes('持续前进'));
 data.motion.hardware.mode='hold';h.set(data);h.ownMotion();
 assert.equal(h.eval('motionIndependentHold()'),false);
 assert(h.nodes.get('motionStopPolicy').textContent.includes('55°'));
 assert(!h.nodes.get('motionStopPolicy').textContent.includes('15°'));
 assert.equal(h.nodes.get('motionStand').textContent,'进入模型默认站姿');
 console.log('PASS real native HTML: direct model start, persistent command, return to zero, ownership, HOLD heartbeat and source recovery captions');
})().catch(e=>{console.error(e);process.exitCode=1;});`;
new Function('require','process','__dirname',harnessSource+checks)(require,process,path.dirname(sourcePath));
