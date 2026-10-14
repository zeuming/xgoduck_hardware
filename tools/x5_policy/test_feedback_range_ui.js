'use strict';
const fs=require('node:fs'),path=require('node:path');
let source=fs.readFileSync(path.join(__dirname,'../x5_recovery/test_feedback_calibration_ui.js'),'utf8').split('// Real pure JointWizard snapshots')[0];
const checks=String.raw`
const h=harness('http:','#motion-models'),s=fixture('closed',19);
s.native_access={full_workbench:true,page_generation:900};
s.calibration.joints[0].min=1000;s.calibration.joints[0].max=3000;
s.motion={phase:'ready',owns_connection:true,feedback_age_ms:0,hardware:{connected:true,generation:7,mode:'off',age_ms:0,armed:false,release_confirmed:true,torque_ids:[],online_ids:[10],rawdata:[{id:10,position:999}]}};
h.set(s);h.eval('renderMotion()');
assert.equal(h.nodes.get('saveRangeBoundaries').disabled,false);
assert(h.nodes.get('motionRangeUpdateHint').textContent.includes('1000 → 999'));
assert.equal(h.eval('motionRangeUpdates()[0].position'),999);
s.motion.hardware.mode='hold';h.set(s);assert.equal(h.nodes.get('saveRangeBoundaries').disabled,true);
s.motion.hardware.mode='off';s.motion.hardware.rawdata[0].position=1000;h.set(s);assert.equal(h.nodes.get('saveRangeBoundaries').disabled,true);
console.log('range button: outlier enabled, boundary preview, running and in-range disabled');
`;
new Function('require','process','__dirname',source+'\n'+checks)(require,process,path.join(__dirname,'../x5_recovery'));
