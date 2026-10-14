'use strict';
// DOM-only validation of actual recorded voltage rows; no network or device access.
const fs=require('node:fs'),path=require('node:path'),Module=require('node:module');
const file=path.resolve(__dirname,'../x5_recovery/test_feedback_calibration_ui.js');
const records=fs.readFileSync(process.argv[2],'utf8').trim().split('\n').map(JSON.parse).at(-1).records;
const sample=records.find(r=>r.voltage_rows.length===15&&r.voltage_rows.every(v=>v.fresh));
if(!sample)throw Error('No recorded live15 sample');
let source=fs.readFileSync(file,'utf8');
const extra=`test('recorded real15 voltage rows render as volts without motion',()=>{
 const h=harness('http:','#motion-models'),s=motionFixture();
 s.motion.hardware.voltage_supported=true;s.motion.hardware.motor_voltages=${JSON.stringify(sample.voltage_rows)};
 h.set(s);for(const row of s.motion.hardware.motor_voltages){assert.equal(h.eval('motionRows.get('+row.id+')[6]').textContent,row.volts.toFixed(1)+' V');}
 assert(h.nodes.get('motionVoltageSummary').textContent.includes('15/15'));
 assert.equal(h.posts().length,0);
});
const selected=tests.filter(([name])=>name.startsWith('recorded real15'));tests.splice(0,tests.length,...selected);
`;
source=source.replace('const testWatchdog =',extra+'const testWatchdog =');
const m=new Module(file,module);m.filename=file;m.paths=module.paths;m._compile(source,file);
