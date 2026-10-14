const fs=require('fs'),vm=require('vm'),assert=require('assert/strict');
const html=fs.readFileSync(__dirname+'/../x5_policy/gamepad_panel.html','utf8');
const script=html.match(/<script>([\s\S]*?)<\/script>/)[1];
(async()=>{
 const nodes={},timers=[],calls=[];
 for(const id of ['x5GamepadPanel','x5GamepadStatus','x5GamepadControl','x5GamepadValues','x5GamepadError','x5ControlMonitor','x5ControlMonitorSummary','x5ControlMonitorData'])nodes[id]={textContent:'',open:false};
 const context={document:{hidden:false,getElementById:id=>nodes[id],querySelector:()=>({append(){}})},location:{hash:'#motion-models'},
  setTimeout:fn=>{timers.push(fn);return fn;},clearTimeout(){},AbortController,
  snapshot:{motion:{owns_connection:true,hardware:{mode:'walk',generation:5},gamepad_control:{active:true,shared_model_buttons:true,message:'共用按钮'}}},
  motionControl:{generation:5},motionReceipt:null,ownsMotionControl:()=>false,
  motionControlHardware:()=>context.snapshot.motion.hardware,
  fetch:async(path,options)=>{calls.push({path,options});return{ok:true,json:async()=>({connected:true,axes:Array(8).fill(0),buttons:Array(15).fill(0)})};}};
 vm.runInNewContext(script,context);await new Promise(setImmediate);
 assert(context.ownsMotionControl(),'web commands stay available after gamepad start');
 for(const mode of ['getup','pick','roulade']){
  context.snapshot.motion.hardware.mode=mode;
  assert(context.ownsMotionControl(),'web controls stay available during '+mode);
 }
 context.snapshot.motion.hardware.mode='walk';
 assert.equal(context.motionControl.generation,5,'display must not revoke browser control');
 context.snapshot.motion.hardware.joint_follow={active:true};assert(!context.ownsMotionControl(),'slider ownership remains separate');
 context.snapshot.motion.hardware.joint_follow={active:false};context.snapshot.motion.gamepad_control.shared_model_buttons=false;
 assert(!context.ownsMotionControl(),'legacy mode retains existing ownership');
 context.snapshot.motion.gamepad_control.shared_model_buttons=true;context.snapshot.motion.owns_connection=false;
 assert(!context.ownsMotionControl(),'disconnected controller grants no controls');
 assert(calls.every(c=>c.path==='/api/gamepad'&&!c.options.method),'panel only reads input');
 assert(!html.includes('id="x5GamepadEnable"')&&!html.includes('id="x5GamepadSupport"'),'no takeover prerequisite');
 for(const t of ['A · 编号 0','X · 编号 3','Y · 编号 4','B · 编号 1','START · 编号 11','SELECT · 编号 10','L1 · 编号 6','R1 · 编号 7','L2 · 编号 8','R2 · 编号 9','右摇杆按下 · 编号 14','连接并载入模型','停止并断开运控','网页与手柄操作同一个当前模型'])assert(html.includes(t));
 assert(script.includes('main.append(panel)'));assert(!html.includes('按住 R1'));
 console.log('8 shared gamepad panel scenarios passed');
})().catch(e=>{console.error(e);process.exit(1);});
