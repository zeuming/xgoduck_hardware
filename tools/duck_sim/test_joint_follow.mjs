import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {JointFollowPanel,followFrame,followStartBlocker} from './joint_follow.js';

const ids=[10,11,12,13,14,30,31,32,33,20,21,22,23,24,34];
const timers=[],originalTimeout=globalThis.setTimeout,originalClear=globalThis.clearTimeout;
globalThis.setTimeout=(fn,ms)=>{timers.push({fn,ms});return timers.length;};
globalThis.clearTimeout=()=>{};
function fixture(){
  const elements=new Map(),posts=[];let now=100;
  const element=()=>({checked:false,disabled:false,_text:'',textWrites:0,get textContent(){return this._text;},set textContent(value){this._text=value;this.textWrites++;},hidden:false,attributes:{},getAttribute(k){return this.attributes[k];},setAttribute(k,v){this.attributes[k]=v;},scrollIntoView(){this.scrolled=true;},after(node){elements.set(node.id,node);}});
  const html=readFileSync(new URL('./index.html',import.meta.url),'utf8');
  for(const match of html.matchAll(/\bid="([^"]+)"/g)){assert.equal(elements.has(match[1]),false,'HTML ids must be unique');elements.set(match[1],element());}
  const doc={hidden:false,addEventListener(){},createElement:element,getElementById:id=>elements.get(id)||null};
  const state={token:'test',native_access:{page_generation:7,joint_follow_protocol:1},
    calibration:{joints:ids.map(id=>({id,min:1000,max:3000}))},motion:{owns_connection:true,phase:'ready',connection_sequence:1,
    hardware:{connected:true,mode:'off',generation:5,armed:false,release_confirmed:true,torque_ids:[],age_ms:1,imu_age_ms:1,imu_ok:true,calibrated:true,online_ids:[...ids],fault_ids:[],
      rawdata:[...ids].sort((a,b)=>a-b).map(id=>({id,position:2047,angle_deg:id/10})),joint_follow:{active:false,joints:[]}}}};
  const p=new JointFollowPanel({ids,doc,clock:()=>now,disconnectViewer:async()=>{},onFrame(){},onChange(){},
    request:async(url,options)=>{
      if(!options.body)return {ok:true,json:async()=>structuredClone(state)};
      const body=JSON.parse(options.body);posts.push(body);
      if(body.action==='start'){
        state.motion.hardware.generation=6;state.motion.hardware.mode='hold';state.motion.hardware.armed=true;
        state.motion.hardware.release_confirmed=false;
        state.motion.control_owner={id:p.client,generation:6};
        state.motion.hardware.joint_follow={active:true,joints:ids.map(id=>({id,requested_deg:id/10,min_deg:-10,max_deg:10}))};
        return {ok:true,json:async()=>({generation:6})};
      }
      if(body.action==='heartbeat')return {ok:true,json:async()=>({heartbeat_accepted:true,generation:state.motion.hardware.generation})};
      if(body.action==='stop'){state.motion.hardware.joint_follow.active=false;state.motion.hardware.mode='off';state.motion.hardware.armed=false;state.motion.hardware.release_confirmed=true;state.motion.hardware.generation++;}
      return {ok:true,json:async()=>({})};
    }});
  p.snapshot=structuredClone(state);p.entered=true;
  return {p,state,doc,posts,elements,advance:dt=>{now+=dt;}};
}
try{
  // Polling and render cannot arm; ids map by identity, not policy/servo order.
  let f=fixture();await f.p.poll();assert.equal(f.posts.length,0);
  assert.deepEqual(followFrame(f.state,ids).q,ids.map(id=>id/10));
  f.state.motion.hardware.age_ms=601;assert.equal(followFrame(f.state,ids),null);
  await f.p.start();assert.equal(f.posts.length,0,'unchecked start is inert');

  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  assert.equal(f.p.active,true);assert.equal(f.posts[0].action,'start');
  assert.equal(f.posts[0].joint_follow,true);assert.equal(f.posts[1].action,'heartbeat');
  assert.equal(f.doc.getElementById('home-pose').disabled,true);
  f.p.target(21,3);for(let i=0;i<12;i++)await Promise.resolve();
  assert.equal(f.posts.filter(p=>p.action==='joint-target').length,1);
  assert.equal(f.posts.at(-1).speed_deg_s,5);
  f.p.target(21,4);f.p.target(21,5);
  assert.equal(f.posts.filter(p=>p.action==='joint-target').length,1,'rapid input is coalesced');
  f.advance(101);await f.p.flush();assert.equal(f.posts.at(-1).angle_deg,5);
  const old=structuredClone(f.state);old.motion.hardware.generation=5;old.motion.hardware.joint_follow.active=false;
  f.p.accept(old);assert.equal(f.p.active,true,'old publication cannot cancel a new start');
  f.state.motion.control_owner.id='another-page';f.p.accept(f.state);
  assert.equal(f.p.active,false);assert.equal(f.p.pending,null);

  // Rate changes preserve the latest user target, including one coalesced
  // behind an in-flight send; a stale published angle must never replace it.
  f=fixture();assert.equal(f.p.setSpeed(31),true);assert.equal(f.p.setSpeed(50),true);assert.equal(f.posts.length,0);
  for(const speed of [0,51,NaN,true])assert.equal(f.p.setSpeed(speed),false);
  f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  f.p.target(34,6);for(let i=0;i<12;i++)await Promise.resolve();
  assert.equal(f.posts.at(-1).speed_deg_s,50);
  f.p.target(34,7);f.p.setSpeed(20);f.advance(101);await f.p.flush();
  assert.equal(f.posts.at(-1).joint_id,34);assert.equal(f.posts.at(-1).angle_deg,7);
  assert.equal(f.posts.at(-1).speed_deg_s,20);
  for(let i=0;i<12;i++)await Promise.resolve();
  f.p.active=false;const speedPosts=f.posts.length;f.p.setSpeed(10);
  assert.equal(f.posts.length,speedPosts,'speed changes never enable motors');

  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  f.p.leave();await Promise.resolve();await Promise.resolve();
  assert.equal(f.p.active,false);assert.equal(f.posts.at(-1).action,'stop');assert.equal(f.posts.at(-1).stop_source,'workspace-leave');
  const count=f.posts.length;f.p.target(10,2);assert.equal(f.posts.length,count);

  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  f.state.motion.hardware.age_ms=700;f.p.accept(f.state);
  assert.equal(f.p.active,true);assert.equal(f.p.feedbackWaiting,true);
  assert.equal(f.p.target(31,2),false,'stale display feedback freezes new targets');
  f.advance(1501);timers.filter(t=>t.ms===50).at(-1).fn();
  assert.equal(f.p.active,false);assert.equal(f.posts.at(-1).action,'stop','sustained display loss still stops within its bound');

  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  const request=f.p.request;let release;
  f.p.request=async(url,opts)=>{if(opts.body&&JSON.parse(opts.body).action==='start'){
    const response=await request(url,opts);await new Promise(resolve=>{release=resolve;});return response;
  }return request(url,opts);};
  const starting=f.p.start();
  for(let i=0;i<10&&!release;i++)await Promise.resolve();
  assert.ok(release);f.p.leave();release();await starting;
  assert.equal(f.p.active,false);assert.equal(f.posts.at(-1).action,'stop','late START acknowledgement cannot rearm page');
  // Actual incident: ID30 was outside its saved range. Explain before START,
  // preserve confirmation, and send neither START nor an automatic target.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  Object.assign(f.state.calibration.joints.find(j=>j.id===30),{min:1720,max:3729});
  f.state.motion.hardware.rawdata.find(j=>j.id===30).position=1603;
  f.p.accept(structuredClone(f.state));
  assert.match(f.doc.getElementById('joint-check-range').textContent,/ID30.*1603.*1720–3729/);
  assert.equal(f.doc.getElementById('joint-follow-start').disabled,false,'blocked START remains clickable to explain');
  await f.p.start();assert.equal(f.posts.length,0);
  assert.match(f.doc.getElementById('joint-follow-message').textContent,/ID30.*1603.*1720–3729/);
  assert.equal(f.doc.getElementById('joint-follow-message').hidden,false);
  assert.equal(f.doc.getElementById('joint-follow-support').checked,true);
  f.state.motion.hardware.rawdata.find(j=>j.id===30).position=1800;
  f.p.accept(structuredClone(f.state));
  assert.equal(f.doc.getElementById('joint-follow-start').disabled,false);
  assert.equal(f.posts.length,0,'moving into range never starts automatically');

  // A missing display frame cannot make Connect blink/reconnect, and cannot
  // use historical feedback to start. Ordinary polling preserves the checkbox.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  for(let i=0;i<8;i++){
    const s=structuredClone(f.state);
    if(i%2){s.motion.last_hardware=s.motion.hardware;s.motion.hardware=null;}
    f.p.accept(s);
    assert.equal(f.doc.getElementById('joint-follow-connect').disabled,false);
    assert.equal(f.doc.getElementById('joint-follow-connect').textContent,'检查连接');
    assert.equal(f.doc.getElementById('joint-follow-support').checked,true);
    if(i%2)assert.match(f.doc.getElementById('joint-check-feedback').textContent,/尚未收到本轮有效反馈/);
  }
  // The initial GET may be stale even if the clicked button was ready.
  const requestFresh=f.p.request;let gets=0;
  f.p.request=async(url,opts)=>{
    if(!opts.body&&gets++===0){const s=structuredClone(f.state);s.motion.hardware=null;return {ok:true,json:async()=>s};}
    return requestFresh(url,opts);
  };
  const timerBase=timers.length,waiting=f.p.start();
  for(let i=0;i<12&&!timers.slice(timerBase).some(t=>t.ms===100);i++)await Promise.resolve();
  assert.equal(f.posts.length,0,'missing publication cannot authorize ARM');
  timers.slice(timerBase).find(t=>t.ms===100).fn();await waiting;
  assert.equal(f.p.active,true,'waits for a fresh frame without asking to recheck');

  // Production trace: the two display publishers can omit a frame after an
  // accepted START while the controller keeps sampling normally. Never turn
  // that temporary display gap into an immediate motor STOP.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  const startGapRequest=f.p.request;let afterStart=false;
  f.p.request=async(url,opts)=>{
    const response=await startGapRequest(url,opts);
    if(opts.body&&JSON.parse(opts.body).action==='start')afterStart=true;
    if(!opts.body&&afterStart){const s=structuredClone(f.state);s.motion.hardware=null;s.motion.sample_age_ms=535;return {ok:true,json:async()=>s};}
    return response;
  };
  await f.p.start();
  assert.equal(f.p.active,true,'a missing initial display packet must not cancel an accepted START');
  assert.equal(f.p.feedbackWaiting,true);
  assert.equal(f.posts.some(p=>p.action==='stop'),false);
  assert.equal(f.p.target(31,4),false,'a missing measured frame cannot authorize a new target');
  f.advance(250);f.p.accept(structuredClone(f.state));
  assert.equal(f.p.feedbackWaiting,false);assert.equal(f.p.active,true);
  assert.equal(f.p.target(31,4),true,'fresh same-session feedback restores target controls');

  // Actual page drag: a display gap must freeze, rather than discard, the
  // last fresh-authorized target waiting for the coalescing interval.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  f.p.nextSendAt=200;f.p.target(34,4);
  const dragGap=structuredClone(f.state);dragGap.motion.hardware=null;f.p.accept(dragGap);
  assert.equal(f.p.pending?.angle_deg,4,'the final drag position survives a temporary display gap');
  assert.equal(f.p.target(34,5),false,'missing feedback cannot authorize a different target');
  assert.equal(f.posts.some(p=>p.action==='joint-target'),false);
  f.advance(150);f.p.accept(structuredClone(f.state));
  for(let i=0;i<20;i++)await Promise.resolve();
  assert.equal(f.posts.at(-1).action,'joint-target');assert.equal(f.posts.at(-1).angle_deg,4);

  // Connect delegates the read-only reader handover to the native service.
  f=fixture();f.state.session={status:'active'};f.state.motion.owns_connection=false;f.state.motion.phase='closed';f.state.motion.hardware=null;
  const requestConnect=f.p.request;
  f.p.request=async(url,opts)=>{
    if(opts.body&&JSON.parse(opts.body).action==='connect'){
      f.posts.push(JSON.parse(opts.body));f.state.motion.owns_connection=true;f.state.motion.phase='ready';
      return {ok:true,json:async()=>structuredClone(f.state)};
    }
    return requestConnect(url,opts);
  };
  assert.match(followStartBlocker(f.state,ids),/只读反馈/);
  await f.p.connect();assert.deepEqual(f.posts.map(p=>p.action),['connect']);
  assert.equal(f.doc.getElementById('joint-follow-connect').textContent,'检查连接');

  // If connection is absent, an attempted precheck keeps confirmation, but a
  // request actually sent with an unknown outcome still requires reconfirming.
  f=fixture();f.state.motion={phase:'closed',owns_connection:false,hardware:null};
  f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  assert.equal(f.posts.length,0);assert.equal(f.doc.getElementById('joint-follow-support').checked,true);
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  const requestFailure=f.p.request;
  f.p.request=async(url,opts)=>opts.body&&JSON.parse(opts.body).action==='start'
    ?{ok:false,json:async()=>({error:'启动结果未确认'})}:requestFailure(url,opts);
  await f.p.start();assert.equal(f.doc.getElementById('joint-follow-support').checked,false);
  assert.equal(f.p.active,false);
  // Unchecked, unauthenticated, offline and malformed responses all provide
  // a visible reason; no silent button or failed request can arm the robot.
  f=fixture();await f.p.start();assert.match(f.doc.getElementById('joint-follow-message').textContent,/请先勾选/);
  assert.equal(f.doc.getElementById('joint-follow-diagnostics').scrollTop,0);
  assert.equal(f.doc.getElementById('joint-follow-message').scrolled,undefined,'error reveals within diagnostics without scrolling the buttons');assert.equal(f.posts.length,0);
  for(const [request,pattern] of [
    [async()=>({status:401}),/页面会话已失效/],
    [async()=>{throw new TypeError('Failed to fetch');},/无法连接 X5 控制服务/],
    [async()=>({ok:false,status:503,json:async()=>({error:'控制服务暂不可用'})}),/控制服务暂不可用/],
    [async()=>({ok:false,status:502,json:async()=>{throw new SyntaxError('bad json');}}),/HTTP 502/],
  ]){
    f=fixture();f.p.request=request;await f.p.poll();
    assert.match(f.doc.getElementById('joint-follow-message').textContent,pattern);
    assert.equal(f.doc.getElementById('joint-follow-message').hidden,false);
    assert.match(f.doc.getElementById('joint-check-service').textContent,/读取失败/);
    assert.equal(f.posts.length,0);
  }

  // Recovering a restarted HTTP session is an explicit read-only action.
  f=fixture();let staleCookie=true;const freshRequest=f.p.request,urls=[];
  f.p.request=async(url,options)=>{
    urls.push(url);if(url==='/'){staleCookie=false;return {ok:true};}
    if(staleCookie)return {status:401};return freshRequest(url,options);
  };
  await f.p.poll();await f.p.refresh();
  assert.ok(urls.includes('/'));assert.equal(f.p.statusError,false);
  assert.equal(f.posts.length,0);assert.equal(f.p.active,false);

  // Missing motors/IMU/calibration are enumerated, with labels, before START.
  f=fixture();f.p.labels={30:'颈部俯仰'};f.state.motion.hardware.online_ids=f.state.motion.hardware.online_ids.filter(id=>id!==30);
  f.state.motion.hardware.imu_ok=false;f.state.motion.hardware.calibrated=false;
  f.p.accept(f.state);const hardwareMessage=f.doc.getElementById('joint-check-hardware').textContent;
  assert.match(hardwareMessage,/ID30 颈部俯仰/);assert.match(hardwareMessage,/IMU/);assert.match(hardwareMessage,/校准尚未完成/);

  // STOP completion must retain the trigger, so a heartbeat error does not
  // disappear behind the generic 'stopped' success message.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  await f.p.stop('page-hidden','随动心跳中断：请求超时');
  assert.match(f.doc.getElementById('joint-follow-message').textContent,/心跳中断.*请求超时.*确认卸力/);
  assert.equal(f.doc.getElementById('joint-follow-message').attributes['data-level'],'error');
  // Readiness is still visible with a cached pre-diagnostics HTML document.
  f=fixture();f.elements.delete('joint-follow-readiness');
  for(const key of ['service','connection','feedback','hardware','range','support'])f.elements.delete('joint-check-'+key);
  const oldDocumentPanel=new JointFollowPanel({ids,doc:f.doc,onFrame(){},onChange(){},disconnectViewer:async()=>{},request:f.p.request});
  oldDocumentPanel.accept(f.state);
  assert.match(f.doc.getElementById('joint-follow-readiness').textContent,/控制服务.*X5/);
  assert.match(f.doc.getElementById('joint-follow-readiness').textContent,/当前位置/);
  // Missing publications preserve the previous range reading for display,
  // while current feedback still blocks every START. Repeated readings do not
  // replace DOM text or disturb a user's scroll through the diagnostics.
  f=fixture();f.state.motion.hardware.rawdata.find(j=>j.id===30).position=999;
  f.p.accept(structuredClone(f.state));
  const rangeNode=f.doc.getElementById('joint-check-range'),rangeText=rangeNode.textContent,writes=rangeNode.textWrites;
  f.doc.getElementById('joint-follow-diagnostics').scrollTop=120;
  for(let i=0;i<10;i++){
    f.advance(200);const s=structuredClone(f.state);if(i%2)s.motion.hardware=null;
    f.p.accept(s);
    assert.equal(rangeNode.textContent,rangeText);
    assert.equal(f.doc.getElementById('joint-follow-diagnostics').scrollTop,120);
    assert.ok(followStartBlocker(s,ids),'display cache does not authorize START');
  }
  assert.equal(rangeNode.textWrites,writes,'identical content is not rewritten');
  assert.equal(f.posts.length,0);

  // Values change at most twice a second, but newly affected joints appear
  // immediately and a previous connection cannot donate cached positions.
  f.state.motion.hardware.rawdata.find(j=>j.id===30).position=998;
  f.p.accept(structuredClone(f.state));
  assert.equal(rangeNode.textContent,rangeText);
  f.advance(500);f.p.accept(structuredClone(f.state));assert.match(rangeNode.textContent,/998/);
  f.state.motion.hardware.rawdata.find(j=>j.id===33).position=999;
  f.p.accept(structuredClone(f.state));assert.match(rangeNode.textContent,/ID33/);
  const next=structuredClone(f.state);next.motion.hardware=null;next.motion.connection_sequence=2;
  f.p.accept(next);assert.doesNotMatch(rangeNode.textContent,/ID30|ID33/);
  // Incident regression: a 350 ms reply must not serialize the next lease
  // request behind that response, nor trigger the old 300 ms HTTP abort.
  const settle=async()=>{for(let i=0;i<30;i++)await Promise.resolve();};
  const latestTick=base=>timers.slice(base).filter(t=>t.ms===50).at(-1).fn;
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  let delayed=[],maxPending=0;const requestDelayed=f.p.request,baseDelayed=timers.length;
  f.p.request=async(url,opts)=>{
    if(opts.body&&JSON.parse(opts.body).action==='heartbeat'){
      f.posts.push(JSON.parse(opts.body));
      return new Promise((resolve,reject)=>{delayed.push({resolve:()=>resolve({ok:true,json:async()=>({heartbeat_accepted:true,generation:6})}),reject});maxPending=Math.max(maxPending,f.p.heartbeatSession.pending);});
    }
    return requestDelayed(url,opts);
  };
  await f.p.start();
  for(let i=0;i<3;i++){f.advance(100);latestTick(baseDelayed)();await settle();}
  assert.equal(delayed.length,3,'in-flight heartbeat requests stay bounded');
  assert.equal(maxPending,3);assert.equal(f.posts.filter(p=>p.action==='stop').length,0);
  f.advance(50);delayed[0].resolve();await settle();
  f.advance(50);latestTick(baseDelayed)();await settle();
  assert.equal(delayed.length,4,'next heartbeat is sent on cadence after slow reply');
  assert.equal(f.p.active,true);assert.equal(f.posts.filter(p=>p.action==='start').length,1);
  await f.p.stop();delayed.slice(1).forEach(p=>p.resolve());await settle();
  assert.equal(f.p.active,false,'late heartbeats cannot reactivate a stopped session');

  // A transient failed HTTP response does not cancel another valid heartbeat.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  delayed=[];const requestTransient=f.p.request,baseTransient=timers.length;
  f.p.request=async(url,opts)=>{
    if(opts.body&&JSON.parse(opts.body).action==='heartbeat'){
      f.posts.push(JSON.parse(opts.body));
      return new Promise((resolve,reject)=>delayed.push({resolve:()=>resolve({ok:true,json:async()=>({heartbeat_accepted:true,generation:6})}),reject}));
    }return requestTransient(url,opts);
  };
  await f.p.start();f.advance(100);latestTick(baseTransient)();
  delayed[0].reject(Object.assign(Error('response timeout'),{name:'AbortError'}));await settle();
  delayed[1].resolve();await settle();
  assert.equal(f.p.active,true);assert.equal(f.posts.some(p=>p.action==='stop'),false);
  await f.p.stop();

  // When every heartbeat is lost, X5's watchdog may already have stopped
  // before the implicit STOP arrives. Read back proof instead of retaining
  // the misleading "stopping is forbidden" / "result unconfirmed" alarm.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  const requestLost=f.p.request,baseLost=timers.length;
  f.p.request=async(url,opts)=>{
    const body=opts.body&&JSON.parse(opts.body);
    if(body?.action==='heartbeat'){f.posts.push(body);return new Promise(()=>{});}
    if(body?.action==='stop'){f.posts.push(body);return {ok:false,json:async()=>({error:'页面切换或关闭不再停止运控；只有明确的停止命令可执行'})};}
    return requestLost(url,opts);
  };
  await f.p.start();f.advance(650);
  Object.assign(f.state.motion.hardware,{mode:'off',generation:7,armed:false,release_confirmed:true});f.state.motion.hardware.joint_follow.active=false;
  latestTick(baseLost)();await settle();
  assert.equal(f.p.active,false);assert.equal(f.p.stopPending,null);
  assert.match(f.doc.getElementById('joint-follow-message').textContent,/心跳.*确认卸力/);
  assert.doesNotMatch(f.doc.getElementById('joint-follow-message').textContent,/只有明确的停止命令/);
  assert.match(f.doc.getElementById('joint-follow-state').textContent,/已确认卸力/);

  // Never infer torque release from a rejected request, an old publication,
  // another connection or a restarted service. Block a new START until proof.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;await f.p.start();
  const requestUnconfirmed=f.p.request;
  f.p.request=async(url,opts)=>opts.body&&JSON.parse(opts.body).action==='stop'
    ?{ok:false,json:async()=>({error:'STOP reply unavailable'})}:requestUnconfirmed(url,opts);
  await f.p.stop('page-hidden','测试失联');
  assert.ok(f.p.stopPending);assert.match(f.doc.getElementById('joint-follow-state').textContent,/等待卸力确认/);
  const starts=f.posts.filter(p=>p.action==='start').length;await f.p.start();
  assert.equal(f.posts.filter(p=>p.action==='start').length,starts);
  Object.assign(f.state.motion.hardware,{mode:'off',generation:7,armed:false});f.state.motion.hardware.joint_follow.active=false;
  f.p.accept(structuredClone(f.state));assert.ok(f.p.stopPending,'off alone is insufficient');
  f.state.motion.hardware.release_confirmed=true;f.state.motion.hardware.age_ms=700;
  f.p.accept(structuredClone(f.state));assert.ok(f.p.stopPending,'stale release observation is insufficient');
  f.state.motion.hardware.age_ms=1;f.state.native_access.page_generation=8;
  f.p.accept(structuredClone(f.state));assert.ok(f.p.stopPending,'new service is not old STOP proof');
  f.state.native_access.page_generation=7;f.state.motion.connection_sequence=2;
  f.p.accept(structuredClone(f.state));assert.ok(f.p.stopPending,'new connection is not old STOP proof');
  f.state.motion.connection_sequence=1;f.state.motion.hardware.generation=5;
  f.p.accept(structuredClone(f.state));assert.ok(f.p.stopPending,'old generation is not STOP proof');
  f.state.motion.hardware.generation=7;f.p.accept(structuredClone(f.state));
  assert.equal(f.p.stopPending,null);assert.match(f.doc.getElementById('joint-follow-message').textContent,/确认卸力/);

  // Actual X5 IPC contract: overlap is rejected with HTTP 409 without being
  // dispatched. Keep the independently in-flight heartbeat and bounded ack
  // deadline; do not confuse lane contention with owner/lease rejection.
  f=fixture();f.doc.getElementById('joint-follow-support').checked=true;
  const overlapRequest=f.p.request;let heartbeatCalls=0;
  f.p.request=async(url,opts)=>{
    if(opts.body&&JSON.parse(opts.body).action==='heartbeat'&&heartbeatCalls++===0)
      return {ok:false,status:409,json:async()=>({error:'当前操作尚未完成；请求未排队、未重发'})};
    return overlapRequest(url,opts);
  };
  const baseOverlap=timers.length;
  await f.p.start();await settle();assert.equal(f.p.active,true);
  f.advance(100);latestTick(baseOverlap)();await settle();
  assert.equal(f.p.active,true);assert.equal(f.posts.some(p=>p.action==='stop'),false);
  console.log('Passed browser-control regressions, including the production START/display-gap trace, delayed heartbeats, bounded dispatch and observed torque release.');
}finally{globalThis.setTimeout=originalTimeout;globalThis.clearTimeout=originalClear;}
