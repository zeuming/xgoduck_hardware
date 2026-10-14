// Native controller remains the only UART owner. No automatic connect/rearm.
export function followFrame(snapshot, ids) {
  const h=snapshot?.motion?.hardware, p=h?.joint_follow;
  if(!h?.connected||![h.age_ms,h.imu_age_ms].every(v=>Number.isFinite(v)&&v>=0&&v<600))return null;
  const measured=new Map((h.rawdata||[]).map(r=>[r.id,r.angle_deg]));
  const targets=new Map((p?.joints||[]).map(r=>[r.id,r]));
  if(!ids.every(id=>Number.isFinite(measured.get(id))))return null;
  return {q:ids.map(id=>measured.get(id)),target:p?.active?ids.map(id=>targets.get(id)?.requested_deg):null,hardware:h};
}

// Connection ownership survives a missing display publication. A historical
// packet may explain the UI, but must never authorize START or a motor target.
export function followConnection(snapshot) {
  const m=snapshot?.motion;
  return !!m&&(m.owns_connection===true||m.phase==='ready'||m.phase==='deploying');
}

export function followChecks(snapshot, ids, labels={}) {
  const m=snapshot?.motion,h=m?.hardware,connected=followConnection(snapshot),fresh=!!followFrame(snapshot,ids);
  const checks=[];
  const add=(key,label,level,detail,blocking=level==='fail')=>checks.push({key,label,level,detail,blocking});
  const named=id=>`ID${id}${labels[id]?' '+labels[id]:''}`;
  add('service','控制服务',snapshot?.native_access?.joint_follow_protocol===1?'pass':'fail',
    !snapshot?'尚未读取到 X5 状态，请点击「刷新状态」。':snapshot.native_access?.joint_follow_protocol!==1
    ?'此服务没有关节随动接口，请从 X5 的 /sim/ 入口打开。':'X5 控制接口可用');
  let connectionProblem=snapshot?.maintenance?.owns_connection?'舵机维护正在占用连接，请先在实机工作台退出维护。':
    !connected?(m?.error&&!m.startup_fault?`连接未就绪：${m.error}`:snapshot?.session?.status==='active'
      ?'当前为只读反馈采集。点击「连接实机」切换到关节控制；连接不会上扭矩。':'尚未连接硬件。请点击「连接实机」；连接不会上扭矩。'):'';
  add('connection','实机连接',connectionProblem?'fail':'pass',connectionProblem||'已连接，控制服务持有连接');
  const age=v=>Number.isFinite(v)?`${Math.round(v)} ms`:'未收到';
  const missing=ids.filter(id=>!h?.rawdata?.some(j=>j.id===id&&Number.isFinite(j.angle_deg)));
  add('feedback','实时反馈',fresh?'pass':connected?'fail':'wait',fresh?`关节 ${age(h.age_ms)} · IMU ${age(h.imu_age_ms)}`:
    !connected?'连接后检查':h?`反馈不可用：关节 ${age(h.age_ms)}，IMU ${age(h.imu_age_ms)}${missing.length?'；缺少 '+missing.map(named).join('、'):''}。请检查 USB 连接和舵机供电。`:
    `尚未收到本轮有效反馈${Number.isFinite(m?.sample_age_ms)?`（距今 ${age(m.sample_age_ms)}）`:''}。点击「刷新状态」重查；仍无反馈请检查 USB 连接和舵机供电。`);
  const hardwareProblems=[];
  if(fresh){
    const offline=ids.filter(id=>!h.online_ids?.includes(id));
    if(offline.length)hardwareProblems.push('未在线：'+offline.map(named).join('、'));
    if(h.fault_ids?.length)hardwareProblems.push('故障：'+h.fault_ids.map(named).join('、'));
    if(h.mode!=='off'||h.armed||h.torque_ids?.length)hardwareProblems.push('尚未全部卸力，请先停止当前动作');
    if(h.pending_stop)hardwareProblems.push('停止确认尚未完成');
    if(h.imu_ok!==true)hardwareProblems.push('IMU 状态无效');
    if(h.calibrated!==true)hardwareProblems.push('机械零点、关节范围/方向或 IMU 校准尚未完成，请到三维校准核对');
    if(h.firmware_error)hardwareProblems.push('下位机故障码：'+h.firmware_error);
  }
  add('hardware','舵机与校准',!fresh?'wait':hardwareProblems.length?'fail':'pass',!fresh?'收到反馈后检查':hardwareProblems.join('；')||`${ids.length}/${ids.length} 在线 · 已卸力 · 校准完整`);
  const positions=new Map((h?.rawdata||[]).map(j=>[j.id,j.position]));
  const calibration=new Map((snapshot?.calibration?.joints||[]).map(j=>[j.id,j]));
  const outside=[];
  for(const id of fresh?ids:[]){
    const j=calibration.get(id),position=positions.get(id);
    if(!j||![position,j.min,j.max].every(Number.isFinite)||j.min>j.max){outside.push(`${named(id)}：关节位置或已保存范围不完整`);continue;}
    if(position<j.min||position>j.max)outside.push(`${named(id)} 当前位置 ${position}，已保存范围 ${j.min}–${j.max}（编码器刻度）`);
  }
  add('range','当前位置',!fresh?'wait':outside.length?'fail':'pass',!fresh?'收到反馈后检查':outside.length
    ?outside.join('；')+'。确认卸力并托稳后移回已保存范围；若范围记录有误，请到「三维校准」核对。':`${ids.length} 个关节均在已保存范围内`);
  return checks;
}

export function followStartBlocker(snapshot,ids,labels) {
  return followChecks(snapshot,ids,labels).filter(c=>c.blocking).map(c=>c.detail).join('\n');
}

function updateText(node,text){if(node&&node.textContent!==text)node.textContent=text;}
function updateAttribute(node,key,value){if(node?.getAttribute?.(key)!==value)node?.setAttribute?.(key,value);}
function transientError(message){return Object.assign(Error(message),{transient:true});}
// Two independent 200 ms publishers feed the display. Their occasional empty
// packet is not a controller disconnect. During a bounded display gap, freeze
// targets while the controller retains its independent page lease/sensor gates.
const DISPLAY_GAP_MS=1500;

// The model and DOM share the page event loop. Keep the short native lease
// independent of a slow WebGL frame, but renew only while fresh observations
// from this exact page/session keep arriving. This worker never starts motors
// or sends joint targets; all native ownership and hardware gates stay in force.
export function followHeartbeatWorker(){
  let session,live=false,pending=0,lastAck=0,lastFrame=0,lastError='';
  const fault=message=>{if(live){live=false;self.postMessage({type:'fault',message});}};
  const tick=()=>{
    if(!live)return;
    const now=performance.now();
    if(now-lastFrame>=session.displayGapMs){fault('画面反馈持续中断，已暂停关节发送');return;}
    if(now-lastAck>=650){fault('随动控制心跳未得到确认'+(lastError?'：'+lastError:''));return;}
    if(pending<3){
      pending++;
      const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1000);
      fetch(session.url,{method:'POST',credentials:'same-origin',cache:'no-store',signal:abort.signal,
        headers:{'Content-Type':'application/json','X-CSRF-Token':session.token},body:JSON.stringify(session.body)})
        .then(async response=>{
          const result=await response.json();
          if(!live)return;
          if(response.status===409&&['当前操作尚未完成；请求未排队、未重发','心跳回复延迟；正在核对原请求，未重发'].includes(result.error)){lastError='上一条心跳正在处理';return;}
          if(!response.ok||result.error){fault('随动心跳被拒绝：'+(result.error||`HTTP ${response.status}`));return;}
          if(result.heartbeat_accepted!==true||result.generation!==session.body.generation){fault('心跳确认与本轮控制会话不匹配');return;}
          lastAck=performance.now();lastError='';
        }).catch(e=>{lastError=e.name==='AbortError'?'X5 请求超时，结果未确认':'无法连接 X5 控制服务';})
        .finally(()=>{clearTimeout(timer);pending--;});
    }
    if(live)setTimeout(tick,50);
  };
  self.onmessage=({data})=>{
    if(data.type==='start'&&!session){session=data;live=true;lastAck=lastFrame=performance.now();tick();}
    else if(data.type==='frame'&&live)lastFrame=performance.now();
  };
}

function createFollowHeartbeatWorker(){
  if(typeof Worker==='undefined'||typeof location==='undefined')return null;
  const url=URL.createObjectURL(new Blob([`(${followHeartbeatWorker.toString()})();`],{type:'text/javascript'}));
  try{
    const worker=new Worker(url);
    return {worker,dispose(){worker.terminate();URL.revokeObjectURL(url);}};
  }catch(e){URL.revokeObjectURL(url);throw e;}
}

export class JointFollowPanel {
  constructor({ids,labels={},onFrame,onChange,disconnectViewer,request=(...args)=>fetch(...args),doc=document,clock=()=>performance.now(),heartbeatWorkerFactory=createFollowHeartbeatWorker}){
    Object.assign(this,{ids,labels,onFrame,onChange,disconnectViewer,request,doc,clock,heartbeatWorkerFactory});
    this.client=Array.from(crypto.getRandomValues(new Uint8Array(16)),v=>v.toString(16).padStart(2,'0')).join('');
    this.active=false;this.entered=false;this.busy=false;this.epoch=0;this.sequence=0;this.pending=null;
    this.speed=5;this.lastTarget=null;
    this.displayChecks=new Map();
    this.$=id=>doc.getElementById(id);
    // A cached document must not silently drop diagnostics from newer JS.
    if(!this.$('joint-follow-readiness')){
      const node=doc.createElement('div');node.id='joint-follow-readiness';node.className='follow-readiness';
      this.$('joint-follow-state').after(node);
    }
    this.$('joint-follow-connect').onclick=()=>this.connect();
    this.$('joint-follow-start').onclick=()=>this.start();
    this.$('joint-follow-stop').onclick=()=>this.stop('button');
    if(this.$('joint-follow-details-stop'))this.$('joint-follow-details-stop').onclick=()=>this.stop('button');
    if(this.$('joint-follow-refresh'))this.$('joint-follow-refresh').onclick=()=>this.refresh();
    this.$('joint-follow-support').onchange=()=>this.render();
    const speed=this.$('joint-follow-speed');
    if(speed){
      speed.oninput=()=>updateText(this.$('joint-follow-speed-value'),`${speed.value}°/秒`);
      speed.onchange=()=>this.setSpeed(Number(speed.value));
    }
    doc.addEventListener('visibilitychange',()=>{if(doc.hidden)this.stop('page-hidden');});
    globalThis.addEventListener?.('pagehide',()=>this.stop('pagehide'));
    this.render();
  }
  async api(body,timeout=2000){
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),timeout);
    try{
      const response=await this.request(body?'/api/motion':'/api/live-status',{credentials:'same-origin',cache:'no-store',signal:abort.signal,
        ...(body?{method:'POST',keepalive:body.action==='stop',headers:{'Content-Type':'application/json','X-CSRF-Token':this.snapshot?.token||''},body:JSON.stringify(body)}:{})});
      if(response.status===404&&!body){const error=Error('真机随动需从 X5 的 /sim/ 入口打开，并安装对应控制接口。');error.unavailable=true;throw error;}
      if(response.status===401)throw Error('页面会话已失效，控制服务可能已重启。点击「刷新状态」重新建立网页会话，再连接实机。');
      if(response.status===403)throw Error('页面会话或请求校验已变化。点击「刷新状态」重查；本次操作未确认。');
      let result;try{result=await response.json();}catch{throw Error(`服务返回无法识别的内容（HTTP ${response.status}），请刷新状态或打开实机工作台检查。`);}
      // The isolated X5 heartbeat lane deliberately rejects overlap without
      // queuing it. This is not an owner/lease rejection; another in-flight
      // heartbeat must still acknowledge within the existing bounded deadline.
      if(body?.action==='heartbeat'&&['当前操作尚未完成；请求未排队、未重发','心跳回复延迟；正在核对原请求，未重发'].includes(result.error))throw transientError('上一条心跳正在处理');
      if(!response.ok||result.error)throw Error(result.error||`服务请求失败（HTTP ${response.status}），请刷新状态重查。`);return result;
    }catch(e){if(e.name==='AbortError')throw transientError('X5 请求超时，结果未确认；正在核对设备状态。');
      if(e instanceof TypeError)throw transientError('无法连接 X5 控制服务，正在等待设备状态恢复。');throw e;}
    finally{clearTimeout(timer);}
  }
  body(action,extra={}){return {action,page_generation:this.snapshot?.native_access?.page_generation,
    generation:this.generation??this.snapshot?.motion?.hardware?.generation,control_id:this.client,...extra};}
  error(text='',level='error'){
    this.notice=text;this.noticeLevel=level;
    const node=this.$('joint-follow-message');updateText(node,text);node.hidden=!text;
    updateAttribute(node,'data-level',level);
    const detail=this.$('joint-follow-message-detail');
    if(detail){updateText(detail,text);detail.hidden=!text;updateAttribute(detail,'data-level',level);}
  }
  explain(text){
    this.error(text);this.render();
    const diagnostics=this.$('joint-follow-diagnostics');
    if(diagnostics)diagnostics.scrollTop=0;
    else this.$('joint-follow-message').scrollIntoView?.({block:'nearest',behavior:'smooth'});
  }
  async refresh(){
    if(this.busy)return;
    if(this.active){this.explain('随动进行中，反馈会自动刷新。请先停止随动，再重新建立网页会话。');return;}
    this.busy=true;this.phaseLabel='正在刷新状态…';this.error();this.render();
    try{
      // Explicit user action renews only the HTTP session. It never acquires
      // UART ownership, restores a control lease or replays START/TARGET.
      const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),3000);
      try{await this.request('/',{credentials:'same-origin',cache:'no-store',signal:abort.signal});}finally{clearTimeout(timer);}
      this.accept(await this.api());this.unavailable=false;this.error('状态已刷新；没有启动实机。','success');
    }catch(e){this.statusError=true;this.explain(e instanceof TypeError?'无法连接 X5 控制服务，请检查设备网络及 8767 服务。':e.name==='AbortError'?'刷新状态超时，请检查设备网络及 8767 服务。':e.message);}
    finally{this.busy=false;this.render();if(this.entered)this.poll();}
  }
  enter(){this.entered=true;this.poll();}
  leave(){this.entered=false;this.stop('workspace-leave');}
  waitForFeedback(){
    this.feedbackWaiting=true;this.render();
  }
  accept(s){
    const h=s.motion?.hardware,owner=s.motion?.control_owner;
    if(this.active&&(s.native_access?.page_generation!==this.sessionPage
      ||s.motion?.connection_sequence!==this.sessionConnection)){
      this.active=false;this.pending=null;this.epoch++;this.feedbackWaiting=false;
      this.endHeartbeat();
      this.$('joint-follow-support').checked=false;
      this.error('控制服务或硬件连接已变化，已停止发送关节目标；本页不会恢复旧会话。');
    }
    // The independent display publisher can briefly lag an accepted START.
    // Old snapshots must neither cancel the new session nor freshen its age.
    if(this.active&&Number.isInteger(h?.generation)&&h.generation<this.generation)return;
    this.snapshot=s;this.received=this.clock();this.statusError=false;
    this.confirmObservedStop(s);
    if(this.active&&!h){
      if(followConnection(s)&&!s.motion?.error){this.waitForFeedback();return;}
      this.stop('page-hidden',s.motion?.error||'实机连接已关闭');return;
    }
    if(this.active&&(!h?.joint_follow?.active||h.generation!==this.generation||owner?.id!==this.client)){
      const reason=h?.joint_follow?.reason||h?.error||s.motion?.error||s.motion?.last_operation?.error
        ||(owner?.id!==this.client?'本页已失去当前动作的控制权':'控制会话已变化');
      const context=this.stopContext('随动已结束：'+reason);
      this.active=false;this.pending=null;this.epoch++;this.stopPending=context;this.feedbackWaiting=false;
      this.endHeartbeat();
      this.error(context.cause+'；正在核对卸力状态。');
      this.$('joint-follow-support').checked=false;
      this.confirmObservedStop(s);
    }
    const frame=followFrame(s,this.ids);
    if(this.active&&!frame){this.waitForFeedback();return;}
    if(this.active&&frame){this.feedbackAt=this.clock();this.feedbackWaiting=false;this.heartbeatWorker?.worker.postMessage({type:'frame'});this.flush();this.onFrame(frame);}
    this.render();
  }
  async poll(){
    if(!this.entered||this.polling)return;
    this.polling=true;const epoch=this.epoch;
    try{const s=await this.api();if(this.entered&&!this.busy&&epoch===this.epoch){this.accept(s);this.unavailable=s.native_access?.joint_follow_protocol!==1;}}
    catch(e){if(this.entered){if(this.active&&e.transient)this.waitForFeedback();else if(this.active)this.stop('page-hidden',e.message);else this.error(e.message);this.unavailable=e.unavailable===true;this.statusError=true;this.render();}}
    finally{this.polling=false;if(this.entered&&!this.unavailable)setTimeout(()=>this.poll(),this.active&&this.feedbackWaiting?100:200);}
  }
  async connect(){
    if(this.busy||this.active||this.stopPending)return;
    this.busy=true;this.phaseLabel='正在连接实机…';this.epoch++;this.error();this.render();
    try{
      await this.disconnectViewer();const s=await this.api();this.accept(s);
      if(s.native_access?.joint_follow_protocol!==1)throw Error('X5 服务尚未安装关节随动控制接口。');
      if(followConnection(s)){this.error('实机连接已建立；逐项检查结果可在「提示与检查详情」查看。','info');return;}
      if(s.maintenance?.owns_connection)throw Error('请先在实机工作台退出舵机维护。');
      // The native connect action owns the transition from read-only feedback
      // to motion; end_for_motion waits for the reader's UART closure first.
      const result=await this.api(this.body('connect'),10000);
      if(result.motion&&result.native_access)this.accept(result);
      else this.accept(await this.api());
      this.error('连接请求已完成；尚未启用随动，请查看逐项检查结果。','info');
    }catch(e){this.explain('连接失败：'+(e instanceof TypeError?'无法连接 X5 服务，请检查设备网络及 8767 服务，再点击「刷新状态」。':e.message));}finally{this.busy=false;this.render();}
  }
  async start(){
    if(this.busy||this.active||this.stopPending)return;
    if(!this.$('joint-follow-support').checked){this.explain('未启用：请先勾选「机身已托稳、活动间隙已确认」。'+(followStartBlocker(this.snapshot,this.ids,this.labels)?'\n同时请在「提示与检查详情」处理未通过的检查项。':''));return;}
    this.busy=true;this.phaseLabel='正在检查启用条件…';const epoch=++this.epoch;let startSent=false;this.pending=null;this.lastTarget=null;this.stopConfirmed=null;this.error();this.render();
    try{
      let s=await this.api();this.accept(s);
      // GET reads an independent publisher; wait briefly for a current frame
      // instead of treating one missing publication as a disconnected robot.
      const deadline=this.clock()+1800;
      while(epoch===this.epoch&&this.entered&&!this.doc.hidden&&followConnection(s)
          &&!followFrame(s,this.ids)&&this.clock()<deadline){
        await new Promise(resolve=>setTimeout(resolve,100));
        s=await this.api();this.accept(s);
      }
      if(epoch!==this.epoch||!this.entered||this.doc.hidden)return;
      const h=s.motion?.hardware;
      if(s.native_access?.joint_follow_protocol!==1)throw Error('X5 服务尚未安装关节随动控制接口。');
      const blocker=followStartBlocker(s,this.ids,this.labels);
      if(blocker){this.explain('未启用：'+blocker);return;}
      this.generation=h.generation;
      this.phaseLabel='正在请求启用，请保持托稳…';this.render();
      startSent=true;
      const result=await this.api(this.body('start',{mode:'hold',joint_follow:true,support_confirmed:true,clearance_confirmed:true}),8000);
      this.generation=result.generation??result.motion?.hardware?.generation;
      if(!Number.isInteger(this.generation))throw Error('启用结果未确认，请检查原工作台状态。');
      if(epoch!==this.epoch||!this.entered||this.doc.hidden){await this.api(this.body('stop',{stop_source:'page-hidden'}));return;}
      this.active=true;this.sequence=0;this.received=this.feedbackAt=this.clock();this.feedbackWaiting=true;
      this.sessionPage=s.native_access.page_generation;this.sessionConnection=s.motion.connection_sequence;
      this.heartbeat();
      try{this.accept(await this.api());}catch(e){if(e.transient)this.waitForFeedback();else await this.stop('page-hidden',e.message);}
    }catch(e){this.active=false;this.pending=null;this.endHeartbeat();if(startSent)this.$('joint-follow-support').checked=false;this.explain('启用失败：'+e.message);}
    finally{this.busy=false;this.render();}
  }
  heartbeat(){
    if(!this.active||this.heartbeatSession?.epoch===this.epoch)return;
    const session={epoch:this.epoch,pending:0,lastAck:this.clock(),lastError:''};
    this.heartbeatSession=session;
    const current=()=>this.active&&session.epoch===this.epoch;
    let transport;
    try{transport=this.heartbeatWorkerFactory();}catch(e){this.stop('page-hidden','无法建立独立控制心跳：'+e.message);return;}
    if(transport){
      this.heartbeatWorker=transport;
      updateAttribute(this.$('joint-follow-state'),'data-heartbeat-transport','worker');
      transport.worker.onmessage=({data})=>{if(current()&&data.type==='fault')this.stop('page-hidden',data.message);};
      transport.worker.onerror=()=>{if(current())this.stop('page-hidden','独立控制心跳异常，已暂停关节发送');};
      transport.worker.postMessage({type:'start',url:new URL('/api/motion',location.href).href,
        token:this.snapshot?.token||'',body:this.body('heartbeat',{background:false}),displayGapMs:DISPLAY_GAP_MS});
      return;
    }
    updateAttribute(this.$('joint-follow-state'),'data-heartbeat-transport','page');
    const tick=()=>{
      if(!current())return;
      if(this.doc.hidden||!this.entered){this.stop('page-hidden','页面已隐藏或离开关节调试');return;}
      if(this.clock()-this.feedbackAt>=DISPLAY_GAP_MS){this.stop('page-hidden','画面反馈持续中断，已暂停关节发送');return;}
      if(this.clock()-session.lastAck>=650){this.stop('page-hidden','随动控制心跳未得到确认'+(session.lastError?'：'+session.lastError:''));return;}
      // Give control delivery priority over the 200 ms display publisher.
      // Dispatch independently of replies, with at most three requests in flight.
      if(session.pending<3){
        session.pending++;
        this.api(this.body('heartbeat',{background:false}),1000).then(result=>{
          if(!current())return;
          if(result.heartbeat_accepted!==true||result.generation!==this.generation)throw Error('心跳确认与本轮控制会话不匹配');
          session.lastAck=this.clock();session.lastError='';
        }).catch(e=>{
          if(!current())return;
          session.lastError=e.message;
          // A delayed HTTP reply is not proof that X5 rejected the lease.
          // Owner/session errors are terminal; network failures are bounded
          // by the ack deadline and X5's independent hardware watchdog.
          if(!e.transient)this.stop('page-hidden','随动心跳被拒绝：'+e.message);
        }).finally(()=>{session.pending--;});
      }
      if(current())setTimeout(tick,50);
    };
    tick();
  }
  endHeartbeat(){this.heartbeatWorker?.dispose();this.heartbeatWorker=null;}
  stopContext(cause=''){
    return {cause,generation:this.generation??this.snapshot?.motion?.hardware?.generation,
      page:this.snapshot?.native_access?.page_generation,connection:this.snapshot?.motion?.connection_sequence};
  }
  confirmStop(context,generation){
    if(this.stopPending!==context)return;
    this.stopPending=null;this.stopConfirmed={...context,generation};
    this.error((context.cause?context.cause+'。':'')+'已停止随动并确认卸力。',context.cause?'error':'success');
  }
  confirmObservedStop(s){
    const p=this.stopPending,h=s?.motion?.hardware,m=s?.motion;
    if(!p||s.native_access?.page_generation!==p.page||m?.connection_sequence!==p.connection)return;
    const sameGeneration=v=>Number.isInteger(v)&&Number.isInteger(p.generation)&&v>=p.generation;
    if(h?.mode==='off'&&h.release_confirmed===true&&h.armed===false&&!h.torque_ids?.length
      &&!h.pending_stop&&!h.joint_follow?.active&&sameGeneration(h.generation)
      &&Number.isFinite(h.age_ms)&&h.age_ms>=0&&h.age_ms<600){this.confirmStop(p,h.generation);return;}
    const proof=m.closed_proof;
    if(!m.owns_connection&&proof?.serial_closed===true&&proof.release_confirmed===true
      &&proof.after?.mode==='off'&&sameGeneration(proof.after.generation))this.confirmStop(p,proof.after.generation);
  }
  async observeStop(context,attempt=0){
    if(this.stopPending!==context)return;
    try{this.accept(await this.api());}catch(e){context.error=context.error||e.message;}
    if(this.stopPending!==context)return;
    this.error((context.cause?context.cause+'。':'')+'停止待确认，已暂停关节发送；正在自动核对 X5 卸力状态。'+(context.error?' '+context.error:''));
    this.render();
    if(!this.entered&&attempt<8)setTimeout(()=>this.observeStop(context,attempt+1),200);
  }
  async stop(source='button',cause=''){
    if(source!=='button'&&!this.active&&!this.busy)return;
    const context=this.stopContext(cause);this.stopPending=context;this.stopConfirmed=null;
    this.active=false;this.pending=null;this.epoch++;this.$('joint-follow-support').checked=false;
    this.endHeartbeat();
    this.feedbackWaiting=false;
    this.error((cause?cause+'；':'')+'正在请求停止并核对卸力。',cause?'error':'info');this.render();
    try{
      const result=await this.api(this.body('stop',{stop_source:source}),2500);
      if(result.action_receipt_protocol===1&&result.action==='stop'&&result.mode==='off'
        &&result.release_confirmed===true&&result.page_generation===context.page
        &&result.connection_sequence===context.connection
        &&Number.isInteger(result.generation)&&result.generation>=context.generation)this.confirmStop(context,result.generation);
    }catch(e){context.error=e.message;}
    await this.observeStop(context);this.render();
  }
  target(id,angle){
    if(!this.active||this.busy||this.feedbackWaiting||!this.joint(id))return false;
    this.lastTarget={joint_id:id,angle_deg:angle};
    this.pending={...this.lastTarget,speed_deg_s:this.speed};this.flush();return true;
  }
  setSpeed(speed){
    if(!Number.isInteger(speed)||speed<1||speed>50)return false;
    this.speed=speed;
    // Only change the rate of the user's existing target. No START or new
    // joint angle is inferred when a speed is chosen before slider input.
    if(this.active&&this.lastTarget){
      this.pending={...this.lastTarget,speed_deg_s:speed};this.flush();
    }
    this.render();return true;
  }
  async flush(){
    if(this.sending||!this.active||this.feedbackWaiting||!this.pending)return;
    const wait=(this.nextSendAt||0)-this.clock();
    if(wait>0){if(!this.sendTimer)this.sendTimer=setTimeout(()=>{this.sendTimer=null;this.flush();},wait);return;}
    const target=this.pending,epoch=this.epoch;this.pending=null;this.sending=true;
    this.nextSendAt=this.clock()+100;
    try{await this.api(this.body('joint-target',{...target,sequence:++this.sequence}),500);}
    catch(e){if(epoch===this.epoch)this.stop('page-hidden','目标未确认：'+e.message);}
    finally{this.sending=false;if(this.active&&epoch===this.epoch&&this.pending)setTimeout(()=>this.flush(),100);}
  }
  joint(id){return this.snapshot?.motion?.hardware?.joint_follow?.joints?.find(j=>j.id===id);}
  render(){
    const connected=followConnection(this.snapshot),checks=followChecks(this.snapshot,this.ids,this.labels);
    const h=this.snapshot?.motion?.hardware;
    if(this.active&&h?.joint_follow?.active&&h.imu_ok&&h.calibrated&&!h.fault_ids?.length&&this.ids.every(id=>h.online_ids?.includes(id))){
      const check=checks.find(c=>c.key==='hardware');Object.assign(check,{level:'pass',blocking:false,detail:`${this.ids.length}/${this.ids.length} 在线 · 随动扭矩已开启`});
    }
    if(this.statusError)checks[0]={...checks[0],level:'fail',blocking:true,detail:'状态读取失败。点击「刷新状态」重查，失败原因见上方提示。'};
    const failures=checks.filter(c=>c.blocking),confirmed=this.$('joint-follow-support').checked;
    // Display continuity only. START always rechecks the latest snapshot and
    // never consults this cache. Do not replace a long range warning with a
    // one-line placeholder each time the display publisher misses a frame.
    const context=JSON.stringify([this.snapshot?.native_access?.page_generation,this.snapshot?.motion?.connection_sequence]);
    const range=checks.find(c=>c.key==='range');
    if(followFrame(this.snapshot,this.ids))this.lastRange={context,check:{...range}};
    else if(!connected||this.lastRange?.context!==context)this.lastRange=null;
    if(connected&&this.lastRange)Object.assign(range,this.lastRange.check,{label:'当前位置（最近有效读数）'});
    this.$('joint-follow-connect').disabled=this.busy||this.active||!!this.stopPending;
    updateText(this.$('joint-follow-connect'),connected?'检查连接':'连接实机');
    // Keep precondition failures clickable: every click explains the reason.
    // Only an in-flight request or an active session disables duplicate START.
    this.$('joint-follow-start').disabled=this.busy||this.active||!!this.stopPending;
    updateText(this.$('joint-follow-start'),this.active?'随动已启用':'检查并启用随动');
    updateAttribute(this.$('joint-follow-start'),'aria-describedby','joint-follow-readiness joint-follow-message');
    this.$('joint-follow-stop').disabled=!this.snapshot?.token;
    if(this.$('joint-follow-details-stop'))this.$('joint-follow-details-stop').disabled=!this.snapshot?.token;
    if(this.$('joint-follow-refresh'))this.$('joint-follow-refresh').disabled=this.busy||this.active;
    this.$('joint-follow-support').disabled=this.busy||this.active||!!this.stopPending;
    this.$('home-pose').disabled=this.active||this.busy||!!this.stopPending;
    const speed=this.$('joint-follow-speed');
    if(speed){
      speed.disabled=this.busy||!!this.stopPending;
      if(this.doc.activeElement!==speed){speed.value=this.speed;updateText(this.$('joint-follow-speed-value'),`${this.speed}°/秒`);}
      const applied=this.active&&h?.joint_follow?.speed_deg_s===this.speed;
      updateText(this.$('joint-follow-speed-hint'),!this.active?'1–50°/秒 · 启用随动后应用于实机'
        :this.feedbackWaiting?'等待反馈恢复后应用速度':applied?'已应用于实机 · 数值越大转动越快'
        :this.lastTarget?'正在应用速度…':'下次拖动关节时应用此速度');
    }
    updateText(this.$('joint-follow-state'),this.active?(this.feedbackWaiting?'随动保持中 · 等待画面反馈':'真机随动已启用'):this.stopPending?'正在停止 · 等待卸力确认':this.busy?this.phaseLabel||'正在处理…':this.stopConfirmed?'随动已停止 · 已确认卸力':failures.length?`暂不能启用 · ${failures.length} 项待处理`:this.notice&&this.noticeLevel==='error'?'随动未启用 · 查看提示':confirmed?'已就绪 · 等待启用':'检查通过 · 等待托稳确认');
    let completeList=true;
    for(const check of checks){
      const node=this.$('joint-check-'+check.key);if(!node){completeList=false;continue;}
      const signature=check.level+'|'+(check.detail.match(/ID\d+/g)||[]).join(','),previous=this.displayChecks.get(check.key),now=this.clock();
      // Numeric readouts update at most twice a second. A new warning or
      // affected joint is immediate; control/feedback checks are not delayed.
      if(['range','feedback'].includes(check.key)&&previous?.signature===signature&&now-previous.at<500)continue;
      updateText(node,`${check.level==='pass'?'✓':check.level==='fail'?'!':'○'} ${check.label}：${check.detail}`);
      updateAttribute(node,'data-level',check.level);
      this.displayChecks.set(check.key,{signature,at:now});
    }
    const support=this.$('joint-check-support');
    if(support){updateText(support,confirmed?'✓ 托稳确认：已勾选':'○ 托稳确认：尚未勾选操作栏中的确认框');updateAttribute(support,'data-level',confirmed?'pass':'wait');}
    const readiness=this.$('joint-follow-readiness');
    updateText(readiness,this.active?(this.feedbackWaiting?'画面正在同步，暂时冻结关节目标；反馈恢复后自动继续显示。':'随动进行中，拖动下方关节滑块改变目标。'):this.stopPending?'已暂停关节发送，正在自动核对 X5 卸力状态。':completeList
      ?failures.length?'打开「提示与检查详情」查看待处理项。点击启用也会显示具体原因。':confirmed?'点击「检查并启用随动」开始。':'请托稳机身并勾选操作栏中的确认框，再启用随动。'
      :checks.map(c=>`${c.level==='pass'?'✓':'!'} ${c.label}：${c.detail}`).join('\n'));
    this.onChange();
  }
}
