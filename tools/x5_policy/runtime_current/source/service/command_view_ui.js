// Commands originate only in explicit button/input events. Rendering and GET
// completion never send motor commands, renew control, or invent success.
let commandViewSequence = 0;
const commandViewResults = [];
let commandViewReceipt = null;
const commandViewButtons = ['motionConnect','motionShadow','motionStand','motionWalk',
  'motionGetup','motionPick','motionRoulade','motionStop','motionDisconnect',
  'walkModelPreview','walkModelForwardLocked','motionZeroCommand',
  'motionForward','motionBackward','motionLeft','motionRight',
  'motionRecheck','motionImuRead','motionImuEnd'];

function commandViewResponse(value) {
  const m=value?.motion, h=m?.hardware || m?.closed_proof?.after;
  return {error:value?.error, action:value?.action, generation:value?.generation,
    mode:value?.mode ?? h?.mode, connected:value?.connected ?? h?.connected,
    release_confirmed:value?.release_confirmed ?? h?.release_confirmed,
    phase:m?.phase, operation:m?.last_operation,
    session:value?.session ? {status:value.session.status,error:value.session.error} : undefined};
}

function commandViewGeneration() {
  const current = snapshot?.motion?.hardware ?? snapshot?.motion?.last_hardware;
  const receipt = commandViewReceipt;
  if (receipt?.page_generation === motionPageGeneration()
      && receipt.connection_sequence === snapshot?.motion?.connection_sequence
      && receipt.generation >= (current?.generation ?? -1)) return receipt.generation;
  return current?.generation;
}
async function commandViewRunMotion(action, extra={}) {
  const entry = {sequence:++commandViewSequence, action, parameters:{...extra}, state:'pending'};
  commandViewResults.push(entry);
  if (commandViewResults.length > 64) commandViewResults.shift();
  const generation = commandViewGeneration();
  const pageGeneration = motionPageGeneration();
  const abort = new AbortController();
  const timer = setTimeout(()=>abort.abort(), action==='connect' ? 20000 : 5000);
  const button = nativeActiveButton;
  nativeActiveButton = null;
  renderMotion();
  try {
    const response = await fetch('/api/motion', {method:'POST', credentials:'same-origin',
      headers:{'Content-Type':'application/json','X-CSRF-Token':snapshot?.token || ''},
      signal:abort.signal, body:JSON.stringify({action, page_generation:pageGeneration,
        generation, control_id:motionControlId, ...extra})});
    const result = await response.json();
    entry.response = commandViewResponse(result);
    if (!response.ok) throw new Error(result.error || 'X5 拒绝了本次命令');
    entry.state = 'complete';
    // Keep acknowledgements distinct from asynchronously sampled telemetry.
    if (result.action_receipt_protocol === 1) {
      if (result.page_generation === motionPageGeneration()
          && (!commandViewReceipt || result.generation >= commandViewReceipt.generation))
        commandViewReceipt = result;
    } else if (result.motion && motionPageGeneration(result) === motionPageGeneration()
        && motionStateNotOlder(result.motion, snapshot?.motion)) {
      snapshotRevision++;
      acceptSnapshot(result);
    }
    nativeButtonMessage(button, 'X5 已返回执行结果');
  } catch (error) {
    entry.state = error.name === 'AbortError' ? 'unknown' : 'failed';
    entry.error = error.name === 'AbortError'
      ? '应答超时，执行结果未知；请查看后台状态和日志。命令不会自动重发。' : error.message;
    nativeButtonMessage(button, entry.error);
  } finally {
    clearTimeout(timer);
    renderMotion();
    void refresh();
  }
  return entry;
}
function motionHeartbeatTick() {}
async function motionHeartbeat() {}
function motionDriveCommand(values) {
  return runMotion('command', {forward:values[0], side:values[1], yaw:values[2]});
}
async function commandViewAux(path, body, button) {
  const entry={sequence:++commandViewSequence,action:path,parameters:body,state:'pending'};
  commandViewResults.push(entry);if(commandViewResults.length>64)commandViewResults.shift();
  const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),8000);
  renderMotion();
  try {
    const response=await fetch(path,{method:'POST',credentials:'same-origin',signal:abort.signal,
      headers:{'Content-Type':'application/json','X-CSRF-Token':snapshot?.token || ''},body:JSON.stringify(body)});
    const result=await response.json();entry.response=commandViewResponse(result);
    if(!response.ok || result.error)throw new Error(result.error || 'X5 拒绝了本次命令');
    entry.state='complete';nativeButtonMessage(button,'X5 已返回结果');
  } catch(error) {
    entry.state=error.name==='AbortError'?'unknown':'failed';
    entry.error=error.name==='AbortError'?'应答超时，结果未知；命令不会自动重发。':error.message;
    nativeButtonMessage(button,entry.error);
  } finally {clearTimeout(timer);renderMotion();void refresh();}
  return entry;
}

// Replace the legacy error wrapper too; only the matching backend receipt is shown.
runMotion=commandViewRunMotion;
const zeroCommandButton=el('motionZeroCommand');
const resultLog=el('motionCommandResults');
const commandViewModelLabels={walk:'行走',getup:'起身',pick:'俯身 / 抓取',roulade:'翻滚'};
for(const name of Object.keys(commandViewModelLabels)) {
  el('modelPreview-'+name).onclick=()=>runMotion('start',{mode:'shadow',preview_model:name});
}

el('motionConnect').onclick=()=>runMotion('connect');
el('motionShadow').onclick=()=>runMotion('start',{mode:'shadow',preview_model:el('motionPreviewModel').value || 'walk'});
el('walkModelPreview').onclick=()=>runMotion('start',{mode:'shadow',preview_model:'walk'});
el('motionStand').onclick=()=>runMotion('start',{mode:'hold'});
el('motionWalk').onclick=()=>runMotion('start',{mode:'walk'});
for(const [id,mode] of [['motionGetup','getup'],['motionPick','pick'],['motionRoulade','roulade']])
  el(id).onclick=()=>runMotion('start',{mode,behavior_confirmed:el('motionBehaviorConfirmed').checked});
el('motionStop').onclick=()=>runMotion('stop',{stop_source:'button'});
el('motionDisconnect').onclick=()=>runMotion('disconnect');
el('walkModelForwardLocked').onclick=()=>runMotion('command',{forward:.3,side:0,yaw:0});
zeroCommandButton.onclick=()=>runMotion('command',{forward:0,side:0,yaw:0});
el('motionRecheck').onclick=()=>commandViewAux('/api/check-connection',
  {generation:snapshot?.session?.generation,purpose:'joints'},el('motionRecheck'));
el('motionImuRead').onclick=()=>commandViewAux('/api/check-connection',
  {generation:snapshot?.session?.generation,purpose:'imu'},el('motionImuRead'));
el('motionImuEnd').onclick=()=>commandViewAux('/api/end-feedback',
  {generation:snapshot?.session?.generation},el('motionImuEnd'));
// Each arrow is also a one-click persistent command. There is no hidden blur,
// pointer-capture-loss or tab-navigation command. Zero is a separate button.
for(const [id,values] of [['motionForward',[.1,0,0]],['motionBackward',[-.1,0,0]],
  ['motionLeft',[0,0,.3]],['motionRight',[0,0,-.3]]]) {
  el(id).onpointerdown=el(id).onpointerup=el(id).onpointercancel=el(id).onlostpointercapture=null;
  el(id).onclick=()=>motionDriveCommand(values);
}

const commandViewRenderBase=renderMotion;
renderMotion=function() {
  commandViewRenderBase();
  for(const id of commandViewButtons) { const button=el(id); if(button) {button.disabled=false;button.title='';} }
  el('motionBehaviorConfirmed').disabled=false;el('motionPreviewModel').disabled=false;
  el('motionConnect').textContent='连接并载入模型';
  el('motionStand').textContent='进入模型默认站姿';
  el('motionWalk').textContent='启动行走模型 · 零速';
  el('motionStop').textContent='停止运动 · 释放扭矩';
  el('walkModelForwardLocked').textContent='持续前进 · 0.30 m/s';
  el('walkModelForwardReason').textContent='指令持续执行；点击「回零速」归零，点击「停止运动」释放扭矩。';
  el('walkModelTestGuide').textContent='本页发送命令并异步显示 X5 状态；执行条件和结果由 X5 返回。';
  el('motionWalkGateStatus').textContent='';
  el('motionBalanceHint').textContent='X5 本地服务维持已启动的运控。切换标签、关闭网页或暂停画面更新不发送停止命令。';
  const m=snapshot?.motion, hardware=m?.hardware ?? m?.last_hardware;
  const connected=!fetchError && m?.phase==='ready' && m?.owns_connection===true
    && m?.hardware===hardware && hardware?.connected===true
    && Number.isFinite(hardware.age_ms) && hardware.age_ms>=0 && hardware.age_ms<500;
  const imuUnavailable=connected && (hardware.imu_ok!==true
    || !Number.isFinite(hardware.imu_age_ms) || hardware.imu_age_ms>=500);
  const missingPositions=connected&&hardware.mode==='off'?motorIds.filter(id=>!hardware.online_ids?.includes(id)):[];
  el('motionConnect').textContent=connected?'已连接 · 模型已载入':'连接并载入模型';
  el('motionConnect').title=connected?'点击刷新当前连接结果，不重复连接串口、不启动动作':'';
  el('motionStatusTitle').textContent=connected
    ?(missingPositions.length?'运控已连接 · 部分位置反馈缺失':imuUnavailable?'运控已连接 · IMU 反馈未就绪':'运控已连接')
    :m?.owns_connection?'运控占用连接 · 等待状态':'运控未连接';
  const loaded=m?.models_loaded || Object.keys(m?.deployment?.model_hashes || {});
  for(const [name,label] of Object.entries(commandViewModelLabels)) {
    const running=m?.owns_connection===true && hardware?.connected===true
      && hardware?.mode!=='off' && hardware?.active_model===name;
    const preview=running && hardware.mode==='shadow';
    el('modelCard-'+name).dataset.running=String(running);
    el('modelState-'+name).textContent=running
      ?(preview?'不动作预览：':'最近运行反馈：')+label+
        (Number.isFinite(hardware.behavior_seconds_remaining)?' · 剩余约 '+hardware.behavior_seconds_remaining.toFixed(1)+' 秒':'')
      :loaded.includes(name)?'模型已加载 · 等待显式启动':'模型尚未加载';
  }
  const failure=m?.last_operation?.state==='failed' ? m.last_operation.error : '';
  const serverError=failure || m?.error || hardware?.error || '';
  const startRejected=connected && !!failure && m.last_operation.action==='start';
  if(startRejected)el('motionStatusTitle').textContent='运控已连接 · 动作启动未通过';
  el('motionStatusMessage').textContent=missingPositions.length
    ?'已连接并载入模型；ID '+missingPositions.join('、')+' 位置反馈缺失，动作仍被阻止。已确认扭矩关闭时，可保存其他关节的有效越界读数。'
    :imuUnavailable
    ?'舵机连接已建立；IMU 没有新鲜有效数据，模型动作仍由 X5 的实时检查阻止。'+(serverError?' '+serverError:'')
    :startRejected?'连接仍然保持，本次动作未启动：'+failure+'。停止并确认扭矩全关后可以设置，无需重新载入或启动模型。'
    :serverError || (connected?'连接已建立，模型已载入；连接不会启动动作。停止并确认扭矩全关后可以设置，无需再次载入。':'实时状态与日志由 X5 异步发布。');
  const last=commandViewResults[commandViewResults.length-1];
  el('motionActionResult').hidden=!last;
  if(last) {
    el('motionActionResult').textContent='命令 #'+last.sequence+' '+last.action+'：'+
      (last.state==='pending'?'等待 X5 结果':last.error || 'X5 已返回结果');
    el('motionActionResult').classList.toggle('error',last.state==='failed'||last.state==='unknown');
  }
  resultLog.textContent=commandViewResults.slice(-8).map(row=>JSON.stringify(row)).join('\n');
};
