#!/usr/bin/env python3
"""X5-native adapter with explicit password or anonymous browser sessions.

The runtime owns the sole UART and implements the hardware/health guards. This
module has no SSH or serial implementation; it adds LAN authentication, HTTP
framing, page/control leases, and the established calibration workflows.
"""
import copy
from contextlib import nullcontext
from http.cookies import CookieError, SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import socket
import stat
import threading
import time

HOST = '192.168.1.11'
PORT = 8767
ORIGIN = 'http://192.168.1.11:8767'
COOKIE = 'duck_native_session'
READ_ONLY = 'X5 原生工作台仅提供运控；已保存校准可查看和导出，采集与舵机维护暂不可用。'
MODEL_NAMES = ('walk', 'getup', 'pick', 'roulade')
STOP_SOURCES = ('button', 'page-hidden', 'pagehide', 'workspace-leave', 'window-blur')
MAX_REQUEST = 4096
MAX_RESPONSE = 2 * 1024 * 1024
MAX_HEADERS = 16384
COOKIE_SECONDS = 12 * 60 * 60
MAX_SESSIONS = 64


def strict_json(raw):
    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError('重复的 JSON 字段')
            value[key] = item
        return value
    def invalid(_value):
        raise ValueError('JSON 数值无效')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=invalid)


def private_secret(path):
    """Read a deployment-created private credential without printing it."""
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_mode & 0o077:
            raise ValueError('工作台访问口令文件必须由当前用户私有保存')
        if not 32 <= info.st_size <= 1024:
            raise ValueError('工作台访问口令文件大小无效')
        value = strict_json(os.read(fd, 1025))
    finally:
        os.close(fd)
    password = value.get('password') if type(value) is dict else None
    if type(password) is not str or not re.fullmatch(r'[A-Za-z0-9_-]{32,128}', password):
        raise ValueError('工作台访问口令必须为部署时生成的随机口令')
    return password


READ_ONLY_SCRIPT = r'''
// Native X5 service owns only motion. Other workspaces retain saved references.
const nativeReadOnlyReason='X5 原生工作台仅提供运控；已保存校准可查看和导出，采集与舵机维护暂不可用。';
connectionTargetText=()=> 'X5 独立工作台 · '+globalThis.location.host+'；已保存校准只读。';
canAction=canCancel=canEndFeedback=canRecoverFeedback=canMaintenance=canJointAction=canJointCancel=canZeroAction=()=>false;
canMotionRecheck=()=>!localFilePage&&!!snapshot&&!fetchError&&!motionBusy&&!requestBusy&&snapshot.motion?.owns_connection===false&&['idle','closed','fault'].includes(snapshot.motion?.phase);
recheckMotionFeedback=async()=>{if(canMotionRecheck())return runMotion('connect');};
el('motionRecheck').onclick=()=>recheckMotionFeedback();
motionRecoveryInfo=()=>snapshot?.motion?.owns_connection?'当前连接仍由 X5 维护；先停止并断开，确认串口已关闭后才能重新检查。':'点击「重新检查连接与舵机」或「连接并载入模型」检查并连接；不会自动启动动作。';
motionStateNotOlder=function(next,current){
  const ns=next?.connection_sequence??0,cs=current?.connection_sequence??0;
  if(ns!==cs)return ns>cs;
  const ng=next?.hardware?.generation??next?.last_hardware?.generation??next?.closed_proof?.after?.generation??0;
  const cg=current?.hardware?.generation??current?.last_hardware?.generation??current?.closed_proof?.after?.generation??0;
  const nd=next?.deployment?.started_monotonic??0,cd=current?.deployment?.started_monotonic??0;
  return nd>cd||nd===cd&&ng>=cg;
};
const nativeOriginalRenderMotion=renderMotion;
renderMotion=function(){nativeOriginalRenderMotion();
  const m=snapshot?.motion,known=Array.isArray(m?.models_loaded)?m.models_loaded:Object.keys(m?.deployment?.model_hashes||{});
  const loaded=[...new Set(known.filter(name=>['walk','getup','pick','roulade'].includes(name)))];
  el('motionModelsCount').textContent=loaded.length+' / 4';el('motionHost').textContent=loaded.length?'X5 · CPU':'尚未部署';
  if(loaded.length&&!motionConnectionEstablished()){
    el('motionCalibration').textContent='已载入 · 连接待复查';
    for(const [name,cells] of motionModelRows)if(loaded.includes(name))cells[1].textContent='X5 已加载 · 连接待复查';
  }
};
for(const route of workspaceRoutes){if(route.key!=='motion-models'){route.instructionTitle='已保存校准 · 只读';route.instructionCopy=nativeReadOnlyReason;}}
const nativeOriginalRender=render;
render=function(){nativeOriginalRender();
  for(const id of ['recoverFeedback','endFeedback','recoverFeedbackInZeros'])el(id).disabled=true;
  el('motionRecoveryStatus').textContent=motionRecoveryInfo();
  el('sessionTime').textContent='X5 独立运行 · 口令登录 · 动作由按钮启动';
  const footer=document.querySelector('footer span:last-child');if(footer)footer.textContent='校准与模型已载入 X5；当前服务在 RAM 中运行。';
};
if(!globalThis.location.hash)selectWorkspace('#motion-models');
render();
'''


WALK_MODEL_TEST_PANEL = r'''
      <div class="capability-note" id="walkModelTest" aria-labelledby="walkModelTestTitle">
        <strong id="walkModelTestTitle">行走模型测试</strong>
        <p class="note">模型：xgoduck_walk.onnx · X5 CPU · <span id="walkModelLoadState">等待模型状态</span></p>
        <p class="note">零速平衡使用这个行走模型，前进、侧移和转向速度均为 0。模型会实际调整关节，也可能小幅踏步。</p>
        <p class="note" id="walkModelTestGuide" role="status" aria-live="polite">先连接并载入模型，再确认防跌准备和头颈间隙。</p>
        <div class="settings-actions" id="walkModelTestActions">
          <button class="button" id="walkModelPreview" type="button" disabled>查看行走模型输出 · 不动作</button>
          <button class="button" id="walkModelForwardLocked" type="button" disabled aria-describedby="walkModelForwardReason" title="先连续零速5秒，再明确触发一次限时前进指令。">前进指令试步 · 1.2秒</button>
        </div>
        <p class="note" id="walkModelForwardResult" role="status" aria-live="polite">尚未触发本轮前进指令。</p>
        <p class="note" id="walkModelForwardReason">先连续零速5秒，再触发一次前进指令；1.2秒后自动回零。本试验不代表已具备可靠行走能力，请全程在旁防跌。</p>
        <p class="note">零速试跑最长 30 秒；首次建议 2–3 秒后点击下方「停止运动 · 释放扭矩」。停止时请托住机身。</p>
      </div>
'''

FULL_WORKBENCH_SCRIPT = r'''
// Keep the original guarded walk button and handler; no second start path.
el('walkModelTestActions').insertBefore(el('motionWalk'),el('walkModelForwardLocked'));
el('walkModelPreview').onclick=()=>{
  if(el('motionShadow').disabled)return;
  el('motionPreviewModel').value='walk';
  el('motionShadow').click();
};
// Both passive calibration and explicit motion run on this X5.
// A browser snapshot is a delayed display, not the controller's start gate.
// Permit one explicit hold request during an ordinary polling delay. The
// runtime still checks the live controller, generation and 100ms feedback
// before ARM; this neither queues an action nor retries a rejected request.
function nativeHoldRequestHardware(){return snapshot?.motion?.hardware??snapshot?.motion?.last_hardware;}
// Published ages already include controller-to-web delay. Add only the time
// elapsed in this browser, never the same publication delay a second time.
// This display path does not participate in any request or control gate.
const nativeOriginalImuDisplayFrame=motionImuDisplayFrame;
motionImuDisplayFrame=function(){
  const m=snapshot?.motion,h=m?.hardware,g=h?.imu?.gravity,p=snapshot?.process_isolation;
  const elapsed=receivedAt===null?Infinity:performance.now()-receivedAt;
  if(!fetchError&&p?.protocol===1&&p.status_source==='published_snapshot'
    &&m?.phase==='ready'&&m.owns_connection===true&&h?.connected===true&&h.imu_ok===true
    &&Number.isFinite(elapsed)&&elapsed>=0
    &&[h.age_ms,h.imu_age_ms].every(age=>Number.isFinite(age)&&age>=0&&age+elapsed<500)
    &&finiteVector(g)&&Math.hypot(...g)>.000001){
    const degrees=180/Math.PI,norm=Math.hypot(...g);
    return {tilt:Math.acos(Math.max(-1,Math.min(1,-g[2]/norm)))*degrees,
      pitch:Math.atan2(g[0],-g[2])*degrees,roll:Math.atan2(-g[1],-g[2])*degrees,
      source:'motion',age_ms:h.imu_age_ms+elapsed};
  }
  return nativeOriginalImuDisplayFrame();
};
const nativeOriginalKnownWalkBlockedReason=motionKnownWalkBlockedReason;
motionKnownWalkBlockedReason=function(h){
  const known=nativeOriginalKnownWalkBlockedReason(h);
  if(h?.mode==='hold'&&h.model_start_blocked_reasons?.walk==='先启动站姿，并等待实际关节接近默认姿态'){
    const target=motionStandDegrees(h);
    const rows=Array.isArray(target)&&target.length===motorIds.length?motorIds.map((id,index)=>({id,
      target:target[index],actual:h.rawdata?.find(row=>row?.id===id)?.angle_deg})):[];
    if(rows.length&&rows.every(row=>Number.isFinite(row.target)&&Number.isFinite(row.actual))){
      rows.sort((a,b)=>Math.abs(b.actual-b.target)-Math.abs(a.actual-a.target));
      const worst=rows[0],gap=Math.abs(worst.actual-worst.target);
      if(gap>=5)return '站姿尚未到位：ID '+worst.id+' 当前 '+worst.actual.toFixed(1)
        +'°，目标 '+worst.target.toFixed(1)+'°，偏差 '+gap.toFixed(2)
        +'°；要求小于 5°。请保持双脚落地，托稳后停止检查。';
    }
  }
  if(h?.mode!=='hold'||h.model_start_blocked_reasons?.walk!=='固定头颈适配：实际头颈偏离记录位置达到 2°，已停止')return known;
  const rows=[30,31,32,33].map(id=>({id,target:h.hold_head_deg?.[id],
    actual:h.rawdata?.find(row=>row?.id===id)?.angle_deg}));
  if(!rows.every(row=>Number.isFinite(row.target)&&Number.isFinite(row.actual)))return known;
  rows.sort((a,b)=>Math.abs(b.actual-b.target)-Math.abs(a.actual-a.target));
  const worst=rows[0];
  return '头颈尚未满足平衡接管条件：ID '+worst.id+' 当前 '+worst.actual.toFixed(1)
    +'°，目标 '+worst.target.toFixed(1)+'°，偏差 '+Math.abs(worst.actual-worst.target).toFixed(2)
    +'°；要求小于 2°。当前仍在保持固定站姿，请托稳后停止检查。';
};
let nativeForwardPending=false;
function nativeForwardResultText(){
  const h=nativeHoldRequestHardware(),f=h?.forward_trial,e=f?.execution;
  if(motionReceiptContextCurrent()&&motionReceipt.action==='forward-trial')return '前进请求已接收，等待 X5 发布实际发送结果；这不表示机器人已经迈步。';
  if(!f?.used)return '尚未触发本轮前进指令。';
  const count=Number.isInteger(e?.successful_nonzero_TARGETs)?e.successful_nonzero_TARGETs:null;
  const sent=count===null?'本轮请求已接收':count>0?'已发送 '+count+' 帧带前进指令的目标':'请求已接收，尚未确认发送非零目标';
  const phase=f.phase==='running'?'；指令执行中':e?.successful_zero_TARGETs_after_forward>0?'；随后已恢复零速指令':f.phase==='cancelled'?'；本轮已停止或取消':'；等待零速目标确认';
  const stopped=h?.mode==='off'&&f.phase==='cancelled'?'，当前已停止':'';
  return sent+phase+stopped+'。是否抬脚和前移需要现场观察，发送成功不代表行走成功。';
}
function nativeForwardBlockedReason(){
  const h=nativeHoldRequestHardware(),f=h?.forward_trial,elapsed=receivedAt===null?Infinity:performance.now()-receivedAt;
  if(!motionConnectionEstablished())return '先连接运控、恢复站姿，再启动零速试跑。';
  if(!el('motionSupportConfirmed').checked)return '请确认本轮防跌准备。';
  if(nativeForwardPending||motionBusy||requestBusy||motionReceiptContextCurrent())return '上一项操作尚未完成，请等待新状态。';
  if(!f||f.protocol!==1)return '当前版本未开放受限试步。';
  if(f.phase==='running')return '正在试验，X5将在1.2秒时回到零速；本轮不能重复触发。';
  if(f.used)return h.mode==='walk'?'本轮试步已使用；结果见下方。托住机身后可点击停止结束。':'本轮试步已结束；结果见下方。';
  if(!f.allowed)return f.blocked_reason||'本轮零速和保护条件尚未通过。';
  if(h.mode!=='walk'||h.generation!==f.mode_generation||h.armed!==true||h.pending_stop||h.error||h.firmware_error)return '当前运行状态不满足试步条件。';
  if(localFilePage||fetchError||!snapshot?.token||!Number.isFinite(elapsed)||elapsed<0||elapsed>=1500||![h.age_ms,h.imu_age_ms].every(v=>Number.isFinite(v)&&v>=0&&v+elapsed<1500))return '页面反馈已过期，等待新反馈后再试。';
  if(!ownsMotionControl())return '请在启动本轮零速试跑的页面操作。';
  return '';
}
el('walkModelForwardLocked').onclick=async()=>{
  if(nativeForwardBlockedReason())return;
  nativeForwardPending=true;
  try{await runMotion('forward-trial',{support_confirmed:el('motionSupportConfirmed').checked});}
  finally{nativeForwardPending=false;renderMotion();}
};
function nativePreviewRequestReady(){
  const h=nativeHoldRequestHardware(),elapsed=receivedAt===null?Infinity:performance.now()-receivedAt;
  // Only permit a single explicit request. The X5 controller still requires
  // live feedback under 100ms and torque off before starting inference.
  return !localFilePage&&!fetchError&&!!snapshot?.token&&motionConnectionEstablished()
    &&Number.isFinite(elapsed)&&elapsed>=0&&elapsed<1500
    &&Number.isFinite(h?.age_ms)&&h.age_ms>=0&&h.age_ms+elapsed<1500
    &&Number.isInteger(h.generation)&&h.calibrated===true&&h.mode==='off'
    &&h.armed===false&&h.pending_stop!==true&&!h.error&&h.firmware_error===0
    &&Array.isArray(h.torque_ids)&&h.torque_ids.length===0
    &&Array.isArray(h.online_ids)&&h.online_ids.length===15&&motorIds.every(id=>h.online_ids.includes(id))
    &&Array.isArray(h.fault_ids)&&h.fault_ids.length===0;
}
function nativeHoldRequestReady(){
  const h=nativeHoldRequestHardware(),elapsed=receivedAt===null?Infinity:performance.now()-receivedAt;
  return !localFilePage&&!fetchError&&!!snapshot?.token&&motionConnectionEstablished()
    &&Number.isFinite(elapsed)&&elapsed>=0&&elapsed<1500
    &&Number.isFinite(h?.age_ms)&&h.age_ms>=0&&h.age_ms+elapsed<1500
    &&Number.isInteger(h.generation)&&h.calibrated===true&&!h.error&&!h.start_blocked_reason
    &&(h.head_clearance_required!==true||h.hold_head_deg!=null)
    &&Array.isArray(h.online_ids)&&h.online_ids.length===15&&motorIds.every(id=>h.online_ids.includes(id))
    &&Array.isArray(h.fault_ids)&&h.fault_ids.length===0
    &&motionStandDegrees(h)!==null&&['off','walk'].includes(h.mode);
}
function nativeZeroBalanceRequestReady(){
  const h=nativeHoldRequestHardware(),elapsed=receivedAt===null?Infinity:performance.now()-receivedAt,g=h?.imu?.gravity;
  // This authorizes ONE request, never a sensor sample. NativeRuntime checks
  // the current generation, HOLD, actual ranges, head and 100ms UART/IMU age.
  return !localFilePage&&!fetchError&&!!snapshot?.token&&motionConnectionEstablished()
    &&Number.isFinite(elapsed)&&elapsed>=0&&elapsed<1500
    &&Number.isFinite(h?.age_ms)&&h.age_ms>=0&&h.age_ms+elapsed<1500
    &&Number.isFinite(h?.imu_age_ms)&&h.imu_age_ms>=0&&h.imu_age_ms+elapsed<1500
    &&Number.isInteger(h.generation)&&h.calibrated===true&&!h.error&&!h.start_blocked_reason
    &&!h.model_start_blocked_reasons?.walk&&h.mode==='hold'&&h.hold_ready===true
    &&h.armed===true&&h.pending_stop!==true&&h.firmware_error===0
    &&Array.isArray(h.online_ids)&&h.online_ids.length===15&&motorIds.every(id=>h.online_ids.includes(id))
    &&Array.isArray(h.torque_ids)&&h.torque_ids.length===15&&motorIds.every(id=>h.torque_ids.includes(id))
    &&Array.isArray(h.fault_ids)&&h.fault_ids.length===0
    &&h.imu_ok===true&&finiteVector(g)&&Math.hypot(...g)>.000001
    &&Math.acos(Math.max(-1,Math.min(1,-g[2]/Math.hypot(...g))))*180/Math.PI<15
    &&motionModelContractReady('walk',h)&&motionFixedHeadWalkReady(h);
}
connectionTargetText=()=> 'X5 独立工作台 · '+globalThis.location.host+'；采集、维护和运控均在 X5 运行。';
const nativeOriginalCanMotionRecheck=canMotionRecheck,nativeOriginalMotionRecheck=recheckMotionFeedback,nativeOriginalRecoveryInfo=motionRecoveryInfo;
function nativeNeedsFeedbackReview(){return snapshot?.maintenance?.fault_hold===true||!!snapshot?.session?.recovery_recheck||snapshot?.session?.status==='fault';}
canMotionRecheck=()=>!localFilePage&&!!snapshot&&!fetchError&&!motionBusy&&!requestBusy&&snapshot.motion?.owns_connection===false&&['idle','closed','fault'].includes(snapshot.motion?.phase)&&snapshot.maintenance?.owns_connection!==true&&(!nativeNeedsFeedbackReview()||nativeOriginalCanMotionRecheck());
recheckMotionFeedback=async()=>{if(canMotionRecheck())return nativeNeedsFeedbackReview()?nativeOriginalMotionRecheck():runMotion('connect');};
el('motionRecheck').onclick=()=>recheckMotionFeedback();
motionRecoveryInfo=()=>snapshot?.motion?.owns_connection?'运控正在独占串口；要重新采集，请先停止并断开运控。':nativeNeedsFeedbackReview()?nativeOriginalRecoveryInfo():'USB 断线后，接好 COM 线与舵机电源，点击「重新检查连接与舵机」。检查通过即可恢复使用，不会自动启动动作。';
motionStateNotOlder=function(next,current){
  const ns=next?.connection_sequence??0,cs=current?.connection_sequence??0;
  if(ns!==cs)return ns>cs;
  const ng=next?.hardware?.generation??next?.last_hardware?.generation??next?.closed_proof?.after?.generation??0;
  const cg=current?.hardware?.generation??current?.last_hardware?.generation??current?.closed_proof?.after?.generation??0;
  return ng>=cg;
};
const nativeOriginalRenderMotion=renderMotion;
renderMotion=function(){nativeOriginalRenderMotion();
  const previewReady=nativePreviewRequestReady();
  el('motionShadow').disabled=!previewReady||motionBusy||requestBusy;
  el('motionPreviewModel').disabled=!previewReady||motionBusy||requestBusy;
  el('motionShadow').title=previewReady?'点击后由 X5 检查当前反馈，只推理，不开启扭矩':'等待连接、扭矩关闭和有效反馈';
  el('motionStand').disabled=!nativeHoldRequestReady()||motionBusy||requestBusy||!el('motionSupportConfirmed').checked;
  el('motionStand').title=!el('motionSupportConfirmed').checked?'请先确认机身有稳固支撑':nativeHoldRequestReady()?'点击后由 X5 检查当前反馈，通过后进入固定站姿':nativeHoldRequestHardware()?.start_blocked_reason||'连接或反馈待更新；支撑确认已保留';
  const zeroReady=nativeZeroBalanceRequestReady();
  if(nativeHoldRequestHardware()?.head_clearance_required===true){
    el('motionWalk').disabled=!zeroReady||motionBusy||requestBusy||!el('motionSupportConfirmed').checked;
    el('motionWalk').title=zeroReady?'点击后由 X5 检查当前100毫秒内的反馈，通过后接管零速平衡':motionKnownWalkBlockedReason(nativeHoldRequestHardware())||'等待站姿到位、头颈记录和有效连接';
    if(!zeroReady&&nativeHoldRequestHardware()?.mode==='hold'&&motionKnownWalkBlockedReason(nativeHoldRequestHardware()))
      setMotionText(el('motionWalkGateStatus'),motionKnownWalkBlockedReason(nativeHoldRequestHardware()));
    if(zeroReady)setMotionText(el('motionWalkGateStatus'),'固定站姿已到位。点击后由 X5 检查实时反馈，再启动固定头颈零速平衡；机身仍需防跌支撑。');
    const trial=nativeHoldRequestHardware()?.candidate_profile;
    if(trial?.backend_trial_limit_s===30){
      el('motionWalk').textContent='零速平衡（30秒）';
      if(zeroReady)setMotionText(el('motionWalkGateStatus'),'固定站姿已到位。零速试验最长30秒，倾角达到15°会提前停止卸力；请保持防跌支撑。');
      if(nativeHoldRequestHardware()?.mode==='walk'&&Number.isFinite(trial.trial_seconds_remaining))
        setMotionText(el('motionWalkGateStatus'),'零速平衡剩余约'+Math.ceil(trial.trial_seconds_remaining)+'秒；到时自动停止卸力，请保持防跌支撑。');
    }
  }
  if(motionReceiptContextCurrent())for(const id of ['motionShadow','motionStand'])el(id).disabled=true;
  if(motionReceiptContextCurrent())el('motionWalk').disabled=true;
  const m=snapshot?.motion,known=Array.isArray(m?.models_loaded)?m.models_loaded:Object.keys(m?.deployment?.model_hashes||{});
  const loaded=[...new Set(known.filter(name=>['walk','getup','pick','roulade'].includes(name)))];
  el('motionModelsCount').textContent=loaded.length+' / 4';el('motionHost').textContent=loaded.length?'X5 · CPU':'尚未部署';
  if(loaded.length&&!motionConnectionEstablished()){
    el('motionCalibration').textContent='已载入 · 连接待复查';
    for(const [name,cells] of motionModelRows)if(loaded.includes(name))cells[1].textContent='X5 已加载 · 连接待复查';
  }
  // Presentation only: unchanged native start/lease/feedback/pose gates.
  el('walkModelLoadState').textContent=loaded.includes('walk')?'已加载':'尚未加载';
  el('walkModelPreview').disabled=el('motionShadow').disabled||motionReceiptContextCurrent();
  el('walkModelForwardResult').textContent=nativeForwardResultText();
  const forwardReason=nativeForwardBlockedReason();
  el('walkModelForwardLocked').disabled=!!forwardReason;
  el('walkModelForwardLocked').title=forwardReason||'一次1.2秒前进指令，自动回零；请在旁防跌';
  el('walkModelForwardReason').textContent=forwardReason||'可以触发一次试验：前进指令峰值0.15 m/s，1.2秒后回零。尚未通过实机行走验收，请在旁防跌。';
  el('motionWalk').textContent='行走模型 · 零速试跑（最长30秒）';
  const walkHardware=nativeHoldRequestHardware();
  let walkGuide;
  const walkDisplayElapsed=receivedAt===null?Infinity:performance.now()-receivedAt;
  if(!motionConnectionEstablished())walkGuide='按钮不可用：尚未连接运控。先点击「连接并载入模型」；连接不会自动启动动作。';
  else if(walkHardware?.mode==='walk')walkGuide=el('motionWalkGateStatus').textContent;
  else if(motionBusy||requestBusy)walkGuide='按钮不可用：上一项操作尚未结束，请等待 X5 返回结果。';
  else if(motionReceiptContextCurrent())walkGuide='按钮不可用：操作已收到，正在等待对应的新状态；请勿重复点击。';
  else if(walkHardware?.mode==='shadow')walkGuide='按钮不可用：当前正在不动作预览，舵机扭矩关闭。先点击「停止运动 · 释放扭矩」结束预览，再点击「缓慢恢复上次站姿」；站姿到位后才能零速试跑。';
  else if(!el('motionSupportConfirmed').checked)walkGuide='按钮不可用：尚未勾选防跌准备。请先托稳机身、双脚平放并勾选确认。';
  else if(walkHardware?.head_clearance_required===true&&walkHardware.hold_head_deg==null)walkGuide='按钮不可用：本次连接尚未记录头颈避让位置。先确认整个恢复路径无遮挡，再点击「记录当前头颈位置 · 不动作」。';
  else if(!walkHardware)walkGuide='按钮不可用：正在等待运控状态更新，暂时无法确认站姿是否到位。';
  else if(walkHardware.mode!=='hold')walkGuide='按钮不可用：尚未进入固定站姿。先点击「缓慢恢复上次站姿」；固定站姿到位后才能零速试跑。'+(walkHardware.error?' 上次停止原因：'+walkHardware.error:'');
  else if(walkHardware?.error||walkHardware?.start_blocked_reason)walkGuide='按钮不可用：'+(walkHardware.error||walkHardware.start_blocked_reason);
  else if(motionKnownWalkBlockedReason(walkHardware))walkGuide='按钮不可用：'+motionKnownWalkBlockedReason(walkHardware);
  else if(localFilePage||fetchError||!snapshot?.token)walkGuide='按钮不可用：网页连接异常，请刷新工作台并重新检查连接。';
  else if(!Number.isFinite(walkDisplayElapsed)||walkDisplayElapsed<0||walkDisplayElapsed>=1500||![walkHardware?.age_ms,walkHardware?.imu_age_ms].every(age=>Number.isFinite(age)&&age>=0&&age+walkDisplayElapsed<1500))walkGuide='按钮不可用：页面反馈已过期，正在等待更新。不要移动脚掌来使按钮变亮。';
  else if(walkHardware?.calibrated!==true)walkGuide='按钮不可用：当前校准未通过检查，需要重新核对已保存校准。';
  else if(walkHardware?.fault_ids?.length)walkGuide='按钮不可用：舵机报告故障，ID '+walkHardware.fault_ids.join('、')+'。';
  else if(!Array.isArray(walkHardware?.online_ids)||walkHardware.online_ids.length!==15||!motorIds.every(id=>walkHardware.online_ids.includes(id)))walkGuide='按钮不可用：尚未确认 15 个舵机全部在线。';
  else if(walkHardware?.pending_stop===true||walkHardware?.armed!==true||walkHardware?.torque_ids?.length!==15)walkGuide='按钮不可用：固定站姿未确认全部舵机保持上力，或正在停止；请先托稳并检查站姿状态。';
  else if(!motionModelContractReady('walk',walkHardware))walkGuide='按钮不可用：'+motionModelContractHint('walk',walkHardware);
  else if(!motionFixedHeadWalkReady(walkHardware))walkGuide='按钮不可用：固定头颈适配尚未就绪，请核对本次头颈记录和连接。';
  else walkGuide=zeroReady?'站姿检查通过，可以启动行走模型的零速试跑；请在旁保护。':'按钮不可用：实时姿态或站姿检查尚未通过，请保持双脚平放并等待状态更新。';
  el('walkModelTestGuide').textContent=walkGuide;
  el('motionWalk').title=walkGuide;
  if(el('motionWalk').disabled)setMotionText(el('motionWalkGateStatus'),walkGuide);
};
const nativeOriginalRender=render;
render=function(){nativeOriginalRender();
  el('sessionTime').textContent='X5 独立运行 · 浏览器控制会话 · 校准明确保存到 X5';
  const footer=document.querySelector('footer span:last-child');if(footer)footer.textContent='已保存校准保留在 X5；模型和当前后台在内存中运行。';
};
render();
'''


def native_page(source, *, authentication_required=True, full_workbench=False):
    """Adapt at serve time; leave the established Mac HTML source unchanged."""
    marker = '</script>'
    if source.count(marker) != 1 or 'function render()' not in source:
        raise ValueError('统一工作台页面结构已变化，原生适配未启动')
    if type(authentication_required) is not bool:
        raise ValueError('访问口令模式无效')
    if type(full_workbench) is not bool:
        raise ValueError('原生工作台功能模式无效')
    script = FULL_WORKBENCH_SCRIPT if full_workbench else READ_ONLY_SCRIPT
    if not authentication_required:
        script = script.replace('口令登录', '浏览器控制会话')
    # Every native poll, including the first one after a page reload, uses the
    # live projection. Full diagnostic evidence remains at /api/status.
    source = source.replace("fetch('/api/status',", "fetch('/api/live-status',")
    if full_workbench:
        panel_marker = '<label class="source-note">不动作预览模型 '
        if source.count(panel_marker) != 1:
            raise ValueError('行走模型测试区页面结构已变化')
        source = source.replace(panel_marker, WALK_MODEL_TEST_PANEL + panel_marker, 1)
        replacements = {
            ":operation?.action==='stop'?'停止运动':'断开运控';":
                ":operation?.action==='stop'?'停止运动':operation?.action==='forward-trial'?'受限前进指令试步':'断开运控';",

            "motionControl=request.action==='start'?{...request,generation:value.generation}:null;":
                "motionControl=['start','forward-trial'].includes(request.action)?{...request,generation:value.generation}:null;",
            "  // This is a control receipt, never a sensor frame.":
                "  if(request.action==='forward-trial'&&(value.mode!=='walk'||value.control_owner?.id!==motionControlId||value.control_owner?.generation!==value.generation))throw new Error('试步控制权未确认');\n  // This is a control receipt, never a sensor frame.",
            "const busy=!['command','stop'].includes(action);if(busy)motionBusy=true;":
                "const busy=!['command','stop','forward-trial'].includes(action);if(busy)motionBusy=true;",
            "const epoch=['start','stop','connect','disconnect','capture-head-clearance'].includes(action)?++motionActionEpoch:motionActionEpoch;":
                "const epoch=['start','stop','connect','disconnect','capture-head-clearance','forward-trial'].includes(action)?++motionActionEpoch:motionActionEpoch;",

            "if(action==='start'&&!motionFresh())return;":
                "if(action==='start'&&!(extra.mode==='hold'?nativeHoldRequestReady():extra.mode==='shadow'?nativePreviewRequestReady():extra.mode==='walk'&&nativeHoldRequestHardware()?.head_clearance_required===true?nativeZeroBalanceRequestReady():motionFresh()))return;",
            "if(action==='start'&&extra.mode==='hold'&&motionStandDegrees()===null)return;":
                "if(action==='start'&&extra.mode==='hold'&&motionStandDegrees(nativeHoldRequestHardware())===null)return;",
            "const generation=(action==='capture-head-clearance'?(snapshot.motion?.hardware??snapshot.motion?.last_hardware):['stop','command'].includes(action)?motionControlHardware():snapshot.motion?.hardware)?.generation,pageGeneration=motionPageGeneration();":
                "const generation=(action==='start'&&['hold','shadow','walk'].includes(extra.mode)||action==='capture-head-clearance'?nativeHoldRequestHardware():['stop','command','forward-trial'].includes(action)?motionControlHardware():snapshot.motion?.hardware)?.generation,pageGeneration=motionPageGeneration();",
            "if(action==='start'&&['walk','getup','pick','roulade'].includes(extra.mode)&&!motionModelContractReady(extra.mode))return;":
                "if(action==='start'&&['walk','getup','pick','roulade'].includes(extra.mode)&&!motionModelContractReady(extra.mode,extra.mode==='walk'?nativeHoldRequestHardware():snapshot?.motion?.hardware))return;",
            "if(action==='start'&&extra.mode==='walk'&&(snapshot.motion.hardware.mode!=='hold'||snapshot.motion.hardware.hold_ready!==true||motionWalkBlockedReason()))return;":
                "if(action==='start'&&extra.mode==='walk'&&(nativeHoldRequestHardware()?.head_clearance_required===true?!nativeZeroBalanceRequestReady():snapshot.motion.hardware.mode!=='hold'||snapshot.motion.hardware.hold_ready!==true||motionWalkBlockedReason()))return;",
        }
        for old, new in replacements.items():
            if source.count(old) != 1:
                raise ValueError('固定站姿请求适配结构已变化')
            source = source.replace(old, new, 1)
    return source.replace(marker, script + '\n' + marker).encode('utf-8')


class NativeWorkbench:
    def __init__(self, runtime, *, web_root, password=None, authentication_required=True,
                 snapshot_template=None, clock=time.monotonic, calibration_service=None):
        self.runtime = runtime
        self.calibration_service = calibration_service
        self.web_root = Path(web_root)
        if type(authentication_required) is not bool:
            raise ValueError('访问口令模式无效')
        self.authentication_required = authentication_required
        self.password = password if authentication_required else None
        if authentication_required and (type(password) is not str or not re.fullmatch(r'[A-Za-z0-9_-]{32,128}', password)):
            raise ValueError('访问口令格式无效')
        if snapshot_template is not None and not isinstance(snapshot_template, dict):
            snapshot_template = strict_json(Path(snapshot_template).read_bytes())
        if snapshot_template is not None and type(snapshot_template) is not dict:
            raise ValueError('页面状态模板必须是 JSON 对象')
        self.template = copy.deepcopy(snapshot_template or {})
        self.clock = clock
        self.token = secrets.token_urlsafe(32)
        self.login_token = secrets.token_urlsafe(32)
        self.generation = secrets.randbelow(0x7ffffffe) + 1
        self.lock = threading.RLock()
        self.io_lock = threading.RLock()
        self.cookies = {}
        self.login_attempts = {}
        self.control_owner = None
        self.last_operation = None
        self.operation_epoch = 0
        self.closed = False
        self.page = native_page((self.web_root / 'index.html').read_text(),
                                authentication_required=authentication_required,
                                full_workbench=calibration_service is not None)
        if calibration_service is not None:
            calibration_service.session.motion = runtime
            previous_gate = calibration_service.before_acquire
            def before_acquire(purpose):
                with self.io_lock:
                    self._acquisition_gate()
                    return previous_gate(purpose)
            calibration_service.before_acquire = before_acquire

    def _acquisition_gate(self):
        motion = self.runtime.snapshot()
        if motion.get('owns_connection') or motion.get('phase') == 'deploying':
            raise ValueError('运控正在使用连接；先在「运控模型」停止并断开。')

    def calibration_action(self, path, body, *, principal=None):
        if self.calibration_service is None:
            raise ValueError(READ_ONLY)
        with self.io_lock:
            with self.lock:
                if self.closed:
                    raise ValueError('原生工作台已关闭')
            # Maintenance exit/release and calibration END retain their own
            # cleanup path; all acquisition/start gates use actual UART owners.
            if path != '/api/end-feedback' and not (
                    path == '/api/servo-maintenance' and isinstance(body, dict)
                    and body.get('action') in ('exit', 'release')):
                self._acquisition_gate()
            self.calibration_service.dispatch(path, body)
            # Mutation replies feed the same live UI as /api/live-status.
            # Full reconnection evidence remains available through /api/status.
            return self.snapshot(live=True)

    def _new_cookie_locked(self):
        now = self.clock()
        if self.closed:
            raise ValueError('原生工作台已关闭')
        self.cookies = {token: expires for token, expires in self.cookies.items() if expires > now}
        if len(self.cookies) >= MAX_SESSIONS:
            # Insertion order tracks last successful use. Abandoned browser
            # and diagnostic sessions must not lock everyone out for 12 hours.
            # Never evict the principal that owns the current motion lease.
            owner = (self.control_owner or {}).get('principal')
            idle = next((token for token in self.cookies if token != owner), None)
            if idle is None:
                raise ValueError('访问设备过多，请稍后重试')
            del self.cookies[idle]
        token = secrets.token_urlsafe(32)
        self.cookies[token] = now + COOKIE_SECONDS
        return token

    def anonymous_session(self, header):
        """Called only by GET /; identify browsers without any login step."""
        if self.authentication_required:
            raise ValueError('当前工作台需要访问口令')
        with self.lock:
            token = self.principal(header)
            if token is not None:
                return token, False
            return self._new_cookie_locked(), True

    def login(self, password, address):
        if not self.authentication_required:
            raise ValueError('当前工作台无需口令；请刷新首页建立浏览器控制会话')
        with self.lock:
            now = self.clock()
            self.login_attempts = {ip: times for ip, times in self.login_attempts.items()
                                   if any(now - value < 60 for value in times)}
            attempts = [value for value in self.login_attempts.get(address, []) if now - value < 60]
            if len(attempts) >= 5 or address not in self.login_attempts and len(self.login_attempts) >= 128:
                raise ValueError('登录次数过多，请稍后重试')
            attempts.append(now)
            self.login_attempts[address] = attempts
            if type(password) is not str or len(password) > 128 or not secrets.compare_digest(
                    password.encode('utf-8'), self.password.encode('utf-8')):
                raise ValueError('访问口令错误')
            return self._new_cookie_locked()

    def authenticated(self, header):
        return self.principal(header) is not None

    def principal(self, header):
        if type(header) is not str or len(header) > 4096:
            return None
        # SimpleCookie permits repeated names; explicitly reject that ambiguity.
        names = [part.split('=', 1)[0].strip() for part in header.split(';') if '=' in part]
        if names.count(COOKIE) != 1:
            return None
        try:
            cookies = SimpleCookie(header)
            token = cookies[COOKIE].value
        except (KeyError, ValueError, CookieError):
            return None
        with self.lock:
            expires = self.cookies.get(token, 0)
            if self.closed or expires <= self.clock():
                return None
            # Touch recency without extending the original expiry or changing
            # the browser identity used by CSRF and motion ownership checks.
            self.cookies.pop(token)
            self.cookies[token] = expires
            return token

    def snapshot(self, *, live=False):
        # Both providers return owned snapshots. Copying the complete result
        # again duplicates historical logs while the control thread needs CPU.
        motion = self.runtime.snapshot(compact=True) if live else self.runtime.snapshot()
        if type(motion) is not dict:
            raise ValueError('原生运控状态无效')
        if not live:
            motion['runtime_last_operation'] = motion.get('last_operation')
        # Never use template hardware, calibration, IMU, or old control leases.
        if self.calibration_service is not None:
            value = self.calibration_service.snapshot(compact=live, include_motion=False)
            value.update(token=self.token, motion=motion,
                         hardware=motion.get('hardware') if live else copy.deepcopy(motion.get('hardware')),
                         native_access=dict(authentication_required=self.authentication_required,
                                            page_generation=self.generation, full_workbench=True),
                         connection_target=dict(kind='wifi', host=HOST, page_url=ORIGIN+'/', native=True))
            if value.get('feedback') is None and motion.get('hardware') is not None:
                value['body_imu'] = (motion['hardware'].get('imu') if live else
                                     copy.deepcopy(motion['hardware'].get('imu')))
            with self.lock:
                motion['control_owner'] = (None if self.control_owner is None else
                    {key: self.control_owner[key] for key in ('id', 'generation')})
                motion['last_operation'] = self._operation_snapshot_locked(compact=live)
            return value
        value = copy.deepcopy(self.template)
        value.update(session=dict(status='closed', generation=self.generation, error=None,
                                  error_details=None, message=READ_ONLY, end_reason='native-read-only',
                                  received_age_ms=None, recovery_allowed=False, recheck_allowed=False,
                                  recovery_pending=False, recovery_blocked_reason=READ_ONLY,
                                  connection_check=dict(allowed=False, phase='idle', blocked_reason=READ_ONLY)),
                     token=self.token, calibration=copy.deepcopy(self.runtime.calibration),
                     motion=motion, feedback=None,
                     hardware=motion.get('hardware') if live else copy.deepcopy(motion.get('hardware')),
                     body_imu=((motion.get('hardware') or {}).get('imu') if live else
                               copy.deepcopy((motion.get('hardware') or {}).get('imu'))),
                     native_access=dict(authentication_required=self.authentication_required),
                     connection_target=dict(kind='wifi', host=HOST, page_url=ORIGIN + '/', native=True),
                     wizard=dict(stage='idle', expected_action=None, busy=False, progress=0,
                                 sample_count=0, message=READ_ONLY, error=None),
                     joint_wizard=dict(stage='idle', id=None, busy=False, error=None),
                     joint_zero=dict(stage='idle', id=None, busy=False, error=None),
                     maintenance=dict(phase='unavailable', connected=False, owns_connection=False,
                                      busy=False, compatible=False, fault_hold=False,
                                      blocked_reason=READ_ONLY, error=None))
        with self.lock:
            motion['control_owner'] = (None if self.control_owner is None else
                                       {key: self.control_owner[key] for key in ('id', 'generation')})
            motion['last_operation'] = self._operation_snapshot_locked(compact=live)
        return value

    def _operation_snapshot_locked(self, *, compact):
        if not compact or self.last_operation is None:
            return copy.deepcopy(self.last_operation)
        # Project before copying: deep historical hardware/STOP evidence stays
        # available from the full status route and the runtime audit record.
        value = {key: self.last_operation[key] for key in (
            'action', 'mode', 'generation', 'stop_source', 'recorded_monotonic',
            'state', 'error') if key in self.last_operation}
        after = self.last_operation.get('after')
        if type(after) is dict:
            value['after'] = {key: after[key] for key in (
                'connected', 'generation', 'mode', 'release_confirmed') if key in after}
        return value

    def initial_connect(self):
        """One explicit bootstrap attempt; faults remain visible without retry."""
        try:
            return self.runtime.connect()
        except Exception as error:
            with self.lock:
                self.last_operation = dict(action='connect', mode=None, state='failed',
                                           error=str(error), recorded_monotonic=self.clock())
            return self.runtime.snapshot()

    def motion_action(self, body, *, principal=None, live=False):
        if type(body) is not dict:
            raise ValueError('运控请求必须是 JSON 对象')
        action = body.get('action')
        fields = {'action', 'page_generation', 'generation', 'control_id'}
        extras = dict(start={'mode', 'support_confirmed', 'behavior_confirmed', 'preview_model'},
                      command={'forward', 'side', 'yaw'}, heartbeat={'background'},
                      stop={'stop_source'}, connect=set(), disconnect=set(),
                      **{'capture-head-clearance': {'clearance_confirmed'}, 'forward-trial': {'support_confirmed'}})
        if action not in extras or set(body) - fields - extras[action]:
            raise ValueError('未知运控操作或参数')
        if type(body.get('page_generation')) is not int or body['page_generation'] != self.generation:
            raise ValueError('页面会话已变化，旧请求未执行；请刷新')
        client = body.get('control_id')
        valid_client = type(client) is str and re.fullmatch(r'[0-9a-f]{32}', client) is not None
        if not valid_client:
            raise ValueError('运控页面版本已更新，请刷新')
        if 'generation' in body and type(body['generation']) is not int:
            raise ValueError('运控状态编号无效')
        if action == 'start':
            if body.get('mode') not in ('shadow', 'hold') + MODEL_NAMES:
                raise ValueError('未知动作模式')
            if body['mode'] != 'shadow' and body.get('support_confirmed') is not True:
                raise ValueError('请先托稳机身并勾选支撑确认')
            if body['mode'] in ('getup', 'roulade') and body.get('behavior_confirmed') is not True:
                raise ValueError('请确认保护装置与动作区域')
            if body.get('preview_model', 'walk') not in MODEL_NAMES:
                raise ValueError('未知预览模型')
            for key in ('support_confirmed', 'behavior_confirmed'):
                if key in body and type(body[key]) is not bool:
                    raise ValueError('支撑确认格式无效')
        if action == 'heartbeat' and 'background' in body and type(body['background']) is not bool:
            raise ValueError('后台心跳参数无效')
        if action == 'capture-head-clearance' and body.get('clearance_confirmed') is not True:
            raise ValueError('请先确认头颈与板卡、线束有活动间隙')
        if action == 'stop' and body.get('stop_source') not in STOP_SOURCES:
            raise ValueError('停止来源无效')
        if action == 'forward-trial' and body.get('support_confirmed') is not True:
            raise ValueError('请确认本轮防跌准备')
        if action == 'command':
            for key, bound in (('forward', .2), ('side', .1), ('yaw', .5)):
                number = body.get(key)
                if type(number) not in (int, float) or not math.isfinite(number) or abs(number) > bound:
                    raise ValueError('行走指令范围无效')
        # STOP must be able to invalidate an in-progress model preflight. The
        # controller already serializes UART operations and generation checks.
        receipt_action = live and action in ('start', 'stop', 'capture-head-clearance', 'forward-trial')
        with (nullcontext() if action in ('stop', 'heartbeat') else self.io_lock):
            with self.lock:
                if self.closed:
                    raise ValueError('原生工作台已关闭')
                owner = copy.deepcopy(self.control_owner)
            before = (self.runtime.control_context() if action == 'heartbeat' or receipt_action
                      else self.runtime.snapshot())
            hardware = before.get('hardware') or {}
            # Delayed publication may hide display feedback. For an explicit
            # hold request use only its generation/ownership context here;
            # runtime.action independently reads the actual controller and
            # start_mode enforces every current feedback and hardware gate.
            if ((action == 'start' and body.get('mode') == 'hold' or action in ('capture-head-clearance', 'heartbeat'))
                    and before.get('phase') == 'ready' and before.get('owns_connection') is True
                    and not hardware):
                hardware = before.get('last_hardware') or {}
            if action in ('start', 'command', 'heartbeat', 'capture-head-clearance', 'forward-trial') and (
                    type(body.get('generation')) is not int or body['generation'] != hardware.get('generation')):
                raise ValueError('Motion state changed; refresh before starting')
            if action == 'start' and hardware.get('mode') != 'off' and (
                    not owner or client != owner['id'] or principal != owner['principal'] or
                    body.get('generation') != owner['generation']):
                raise ValueError('当前动作由另一页面控制；先停止再由本页启动')
            if action == 'capture-head-clearance' and hardware.get('mode') != 'off':
                raise ValueError('请先停止运动并确认扭矩关闭，再记录头颈位置')
            if action in ('command', 'heartbeat', 'forward-trial') or action == 'stop' and body['stop_source'] != 'button':
                if action == 'stop' and body['stop_source'] == 'window-blur':
                    raise ValueError('窗口失焦不再停止运动；请刷新旧页面')
                if not owner or client != owner['id'] or principal != owner['principal'] or body.get('generation') != owner['generation'] or hardware.get('generation') != owner['generation']:
                    raise ValueError('该页面未启动当前动作；旧页面请求未执行')
                if action == 'stop' and hardware.get('mode') == 'hold' and (
                        body['stop_source'] == 'page-hidden' or hardware.get('hold_control_protocol') == 2):
                    raise ValueError('固定站姿由 X5 独立保持；请使用停止按钮结束站姿')
            audited = action in ('start', 'stop', 'disconnect', 'connect', 'capture-head-clearance', 'forward-trial')
            operation = None
            if audited:
                with self.lock:
                    self.operation_epoch += 1
                    operation_epoch = self.operation_epoch
                    operation = dict(action=action, mode=body.get('mode'),
                                               generation=body.get('generation'), stop_source=body.get('stop_source'),
                                               recorded_monotonic=self.clock(), state='pending',
                                               error=None, before=copy.deepcopy(hardware))
                    self.last_operation = operation
            try:
                if action == 'connect':
                    if self.calibration_service is not None:
                        calibration_session = self.calibration_service.session
                        if (calibration_session.maintenance_mapping_changed
                                or calibration_session.maintenance_fault_hold
                                or calibration_session.recovery_recheck is not None):
                            raise ValueError('当前连接、维护故障或编号映射尚未复查通过，不能连接运控')
                        if self.calibration_service.session.recovery_pending or self.calibration_service.session.source_transmitting:
                            raise ValueError('采集连接正在检查或启动，请等本次连接完成后再连接运控')
                        maintenance = self.calibration_service.session.maintenance
                        if maintenance is not None and maintenance.owns_connection:
                            raise ValueError('舵机维护占用连接；先停止并退出维护，等待串口关闭。')
                        self.calibration_service.end_for_motion()
                    result = self.runtime.connect()
                elif action == 'disconnect':
                    result = self.runtime.disconnect()
                else:
                    result = (self.runtime.action_receipt(body) if receipt_action
                              else self.runtime.action(copy.deepcopy(body)))
                if action == 'heartbeat':
                    # Authentication, owner and generation were checked above;
                    # runtime.action checked the actual controller generation.
                    # Status projection/calibration serialization belongs to
                    # GET /api/live-status, not this latency-sensitive lease.
                    return dict(heartbeat_accepted=True, page_generation=self.generation,
                                generation=body['generation'])
                after = result if receipt_action else (result or self.runtime.snapshot()).get('hardware') or {}
                latest_context = self.runtime.control_context() if receipt_action else self.runtime.snapshot()
                latest = latest_context.get('hardware') or {}
                with self.lock:
                    current_operation = not audited or operation_epoch == self.operation_epoch
                    captured_head = result.get('captured_head_deg') if receipt_action else None
                    captured_head_valid = (type(captured_head) is dict
                        and (set(captured_head) == {30, 31, 32, 33}
                             or set(captured_head) == {'30', '31', '32', '33'})
                        and all(type(v) in (int, float) and math.isfinite(v)
                                for v in captured_head.values()))
                    if receipt_action and (not current_operation
                            or result.get('action_receipt_protocol') != 1
                            or result.get('action') != action
                            or type(after.get('generation')) is not int
                            or result.get('connection_sequence') != latest_context.get('connection_sequence')
                            or result.get('connection_sequence') != before.get('connection_sequence')
                            or after.get('generation') != latest.get('generation')
                            or after.get('mode') != latest.get('mode')
                            or after.get('connected') is not True
                            or latest.get('connected') is not True
                            or action == 'start' and after.get('mode') != body['mode']
                            or action in ('stop', 'capture-head-clearance') and (after.get('mode') != 'off'
                                or after.get('release_confirmed') is not True
                                or latest.get('release_confirmed') is not True)
                            or action == 'capture-head-clearance' and not captured_head_valid):
                        raise ValueError('动作已被更新状态取代；旧确认未采用')
                    if action == 'start' and current_operation and after.get('connected') is True and after.get('mode') == body['mode'] and after.get('generation') == latest.get('generation'):
                        self.control_owner = dict(id=client, generation=after['generation'], principal=principal)
                    elif action in ('stop', 'disconnect', 'connect', 'capture-head-clearance') and current_operation:
                        self.control_owner = None
                    if audited:
                        operation.update(state='complete', after=copy.deepcopy(after))
                    if receipt_action:
                        return dict(result, page_generation=self.generation,
                                    connection_sequence=latest_context['connection_sequence'],
                                    operation_epoch=operation_epoch,
                                    control_owner=(None if self.control_owner is None else
                                        {key: self.control_owner[key] for key in ('id', 'generation')}))
            except Exception as error:
                with self.lock:
                    if audited:
                        operation.update(state='failed', error=str(error))
                raise
            return self.snapshot(live=live)

    def close(self):
        with self.io_lock:
            with self.lock:
                self.closed = True
                self.cookies.clear()
                self.control_owner = None
            self.runtime.close()
            if self.calibration_service is not None:
                maintenance = self.calibration_service.session.maintenance
                if maintenance is not None:
                    maintenance.close(timeout=3)
                    if maintenance.owns_connection:
                        raise ValueError('关闭后台时维护串口关闭尚未确认')
                self.calibration_service.close()


def login_page(token):
    # Token is embedded only in the unauthenticated same-origin login page.
    return ('''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>鸭子工作台登录</title>
<style>body{font:16px system-ui;background:#f5f7f2;color:#26382e;margin:12vh auto;padding:24px;max-width:420px}input,button{font:inherit;box-sizing:border-box;width:100%;padding:14px;margin:8px 0}button{background:#08786c;color:white;border:0;border-radius:8px}#error{color:#a83123}</style>
<h1>鸭子工作台</h1><p>X5 独立运行。输入部署时提供的访问口令。</p>
<form id="login"><label>访问口令<input id="password" type="password" autocomplete="current-password" required maxlength="128"></label><button>登录</button><p id="error" role="alert"></p></form>
<script>document.getElementById('login').onsubmit=async function(event){event.preventDefault();const error=document.getElementById('error');error.textContent='';try{const response=await fetch('/api/login',{method:'POST',credentials:'same-origin',headers:{'Content-Type':'application/json','X-CSRF-Token':''' + json.dumps(token) + '''},body:JSON.stringify({password:document.getElementById('password').value})});const result=await response.json();if(!response.ok)throw new Error(result.error||'登录失败');location.replace('/#motion-models');}catch(problem){error.textContent=problem.message;}};</script></html>''').encode()


def handler_for(workbench):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'
        server_version = 'DuckWorkbench'

        def log_message(self, *_args):
            pass  # Passwords, tokens and request bodies never enter logs.

        def setup(self):
            super().setup()
            self.connection.settimeout(3.)

        def reply(self, code, value, *, kind='application/json; charset=utf-8', cookie=None, download=False):
            data = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            if len(data) > MAX_RESPONSE:
                code, data, kind = 503, b'{"error":"response exceeds bound"}', 'application/json'
            self.send_response(code)
            for key, item in (('Content-Type', kind), ('Content-Length', str(len(data))),
                              ('Cache-Control', 'no-store'), ('X-Content-Type-Options', 'nosniff'),
                              ('X-Frame-Options', 'DENY'), ('Referrer-Policy', 'no-referrer'), ('Connection', 'close')):
                self.send_header(key, item)
            if cookie:
                self.send_header('Set-Cookie', cookie)
            if download:
                self.send_header('Content-Disposition', 'attachment; filename="duck-calibration.json"')
            self.end_headers()
            self.close_connection = True
            try:
                self.wfile.write(data)
            except (OSError, socket.timeout):
                pass

        def framing(self, *, post=False):
            items = list(self.headers.raw_items())
            names = [name.lower() for name, _value in items]
            if len(items) > 64 or sum(len(name) + len(value) + 4 for name, value in items) > MAX_HEADERS:
                raise ValueError('请求头超过限制')
            if len(names) != len(set(names)):
                raise ValueError('重复请求头无效')
            if any(any(ord(char) < 32 or ord(char) == 127 for char in name + value) for name, value in items):
                raise ValueError('请求头格式无效')
            if self.headers.get('Host') != HOST + ':' + str(PORT):
                raise ValueError('工作台地址无效')
            if 'transfer-encoding' in names or 'expect' in names:
                raise ValueError('不支持分块或预期请求')
            if self.path.startswith('//') or '?' in self.path or '#' in self.path:
                raise ValueError('工作台路径无效')
            length = self.headers.get('Content-Length', '0')
            if not re.fullmatch(r'0|[1-9][0-9]{0,3}', length):
                raise ValueError('请求长度无效')
            size = int(length)
            if post:
                if self.headers.get('Origin') != ORIGIN:
                    raise ValueError('仅允许工作台同源请求')
                if not 0 < size <= MAX_REQUEST or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('请求大小或类型无效')
            elif size != 0:
                raise ValueError('读取请求不能包含正文')
            return size

        def do_GET(self):
            try:
                self.framing()
                authenticated = workbench.authenticated(self.headers.get('Cookie'))
                if not workbench.authentication_required and self.path == '/':
                    token, created = workbench.anonymous_session(self.headers.get('Cookie'))
                    cookie = session_cookie(token) if created else None
                    return self.reply(200, workbench.page, kind='text/html; charset=utf-8', cookie=cookie)
                if not workbench.authentication_required and self.path == '/login':
                    return self.reply(404, dict(error='工作台无需口令，请打开首页'))
                if self.path in ('/', '/login'):
                    return self.reply(200, workbench.page if authenticated else login_page(workbench.login_token),
                                      kind='text/html; charset=utf-8')
                if not authenticated:
                    return self.reply(401, dict(error=access_message(workbench)))
                if self.path in ('/api/status', '/api/live-status'):
                    return self.reply(200, workbench.snapshot(live=self.path == '/api/live-status'))
                if self.path == '/api/calibration':
                    return self.reply(200, copy.deepcopy(workbench.runtime.calibration), download=True)
                if self.path in ('/assets/zero-reference.png', '/assets/positive-direction.png',
                                  '/zero-reference.png', '/positive-direction.png'):
                    asset = workbench.web_root / 'assets' / self.path.rsplit('/', 1)[-1]
                    if asset.stat().st_size > MAX_RESPONSE:
                        raise ValueError('参考图超过限制')
                    return self.reply(200, asset.read_bytes(), kind='image/png')
                return self.reply(404, dict(error='找不到页面'))
            except (ValueError, OSError) as error:
                return self.reply(403, dict(error=str(error)))

        def do_POST(self):
            try:
                size = self.framing(post=True)
                login = self.path == '/api/login'
                if login and not workbench.authentication_required:
                    return self.reply(403, dict(error='工作台无需口令，请打开首页建立浏览器控制会话'))
                if not login and not workbench.authenticated(self.headers.get('Cookie')):
                    return self.reply(401, dict(error=access_message(workbench)))
                token = workbench.login_token if login else workbench.token
                if not secrets.compare_digest(self.headers.get('X-CSRF-Token', '').encode('utf-8'), token.encode('utf-8')):
                    return self.reply(403, dict(error='页面会话已变化，请刷新'))
                calibration_paths = ('/api/imu-wizard', '/api/joint-calibration', '/api/joint-zero',
                    '/api/end-feedback', '/api/recover-feedback', '/api/recheck-feedback',
                    '/api/check-connection', '/api/servo-maintenance')
                if self.path not in ('/api/login', '/api/motion') + calibration_paths:
                    return self.reply(404, dict(error='未知工作台操作'))
                if self.path in calibration_paths and workbench.calibration_service is None:
                    return self.reply(403, dict(error=READ_ONLY))
                raw = self.rfile.read(size)
                if len(raw) != size:
                    raise ValueError('请求正文不完整；不会重发')
                body = strict_json(raw)
                if login:
                    if type(body) is not dict or set(body) != {'password'}:
                        raise ValueError('登录请求格式无效')
                    token = workbench.login(body['password'], self.client_address[0])
                    cookie = session_cookie(token)
                    return self.reply(200, dict(logged_in=True), cookie=cookie)
                principal = workbench.principal(self.headers.get('Cookie'))
                if self.path in calibration_paths:
                    return self.reply(200, workbench.calibration_action(self.path, body, principal=principal))
                return self.reply(200, workbench.motion_action(body, principal=principal, live=True))
            except Exception as error:
                return self.reply(409, dict(error=str(error)))

    return Handler


def access_message(workbench):
    return ('请先输入访问口令登录' if workbench.authentication_required else
            '浏览器控制会话尚未建立或已失效；请刷新工作台首页，无需口令')


def session_cookie(token):
    return COOKIE + '=' + token + '; Path=/; HttpOnly; SameSite=Strict; Max-Age=' + str(COOKIE_SECONDS)


class BoundedHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    block_on_close = False
    request_queue_size = 16

    def __init__(self, address, handler, *, max_threads=12, listener=None):
        if not 8 <= max_threads <= 32:
            raise ValueError('工作台并发线程数无效')
        self.slots = threading.BoundedSemaphore(max_threads)
        super().__init__(address, handler, bind_and_activate=listener is None)
        if listener is not None:
            self.socket.close()
            self.socket = listener
            self.server_address = listener.getsockname()

    def process_request(self, request, client_address):
        if not self.slots.acquire(blocking=False):
            try:
                request.settimeout(.2)
                request.sendall(b'HTTP/1.0 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n')
            except OSError:
                pass
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.slots.release()


def serve(runtime, *, web_root, secret_file=None, host=HOST, port=PORT,
          snapshot_template=None, auto_connect=True, ready_callback=None, authentication_required=True,
          calibration_service=None):
    """Block as an independent native service; bootstrap supplies the runtime."""
    if host != HOST or type(port) is not int or port != PORT:
        raise ValueError('仅允许部署时审查的 X5 Wi-Fi 地址与端口')
    if type(authentication_required) is not bool:
        raise ValueError('访问口令模式无效')
    password = None
    if authentication_required:
        if secret_file is None:
            raise ValueError('原生工作台访问口令文件缺失')
        secret_path = Path(secret_file).resolve()
        if not secret_path.is_relative_to('/dev/shm'):
            raise ValueError('原生工作台访问口令必须保存在 RAM 中')
        password = private_secret(secret_path)
    workbench = NativeWorkbench(runtime, web_root=web_root, password=password,
                                authentication_required=authentication_required, snapshot_template=snapshot_template,
                                calibration_service=calibration_service)
    server = None
    saved_signals = {}
    try:
        server = BoundedHTTPServer((host, port), handler_for(workbench))
        if threading.current_thread() is threading.main_thread():
            def stop_service(_signum, _frame):
                raise SystemExit(0)
            for signum in (signal.SIGTERM, signal.SIGINT):
                saved_signals[signum] = signal.signal(signum, stop_service)
        if auto_connect:
            workbench.initial_connect()  # STOP/CONFIG/STATE only; a fault remains visible.
        info = dict(pid=os.getpid(), address=server.server_address[0], port=server.server_address[1],
                    listener_fd=server.socket.fileno(), listener_inode=os.fstat(server.socket.fileno()).st_ino,
                    auth_required=authentication_required, motion=runtime.snapshot())
        if ready_callback:
            ready_callback(info)
        server.serve_forever(poll_interval=.2)
    finally:
        if server is not None:
            server.server_close()
        workbench.close()
        for signum, previous in saved_signals.items():
            signal.signal(signum, previous)
