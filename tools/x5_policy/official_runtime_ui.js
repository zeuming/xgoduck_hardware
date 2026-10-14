// Normal control replaces the former attended-trial UI. Backend owns validation.
function officialCommandValues() {
  const c = nativeHoldRequestHardware()?.command || {};
  return [c.forward || 0, c.side || 0, c.yaw || 0];
}
nativeZeroBalanceRequestReady = function () {
  const h = nativeHoldRequestHardware();
  return motionConnectionEstablished() && ['off', 'hold'].includes(h?.mode)
    && !!snapshot?.token && !fetchError && !motionBusy && !requestBusy
    && !nativeZeroBalanceBlockingReason(h);
};
nativeZeroBalanceStatus = function (h, ready) {
  if (h?.mode === 'walk') {
    const phase = h.official_runtime?.recovery?.phase || 'walk';
    const label = phase === 'getup' ? '官方恢复流程 · 起身'
      : phase === 'default_pose' ? '官方恢复流程 · 默认姿态' : '行走模型运行中';
    return {kind: 'running', label, text: label + '。无30秒时限；速度为零时模型继续平衡。'};
  }
  return {kind: ready ? 'ready' : 'waiting', label: '启动行走模型 · 零速',
    text: nativeZeroBalanceBlockingReason(h) || (ready ? '点击后启动官方行走模型，不需要等待5秒或满足5°误差门槛。' : '先连接并载入模型。')};
};
nativeForwardBlockedReason = function () {
  const h = nativeHoldRequestHardware();
  if (!motionConnectionEstablished()) return '请先连接并载入模型。';
  if (h?.mode !== 'walk') return '请先启动行走模型。';
  if (!ownsMotionControl()) return '当前控制权属于其他页面或手柄。';
  if (nativeForwardPending || motionBusy || requestBusy) return '正在处理上一项请求。';
  return '';
};
nativeForwardResultText = function () {
  const h = nativeHoldRequestHardware();
  if (h?.mode !== 'walk') return '模型尚未运行。';
  const c = officialCommandValues();
  return '当前速度指令：前后 ' + Number(c[0]).toFixed(2) + ' m/s，侧向 '
    + Number(c[1]).toFixed(2) + ' m/s，转向 ' + Number(c[2]).toFixed(2)
    + ' rad/s。指令送达不代表已完成实机行走验收。';
};
nativeRenderTrialStopPolicy = function () {
  el('motionStopPolicy').textContent = '官方恢复逻辑：倾角超过55°持续约0.15秒后进入默认姿态和起身模型；恢复站稳后返回行走。头颈目标遵守已保存范围，腿部使用原模型目标。保留急停、通信超时及保存范围检查；急停会释放扭矩。';
};
el('walkModelForwardLocked').onclick = async function () {
  const reason = nativeForwardBlockedReason();
  if (reason) { el('walkModelForwardReason').textContent = reason; return; }
  const moving = officialCommandValues().some(v => v !== 0);
  nativeForwardPending = true;
  try { await runMotion('command', {forward: moving ? 0 : .3, side: 0, yaw: 0}); }
  finally { nativeForwardPending = false; renderMotion(); }
};
const officialOriginalRenderMotion = renderMotion;
renderMotion = function () {
  officialOriginalRenderMotion();
  const h = nativeHoldRequestHardware();
  const moving = officialCommandValues().some(v => v !== 0);
  el('motionStand').textContent = '进入模型默认站姿';
  el('walkModelForwardLocked').textContent = moving ? '回零速 · 保持平衡' : '持续前进 · 0.30 m/s';
  el('walkModelForwardLocked').title = '再次点击回零速；没有2秒或每轮一次限制';
  el('walkModelForwardReason').textContent = nativeForwardBlockedReason()
    || '持续执行，直到点击回零速；紧急停止使用红色停止按钮。';
  el('walkModelTestGuide').textContent = '连接后启动行走模型；零速时保持平衡，再点击持续前进或交给手柄。';
  el('motionWalkGateStatus').textContent = nativeZeroBalanceBlockingReason(h)
    || '按官方恢复流程运行；启动与每帧控制均由 X5 检查实时反馈。';
  // Experimental joint probes are separate maintenance operations, not gates
  // on the normal controller. Keep the rest of the existing workbench intact.
  const panel = el('trackingProbePanel'); if (panel) panel.hidden = true;
};
