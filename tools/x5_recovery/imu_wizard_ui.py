#!/usr/bin/env python3
"""Pure HTML adapter for the supported, torque-off IMU calibration wizard.

This module never contacts hardware. The caller embeds the returned HTML in the
existing RAM-only calibration service; every sample remains an explicit click.
"""

import inspect
import re


# Crop the same official-model rendering already shown by this calibration
# page. Only the model crop rotates; the support and floor stay stationary.
POSE_SUPPORT = '''<g class="imu-pose-support"><path d="M29 245H271" stroke="#c6d1d6" stroke-width="2"/><path d="M137 243V171H163V243M127 171H173" fill="#e3eaee" stroke="#9eafb7" stroke-width="2"/></g>'''
SIDE_MODEL = '''<svg x="68" y="26" width="164" height="204" viewBox="525 175 335 505" preserveAspectRatio="xMidYMid meet" overflow="hidden"><image href="/zero-reference.png" x="0" y="0" width="1400" height="840"/></svg>'''
FRONT_MODEL = '''<svg x="68" y="26" width="164" height="204" viewBox="75 175 355 505" preserveAspectRatio="xMidYMid meet" overflow="hidden"><image href="/zero-reference.png" x="0" y="0" width="1400" height="840"/></svg>'''

LEVEL_SVG = '''<svg viewBox="0 0 300 270" role="img" aria-label="复用模型参考图的侧视鸭子，机身放平、躯干由支架承重，前端朝图右方"><g class="imu-pose-model">''' + SIDE_MODEL + '''</g>''' + POSE_SUPPORT + '''<text x="20" y="17" fill="currentColor" font-size="12">侧面看 · 前端 / 鸭嘴方向 →</text><text x="76" y="262" fill="currentColor" font-size="11">躯干由支架承重，脚不承重</text></svg>'''

FRONT_UP_SVG = '''<svg viewBox="0 0 300 270" role="img" aria-label="复用模型参考图的侧视鸭子，前端朝图右方，将这一端抬高约二十到四十度"><g class="imu-pose-model" transform="rotate(-25 150 150)">''' + SIDE_MODEL + '''</g>''' + POSE_SUPPORT + '''<path d="M258 113V68M252 76L258 65L264 76" fill="none" stroke="#25767f" stroke-width="3"/><text x="20" y="17" fill="currentColor" font-size="12">侧面看 · 前端 / 鸭嘴方向 →</text><text x="78" y="262" fill="currentColor" font-size="11">抬高图右侧约 20–40°，停稳</text></svg>'''

LEFT_UP_SVG = '''<svg viewBox="0 0 300 270" role="img" aria-label="复用模型参考图的正视鸭子，从鸭嘴正前方看，鸭子自身左侧是图右侧，将这一侧抬高二十到四十度"><g class="imu-pose-model" transform="rotate(-25 150 150)">''' + FRONT_MODEL + '''</g>''' + POSE_SUPPORT + '''<path d="M258 113V68M252 76L258 65L264 76" fill="none" stroke="#25767f" stroke-width="3"/><text x="20" y="17" fill="currentColor" font-size="12">从鸭嘴正前方看</text><text x="72" y="262" fill="currentColor" font-size="11">鸭子左侧（图右边）抬高</text></svg>'''


IMU_WIZARD_PANEL = '''<section class="card" aria-labelledby="imuCalTitle">
<div class="card-top"><h2 class="card-title" id="imuCalTitle"><span class="step">03</span>机身感知 · 跟着摆 4 次</h2><span class="cal-count" id="imuCalState">未校准</span></div>
<div class="motion-body" id="imuWizardPanel">
<div class="imu-live-panel" aria-labelledby="imuLiveTitle">
<div class="imu-live-heading"><h3 id="imuLiveTitle">IMU 实时读数</h3><span id="imuLiveState" role="status">等待反馈</span></div>
<p id="imuLiveDetail" class="motion-note">连接后会显示传感器读数；数值无需手动填写。</p>
<div class="imu-live-grid"><div>传感器加速度<strong id="imuSensorAcceleration">—</strong><small>传感器 X / Y / Z · m/s² · 含重力</small></div><div>传感器角速度<strong id="imuSensorGyro">—</strong><small>传感器 X / Y / Z · °/s</small></div></div>
<p id="imuBodyState" class="motion-note">完成引导后，再显示相对机身的姿态。</p>
<div class="imu-live-grid imu-body-readings"><div>机身加速度<strong id="imuBodyAcceleration">— · 待校准</strong><small>前 / 左 / 上 · m/s²</small></div><div>机身角速度<strong id="imuBodyGyro">— · 待校准</strong><small>绕前后 / 左右 / 上下 · °/s</small></div><div>侧倾<strong id="imuLiveRoll">— · 待校准</strong></div><div>前后俯仰<strong id="imuLivePitch">— · 待校准</strong></div><div class="imu-live-wide">重力方向<strong id="imuLiveGravity">— · 待校准</strong><small>前 / 左 / 上 · 单位向量</small></div></div>
</div>
<p class="motion-help">先把 IMU 牢固固定在躯干上，靠近机身中部即可。推荐平贴在躯干平整面上，板边沿机身前后方向摆正。跟着下面的图摆机身，网页会识别朝向，不用选择 X / Y / Z。</p>
<label class="support-check"><input id="imuMountedConfirmed" type="checkbox"><span>IMU 已固定在躯干上，不会晃动</span></label>
<p class="motion-note">用稳固垫块或支架承重；每次摆稳后点击，采样约 2 秒，不用一直手托。慢慢调整支撑，不拉扯电线。全程舵机保持释放。</p>
<p class="motion-note">IMU 固定后的位置或朝向改变了，需要重新校准。</p>
<p class="motion-note">下图使用本页模型参考图，只看机身的倾斜方向；支架始终托住躯干，脚不承重。</p>
<div class="imu-wizard-steps">
<article class="imu-wizard-step" id="imuWizardStepLevel"><h3><b>1</b> 放平机身</h3>''' + LEVEL_SVG + '''<p>机身放平，保持静止。</p><button class="button primary" id="imuWizardLevel" type="button" disabled>采集水平姿态</button></article>
<article class="imu-wizard-step" id="imuWizardStepFront"><h3><b>2</b> 抬高前端</h3>''' + FRONT_UP_SVG + '''<p>抬高鸭嘴那一端约 20–40°，停稳。</p><button class="button" id="imuWizardFront" type="button" disabled>采集前端抬高</button></article>
<article class="imu-wizard-step" id="imuWizardStepLeft"><h3><b>3</b> 抬高左侧</h3>''' + LEFT_UP_SVG + '''<p>先回水平，再抬高鸭子自身左侧约 20–40°，停稳。</p><button class="button" id="imuWizardLeft" type="button" disabled>验证左侧抬高</button></article>
<article class="imu-wizard-step" id="imuWizardStepFinish"><h3><b>4</b> 回到水平</h3>''' + LEVEL_SVG + '''<p>恢复水平，保持静止，保存校准。</p><button class="button" id="imuWizardFinish" type="button" disabled>完成静止校准</button></article>
</div>
<p id="imuWizardStatus" class="motion-note" role="status" aria-live="polite">固定 IMU 并确认支架承重后，从第 1 步开始。</p>
<progress id="imuWizardProgress" max="1" value="0" hidden aria-label="当前姿态采样进度"></progress>
<p id="imuWizardGate" class="motion-note" role="status">等待真实反馈。</p>
<div class="motion-actions"><button class="button" id="imuWizardCancel" type="button" disabled>取消本次 / 重新开始</button></div>
<details id="imuAdvancedSettings"><summary>高级设置 · 手动核对传感器轴向</summary><p class="motion-note">使用上面的引导就无需操作这些设置。引导进行中，这些设置会暂时锁定。</p><!--IMU_ADVANCED_CONTENT--></details>
</div></section>'''


IMU_WIZARD_SCRIPT = r'''<style>
.imu-wizard-steps{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin:16px 0}
.imu-live-panel{margin:0 0 18px;padding:14px;background:var(--panel,#f7fafb);border:1px solid var(--line,#d7e0e5);border-radius:12px}.imu-live-heading{display:flex;align-items:center;justify-content:space-between;gap:10px}.imu-live-heading h3{margin:0;font-size:15px}#imuLiveState{font-size:12px;color:#52798c}.imu-live-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.imu-live-grid>div{font-size:12px;min-width:0}.imu-live-grid strong{display:block;margin:5px 0;font-size:14px;font-weight:600;overflow-wrap:anywhere;font-variant-numeric:tabular-nums}.imu-live-grid small{display:block;color:var(--muted,#687b86);font-size:11px}.imu-live-wide{grid-column:1/-1}.imu-body-readings{border-top:1px solid var(--line,#d7e0e5);padding-top:12px}
.imu-wizard-step{padding:14px;border:1px solid var(--line,#d7e0e5);border-radius:12px;min-width:0}
.imu-wizard-step h3{margin:0 0 8px;font-size:15px}.imu-wizard-step h3 b{display:inline-flex;width:24px;height:24px;align-items:center;justify-content:center;margin-right:6px;border-radius:50%;background:#edf2f4;color:#4b6575}
.imu-wizard-step>svg{display:block;width:100%;max-height:230px}.imu-wizard-step p{min-height:3em;font-size:13px;line-height:1.5}.imu-wizard-step .button{width:100%}
.imu-wizard-step[data-active="true"]{border-color:#25767f;box-shadow:0 0 0 1px #25767f}.imu-wizard-step[data-complete="true"] h3 b{background:#dceee8;color:#28644f}
#imuWizardProgress{display:block;width:100%;height:12px;margin:10px 0}#imuWizardProgress[hidden]{display:none}#imuWizardGate{color:#9c642b}#imuAdvancedSettings{margin-top:20px}#imuAdvancedSettings>summary{cursor:pointer;font-size:13px}
@media(max-width:600px){.imu-wizard-steps{grid-template-columns:1fr}.imu-wizard-step p{min-height:0}}@media(max-width:360px){.imu-live-grid{grid-template-columns:1fr}}
</style><script>
let imuWizardRequestBusy=false;
let imuWizardCancelBusy=false;
let imuWizardRequestAction=null;
let imuWizardLocalError='';
let imuWizardLastReceived=null;
let imuWizardEpoch=0;
const imuWizardSteps=[['level','imuWizardLevel','imuWizardStepLevel'],['front-up','imuWizardFront','imuWizardStepFront'],['left-up','imuWizardLeft','imuWizardStepLeft'],['finish','imuWizardFinish','imuWizardStepFinish']];
const imuWizardLabels={'level':'采集水平姿态','front-up':'采集前端抬高','left-up':'验证左侧抬高','finish':'完成静止校准'};
const imuWizardReceiveBefore=receiveMotion;
receiveMotion=function(status){
  if(status&&typeof status==='object'&&typeof status.mode==='string')imuWizardLastReceived=performance.now();
  return imuWizardReceiveBefore(status);
};
function imuWizardState(){return motion.status?.imu_wizard||null}
function imuWizardActive(){const w=imuWizardState();return imuWizardRequestBusy||!!w&&(w.busy===true||!['idle','complete'].includes(w.stage))}
function imuLiveVector(value){return Array.isArray(value)&&value.length===3&&value.every(Number.isFinite)?value:null}
function imuLiveAxes(axes){
  const allowed=['x','-x','y','-y','z','-z'];
  if(!Array.isArray(axes)||axes.length!==3||!axes.every(a=>typeof a==='string'&&allowed.includes(a)))return null;
  const indices=axes.map(a=>'xyz'.indexOf(a.slice(-1)));
  if(new Set(indices).size!==3)return null;
  let inversions=0;for(let i=0;i<3;i++)for(let j=i+1;j<3;j++)if(indices[i]>indices[j])inversions++;
  if((inversions+axes.filter(a=>a.startsWith('-')).length)%2!==0)return null;
  return axes.map((axis,i)=>({index:indices[i],sign:axis.startsWith('-')?-1:1}));
}
function imuLiveInverse(axes,vector){
  const mapping=imuLiveAxes(axes);const values=imuLiveVector(vector);if(!mapping||!values)return null;
  const raw=[0,0,0];mapping.forEach((axis,i)=>raw[axis.index]=axis.sign*values[i]);return raw;
}
function imuLiveSensorVectors(imu){
  const hasAcc=Object.prototype.hasOwnProperty.call(imu,'acc_sensor');const hasGyro=Object.prototype.hasOwnProperty.call(imu,'gyro_sensor');
  const acc=hasAcc?imuLiveVector(imu.acc_sensor):imuLiveInverse(imu.axes,imu.acc_body);
  let gyro=hasGyro?imuLiveVector(imu.gyro_sensor):imuLiveInverse(imu.axes,imu.gyro_body);
  if(!hasGyro){const bias=imuLiveVector(imu.bias_dps);gyro=gyro&&bias?gyro.map((v,i)=>v+bias[i]):null;if(gyro&&!gyro.every(Number.isFinite))gyro=null}
  return {acc,gyro};
}
function imuLiveFormat(vector){const value=imuLiveVector(vector);return value?value.map(v=>v.toFixed(2)).join(' / '):'— · 读数缺失'}
function imuLiveAvailability(){
  const s=motion.status;
  if(s?.paused===true||s?.read_only===true)return {fresh:false,label:'反馈已暂停',detail:(s.pause_reason||s.error||'反馈采样已停止')+'；旧读数已隐藏，已保存校准保留。'};
  if(!s?.connected)return {fresh:false,label:'反馈离线',detail:'没有实时 IMU 反馈。连接恢复前不显示旧数值。'};
  const elapsed=imuWizardLastReceived===null?Infinity:performance.now()-imuWizardLastReceived;
  if(!hardwareFresh()||!Number.isFinite(elapsed)||elapsed<0||elapsed>=1500||!Number.isFinite(s.age_ms)||s.age_ms<0||s.age_ms>=100||!Number.isFinite(s.imu_age_ms)||s.imu_age_ms<0||s.imu_age_ms>=100)return {fresh:false,label:'反馈已陈旧',detail:'等待新数据；旧的加速度、角速度和姿态数值已隐藏。'};
  if(s.imu_feedback_ok!==true)return {fresh:false,label:'IMU 未就绪',detail:'暂未收到有效 IMU 数据，请检查 IMU 连接。'};
  const rate=Number.isFinite(s.rates?.state_hz)&&s.rates.state_hz>=0?' · 状态反馈 '+s.rates.state_hz+' Hz':'';
  return {fresh:true,label:'传感器实时反馈',detail:'距 IMU 采样约 '+Math.round(s.imu_age_ms+elapsed)+' ms · '+Math.round(elapsed)+' ms 前收到状态'+rate+'。'};
}
function renderImuLive(){
  const s=motion.status||{};const imu=s.imu||{};const live=imuLiveAvailability();
  $('imuLiveState').textContent=live.label;$('imuLiveDetail').textContent=live.detail;
  const raw=live.fresh?imuLiveSensorVectors(imu):{acc:null,gyro:null};
  $('imuSensorAcceleration').textContent=live.fresh?imuLiveFormat(raw.acc):'— · '+live.label;
  $('imuSensorGyro').textContent=live.fresh?imuLiveFormat(raw.gyro):'— · '+live.label;
  const axesConfirmed=imu.axes_confirmed===true&&!!imuLiveAxes(imu.axes);
  const bodyConfirmed=axesConfirmed&&imu.calibrated===true&&!imuWizardActive();
  const bodyFresh=live.fresh&&bodyConfirmed;
  const unavailable=live.fresh?'待校准':live.label;
  const bodyUnavailable=live.fresh?(imuWizardActive()?'引导校准中':axesConfirmed?'待静止校准':'待朝向识别'):live.label;
  $('imuBodyAcceleration').textContent=bodyFresh?imuLiveFormat(imu.acc_body):'— · '+bodyUnavailable;
  $('imuBodyGyro').textContent=bodyFresh?imuLiveFormat(imu.gyro_body):'— · '+bodyUnavailable;
  $('imuLiveRoll').textContent=bodyFresh&&Number.isFinite(imu.roll_deg)?imu.roll_deg.toFixed(1)+'°':'— · '+(bodyFresh?'等待姿态反馈':unavailable);
  $('imuLivePitch').textContent=bodyFresh&&Number.isFinite(imu.pitch_deg)?imu.pitch_deg.toFixed(1)+'°':'— · '+(bodyFresh?'等待姿态反馈':unavailable);
  $('imuLiveGravity').textContent=bodyFresh&&imuLiveVector(imu.gravity)?imuLiveFormat(imu.gravity):'— · '+(bodyFresh?'等待姿态反馈':unavailable);
  $('imuBodyState').textContent=!live.fresh?'实时数值已隐藏，校准配置仍保留。':!bodyConfirmed?'当前显示传感器读数；完成 4 步引导后，才按机身的前、左、上解释方向。':Number.isFinite(imu.roll_deg)&&Number.isFinite(imu.pitch_deg)&&imuLiveVector(imu.gravity)?'机身朝向与静止校准已确认，下面显示机身数据和姿态。':'机身朝向与静止校准已确认；姿态尚待模型反馈就绪，已显示机身加速度与角速度。';
  if(!bodyFresh){$('realRoll').textContent='— · '+unavailable;$('realPitch').textContent='— · '+unavailable;$('realAcceleration').textContent='— · '+bodyUnavailable;$('realGyro').textContent='— · '+bodyUnavailable}
}
function imuWizardGateReason(action){
  const s=motion.status;const w=imuWizardState();
  if(!state.token)return '测试服务尚未连接，已保存的关节零点不会丢。';
  if(action==='cancel')return '';
  if(!s?.connected)return '硬件反馈已暂停'+(s?.error?'：'+s.error:'。')+' 已保存的关节零点不会丢。';
  if(!w)return '当前服务尚未提供引导校准，请等待服务更新。';
  if(!Number.isInteger(s.generation)||!Number.isInteger(w.generation)||w.generation!==s.generation)return '校准会话已变化，请等待最新状态后再操作。';
  const elapsed=imuWizardLastReceived===null?Infinity:performance.now()-imuWizardLastReceived;
  if(!hardwareFresh()||!Number.isFinite(elapsed)||elapsed<0||elapsed>=1500)return '反馈已过期，请等待新状态。已保存的关节零点不会丢。';
  if(s.mode!=='off'||s.armed!==false||s.motor_output!==false||!Array.isArray(s.torque_ids)||s.torque_ids.length)return '需要先停止并确认全部舵机已释放。';
  if(!Array.isArray(s.fault_ids)||s.fault_ids.length)return '舵机反馈有故障'+(s.fault_ids?.length?'：ID '+s.fault_ids.join('、'):'')+'。请先处理，零点仍已保存。';
  if(!Array.isArray(s.online_ids)||s.online_ids.length!==15||!motorIds.every(id=>s.online_ids.includes(id))){const missing=motorIds.filter(id=>!s.online_ids?.includes(id));return '等待全部舵机在线'+(missing.length?'，缺少 ID '+missing.join('、'):'')+'。零点仍已保存。'}
  if(!Number.isFinite(s.age_ms)||s.age_ms<0||s.age_ms>=100)return '舵机反馈不够新，请等待连接恢复稳定。';
  if(s.imu_feedback_ok!==true||!Number.isFinite(s.imu_age_ms)||s.imu_age_ms<0||s.imu_age_ms>=100)return 'IMU 反馈未就绪或已过期，请检查 IMU 连接。';
  if(!$('imuMountedConfirmed').checked)return '先把 IMU 固定在躯干上，再勾选安装确认。';
  if(!$('supportConfirmed').checked)return '先用垫块 / 支架承重，并勾选上方的机身支撑确认。';
  if(motion.axesDirty)return '高级轴向设置有未保存修改。请还原或刷新页面，再使用引导校准。';
  if(s.imu?.calibrating===true&&!w.busy)return '当前正在做静止采样，请等待结束。';
  return '';
}
function imuWizardCan(action){
  const w=imuWizardState();
  if(action==='cancel')return !!state.token&&!imuWizardCancelBusy&&imuWizardActive();
  if(imuWizardRequestBusy||motion.busy||motion.stopping||w?.busy===true||imuWizardGateReason(action))return false;
  if(action==='level'&&w?.stage==='complete')return true;
  return w?.expected_action===action;
}
const imuWizardRenderBefore=renderMotion;
renderMotion=function(){
  imuWizardRenderBefore();
  renderImuLive();
  const w=imuWizardState();const busy=imuWizardRequestBusy||w?.busy===true;
  const activeAction=imuWizardRequestAction||(w?.busy?w.stage:w?.expected_action);
  const nextIndex=w?.stage==='complete'?4:imuWizardSteps.findIndex(step=>step[0]===w?.expected_action);
  for(const [index,[action,buttonId,stepId]] of imuWizardSteps.entries()){
    const button=$(buttonId);button.disabled=!imuWizardCan(action);
    button.textContent=busy&&action===activeAction?'采样中，请保持静止…':action==='level'&&w?.stage==='complete'?'重新校准 · 从第 1 步开始':(w?.stage==='failed'&&w.expected_action===action?'重试 · ':'')+imuWizardLabels[action];
    $(stepId).setAttribute('data-active',String(action===activeAction));$(stepId).setAttribute('data-complete',String(index<nextIndex));
  }
  $('imuWizardCancel').disabled=!imuWizardCan('cancel');
  $('imuWizardCancel').textContent=imuWizardCancelBusy?'正在取消…':'取消本次 / 重新开始';
  $('imuMountedConfirmed').disabled=busy||motion.busy||motion.stopping||imuWizardActive();
  if(imuWizardActive()||busy){
    for(const id of ['imuAxisX','imuAxisY','imuAxisZ','imuAxesConfirmed','saveImuAxes','calibrateImu','shadowMotion','zeroReferenceConfirmed'])$(id).disabled=true;
    if(typeof canCaptureAllZeros==='function'){$('captureAllZeros').disabled=true;$('captureZerosReference').disabled=true}
    for(const row of motion.rows.values()){
      for(const field of ['zero','min','max'])row.captures[field].button.disabled=true;
      row.sign.disabled=true;row.verified.disabled=true;
    }
  }
  const progress=Number.isFinite(w?.progress)?Math.max(0,Math.min(1,w.progress)):0;
  $('imuWizardProgress').hidden=!busy;$('imuWizardProgress').value=progress;
  const message=imuWizardLocalError||w?.error||w?.message||'固定 IMU 并确认支架承重后，从第 1 步开始。';
  $('imuWizardStatus').textContent=busy?(w?.message||'正在采集，请保持机身静止。')+' · '+Math.round(progress*100)+'%':message;
  const reason=imuWizardGateReason(w?.expected_action||'level');
  $('imuWizardGate').textContent=reason||((w?.stage==='complete')?'朝向和静止校准已保存；机身继续由支架承重，无需手托。':busy?'等待本步采样完成，不要调整姿态。':'摆稳后点击对应按钮；每一步都由你点击开始。');
  if(w?.stage==='complete'&&!busy)$('imuCalState').textContent='引导校准已保存 ✓';
};
async function imuWizardRun(action){
  if(action==='cancel')return imuWizardCancel();
  if(!imuWizardCan(action))return;
  const s=motion.status;
  const epoch=imuWizardEpoch;const motionEpoch=motion.epoch;
  const body={action,generation:s.generation,support_confirmed:$('supportConfirmed').checked,mounted_confirmed:$('imuMountedConfirmed').checked};
  imuWizardRequestBusy=true;imuWizardRequestAction=action;imuWizardLocalError='';motion.busy=true;renderMotion();
  const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),10000);
  try{
    const status=await api('/api/motion/imu-wizard',body,controller);
    if(epoch!==imuWizardEpoch||motionEpoch!==motion.epoch)return;
    receiveMotion(status);
    imuWizardLocalError=status.imu_wizard?.error||status.error||'';motionError(imuWizardLocalError);
  }catch(error){
    if(epoch!==imuWizardEpoch||motionEpoch!==motion.epoch)return;
    imuWizardLocalError=error.name==='AbortError'?'采样请求超时。请等待状态刷新，确认本步结果后再点击；不会自动重试。':error.message;
    motionError(imuWizardLocalError);
  }finally{clearTimeout(timeout);imuWizardRequestBusy=false;imuWizardRequestAction=null;motion.busy=imuWizardCancelBusy;renderMotion();}
}
async function imuWizardCancel(){
  if(!imuWizardCan('cancel'))return;
  const epoch=++imuWizardEpoch;
  const body={action:'cancel',generation:Number.isInteger(motion.status?.generation)?motion.status.generation:0,support_confirmed:$('supportConfirmed').checked,mounted_confirmed:$('imuMountedConfirmed').checked};
  imuWizardCancelBusy=true;imuWizardLocalError='';renderMotion();
  const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),5000);
  try{
    const status=await api('/api/motion/imu-wizard',body,controller);
    if(epoch!==imuWizardEpoch)return;
    receiveMotion(status);imuWizardLocalError=status.imu_wizard?.error||status.error||'';
    motionError(imuWizardLocalError);
  }catch(error){
    if(epoch!==imuWizardEpoch)return;
    imuWizardLocalError=error.name==='AbortError'?'取消请求超时。请等待状态刷新确认结果；不会自动重新采样。':error.message;
    motionError(imuWizardLocalError);
  }finally{clearTimeout(timeout);imuWizardCancelBusy=false;motion.busy=imuWizardRequestBusy;renderMotion();}
}
for(const [action,buttonId] of imuWizardSteps)$(buttonId).onclick=()=>imuWizardRun(action);
$('imuWizardCancel').onclick=()=>imuWizardRun('cancel');
$('imuMountedConfirmed').onchange=()=>renderMotion();
renderMotion();
setInterval(renderMotion,250);
</script>'''


def transform_html(content):
    """Replace exactly one known IMU card while retaining its legacy DOM IDs."""
    pattern = r'<section class="card" aria-labelledby="imuCalTitle">([\s\S]*?)</section>'
    matches = list(re.finditer(pattern, content))
    if len(matches) != 1:
        raise RuntimeError("Reviewed IMU calibration panel changed")
    old = matches[0].group(0)
    body_marker = '<div class="motion-body">'
    if old.count(body_marker) != 1 or not old.endswith('</div></section>'):
        raise RuntimeError("Reviewed IMU calibration body changed")
    legacy = old.split(body_marker, 1)[1][:-len('</div></section>')]
    for dom_id in ('imuAxisX', 'imuAxisY', 'imuAxisZ', 'imuAxesConfirmed',
                   'saveImuAxes', 'calibrateImu', 'imuProgress', 'realRoll',
                   'realPitch', 'realAcceleration', 'realGyro'):
        if legacy.count('id="' + dom_id + '"') != 1:
            raise RuntimeError("Reviewed IMU DOM ID changed: " + dom_id)
    if content.count('</body>') != 1 or 'id="imuWizardPanel"' in content:
        raise RuntimeError("Reviewed page body changed or wizard already installed")
    panel = IMU_WIZARD_PANEL.replace('<!--IMU_ADVANCED_CONTENT-->', legacy, 1)
    result = content[:matches[0].start()] + panel + content[matches[0].end():]
    return result.replace('</body>', IMU_WIZARD_SCRIPT + '</body>', 1)


def runtime_source():
    """Return the same pure transform as source for the RAM supervisor."""
    return ("import re\nIMU_WIZARD_PANEL=" + repr(IMU_WIZARD_PANEL)
            + "\nIMU_WIZARD_SCRIPT=" + repr(IMU_WIZARD_SCRIPT) + "\n"
            + inspect.getsource(transform_html)
            + "\ntransform_imu_wizard_html=transform_html\n")
