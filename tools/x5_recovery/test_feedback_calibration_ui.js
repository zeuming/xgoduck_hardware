'use strict';

// Pure local UI tests. All fetches, timers, images and DOM nodes are fake.
// This file never imports the host/reader or contacts HTTP, SSH or serial ports.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {execFileSync} = require('node:child_process');
const html = fs.readFileSync(path.join(__dirname, 'feedback_calibration.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script);
const clone = value => JSON.parse(JSON.stringify(value));
const tick = () => new Promise(resolve => setImmediate(resolve));
function deferred() { let resolve; const promise = new Promise(r => { resolve = r; }); return {promise, resolve}; }
const response = (data, status = 200) => ({ok: status >= 200 && status < 300, status, json: async () => clone(data)});
function fixture(status = 'fault', generation = 101) {
  return {
    token: 'token-' + generation,
    session: {status, generation, error: status === 'fault' ? '真实反馈超时' : null, received_age_ms: 0, recovery_allowed: ['fault', 'closed'].includes(status), recovery_pending: false, recovery_blocked_reason: null},
    feedback: {sample_us: 12000, imu_us: 11990, online_mask: 0, fault_mask: 0, torque_mask: 0, arm_cookie: 0, armed: false, imu_ok: true, positions: Array(15).fill(2048), accel_ms2: [0, 0, 9.81], gyro_dps: [0.01, 0.02, 0.03]},
    calibration: {joints: [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34].map(id => ({id, zero: 2048, direction_confirmed: false})), imu: {axes_confirmed: false, calibrated: false}},
    wizard: {stage: 'idle', expected_action: 'level', busy: false, progress: 0, sample_count: 0, message: '', error: null}
  };
}
function harness(protocol='http:', hash='', proofHTML=null) {
  const activeHTML=proofHTML||html,activeScript=proofHTML?proofHTML.match(/<script>([\s\S]*?)<\/script>/)[1]:script;
  const nodes = new Map(), calls = [], timers = new Map(), intervals = [], oldGet = deferred();
  const listeners = new Map();
  let timerId = 0, now = 0, firstGet = true, current = fixture(), post = async () => response(current);
  function node(tagName='div') {
    const classes = new Set(), attributes = new Map(), svg = tagName.toLowerCase()==='svg';
    // SVG .hidden is an unreflected expando in browsers. Only the attribute
    // affects [hidden] CSS; this distinction catches stale SVG illustrations.
    const result = {insertBefore(child, reference) { this.append(child);const index=this.children.indexOf(reference);if(index>=0){this.children=this.children.filter(item=>item!==child);this.children.splice(index,0,child);}return child; }, scrollIntoView(options) { this.lastScroll = options; }, focus() { this.focused = true; }, textContent: '', hidden: svg?undefined:false, disabled: false, checked: false, dataset: {}, children: [], value: 0, append(...children) { for(const child of children){if(child.parentElement)child.parentElement.children=child.parentElement.children.filter(item=>item!==child);child.parentElement=this;this.children.push(child);} }, replaceChildren(...children) { for(const child of this.children)child.parentElement=null;this.children=[];this.append(...children); }, setAttribute(name, value) { attributes.set(name,String(value));if(name==='hidden'){if(!svg)this.hidden=true;}else this[name] = String(value); }, getAttribute(name) { return name==='hidden'?(attributes.get(name)??null):(this[name]??null); }, removeAttribute(name) { attributes.delete(name);if(name==='hidden'){if(!svg)this.hidden=false;}else delete this[name]; }, classList: {toggle(name, enabled) { enabled ? classes.add(name) : classes.delete(name); }, contains(name) { return classes.has(name); }}};
    Object.defineProperty(result, 'innerHTML', {set() { throw new Error('Unsafe innerHTML write'); }});
    return result;
  }
  for (const match of activeHTML.matchAll(/<([a-zA-Z][\w:-]*)\b[^>]*\bid="([^"]+)"[^>]*>/g)) {
    const element=node(match[1]);if(/\bhidden(?:\s|=|>)/.test(match[0]))element.setAttribute('hidden','');
    nodes.set(match[2],element);
  }
  const ctx = vm.createContext({
    location: {protocol, hash, host:'192.168.1.11:8767'},
    document: {querySelector() { return null; }, getElementById(id) { assert(nodes.has(id), 'Unknown DOM id ' + id); return nodes.get(id); }, createElement: node,
      addEventListener(name, listener) { if (!listeners.has(name)) listeners.set(name, []); listeners.get(name).push(listener); }},
    addEventListener(name, listener) { if (!listeners.has(name)) listeners.set(name, []); listeners.get(name).push(listener); },
    Image: class {}, performance: {now: () => now}, AbortController,
    setInterval(callback) { intervals.push(callback); },
    setTimeout(callback, ms) { timers.set(++timerId, {callback, ms}); return timerId; }, clearTimeout(id) { timers.delete(id); },
    async fetch(url, options) {
      calls.push({url, options});
      if (url === '/api/status' || url === '/api/live-status') { if (firstGet) { firstGet = false; return oldGet.promise; } return response(current); }
      return post(url, options);
    }
  });
  ctx.window = ctx;
  vm.runInContext(activeScript, ctx);
  return {
    nodes, calls, timers, intervals, oldGet,
    eval: source => vm.runInContext(source, ctx),
    ownMotion() { vm.runInContext('claimMotionControl(snapshot);renderMotion();',ctx); },
    emit(name) { for (const listener of listeners.get(name) || []) listener({type:name}); },
    navigate(nextHash) { ctx.location.hash = nextHash; for (const listener of listeners.get('hashchange') || []) listener({type:'hashchange'}); if (typeof ctx.onhashchange === 'function') ctx.onhashchange({type:'hashchange'}); },
    set(data) { current = clone(data); ctx.fixture = clone(data); vm.runInContext('acceptSnapshot(fixture); assetOk=true; positiveAssetOk=true; render();', ctx); },
    post(handler) { post = handler; },
    age(ms) { now += ms; vm.runInContext('render();', ctx); },
    async settleOld(data) { oldGet.resolve(response(data)); await tick(); },
    posts: () => calls.filter(call => call.options?.method === 'POST')
  };
}
const stepIds = ['wizardLevel', 'wizardFront', 'wizardLeft', 'wizardFinish'];
function locked(h) { for (const id of stepIds) assert.equal(h.nodes.get(id).disabled, true, id); }
const tests = [];
const test = (name, fn) => tests.push([name, fn]);

// Optional local proof from an actual saved status + served native HTML pair.
// Render only; never print the snapshot, token, raw evidence or request bodies.
if(process.env.XGODUCK_UI_SNAPSHOT){
  assert(process.env.XGODUCK_UI_HTML,'Supply the saved served HTML with the saved status');
  const raw=fs.readFileSync(process.env.XGODUCK_UI_SNAPSHOT,'utf8');assert(raw.length<=2*1024*1024,'Saved snapshot exceeds proof limit');
  const actual=JSON.parse(raw),proofHTML=fs.readFileSync(process.env.XGODUCK_UI_HTML,'utf8');if(actual.native_access?.full_workbench===true)assert(proofHTML.includes('Both passive calibration and explicit motion run on this X5.'),'Full native adaptation is required for this snapshot');
  const h=harness('http:','#motion-models',proofHTML);h.set(actual);
  const caption=h.nodes.get('x5ConnectionTarget'),connectionText=caption.textContent;
  if(actual.native_access?.full_workbench===true){
    assert.equal(connectionText,'X5 独立工作台 · 192.168.1.11:8767；采集、维护和运控均在 X5 运行。');
    for(const route of ['device-status','servo-settings','joint-zeros','joint-steps','imu-steps','motion-models']){
      h.navigate('#'+route);
      for(let repeat=0;repeat<10;repeat++){h.intervals[1]();h.eval('nativeOriginalRender()');assert.equal(caption.textContent,connectionText);assert.equal(caption.hidden,false);}
    }
  }
  let stableMotionConnection=null;
  let supportConfirmationRetained=null;
  let savedFramesRendered=0;
  if(h.eval('motionConnectionEstablished()')){
    h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionSupportConfirmed').onchange();
    const calibrationText=h.nodes.get('motionCalibration').textContent,recoveryHidden=h.nodes.get('motionRecoveryControls').hidden;
    const anglesBefore=['motionImuTilt','motionImuPitch','motionImuRoll'].map(id=>h.nodes.get(id).textContent);
    const positionsBefore=h.eval('Array.from(motionRows.values(),cells=>cells[1].textContent)');
    for(let repeat=0;repeat<10;repeat++){
      h.age(150);h.intervals[1]();h.eval('nativeOriginalRender()');
      assert.equal(h.nodes.get('motionConnect').textContent,'运控已连接');
      assert.equal(h.nodes.get('motionRecoveryControls').hidden,recoveryHidden);
      assert.equal(h.nodes.get('motionCalibration').textContent,calibrationText);
      assert.equal(h.nodes.get('motionSupportConfirmed').checked,true);
      assert.deepEqual(['motionImuTilt','motionImuPitch','motionImuRoll'].map(id=>h.nodes.get(id).textContent),anglesBefore);
      assert.equal(JSON.stringify(h.eval('Array.from(motionRows.values(),cells=>cells[1].textContent)')),JSON.stringify(positionsBefore));
    }
    assert.equal(h.eval('motionFresh()'),false);assert.equal(h.nodes.get('motionStand').disabled,true);
    h.set(actual);assert.equal(h.nodes.get('motionSupportConfirmed').checked,true);
    if(h.eval('motionFresh()')&&!actual.motion.hardware.start_blocked_reason)assert.equal(h.nodes.get('motionStand').disabled,false);
    supportConfirmationRetained=true;
    stableMotionConnection=true;
  }
  if(process.env.XGODUCK_UI_FRAMES){
    const frames=JSON.parse(fs.readFileSync(process.env.XGODUCK_UI_FRAMES,'utf8'));assert(Array.isArray(frames)&&frames.length<=100);
    for(const frame of frames){
      h.set(frame);const hardware=frame.motion.hardware,g=hardware.imu.gravity;
      const pitch=Math.atan2(g[0],-g[2])*180/Math.PI,expectedPitch=(Math.abs(pitch)<.05?0:pitch).toFixed(1)+'°';
      assert.equal(h.nodes.get('motionImuPitch').textContent,expectedPitch);
      for(const row of hardware.rawdata)assert.equal(h.eval('motionRows.get('+row.id+')[1].textContent'),row.position);
      h.age(2000);h.intervals[1]();
      assert.equal(h.nodes.get('motionImuPitch').textContent,expectedPitch);
      for(const row of hardware.rawdata)assert.equal(h.eval('motionRows.get('+row.id+')[1].textContent'),row.position);
      assert.equal(h.eval('motionFresh()'),false);assert.equal(h.nodes.get('motionStand').disabled,true);savedFramesRendered++;
    }
  }
  process.stdout.write(JSON.stringify({connection_target_text:connectionText,connection_caption_stable:true,motion_connection_stable_during_slow_poll:stableMotionConnection,support_confirmation_retained_through_slow_polls:supportConfirmationRetained,saved_frames_rendered_without_blanks:savedFramesRendered,tilt:h.nodes.get('motionImuTilt').textContent,pitch:h.nodes.get('motionImuPitch').textContent,roll:h.nodes.get('motionImuRoll').textContent,current_status:h.nodes.get('motionImuFreshness').textContent+' '+h.nodes.get('motionImuReadStatus').textContent,read_disabled:h.nodes.get('motionImuRead').disabled,end_disabled:h.nodes.get('motionImuEnd').disabled,motion_recheck_disabled:h.nodes.get('motionRecheck').disabled,motion_recheck_status:h.nodes.get('motionRecoveryStatus').textContent,motion_connect_disabled:h.nodes.get('motionConnect').disabled,motion_stand_disabled:h.nodes.get('motionStand').disabled,motion_walk_disabled:h.nodes.get('motionWalk').disabled,request_count:h.posts().length})+'\n');
  process.exit(0);
}

// Real pure JointWizard snapshots, generated without importing any transport.
const realJointFixtures = JSON.parse(execFileSync('python3', ['-B', '-c', `
import copy, json
from joint_calibration import JointWizard, IDS, completed_count
cfg = {'version':1, 'mechanical_reference_confirmed':True,
       'joints':[{'id':sid,'zero':2048,'sign':-1,'min':None,'max':None,'direction_confirmed':False} for sid in IDS],
       'imu':{'axes':['-z','-x','y'],'axes_confirmed':True,'calibrated':True,'bias_dps':[.01,.02,.03]}}
state = {'sample_us':0,'positions':[2048]*15}
wizard = JointWizard(); now = 0.; snapshots = {}
def snap(name):
    snapshots[name] = {'joint_wizard':wizard.snapshot(cfg),'calibration':copy.deepcopy(cfg),'positions':list(state['positions'])}
def capture(action, sid, value, name):
    global now
    state['positions'][IDS.index(sid)] = value
    wizard.start(action,sid,state,now); snap(action+'-busy'+('-mouth' if sid==34 else ''))
    for _ in range(20):
        now += .02; state['sample_us'] += 20000; wizard.tick(state,now,cfg)
    snap(name)
snap('idle')
capture('direction-start',10,2048,'await-positive')
capture('direction-end',10,2078,'await-range-a')
capture('range-a',10,1600,'await-range-b')
capture('range-b',10,2400,'ready')
cfg = wizard.proposed_config(cfg,10); wizard.saved(); snap('complete')
wizard.cancel(); snap('cancelled')
wizard.fail('采样时关节还在移动，本关节未保存'); snap('failed')
wizard.clear(); capture('direction-start',34,2048,'await-positive-mouth')
capture('direction-end',34,2080,'await-range-a-mouth')
capture('range-a',34,2048,'await-range-b-mouth')
capture('range-b',34,2110,'ready-mouth')
cfg = wizard.proposed_config(cfg,34); wizard.saved(); snap('complete-mouth')
good = copy.deepcopy(cfg)
for j in good['joints']: j.update(min=1500,max=2600,direction_confirmed=True)
snapshots['all-complete'] = {'calibration':good,'joint_wizard':dict(wizard.snapshot(good),completed_count=completed_count(good)),'positions':state['positions']}
bad = copy.deepcopy(good); head = bad['joints'][IDS.index(31)]; head.update(sign=-1,min=1822,max=2048)
snapshots['old-stand-insufficient'] = {'calibration':bad,'joint_wizard':dict(wizard.snapshot(bad),completed_count=completed_count(bad)),'positions':state['positions']}
cfg = copy.deepcopy(snapshots['idle']['calibration'])
for j in cfg['joints'][:2]: j.update(sign=-1,min=1500,max=2600,direction_confirmed=True)
cfg['joints'][IDS.index(12)]['zero'] = 2032
wizard.clear()
def retry_capture(action, value, name):
    global now
    state['positions'][IDS.index(12)] = value
    wizard.start(action,12,state,now); snap(name+'-busy')
    for _ in range(20):
        now += .02; state['sample_us'] += 20000; wizard.tick(state,now,cfg)
    snap(name)
retry_capture('direction-start',2200,'retry-direction-start')
retry_capture('direction-end',2160,'retry-direction-end')
retry_capture('range-a',2180,'retry-first-a')
retry_capture('range-b',2361,'range-retry')
retry_capture('range-a',1900,'range-retry-a')
retry_capture('range-b',2450,'range-retry-ready')
cfg = wizard.proposed_config(cfg,12); wizard.saved(); snap('range-retry-saved')
cfg = copy.deepcopy(snapshots['all-complete']['calibration']); wizard.clear()
for action, raw in [('direction-start',2048),('direction-end',2098)]:
    state['positions'][0] = raw; wizard.start(action,10,state,now)
    for _ in range(20):
        now += .02; state['sample_us'] += 20000; wizard.tick(state,now,cfg)
snap('direction-reuse')
cfg = wizard.proposed_direction_config(cfg,10); wizard.saved('方向已保存，沿用已保存范围'); snap('direction-reuse-saved')
print(json.dumps(snapshots))
`], {cwd: __dirname, encoding: 'utf8'}));
function jointFixture(stage = 'idle', generation = 101) {
  const s = fixture('active', generation), pure = clone(realJointFixtures[stage]);
  s.feedback.online_mask = 32767; s.feedback.positions = pure.positions;
  s.calibration = pure.calibration; s.joint_wizard = pure.joint_wizard;
  s.wizard = {stage: 'complete', expected_action: null, busy: false, progress: 1, sample_count: 100, message: 'IMU 已保存', error: null};
  return s;
}
const jointButtonIds = ['jointDirectionStart','jointDirectionEnd','jointRangeA','jointRangeB','jointSave'];
function jointLocked(h) { for (const id of jointButtonIds) assert.equal(h.nodes.get(id).disabled, true, id); }
function supportedJoint(h) { h.nodes.get('jointSupportConfirmed').checked = true; h.nodes.get('jointSupportConfirmed').onchange(); }

test('file URL shows the service link without polling, navigation or API requests', async () => {
  const h=harness('file:');
  assert.equal(h.nodes.get('localFileNotice').hidden,false);
  assert.equal(h.nodes.get('exportCalibration').hidden,true);
  assert(h.nodes.get('connectionLabel').textContent.includes('本地文件'));
  assert(html.includes('id="openCalibrationService" href="http://127.0.0.1:8767/#joint-steps"'));
  assert.equal(h.calls.length,0);assert.equal(h.intervals.length,0);assert.equal(h.timers.size,0);
  locked(h);jointLocked(h);
  await h.eval('refresh()');
  h.set(jointFixture('ready'));supportedJoint(h);h.nodes.get('jointPositiveConfirmed').checked=true;
  for(const action of ['direction-start','save','cancel'])await h.eval('runJoint('+JSON.stringify(action)+')');
  for(const action of ['level','cancel'])await h.eval('run('+JSON.stringify(action)+')');
  await h.eval('endFeedback()');h.set(fixture('fault'));await h.eval('recoverFeedback()');
  assert.equal(h.calls.length,0);assert.equal(h.intervals.length,0);assert.equal(h.timers.size,0);
  assert.equal(h.eval('location.protocol'),'file:');
  const normal=harness();assert.equal(normal.calls.length,1);assert.equal(normal.intervals.length,2);
});

test('mount and polling never recover; missing history refuses and displays raw reason safely', async () => {
  const h = harness(), s = fixture();
  s.session.recovery_allowed = false;
  s.session.recovery_blocked_reason = '原故障历史已丢失，拒绝恢复 <script>bad()</script>';
  s.session.error = s.session.recovery_blocked_reason;
  h.set(s); await h.settleOld(s); await h.eval('refresh()');
  assert.equal(h.posts().length, 0);
  assert.equal(h.nodes.get('recoverFeedback').disabled, true);
  assert.equal(h.nodes.get('recoverFeedbackNearFault').disabled, true);
  assert.equal(h.nodes.get('recoverFeedbackInWizard').disabled, true);
  assert(h.nodes.get('recoveryStatus').textContent.includes(s.session.recovery_blocked_reason));
  assert(h.nodes.get('recoveryWizardStatus').textContent.includes(s.session.recovery_blocked_reason));
  assert.equal(h.nodes.get('recoveryWizardControls').hidden, false);
  assert.equal(h.nodes.get('sessionAlert').textContent, s.session.error);
  await h.nodes.get('recoverFeedback').onclick();
  assert.equal(h.posts().length, 0);
  locked(h); assert.equal(h.nodes.get('accZ').textContent, '—');
});

test('partial feedback identifies ID 32 from the stopped frame without treating it as live data', () => {
  const h=harness(),s=jointFixture('complete');
  const reason='Partial servo communication; reader will close';
  s.session.status='fault';s.session.error=reason;s.session.error_details=reason;
  s.session.fault_feedback={sequence:5279,sample_us:4264335282,imu_us:4264342349,online_mask:28671,fault_mask:0,torque_mask:0,armed:false,arm_cookie:0,imu_ok:true,last_error:0};
  s.feedback=null;s.joint_wizard.stage='failed';s.joint_wizard.error=reason;
  const saved=JSON.stringify(s.calibration);h.set(s);
  for(const id of ['sessionAlert','jointCurrentResult']){
    assert(h.nodes.get(id).textContent.includes('ID：32'),id);
    assert(h.nodes.get(id).textContent.includes('停止时的记录'),id);
    assert(!h.nodes.get(id).textContent.includes('Partial servo communication'),id);
  }
  jointLocked(h);assert.equal(h.nodes.get('jointLivePosition').textContent,'—');
  assert.equal(h.nodes.get('jointOnlineStatus').textContent,'— / 15');
  assert.equal(h.nodes.get('accZ').textContent,'—');
  assert.equal(JSON.stringify(h.eval('snapshot.calibration')),saved);
  assert.equal(h.posts().length,0);
});

test('partial feedback without a valid partial mask does not invent a failed servo ID',()=>{
  const h=harness(),s=fixture();s.feedback=null;
  s.session.error=s.session.error_details='Partial servo communication; reader will close';
  for(const online_mask of [undefined,null,0,32767,-1,32768,'28671']){
    s.session.fault_feedback={online_mask};h.set(s);
    assert(h.nodes.get('sessionAlert').textContent.includes('触发编号未确认'));
    assert(!h.nodes.get('sessionAlert').textContent.includes('ID：32'));
  }
  s.session.fault_feedback={online_mask:32761};h.set(s);
  assert(h.nodes.get('sessionAlert').textContent.includes('ID：11、12'));
  assert.equal(h.posts().length,0);
});

test('new feedback and unrelated faults do not reuse the old ID 32 failure explanation',()=>{
  const h=harness(),s=fixture();s.feedback=null;
  s.session.error=s.session.error_details='Partial servo communication; reader will close';
  s.session.fault_feedback={online_mask:28671};h.set(s);
  h.set(jointFixture('idle',102));assert.equal(h.nodes.get('sessionAlert').hidden,true);
  assert.equal(h.nodes.get('jointLivePosition').textContent,'2048');
  const other=fixture('fault',103);other.session.error='Unsafe armed/cookie/fault/torque feedback; reader will close';
  other.session.fault_feedback={online_mask:28671};h.set(other);
  assert.equal(h.nodes.get('sessionAlert').textContent,other.session.error);
  assert.equal(h.posts().length,0);
});

test('saved historical voltage diagnosis explains a held session without creating live data or recovery permission', async () => {
  const h=harness(), s=jointFixture('complete');
  const message='历史诊断：15 个舵机均报 0x01 电压报警；这不是实时读数，尚未重新检查。当前采集保持停止，需核对供电后复查。';
  s.session.status='fault';s.session.error=message;s.session.error_details='Unsafe armed/cookie/fault/torque feedback; reader will close';
  s.session.recovery_allowed=false;s.session.recovery_blocked_reason=message;s.feedback=null;
  s.session.held_diagnostic={scope:'historical_diagnostic',message,voltage_alarm_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]};
  h.set(s);await h.settleOld(s);
  assert.equal(h.nodes.get('sessionAlert').textContent,message);
  assert(!h.nodes.get('sessionAlert').textContent.includes('触发值未保存'));
  assert.equal(h.nodes.get('jointOnlineStatus').textContent,'— / 15');
  assert.equal(h.nodes.get('jointTorqueStatus').textContent,'未确认');
  assert.equal(h.nodes.get('jointLivePosition').textContent,'—');
  assert.equal(h.nodes.get('accZ').textContent,'—');jointLocked(h);locked(h);
  assert(h.nodes.get('jointSavedSummary').textContent.includes('ID 10 · 左髋旋转'));
  for(const id of ['recoverFeedback','recoverFeedbackNearFault','recoverFeedbackInWizard','recoverFeedbackInJoints']){
    assert.equal(h.nodes.get(id).disabled,true);await h.nodes.get(id).onclick();
  }
  assert.equal(h.posts().length,0);
});

test('one explicit recovery POST, shared buttons, exact protocol, stale GET cannot undo new generation', async () => {
  const h = harness(), old = fixture();
  old.wizard = {stage: 'failed', expected_action: 'level', busy: false, error: '上次水平歪斜'};
  h.set(old); h.nodes.get('mountedConfirmed').checked = true; h.nodes.get('supportConfirmed').checked = true;
  assert.equal(h.nodes.get('recoverFeedback').onclick, h.nodes.get('recoverFeedbackNearFault').onclick);
  assert.equal(h.nodes.get('recoverFeedback').onclick, h.nodes.get('recoverFeedbackInWizard').onclick);
  assert.equal(h.nodes.get('recoverFeedback').disabled, false); locked(h);
  const pending = deferred(); h.post(() => pending.promise);
  const operation = h.nodes.get('recoverFeedback').onclick();
  await h.nodes.get('recoverFeedbackInWizard').onclick();
  assert.equal(h.posts().length, 1);
  const call = h.posts()[0];
  assert.equal(call.url, '/api/recover-feedback');
  assert.deepEqual(JSON.parse(call.options.body), {generation: 101});
  assert.equal(call.options.headers['X-CSRF-Token'], 'token-101');
  assert.equal(call.options.credentials, 'same-origin');
  assert.equal(h.nodes.get('recoverFeedback').disabled, true);
  assert(h.nodes.get('wizardCurrentResult').textContent.includes('正在检查'));
  assert(!h.nodes.get('wizardCurrentResult').textContent.includes('第 0 步'));
  assert.equal(h.nodes.get('accZ').textContent, '—'); locked(h);
  const waiting = fixture('waiting', 202); waiting.session.recovery_pending = true; waiting.session.recovery_allowed = false;
  pending.resolve(response(waiting)); await operation;
  assert.equal(h.nodes.get('mountedConfirmed').checked, false);
  assert.equal(h.nodes.get('supportConfirmed').checked, false);
  assert.equal(h.nodes.get('zeroCount').textContent, '15 / 15');
  assert.equal(h.eval('snapshot.token'), 'token-202');
  assert.equal(h.nodes.get('endFeedback').disabled, true);
  await h.settleOld(old);
  assert.equal(h.eval('snapshot.session.generation'), 202);
  assert.equal(h.eval('snapshot.wizard.stage'), 'idle');
  locked(h);
  const live = fixture('active', 202); live.session.recovery_allowed = false;
  h.set(live);
  assert(h.nodes.get('recoveryStatus').textContent.includes('真实反馈已恢复'));
  assert.equal(h.nodes.get('mountedConfirmed').disabled, false); locked(h);
  h.nodes.get('mountedConfirmed').checked = true; h.nodes.get('supportConfirmed').checked = true; h.eval('render()');
  assert.equal(h.nodes.get('wizardLevel').disabled, false);
  for (const id of stepIds.slice(1)) assert.equal(h.nodes.get(id).disabled, true);
  h.age(501); locked(h); assert.equal(h.nodes.get('accZ').textContent, '—');
  assert.equal(h.posts().length, 1);
});

test('recovery does not require physical confirmations but rejects busy, pending, waiting, closing and absent allow', async () => {
  const h = harness();
  h.set(fixture());
  assert.equal(h.nodes.get('mountedConfirmed').checked, false);
  assert.equal(h.nodes.get('recoverFeedback').disabled, false);
  for (const mutate of [s => { s.wizard.busy = true; }, s => { s.session.recovery_pending = true; }, s => { s.session.status = 'waiting'; }, s => { s.session.status = 'stopping'; s.session.recovery_blocked_reason = '旧连接尚未关闭'; }, s => { s.session.recovery_blocked_reason = '旧连接尚未关闭'; }, s => { delete s.session.recovery_allowed; }]) {
    const s = fixture(); mutate(s); h.set(s);
    assert.equal(h.nodes.get('recoverFeedback').disabled, true);
    assert(h.nodes.get('recoveryStatus').textContent.length > 0);
    await h.nodes.get('recoverFeedback').onclick();
  }
  assert.equal(h.posts().length, 0);
});

test('409 reason is visible and no retry or next step happens', async () => {
  const h = harness(), s = fixture(); h.set(s); await h.settleOld(s);
  h.post(async () => response({error: '故障历史前缀已丢失，恢复被拒绝'}, 409));
  await h.nodes.get('recoverFeedbackNearFault').onclick();
  assert(h.nodes.get('recoveryStatus').textContent.includes('故障历史前缀已丢失，恢复被拒绝'));
  assert.equal(h.nodes.get('recoveryPanel').dataset.kind, 'error');
  await h.eval('refresh()'); h.age(800);
  assert.equal(h.posts().length, 1); locked(h);
});

test('8-second bounded recovery timeout is visible and does not retry', async () => {
  const h = harness(), s = fixture(); h.set(s); await h.settleOld(s);
  h.post((url, options) => new Promise((resolve, reject) => options.signal.addEventListener('abort', () => { const error = new Error('aborted'); error.name = 'AbortError'; reject(error); }, {once: true})));
  const operation = h.nodes.get('recoverFeedback').onclick();
  const timer = [...h.timers.values()].find(t => t.ms === 8000);
  assert(timer); timer.callback(); await operation;
  assert(h.nodes.get('recoveryStatus').textContent.includes('请求超时'));
  assert(h.nodes.get('recoveryStatus').textContent.includes('不会自动重试'));
  await h.eval('refresh()');
  assert.equal(h.posts().length, 1); locked(h);
});

test('late new-generation active acknowledgment clears lost POST error even if a GET arrived before timeout', async () => {
  for (const acknowledgmentBeforeTimeout of [false, true]) {
    const h = harness(), old = fixture(); h.set(old); await h.settleOld(old);
    h.post((url, options) => new Promise((resolve, reject) => options.signal.addEventListener('abort', () => { const error = new Error('lost POST response'); error.name = 'AbortError'; reject(error); }, {once: true})));
    const operation = h.nodes.get('recoverFeedback').onclick();
    const active = fixture('active', 701); active.session.recovery_allowed = false;
    if (acknowledgmentBeforeTimeout) h.set(active);
    [...h.timers.values()].find(t => t.ms === 8000).callback(); await operation;
    if (!acknowledgmentBeforeTimeout) assert(h.nodes.get('recoveryStatus').textContent.includes('请求超时'));
    else assert(h.nodes.get('recoveryStatus').textContent.includes('真实反馈已恢复'));
    h.set(active);
    assert.equal(h.eval('recoveryPhase'), 'ready');
    assert.equal(h.eval('recoveryError'), '');
    assert(h.nodes.get('recoveryStatus').textContent.includes('真实反馈已恢复'));
    assert.equal(h.nodes.get('recoveryPanel').dataset.kind, 'success');
    assert.equal(h.nodes.get('mountedConfirmed').checked, false);
    assert.equal(h.nodes.get('supportConfirmed').checked, false);
    locked(h); assert.equal(h.posts().length, 1);
  }
});

test('late actual new-generation fault replaces timeout but same old generation active cannot acknowledge recovery', async () => {
  const h = harness(), old = fixture(); h.set(old); await h.settleOld(old);
  h.post((url, options) => new Promise((resolve, reject) => options.signal.addEventListener('abort', () => { const error = new Error('lost POST response'); error.name = 'AbortError'; reject(error); }, {once: true})));
  const operation = h.nodes.get('recoverFeedback').onclick();
  const failed = fixture('fault', 702); failed.session.recovery_allowed = false;
  failed.session.error = '原故障日志历史缺失，拒绝恢复'; failed.session.recovery_blocked_reason = failed.session.error;
  h.set(failed); // The new generation can be seen before the POST abort arrives.
  [...h.timers.values()].find(t => t.ms === 8000).callback(); await operation;
  assert(h.nodes.get('recoveryStatus').textContent.includes(failed.session.error));
  h.set(failed);
  assert.equal(h.eval('recoveryError'), '');
  assert(h.nodes.get('recoveryStatus').textContent.includes(failed.session.error));
  assert(!h.nodes.get('recoveryStatus').textContent.includes('请求超时'));
  locked(h); assert.equal(h.posts().length, 1);

  const same = harness(); same.set(old); await same.settleOld(old);
  same.post(async () => response({error: '恢复响应丢失'}, 409));
  await same.nodes.get('recoverFeedback').onclick();
  same.set(fixture('active', 101));
  assert.equal(same.eval('recoveryPhase'), 'failed');
  assert.equal(same.eval('recoveryError'), '恢复响应丢失');
  assert(same.nodes.get('recoveryStatus').textContent.includes('恢复响应丢失'));
  assert(!same.nodes.get('recoveryStatus').textContent.includes('真实反馈已恢复'));
  assert.equal(same.posts().length, 1);
});

test('200 claim followed by rejected checks retains actual fault and locks recovery', async () => {
  const h = harness(), s = fixture(); h.set(s); await h.settleOld(s);
  const waiting = fixture('waiting', 203); waiting.session.recovery_pending = true;
  h.post(async () => response(waiting)); await h.nodes.get('recoverFeedback').onclick();
  const failed = fixture('fault', 203); failed.session.recovery_allowed = false;
  failed.session.error = '原内核日志前缀缺失，拒绝连接'; failed.session.recovery_blocked_reason = failed.session.error;
  h.set(failed);
  assert(h.nodes.get('recoveryStatus').textContent.includes(failed.session.error));
  assert(h.nodes.get('wizardCurrentResult').textContent.includes(failed.session.error));
  assert.equal(h.nodes.get('recoverFeedback').disabled, true); locked(h);
  assert.equal(h.posts().length, 1);
});

test('closed completed calibration and JSON export remain visible while recovery is available', () => {
  const h = harness(), s = fixture('closed');
  s.wizard.stage = 'complete'; s.wizard.expected_action = null;
  s.session.end_reason = 'completed'; s.session.message = '四步保存成功后自动结束';
  s.calibration.imu = {axes_confirmed: true, calibrated: true, axes: ['x', 'y', 'z'], bias_dps: [0.1, 0.2, 0.3]};
  h.set(s);
  assert(h.nodes.get('wizardCurrentResult').textContent.includes('校准已保存'));
  assert(h.nodes.get('wizardCurrentResult').textContent.includes('采集已自动结束'));
  assert.equal(h.nodes.get('wizardCurrentResult').classList.contains('error'), false);
  assert.equal(h.nodes.get('imuSavedState').textContent, '已保存');
  assert.equal(h.nodes.get('zeroCount').textContent, '15 / 15');
  assert.equal(h.nodes.get('recoverFeedback').disabled, false); locked(h);
  assert(/<a\b[^>]*id="exportCalibration"[^>]*href="\/api\/calibration"[^>]*download=/.test(html));
  assert(!/<a\b[^>]*id="exportCalibration"[^>]*disabled/.test(html));
  assert(!html.includes('remaining_seconds'));
});

test('new-generation refresh clears physical confirmations and old local errors; hardware sample gates stay strict', () => {
  const h = harness(); h.set(fixture());
  h.nodes.get('mountedConfirmed').checked = true; h.nodes.get('supportConfirmed').checked = true;
  h.eval("actionError='旧请求失败'; actionErrorAction='level'; actionErrorGeneration=101;");
  h.set(fixture('active', 999));
  assert.equal(h.nodes.get('mountedConfirmed').checked, false);
  assert.equal(h.nodes.get('supportConfirmed').checked, false);
  assert(!h.nodes.get('wizardCurrentResult').textContent.includes('旧请求失败'));
  for (const mutate of [s => { s.feedback.armed = true; }, s => { s.feedback.torque_mask = 1; }, s => { s.feedback.fault_mask = 1; }, s => { s.feedback.imu_ok = false; }, s => { s.session.received_age_ms = 501; }]) {
    const s = fixture('active', 999); mutate(s); h.set(s);
    h.nodes.get('mountedConfirmed').checked = true; h.nodes.get('supportConfirmed').checked = true; h.eval('render()');
    locked(h);
  }
});

test('real pure wizard stages drive one manual joint workflow, atomic save and manual next joint', async () => {
  const h = harness(), initial = jointFixture(); h.set(initial); await h.settleOld(initial);
  const zeros = initial.calibration.joints.map(j => j.zero), imu = clone(initial.calibration.imu);
  assert.equal(h.posts().length, 0); jointLocked(h);
  assert(h.nodes.get('savedMountingDetail').textContent.includes('校准已保存'));
  supportedJoint(h);
  assert.equal(h.nodes.get('jointDirectionStart').disabled, false);
  for (const id of jointButtonIds.slice(1)) assert.equal(h.nodes.get(id).disabled, true);
  const captures = [['direction-start','jointDirectionStart','await-positive'],['direction-end','jointDirectionEnd','await-range-a'],['range-a','jointRangeA','await-range-b'],['range-b','jointRangeB','ready']];
  h.post(async (url, options) => { assert.equal(url, '/api/joint-calibration'); const body = JSON.parse(options.body); return response(jointFixture(body.action+'-busy')); });
  for (const [action, buttonId, successfulStage] of captures) {
    if (action === 'direction-end') {
      assert.equal(h.nodes.get(buttonId).disabled, true);
      h.nodes.get('jointPositiveConfirmed').checked = true; h.eval('render()');
    }
    assert.equal(h.nodes.get(buttonId).disabled, false, successfulStage);
    await h.nodes.get(buttonId).onclick();
    const call = h.posts().at(-1), body = JSON.parse(call.options.body);
    assert.deepEqual(Object.keys(body).sort(), ['action','generation','id','positive_confirmed','support_confirmed']);
    assert.equal(body.action, action); assert.equal(body.id, 10); assert.equal(body.generation, 101);
    assert.equal(body.support_confirmed, true); assert.equal(call.options.headers['X-CSRF-Token'], 'token-101');
    assert.equal(call.options.credentials, 'same-origin');
    jointLocked(h); assert.equal(h.nodes.get('jointLivePosition').textContent, '—');
    assert.equal(h.nodes.get('jointSupportConfirmed').checked, true);
    assert.equal(h.nodes.get('jointSelect').disabled, true);
    h.set(jointFixture(successfulStage));
    assert.equal(h.eval('expectedJointAction()'), {'await-positive':'direction-end','await-range-a':'range-a','await-range-b':'range-b','ready':'save'}[successfulStage]);
  }
  assert.equal(h.posts().length, 4);
  h.nodes.get('jointPositiveConfirmed').checked = false; h.eval('render()'); assert.equal(h.nodes.get('jointSave').disabled, true);
  h.nodes.get('jointPositiveConfirmed').checked = true; h.eval('render()');
  h.post(async () => response(jointFixture('complete'))); await h.nodes.get('jointSave').onclick();
  assert.equal(h.posts().length, 5);
  assert.deepEqual(clone(h.eval('snapshot.calibration.joints.map(j=>j.zero)')), zeros);
  assert.deepEqual(clone(h.eval('snapshot.calibration.imu')), imu);
  assert.equal(h.eval('snapshot.calibration.mechanical_reference_confirmed'), true);
  assert.equal(h.nodes.get('jointCompletedCount').textContent, '1 / 15 完成');
  assert(h.nodes.get('jointCurrentResult').textContent.includes('已保存'));
  assert.equal(h.nodes.get('jointNext').disabled, false); assert.equal(h.nodes.get('jointSelect').disabled, false);
  h.nodes.get('jointNext').onclick();
  assert.equal(h.eval('selectedJointId'), 11); assert.equal(h.nodes.get('jointPositiveConfirmed').checked, false);
  assert.equal(h.nodes.get('jointDirectionStart').disabled, false);
  assert.equal(h.posts().length, 5); // no auto-sample, auto-save, auto-end or auto-next POST
});

test('completion labels use saved configuration and stay visible after refresh, stale feedback or a fault', () => {
  const h=harness(), options=()=>h.nodes.get('jointSelect').children;
  assert(options().every(option=>option.textContent.includes('等待读取')));
  h.set(jointFixture('ready'));
  assert(options()[0].textContent.endsWith('待校准'));
  assert(h.nodes.get('jointSavedSummary').textContent.includes('尚无完成并保存'));
  assert(h.nodes.get('jointSelectedState').textContent.includes('本次采样未保存'));
  assert(!h.nodes.get('jointSelectedState').textContent.includes('已完成并保存'));
  const saved=jointFixture('complete');h.set(saved);
  assert(options()[0].textContent.endsWith('✓ 已完成并保存'));
  assert(options()[1].textContent.endsWith('待校准'));
  assert.equal(h.nodes.get('jointSavedSummary').textContent,'已完成并保存（1 / 15）：ID 10 · 左髋旋转。');
  h.age(501);jointLocked(h);
  assert(options()[0].textContent.endsWith('✓ 已完成并保存'));
  for(const status of ['closed','fault']){
    const stopped=clone(saved);stopped.session.status=status;stopped.feedback=null;h.set(stopped);jointLocked(h);
    assert(options()[0].textContent.endsWith('✓ 已完成并保存'));
    assert(h.nodes.get('jointSelectedState').textContent.includes('已完成并保存'));
    assert(h.nodes.get('jointSavedSummary').textContent.includes('ID 10 · 左髋旋转'));
  }
  const legacy=jointFixture('old-stand-insufficient');h.set(legacy);
  assert(options().find(option=>option.value==='31').textContent.endsWith('已有记录，需复核'));
  assert(!h.nodes.get('jointSavedSummary').textContent.includes('ID 31 ·'));
  assert(h.nodes.get('jointSavedSummary').textContent.includes('14 / 15'));
  assert.equal(h.posts().length,0);
});

test('recalibration retains the previous saved label and range while distinguishing every unsaved draft', () => {
  const h=harness(), saved=jointFixture('complete');h.set(saved);h.eval('chooseJoint(10)');
  const previous=clone(saved.calibration), oldRange=h.nodes.get('jointSavedRange').textContent;
  for(const stage of ['direction-start-busy','await-positive','await-range-a','await-range-b','ready']){
    const retry=jointFixture(stage);retry.calibration=clone(previous);retry.joint_wizard.completed_count=saved.joint_wizard.completed_count;h.set(retry);
    assert.equal(h.eval('selectedJointId'),10);
    assert(h.nodes.get('jointSelect').children[0].textContent.endsWith('✓ 已完成并保存'));
    assert(h.nodes.get('jointSelectedState').textContent.includes('此前已完成并保存；本次重采未保存'));
    assert(h.nodes.get('jointSelectionHint').textContent.includes('只有再次点击「保存本关节」才会更新'));
    assert.equal(h.nodes.get('jointSavedRange').textContent,oldRange);
    assert.deepEqual(clone(h.eval('snapshot.calibration')),previous);
    assert.equal(h.nodes.get('jointSelect').disabled,true);
    assert.equal(h.nodes.get('jointCompletedCount').textContent,'1 / 15 完成');
  }
  for(const stage of ['cancelled','failed']){
    const abandoned=jointFixture(stage);abandoned.calibration=clone(previous);h.set(abandoned);
    assert.equal(h.nodes.get('jointSelectedState').textContent,'✓ 已完成并保存');
    assert.equal(h.nodes.get('jointSavedRange').textContent,oldRange);
    assert.deepEqual(clone(h.eval('snapshot.calibration')),previous);
  }
  assert.equal(h.posts().length,0);
});

test('every joint feedback gate fails closed, and physical support is required', async () => {
  const variants = [
    s => { s.feedback.online_mask = 0; }, s => { s.feedback.online_mask = 32766; },
    s => { s.feedback.torque_mask = 1; }, s => { s.feedback.armed = true; },
    s => { s.feedback.arm_cookie = 1; }, s => { delete s.feedback.arm_cookie; },
    s => { s.feedback.fault_mask = 1; }, s => { s.feedback.last_error = 1; },
    s => { s.feedback.imu_ok = false; }, s => { s.calibration.imu.calibrated = false; },
    s => { s.calibration.imu.axes_confirmed = false; }, s => { s.session.received_age_ms = 500; },
    s => { s.session.status = 'fault'; }, s => { s.session.status = 'closed'; },
    s => { s.session.status = 'waiting'; }, s => { s.session.recovery_pending = true; },
    s => { s.feedback.positions = [2048]; }, s => { s.feedback.positions[0] = 4096; },
    s => { s.calibration.joints[0].zero = null; }
  ];
  for (const mutate of variants) {
    const h = harness(), s = jointFixture(); mutate(s); h.set(s); supportedJoint(h);
    jointLocked(h); assert.equal(h.nodes.get('jointLivePosition').textContent, '—');
    if ([0,32766].includes(s.feedback.online_mask)&&s.feedback.torque_mask===0) {
      assert.equal(h.nodes.get('jointTorqueStatus').textContent, '未确认');
      assert.equal(h.nodes.get('jointTorqueStatusCard').dataset.state, 'unknown');
    }
    await h.nodes.get('jointDirectionStart').onclick(); assert.equal(h.posts().length, 0);
  }
  const h = harness(); h.set(jointFixture()); jointLocked(h);
  h.eval('positiveAssetOk=false; render()'); supportedJoint(h); jointLocked(h);
  assert.equal(h.nodes.get('jointAssetError').hidden, false);
});

test('automatic state cards show real all-online torque-off proof and preserve support checkbox events', () => {
  const h = harness(); h.set(jointFixture());
  assert.equal(h.nodes.get('jointOnlineStatus').textContent,'15 / 15');
  assert.equal(h.nodes.get('jointTorqueStatus').textContent,'已关闭');
  assert.equal(h.nodes.get('jointControllerStatus').textContent,'未使能');
  assert.equal(h.nodes.get('jointTorqueStatusCard').dataset.state,'good');
  assert.equal(h.nodes.get('jointSupportConfirmed').disabled,false); jointLocked(h);
  h.nodes.get('jointSupportConfirmed').checked=true; h.nodes.get('jointSupportConfirmed').onchange();
  assert.equal(h.nodes.get('jointDirectionStart').disabled,false);
  const partial=jointFixture(); partial.feedback.online_mask=1; partial.feedback.torque_mask=1;
  h.set(partial);
  assert.equal(h.nodes.get('jointOnlineStatus').textContent,'1 / 15');
  assert(h.nodes.get('jointTorqueStatus').textContent.includes('禁止轻摆'));
  assert.equal(h.nodes.get('jointTorqueStatusCard').dataset.state,'bad'); jointLocked(h);
  const armed=jointFixture(); armed.feedback.armed=true; armed.feedback.arm_cookie=123;
  h.set(armed);
  assert(h.nodes.get('jointControllerStatus').textContent.includes('禁止轻摆'));
  assert.equal(h.nodes.get('jointTorqueStatus').textContent,'未确认');
  assert.equal(h.nodes.get('jointSupportConfirmed').disabled,true);
});

test('automatic cards remove all old green states on stale, recovery, new-generation waiting and failed reads', () => {
  function unknown(h) {
    assert.equal(h.nodes.get('jointOnlineStatus').textContent,'— / 15');
    assert.equal(h.nodes.get('jointTorqueStatus').textContent,'未确认');
    assert.equal(h.nodes.get('jointControllerStatus').textContent,'未确认');
    for(const id of ['jointOnlineStatusCard','jointTorqueStatusCard','jointControllerStatusCard']) assert.equal(h.nodes.get(id).dataset.state,'unknown');
    assert.equal(h.nodes.get('jointSupportConfirmed').disabled,true);
  }
  const h=harness(); h.set(jointFixture()); h.age(501); unknown(h);
  h.set(jointFixture()); h.eval("fetchError='状态读取超时'; render()"); unknown(h);
  const recovering=jointFixture(); recovering.session.recovery_pending=true;
  h.set(recovering); unknown(h);
  const waiting=jointFixture('idle',999); waiting.session.status='waiting';
  h.set(waiting); unknown(h);
  for(const mutate of [s=>{s.feedback.online_mask=0},s=>{s.feedback.online_mask=32766},s=>{s.feedback.fault_mask=1},s=>{delete s.feedback.arm_cookie},s=>{s.feedback.torque_mask='0'}]) {
    const s=jointFixture(); mutate(s); h.set(s);
    assert.equal(h.nodes.get('jointTorqueStatus').textContent,'未确认');
    assert.equal(h.nodes.get('jointTorqueStatusCard').dataset.state,'unknown');
  }
});

test('real failed and cancelled stages restart safely; cancel works without support or fresh data', async () => {
  const h = harness(); h.set(jointFixture('failed')); supportedJoint(h);
  assert.equal(h.nodes.get('jointDirectionStart').disabled, false);
  assert(h.nodes.get('jointCurrentResult').textContent.includes('采样时关节还在移动'));
  assert(h.nodes.get('jointCurrentResult').textContent.includes('从起点重新采集'));
  assert.equal(h.nodes.get('jointPositiveConfirmed').checked, false);
  const busy = jointFixture('direction-start-busy'); busy.session.received_age_ms = 1000;
  h.set(busy); assert.equal(h.nodes.get('jointCancel').disabled, false);
  assert.equal(h.nodes.get('jointSupportConfirmed').checked, true); jointLocked(h);
  h.post(async () => response(jointFixture('cancelled'))); await h.nodes.get('jointCancel').onclick();
  assert.deepEqual(JSON.parse(h.posts()[0].options.body), {generation:101,action:'cancel',id:10,support_confirmed:true,positive_confirmed:false});
  assert.equal(h.eval('expectedJointAction()'), 'direction-start');
  assert.equal(h.nodes.get('jointSupportConfirmed').checked,true);assert.equal(h.nodes.get('jointDirectionStart').disabled,false);
  assert.equal(h.nodes.get('jointSelect').disabled, false);
});

test('drafts lock joint selection and next buttons; user cannot skip to later captures', async () => {
  const h = harness(); h.set(jointFixture('await-positive')); supportedJoint(h);
  assert.equal(h.nodes.get('jointSelect').disabled, true); assert.equal(h.nodes.get('jointNext').disabled, true);
  h.eval('chooseJoint(11)'); assert.equal(h.eval('selectedJointId'), 10);
  h.nodes.get('jointNext').onclick(); assert.equal(h.eval('selectedJointId'), 10);
  for (const id of ['jointRangeA','jointRangeB','jointSave']) { assert.equal(h.nodes.get(id).disabled, true); await h.nodes.get(id).onclick(); }
  assert.equal(h.posts().length, 0);
});

test('stale display blocks calibration while preserving user inputs; ended sessions clear confirmations', () => {
  const h = harness(); h.set(jointFixture('await-positive')); supportedJoint(h);
  h.nodes.get('jointPositiveConfirmed').checked = true; h.nodes.get('mountedConfirmed').checked = true; h.nodes.get('supportConfirmed').checked = true;
  h.eval('render()'); assert.notEqual(h.nodes.get('jointLivePosition').textContent, '—');
  h.age(501); jointLocked(h); assert.equal(h.nodes.get('jointLivePosition').textContent, '—');
  for (const id of ['mountedConfirmed','supportConfirmed','jointSupportConfirmed','jointPositiveConfirmed']) assert.equal(h.nodes.get(id).checked, true);
  assert.equal(h.nodes.get('zeroCount').textContent, '15 / 15'); assert.equal(h.nodes.get('imuSavedState').textContent, '已保存');
  const ended = jointFixture('complete'); ended.session.status = 'closed'; ended.session.end_reason = 'manual'; ended.session.recovery_allowed = true;
  h.set(ended); assert(h.nodes.get('jointCurrentResult').textContent.includes('本关节已保存'));
  assert.equal(h.nodes.get('jointLivePosition').textContent, '—'); jointLocked(h);
  for (const id of ['mountedConfirmed','supportConfirmed','jointSupportConfirmed','jointPositiveConfirmed']) assert.equal(h.nodes.get(id).checked, false);
  assert.equal(h.nodes.get('recoverFeedbackInJoints').disabled, false);
  assert.equal(h.nodes.get('recoverFeedbackInJoints').onclick, h.nodes.get('recoverFeedback').onclick);
});

test('joint POST is one request, timeout does not repeat, and old POST cannot overwrite a new generation', async () => {
  const h = harness(), initial = jointFixture(); h.set(initial); await h.settleOld(initial); supportedJoint(h);
  const post = deferred(); h.post(() => post.promise);
  const operation = h.nodes.get('jointDirectionStart').onclick(); await h.nodes.get('jointDirectionStart').onclick();
  assert.equal(h.posts().length, 1);
  const newer = jointFixture('idle', 900); h.set(newer);
  assert.equal(h.nodes.get('jointSupportConfirmed').checked, false);
  post.resolve(response(jointFixture('await-positive', 101))); await operation;
  assert.equal(h.eval('snapshot.session.generation'), 900); assert.equal(h.eval('snapshot.joint_wizard.stage'), 'idle');
  assert.equal(h.eval('snapshot.token'), 'token-900'); jointLocked(h);
  assert(!h.nodes.get('jointCurrentResult').textContent.includes('旧请求')); assert.equal(h.posts().length, 1);

  const timeout = harness(); timeout.set(initial); await timeout.settleOld(initial); supportedJoint(timeout);
  timeout.post((url, options) => new Promise((resolve, reject) => options.signal.addEventListener('abort', () => { const error = new Error('aborted'); error.name='AbortError'; reject(error); }, {once:true})));
  const pending = timeout.nodes.get('jointDirectionStart').onclick(); [...timeout.timers.values()].find(t=>t.ms===8000).callback(); await pending;
  assert(timeout.nodes.get('jointCurrentResult').textContent.includes('请求超时')); assert(timeout.nodes.get('jointCurrentResult').textContent.includes('不会自动'));
  await timeout.eval('refresh()'); assert.equal(timeout.posts().length, 1);
});

test('joint POST snapshot wins against an older outstanding GET, and actual range failure is visible', async () => {
  const h = harness(), initial = jointFixture(); h.set(initial); supportedJoint(h);
  h.post(async () => response(jointFixture('await-positive'))); await h.nodes.get('jointDirectionStart').onclick();
  await h.settleOld(initial); assert.equal(h.eval('snapshot.joint_wizard.stage'), 'await-positive');
  assert.equal(h.nodes.get('jointRangeA').disabled, true);
  h.nodes.get('jointPositiveConfirmed').checked = true; h.eval('render()');
  h.post(async () => response({error:'默认站姿的余量不足，本关节未保存；请核对方向与两侧安全位置'},409));
  await h.nodes.get('jointDirectionEnd').onclick();
  assert(h.nodes.get('jointCurrentResult').textContent.includes('默认站姿的余量不足'));
  assert.equal(h.nodes.get('jointCurrentResult').classList.contains('error'), true);
  assert.equal(h.nodes.get('jointSave').disabled, true);
  assert.equal(h.eval('snapshot.calibration.joints[0].direction_confirmed'), false);
});

test('all 15 completion uses real host count and model stand checks; saved compatibility unlocks selection', () => {
  const h = harness(); h.set(jointFixture('all-complete'));
  assert.equal(h.nodes.get('jointCompletedCount').textContent, '15 / 15 完成');
  assert.equal(h.nodes.get('jointAllDone').hidden, false);
  assert(h.nodes.get('jointAllDone').textContent.includes('等待下一步受控实机检查'));
  assert(!h.nodes.get('jointAllDone').textContent.includes('仍需核对机械零姿态'));
  const old = jointFixture('old-stand-insufficient'); h.set(old);
  assert.equal(h.nodes.get('jointCompletedCount').textContent, '14 / 15 完成');
  assert.equal(h.nodes.get('jointAllDone').hidden, true); assert.equal(h.eval('jointSaved(31)'), false);
  assert.equal(h.eval('roundToEven(1820.5)'), 1820);
  const compatibility = jointFixture('complete'); compatibility.joint_wizard.stage='saved'; h.set(compatibility);
  assert.equal(h.nodes.get('jointSelect').disabled, false); assert.equal(h.eval('jointDraftActive()'), false);
});

test('rejected range displays both readings, preserves direction and retries endpoints without automatic save', async () => {
  const h = harness(), rejected = jointFixture('range-retry'); h.set(rejected); await h.settleOld(rejected); supportedJoint(h);
  assert.equal(h.eval('selectedJointId'),12);
  assert.equal(h.nodes.get('jointOriginalZero').textContent,'2032');
  assert.equal(h.nodes.get('jointRangeAReading').textContent,'2180');
  assert.equal(h.nodes.get('jointRangeBReading').textContent,'2361');
  assert(h.nodes.get('jointRangeBLabel').textContent.includes('上次未通过'));
  assert.equal(h.nodes.get('jointRangePreview').textContent,'2192 – 2349');
  assert(h.nodes.get('jointRangeGuide').textContent.includes('2032'));
  assert(h.nodes.get('jointRangeGuide').textContent.includes('2248'));
  assert(h.nodes.get('jointRangeGuide').textContent.includes('≤ 2020'));
  assert(h.nodes.get('jointRangeGuide').textContent.includes('≥ 2260'));
  assert(h.nodes.get('jointCurrentResult').textContent.includes('方向采样已保留'));
  assert(!h.nodes.get('jointCurrentResult').textContent.includes('从起点重新采集'));
  assert.equal(h.nodes.get('jointRangeA').disabled,false); assert.equal(h.nodes.get('jointRangeB').disabled,false);
  assert.equal(h.nodes.get('jointDirectionStart').disabled,true); assert.equal(h.nodes.get('jointSave').disabled,true);
  assert.equal(h.posts().length,0);
  h.post(async (url, options) => {assert.equal(url,'/api/joint-calibration');assert.equal(JSON.parse(options.body).action,'range-a');return response(jointFixture('range-retry-a-busy'));});
  await h.nodes.get('jointRangeA').onclick(); jointLocked(h); assert.equal(h.posts().length,1);
  h.set(jointFixture('range-retry-a')); supportedJoint(h);
  assert.equal(h.nodes.get('jointRangeAReading').textContent,'1900');
  assert.equal(h.nodes.get('jointRangeB').disabled,false); assert.equal(h.nodes.get('jointSave').disabled,true);
  h.set(jointFixture('range-retry-ready')); supportedJoint(h);
  assert.equal(h.nodes.get('jointRangePreview').textContent,'1912 – 2438');
  assert.equal(h.nodes.get('jointSave').disabled,true);
  h.nodes.get('jointPositiveConfirmed').checked=true;h.eval('render()');assert.equal(h.nodes.get('jointSave').disabled,false);
  assert.equal(h.posts().length,1); // No B sample or save until the user's explicit click.
  const saved=jointFixture('range-retry-saved');
  assert.deepEqual(saved.calibration.joints.slice(0,2),rejected.calibration.joints.slice(0,2));
  assert.deepEqual(saved.calibration.joints.map(j=>j.zero),rejected.calibration.joints.map(j=>j.zero));
  assert.deepEqual(saved.calibration.imu,rejected.calibration.imu);
});

test('endpoint retry never overrides live feedback, support, busy, stage or selected-joint gates', async () => {
  const h=harness();
  const variants=[s=>{s.feedback.online_mask=32763;},s=>{s.feedback.fault_mask=4;},s=>{s.feedback.torque_mask=4;},s=>{s.feedback.armed=true;},s=>{s.feedback.arm_cookie=1;},s=>{s.feedback.imu_ok=false;},s=>{s.session.received_age_ms=501;},s=>{s.session.status='closed';},s=>{s.joint_wizard.busy=true;}];
  for(const mutate of variants){const s=jointFixture('range-retry');mutate(s);h.set(s);supportedJoint(h);jointLocked(h);await h.nodes.get('jointRangeA').onclick();await h.nodes.get('jointRangeB').onclick();}
  h.set(jointFixture('range-retry'));h.nodes.get('jointSupportConfirmed').checked=false;h.eval('render()');jointLocked(h);
  h.set(jointFixture('range-retry'));supportedJoint(h);h.eval('selectedJointId=13;render()');jointLocked(h);
  const failed=jointFixture('range-retry');failed.joint_wizard.stage='failed';h.set(failed);h.eval('selectedJointId=12;render()');supportedJoint(h);
  assert.equal(h.nodes.get('jointRangeA').disabled,true);assert.equal(h.nodes.get('jointRangeB').disabled,true);
  assert.equal(h.posts().length,0);
});

test('range records remain visibly historical when live data expires and clear after cancellation', () => {
  const h=harness();h.set(jointFixture('range-retry'));supportedJoint(h);h.age(501);
  jointLocked(h);assert.equal(h.nodes.get('jointLivePosition').textContent,'—');
  assert.equal(h.nodes.get('jointRangeAReading').textContent,'2180');assert.equal(h.nodes.get('jointRangeBReading').textContent,'2361');
  const cancelled=jointFixture('range-retry');cancelled.joint_wizard=clone(realJointFixtures.cancelled.joint_wizard);h.set(cancelled);supportedJoint(h);
  assert.equal(h.nodes.get('jointRangeAReading').textContent,'—');assert.equal(h.nodes.get('jointRangeBReading').textContent,'—');
  assert.equal(h.nodes.get('jointRangeA').disabled,true);assert.equal(h.nodes.get('jointRangeB').disabled,true);
  assert.equal(h.posts().length,0);
});

test('actual duck tile mapping switches to a dedicated mouth diagram using SVG attributes', () => {
  const h = harness(); h.set(jointFixture()); h.eval('chooseJoint(20)');
  assert.equal(h.nodes.get('jointDirectionDiagram').viewBox, '1284 491 303 371');
  h.eval('chooseJoint(34)'); h.set(jointFixture('ready-mouth'));
  assert.equal(h.nodes.get('jointDirectionDiagram').getAttribute('hidden'), ''); assert.equal(h.nodes.get('jointMouthDiagram').getAttribute('hidden'), null);
  assert.equal(h.nodes.get('jointDirectionDiagram').hidden,undefined);assert.equal(h.nodes.get('jointMouthDiagram').hidden,undefined);
  assert(h.nodes.get('jointReferenceCopy').textContent.includes('绿色嘴部张开为正方向'));
  assert.equal(h.nodes.get('jointFullReferenceLink').hidden,true);
  assert(h.nodes.get('jointPositiveLabel').textContent.includes('张开'));
  assert(h.nodes.get('jointMarginNote').textContent.includes('6 刻度'));
  assert(h.nodes.get('jointMarginNote').textContent.includes('36 刻度'));
  assert(h.nodes.get('jointMarginNote').textContent.includes('原零点不改'));
  assert(/id="joint-steps"/.test(html));
  assert(!/href="[^"]*cal\/enter|\/api\/motion\/|\/api\/cal\/targets/.test(html));
});

test('live refresh leaves the selected reference and unchanged options untouched', () => {
  const h=harness(),s=jointFixture();h.set(s);h.eval('chooseJoint(10)');
  const writes=[];
  for(const id of ['jointDirectionDiagram','jointMouthDiagram']){
    const node=h.nodes.get(id);
    for(const method of ['setAttribute','removeAttribute']){
      const original=node[method];node[method]=function(...args){writes.push([id,method,...args]);return original.apply(this,args);};
    }
  }
  function watchProperty(node,key,label){let value=node[key];Object.defineProperty(node,key,{get(){return value;},set(next){writes.push([label,key,next]);value=next;},configurable:true});}
  watchProperty(h.nodes.get('jointSelect'),'value','select');
  for(const option of h.nodes.get('jointSelect').children)watchProperty(option,'textContent','option-'+option.value);
  for(let i=0;i<100;i++)h.eval('render()');
  const next=clone(s);next.feedback.positions[0]=2058;h.set(next);
  assert.equal(h.nodes.get('jointLivePosition').textContent,'2058');
  assert.equal(writes.length,0,'status polling must not repaint the static diagram or option labels');
  h.eval('chooseJoint(14)');
  assert.equal(writes.filter(row=>row[1]==='setAttribute'&&row[2]==='viewBox').length,1);
  assert.equal(h.nodes.get('jointSelect').value,'14');
  writes.length=0;h.eval('chooseJoint(34)');
  assert.equal(h.nodes.get('jointDirectionDiagram').getAttribute('hidden'),'');
  assert.equal(h.nodes.get('jointMouthDiagram').getAttribute('hidden'),null);
  assert.equal(writes.filter(row=>row[2]==='hidden').length,2);
  writes.length=0;for(let i=0;i<100;i++)h.eval('render()');assert.equal(writes.length,0);
  assert.equal(h.posts().length,0);
});

test('33 to 34 and back shows exactly one diagram and preserves calibration without requests',()=>{
  const h=harness(),s=jointFixture();h.set(s);const saved=JSON.stringify(s.calibration);
  const shown=id=>h.nodes.get(id).getAttribute('hidden')===null;
  for(const id of [33,34,33,34,10,34,32,12,22]){
    h.eval('chooseJoint('+id+')');
    assert.equal(shown('jointDirectionDiagram'),id!==34,'direction ID '+id);
    assert.equal(shown('jointMouthDiagram'),id===34,'mouth ID '+id);
    assert.equal(h.nodes.get('jointFullReferenceLink').hidden,id===34);
    if(id===33)assert.equal(h.nodes.get('jointDirectionDiagram').viewBox,'968 491 303 371');
    if(id===12)assert(h.nodes.get('jointReferenceCopy').textContent.includes('往鸭子身后摆'));
    if(id===22)assert(h.nodes.get('jointReferenceCopy').textContent.includes('往鸭嘴方向摆'));
    assert.equal(JSON.stringify(h.eval('snapshot.calibration')),saved);
  }
  assert.equal(h.posts().length,0);
});

test('completed joint reuses existing limits after two direction samples with one explicit save',async()=>{
  const h=harness('http:','#joint-steps');h.set(jointFixture('all-complete'));
  assert.equal(h.nodes.get('jointDirectionReuseRow').hidden,true);
  const s=jointFixture('direction-reuse'),old=clone(s.calibration);h.set(s);supportedJoint(h);
  assert.equal(h.nodes.get('jointDirectionReuseRow').hidden,false);
  assert.equal(h.nodes.get('jointReuseDirection').disabled,true);
  assert(h.nodes.get('jointDirectionReuseDetail').textContent.includes('1500–2600'));
  h.nodes.get('jointPositiveConfirmed').checked=true;h.eval('render()');
  assert.equal(h.nodes.get('jointReuseDirection').disabled,false);assert.equal(h.posts().length,0);
  h.post(async(url,options)=>{assert.equal(url,'/api/joint-calibration');assert.deepEqual(JSON.parse(options.body),{generation:101,action:'save-direction',id:10,support_confirmed:true,positive_confirmed:true});return response(jointFixture('direction-reuse-saved'));});
  await h.nodes.get('jointReuseDirection').onclick();assert.equal(h.posts().length,1);
  const current=h.eval('snapshot.calibration');assert.equal(current.joints[0].sign,1);
  const expected=clone(old);expected.joints[0].sign=1;assert.deepEqual(JSON.parse(JSON.stringify(current)),expected);
  assert.equal(h.nodes.get('jointDirectionReuseRow').hidden,true);
});

test('direction reuse refuses missing backend proof, unsafe feedback, stale state and other joint drafts',async()=>{
  for(const kind of ['missing','rejected','torque','stale','other-joint']){
    const h=harness('http:','#joint-steps'),s=jointFixture('direction-reuse');
    if(kind==='missing')delete s.joint_wizard.direction_reuse_allowed;
    if(kind==='rejected'){s.joint_wizard.direction_reuse_allowed=false;s.joint_wizard.direction_reuse_reason='新方向站姿超出已保存范围';}
    if(kind==='torque')s.feedback.torque_mask=1;
    if(kind==='other-joint')s.joint_wizard.id=12;
    h.set(s);if(kind==='other-joint')h.eval('selectedJointId=10;render()');supportedJoint(h);h.nodes.get('jointPositiveConfirmed').checked=true;h.eval('render()');
    if(kind==='stale')h.age(501);
    assert.equal(h.nodes.get('jointReuseDirection').disabled,true,kind);
    await h.nodes.get('jointReuseDirection').onclick();assert.equal(h.posts().length,0,kind);
    if(kind==='rejected')assert(h.nodes.get('jointDirectionReuseDetail').textContent.includes('超出已保存范围'));
  }
});

test('mouth diagram stays correct after a stale frame and refresh and does not depend on unrelated PNGs',()=>{
  const h=harness(),s=jointFixture();h.set(s);h.eval('chooseJoint(34)');
  h.eval('assetOk=false; positiveAssetOk=false; render()');supportedJoint(h);
  assert.equal(h.nodes.get('jointDirectionStart').disabled,false);
  assert.equal(h.nodes.get('jointAssetError').hidden,true);
  h.age(501);jointLocked(h);
  assert.equal(h.nodes.get('jointDirectionDiagram').getAttribute('hidden'),'');
  assert.equal(h.nodes.get('jointMouthDiagram').getAttribute('hidden'),null);
  h.set(s);assert.equal(h.nodes.get('jointSelect').value,'34');
  assert.equal(h.nodes.get('jointDirectionDiagram').getAttribute('hidden'),'');
  assert.equal(h.nodes.get('jointMouthDiagram').getAttribute('hidden'),null);
  h.eval('chooseJoint(33); positiveAssetOk=false; render()');supportedJoint(h);jointLocked(h);
  assert.equal(h.nodes.get('jointAssetError').hidden,false);assert.equal(h.posts().length,0);
});

test('late old IMU and end POST responses cannot roll back new generation or token', async () => {
  for (const operationName of ['imu','end']) {
    const h = harness(), old = fixture('active',101); h.set(old); await h.settleOld(old);
    h.nodes.get('mountedConfirmed').checked=true; h.nodes.get('supportConfirmed').checked=true; h.eval('render()');
    const pending=deferred(); h.post(()=>pending.promise);
    const operation=operationName==='imu'?h.nodes.get('wizardLevel').onclick():h.nodes.get('endFeedback').onclick();
    assert.equal(h.posts().length,1);
    h.set(fixture('active',901)); pending.resolve(response(old)); await operation;
    assert.equal(h.eval('snapshot.session.generation'),901); assert.equal(h.eval('snapshot.token'),'token-901');
    assert.equal(h.eval('actionError'), ''); assert.equal(h.eval('endError'), '');
  }
});

test('late older recovery POST cannot replace an already acknowledged newer recovery generation', async () => {
  const h = harness(), old = fixture(); h.set(old); await h.settleOld(old);
  const pending = deferred(); h.post(()=>pending.promise);
  const operation = h.nodes.get('recoverFeedback').onclick();
  h.set(fixture('active',902));
  const olderClaim = fixture('waiting',901); olderClaim.session.recovery_pending=true;
  pending.resolve(response(olderClaim)); await operation;
  assert.equal(h.eval('snapshot.session.generation'),902);
  assert.equal(h.eval('snapshot.token'),'token-902');
  assert.equal(h.nodes.get('accZ').textContent,'9.81');
  assert.equal(h.posts().length,1);
});

const workspaceNavigation = {
  'device-status':'navDevice', 'servo-settings':'navServos',
  'joint-steps':'navJoints', 'imu-steps':'navImu'
};
function workspaceVisible(h, key) {
  assert.equal(h.eval('workspaceView'),key);
  for(const section of ['device-status','servo-settings','joint-steps','imu-live','imu-steps']) {
    assert.equal(h.nodes.get(section).hidden,section==='imu-live'?key!=='imu-steps':section!==key,section);
  }
  for(const [view,id] of Object.entries(workspaceNavigation)) {
    assert.equal(h.nodes.get(id).getAttribute('aria-current')==='page',view===key,id);
  }
}
function unknownHardware(h) {
  for(const id of ['statusServoCount','statusFaultCount','statusTorque','statusImu'])assert.equal(h.nodes.get(id).textContent,'未确认',id);
  for(const id of [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]) {
    assert.equal(h.eval(`deviceElements.get(${id}).state.textContent`),'未确认',String(id));
    assert.equal(h.eval(`deviceElements.get(${id}).position.textContent`),'—',String(id));
    assert.equal(h.eval(`deviceElements.get(${id}).torque.textContent`),'未确认',String(id));
  }
}

test('workspace initial deep links and unknown hashes select exactly one view without new transport requests', () => {
  for(const key of Object.keys(workspaceNavigation)) {
    const h=harness('http:','#'+key);workspaceVisible(h,key);
    assert.equal(h.calls.length,1);assert.equal(h.posts().length,0);
  }
  for(const hash of ['', '#missing', '#joint-steps?arm=1', '#api/motion']) {
    const h=harness('http:',hash);workspaceVisible(h,'device-status');
    assert.equal(h.calls.length,1);assert.equal(h.posts().length,0);
  }
});

test('workspace navigation preserves joint drafts, saved calibration and confirmations without a GET or POST', () => {
  const h=harness('http:','#joint-steps');h.set(jointFixture('range-retry'));supportedJoint(h);
  h.nodes.get('jointPositiveConfirmed').checked=true;h.eval('render()');
  const before=clone(h.eval('snapshot')), readings=['jointRangeAReading','jointRangeBReading','jointRangePreview'].map(id=>h.nodes.get(id).textContent);
  const requestCount=h.calls.length;
  for(const key of ['device-status','servo-settings','imu-steps','joint-steps']) {
    h.navigate('#'+key);workspaceVisible(h,key);
    assert.equal(h.eval('selectedJointId'),12);
    assert.deepEqual(clone(h.eval('snapshot')),before);
    assert.equal(h.nodes.get('jointSupportConfirmed').checked,true);
    assert.equal(h.nodes.get('jointPositiveConfirmed').checked,true);
    assert.deepEqual(['jointRangeAReading','jointRangeBReading','jointRangePreview'].map(id=>h.nodes.get(id).textContent),readings);
    assert.equal(h.calls.length,requestCount);assert.equal(h.posts().length,0);
  }
  h.eval("selectWorkspace('servo-settings')");workspaceVisible(h,'servo-settings');
  h.eval("selectWorkspace('#unknown')");workspaceVisible(h,'device-status');
  assert.equal(h.calls.length,requestCount);assert.equal(h.posts().length,0);
});

test('device overview uses fresh feedback and clears every old online position and torque claim after expiry', () => {
  const h=harness(),live=jointFixture('complete');h.set(live);
  assert.equal(h.nodes.get('statusServoCount').textContent,'15 / 15');
  assert.equal(h.nodes.get('statusFaultCount').textContent,'0 / 15');
  assert.equal(h.nodes.get('statusTorque').textContent,'全部关闭');
  assert.equal(h.nodes.get('statusImu').textContent,'已就绪');
  for(const [index,id] of [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34].entries()) {
    assert.equal(h.eval(`deviceElements.get(${id}).position.textContent`),String(live.feedback.positions[index]));
    assert.notEqual(h.eval(`deviceElements.get(${id}).state.textContent`),'未确认');
  }
  const savedLabel=h.eval('deviceElements.get(10).calibration.textContent');
  assert(savedLabel.includes('已'));
  h.age(500);unknownHardware(h);
  assert.equal(h.eval('deviceElements.get(10).calibration.textContent'),savedLabel);
  assert.equal(h.posts().length,0);
});

test('historical voltage evidence stays visible independently and never turns stopped data into live hardware proof', () => {
  const h=harness(),s=jointFixture('complete');
  s.session.status='fault';s.session.recovery_allowed=false;
  const message='历史诊断：15 个舵机均报 0x01 电压报警；这不是实时读数，尚未重新检查。';
  s.session.error=message;s.session.recovery_blocked_reason=message;
  s.session.held_diagnostic={scope:'historical_diagnostic',message,voltage_alarm_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]};
  // An old healthy payload must not establish current hardware state after a fault.
  h.set(s);unknownHardware(h);
  assert.equal(h.nodes.get('statusHistory').hidden,false);
  assert(h.nodes.get('statusHistoryText').textContent.includes(message));
  assert(h.nodes.get('statusDataSource').textContent.length>0);
  assert.equal(h.nodes.get('recoverFeedback').disabled,true);
  assert(h.eval('deviceElements.get(10).calibration.textContent').includes('已'));
  s.feedback=null;h.set(s);unknownHardware(h);
  assert.equal(h.nodes.get('statusHistory').hidden,false);
  assert.equal(h.posts().length,0);
});

test('partial or faulty live servo feedback never establishes an all-torque-off status', () => {
  const h=harness(),partial=jointFixture();partial.feedback.online_mask=32763;
  h.set(partial);
  assert.equal(h.nodes.get('statusServoCount').textContent,'14 / 15');
  assert.equal(h.nodes.get('statusTorque').textContent,'未确认');
  assert.equal(h.eval('deviceElements.get(12).position.textContent'),'—');
  assert.equal(h.eval('deviceElements.get(12).torque.textContent'),'未确认');
  const faulty=jointFixture();faulty.feedback.fault_mask=4;h.set(faulty);
  assert.equal(h.nodes.get('statusFaultCount').textContent,'1 / 15');
  assert.notEqual(h.nodes.get('statusTorque').textContent,'全部关闭');
  assert.equal(h.eval('deviceElements.get(12).position.textContent'),'—');
  const invalid=jointFixture();invalid.feedback.online_mask='32767';h.set(invalid);
  assert.equal(h.nodes.get('statusServoCount').textContent,'未确认');
  assert.equal(h.nodes.get('statusTorque').textContent,'未确认');
  assert.equal(h.eval('deviceElements.get(12).position.textContent'),'—');
  assert.equal(h.posts().length,0);
});

test('servo maintenance inputs and motion or ID buttons remain locked in every feedback state', () => {
  const h=harness();
  for(const data of [fixture('fault'),jointFixture(),jointFixture('all-complete'),fixture('closed')]) {
    h.set(data);h.navigate('#servo-settings');workspaceVisible(h,'servo-settings');
    for(const id of ['servoNewId','servoRename','servoCenter'])assert.equal(h.nodes.get(id).disabled,true,id);
  }
  const count=h.calls.length;
  for(const id of ['servoRename','servoCenter']) {
    const handler=h.nodes.get(id).onclick;
    if(typeof handler==='function')handler();
  }
  assert.equal(h.calls.length,count);assert.equal(h.posts().length,0);
});

test('file-origin workspace navigation remains local and cannot start status or hardware requests', () => {
  const h=harness('file:','#joint-steps');workspaceVisible(h,'joint-steps');
  for(const key of Object.keys(workspaceNavigation)) {
    h.navigate('#'+key);workspaceVisible(h,key);
    for(const id of ['servoNewId','servoRename','servoCenter'])assert.equal(h.nodes.get(id).disabled,true,id);
  }
  assert.equal(h.calls.length,0);assert.equal(h.intervals.length,0);assert.equal(h.timers.size,0);
});

function maintenanceFixture(connected=true){
  const s=jointFixture('all-complete');s.session.status='closed';s.session.error=null;
  s.maintenance={phase:connected?'ready':'idle',owns_connection:connected,connected,compatible:connected?true:null,busy:false,blocked_reason:null,identity_changed:false,received_age_ms:0,servo:connected?{id:10,state:0,error:0,position:2058,torque:0,online:true}:null,results:[],message:'维护状态'};
  return s;
}

test('maintenance navigation and refresh never connect; explicit enter sends one scoped request',async()=>{
  const h=harness(),s=maintenanceFixture(false);h.set(s);await h.settleOld(s);
  h.navigate('#servo-settings');await h.eval('refresh()');assert.equal(h.posts().length,0);
  assert.equal(h.nodes.get('servoEnter').disabled,false);
  h.post(async(url,options)=>{assert.equal(url,'/api/servo-maintenance');assert.deepEqual(JSON.parse(options.body),{action:'enter',generation:101});assert.equal(options.headers['X-CSRF-Token'],'token-101');return response(maintenanceFixture());});
  await h.nodes.get('servoEnter').onclick();assert.equal(h.posts().length,1);
  assert.equal(h.nodes.get('servoRead').disabled,false);assert.equal(h.nodes.get('recoverFeedback').disabled,true);
  assert.equal(h.nodes.get('servoSelectedPosition').textContent,'2058');
});

test('maintenance fault shows its exact cause and explicit review without permitting motion or automatic retry',async()=>{
  const h=harness(),s=maintenanceFixture(false);
  s.maintenance.fault_hold=true;s.maintenance.fault_review_allowed=true;
  s.maintenance.blocked_reason='上次维护出现故障';
  s.maintenance.error='回中未完成：保持目标位置读回不一致（步骤码 18）';
  h.set(s);h.navigate('#servo-settings');
  assert.equal(h.posts().length,0);
  assert.equal(h.nodes.get('servoReview').hidden,false);
  assert.equal(h.nodes.get('servoReview').disabled,false);
  assert.equal(h.nodes.get('servoEnter').disabled,true);
  assert.equal(h.nodes.get('servoCenter').disabled,true);
  assert(h.nodes.get('servoMaintenanceMessage').textContent.includes('步骤码 18'));
  assert(h.nodes.get('servoMaintenanceMessage').textContent.includes('复查舵机反馈'));
  h.post(async(url,options)=>{
    assert.deepEqual(JSON.parse(options.body),{action:'review',generation:101});
    const next=structuredClone(s);Object.assign(next.maintenance,{owns_connection:true,busy:true,action:'review',error:null,message:'正在复查舵机反馈：1 / 15。'});
    return response(next);
  });
  await h.nodes.get('servoReview').onclick();
  assert.equal(h.posts().length,1);assert.equal(h.nodes.get('servoReview').disabled,true);
  assert(h.nodes.get('servoMaintenanceMessage').textContent.includes('1 / 15'));
  assert.equal(h.nodes.get('servoCenter').disabled,true);
  await h.nodes.get('servoReview').onclick();assert.equal(h.posts().length,1);
  const closed=maintenanceFixture(false);closed.maintenance.fault_hold=false;closed.maintenance.fault_review_allowed=false;
  h.set(closed);assert.equal(h.nodes.get('servoReview').hidden,true);
  assert.equal(h.nodes.get('servoEnter').disabled,false);assert.equal(h.nodes.get('servoCenter').disabled,true);
});

test('maintenance respects holds, unsupported firmware and freshness without fake live state',async()=>{
  const h=harness(),held=maintenanceFixture(false);held.maintenance.blocked_reason='历史电压故障需核对';h.set(held);
  await h.nodes.get('servoEnter').onclick();assert.equal(h.posts().length,0);
  assert(h.nodes.get('servoMaintenanceMessage').textContent.includes('电压故障'));
  const s=maintenanceFixture();s.maintenance.compatible=false;h.set(s);
  assert.equal(h.nodes.get('servoRead').disabled,true);assert.equal(h.nodes.get('servoCenter').disabled,true);
  assert(h.nodes.get('servoMaintenanceTitle').textContent.includes('固件不支持'));
  h.set(maintenanceFixture());h.nodes.get('servoSupportConfirmed').checked=true;h.eval('renderMaintenance()');assert.equal(h.nodes.get('servoCenter').disabled,false);
  h.age(501);assert.equal(h.nodes.get('servoSelectedPosition').textContent,'—');assert.equal(h.nodes.get('servoCenter').disabled,true);assert.equal(h.nodes.get('servoSupportConfirmed').checked,false);
  assert.equal(h.nodes.get('servoExit').disabled,false);
});

test('maintenance selected ID read and center send explicit target within saved range',async()=>{
  const h=harness(),s=maintenanceFixture();h.set(s);
  h.nodes.get('servoQueryId').value='12';h.nodes.get('servoQueryId').oninput();assert.equal(h.nodes.get('servoSelectedPosition').textContent,'—');
  h.post(async(url,options)=>{assert.deepEqual(JSON.parse(options.body),{action:'read',generation:101,id:12});const next=maintenanceFixture();next.maintenance.servo.id=12;return response(next);});
  await h.nodes.get('servoRead').onclick();h.nodes.get('servoSupportConfirmed').checked=true;h.eval('renderMaintenance()');
  h.post(async(url,options)=>{assert.deepEqual(JSON.parse(options.body),{action:'center',generation:101,id:12,target:2047,support_confirmed:true});return response(maintenanceFixture());});
  await h.nodes.get('servoCenter').onclick();assert.equal(h.posts().length,2);assert.equal(h.nodes.get('servoSupportConfirmed').checked,false);
  h.nodes.get('servoQueryId').value='254';h.nodes.get('servoQueryId').oninput();assert.equal(h.nodes.get('servoRead').disabled,true);
});

test('rename requires sole servo and mapping confirmations; saved configuration alone cannot authorize',async()=>{
  const h=harness(),s=maintenanceFixture();h.set(s);h.nodes.get('servoNewId').value='20';h.nodes.get('servoNewId').oninput();
  await h.nodes.get('servoRename').onclick();assert.equal(h.posts().length,0);
  h.nodes.get('servoIsolatedConfirmed').checked=true;h.eval('renderMaintenance()');assert.equal(h.nodes.get('servoRename').disabled,true);
  h.nodes.get('servoMappingConfirmed').checked=true;h.eval('renderMaintenance()');assert.equal(h.nodes.get('servoRename').disabled,false);
  h.post(async(url,options)=>{assert.deepEqual(JSON.parse(options.body),{action:'rename',generation:101,id:10,new_id:20,isolated_confirmed:true,mapping_acknowledged:true});const next=maintenanceFixture();next.maintenance.identity_changed=true;return response(next);});
  await h.nodes.get('servoRename').onclick();assert.equal(h.posts().length,1);assert.equal(h.nodes.get('servoMappingConfirmed').checked,false);
  h.nodes.get('servoSupportConfirmed').checked=true;h.eval('renderMaintenance()');assert.equal(h.nodes.get('servoCenter').disabled,true);
  assert(h.nodes.get('servoCenterHint').textContent.includes('待核对'));
});

test('stop remains actionable during maintenance operation and uses exit rather than feedback endpoint',async()=>{
  const h=harness(),s=maintenanceFixture();s.maintenance.busy=true;s.maintenance.blocked_reason='故障待核对';h.set(s);
  assert.equal(h.nodes.get('servoExit').disabled,false);assert.equal(h.nodes.get('endFeedback').disabled,false);
  assert.equal(h.nodes.get('servoScan').disabled,true);
  h.post(async(url,options)=>{assert.equal(url,'/api/servo-maintenance');assert.deepEqual(JSON.parse(options.body),{action:'exit',generation:101});return response(maintenanceFixture(false));});
  await h.nodes.get('endFeedback').onclick();assert.equal(h.posts().length,1);
});

test('maintenance timeouts never repeat an action and old responses never overwrite newer session',async()=>{
  const h=harness();h.set(maintenanceFixture());
  h.post(async()=>{const error=new Error('aborted');error.name='AbortError';throw error;});
  await h.nodes.get('servoRead').onclick();assert.equal(h.posts().length,1);assert(h.nodes.get('servoMaintenanceMessage').textContent.includes('不会自动重试'));
  const pending=deferred();h.post(()=>pending.promise);const action=h.nodes.get('servoRead').onclick();
  const newer=maintenanceFixture();newer.session.generation=102;newer.token='new';h.set(newer);pending.resolve(response(maintenanceFixture()));await action;
  assert.equal(h.eval('snapshot.session.generation'),102);assert.equal(h.eval('snapshot.token'),'new');assert.equal(h.posts().length,2);
});

test('status fetch failure and pending request cannot disable maintenance exit or restore an old action reply',async()=>{
  const h=harness();h.set(maintenanceFixture());h.eval("fetchError='状态读取失败';render()");
  assert.equal(h.nodes.get('servoExit').disabled,false);assert.equal(h.nodes.get('endFeedback').disabled,false);
  h.set(maintenanceFixture());const pending=deferred();
  h.post(async(url,options)=>JSON.parse(options.body).action==='exit'?response(maintenanceFixture(false)):pending.promise);
  const read=h.nodes.get('servoRead').onclick();assert.equal(h.nodes.get('servoExit').disabled,false);
  await h.nodes.get('servoExit').onclick();assert.equal(h.eval('snapshot.maintenance.owns_connection'),false);
  pending.resolve(response(maintenanceFixture()));await read;
  assert.equal(h.eval('snapshot.maintenance.owns_connection'),false);assert.equal(h.posts().length,2);
});

test('one-click scan connects and scans using one explicit request, while fault hold still refuses',async()=>{
  const h=harness(),s=maintenanceFixture(false);h.set(s);
  assert.equal(h.nodes.get('servoScan').textContent,'一键扫描 ID 和位置');assert.equal(h.nodes.get('servoScan').disabled,false);
  h.post(async(url,options)=>{assert.deepEqual(JSON.parse(options.body),{action:'scan',generation:101});const next=maintenanceFixture();next.maintenance.busy=true;next.maintenance.action='scan';return response(next);});
  await h.nodes.get('servoScan').onclick();assert.equal(h.posts().length,1);assert(h.nodes.get('servoScan').textContent.includes('正在扫描'));assert.equal(h.nodes.get('servoExit').disabled,false);
  const held=maintenanceFixture(false);held.maintenance.blocked_reason='电压故障未核对';h.set(held);await h.nodes.get('servoScan').onclick();assert.equal(h.posts().length,1);
});

test('editable return target is preserved per ID and transmitted without changing saved calibration',async()=>{
  const h=harness(),s=maintenanceFixture();h.set(s);
  const saved=JSON.stringify(s.calibration);h.nodes.get('servoTarget').value='2150';h.nodes.get('servoTarget').oninput();
  h.nodes.get('servoSupportConfirmed').checked=true;h.eval('renderMaintenance()');assert.equal(h.nodes.get('servoCenter').disabled,false);assert(h.nodes.get('servoCenter').textContent.includes('2150'));
  h.post(async(url,options)=>{assert.deepEqual(JSON.parse(options.body),{action:'center',generation:101,id:10,target:2150,support_confirmed:true});return response(s);});
  await h.nodes.get('servoCenter').onclick();assert.equal(JSON.stringify(h.eval('snapshot.calibration')),saved);
  h.nodes.get('servoQueryId').value='12';h.nodes.get('servoQueryId').oninput();assert.equal(h.nodes.get('servoTarget').value,'2047');
  h.nodes.get('servoTarget').value='2200';h.nodes.get('servoTarget').oninput();h.set(s);assert.equal(h.nodes.get('servoTarget').value,'2200');
  h.nodes.get('servoQueryId').value='10';h.nodes.get('servoQueryId').oninput();assert.equal(h.nodes.get('servoTarget').value,'2150');
  h.navigate('#device-status');h.navigate('#servo-settings');assert.equal(h.nodes.get('servoTarget').value,'2150');
});

test('default 2047, saved zero and current position presets fill target without commanding movement',()=>{
  const h=harness();h.set(maintenanceFixture());
  h.nodes.get('servoUseZero').onclick();assert.equal(h.nodes.get('servoTarget').value,'2048');
  h.nodes.get('servoUseCurrent').onclick();assert.equal(h.nodes.get('servoTarget').value,'2058');
  h.nodes.get('servoResetTarget').onclick();assert.equal(h.nodes.get('servoTarget').value,'2047');assert.equal(h.posts().length,0);
  assert.equal(h.nodes.get('servoPositionDegrees').textContent,(2058/4096*360).toFixed(1)+'°');assert(h.nodes.get('servoPositionReadTime').textContent.includes('自动读取中'));
});

test('invalid and unsafe targets refuse motion even when current servo and saved calibration are valid',async()=>{
  const h=harness();h.set(maintenanceFixture());
  for(const value of ['', '-1','4096','2.5','abc','1499','2601']){
    h.nodes.get('servoTarget').value=value;h.nodes.get('servoTarget').oninput();h.nodes.get('servoSupportConfirmed').checked=true;h.eval('renderMaintenance()');
    assert.equal(h.nodes.get('servoCenter').disabled,true,value);await h.nodes.get('servoCenter').onclick();
  }
  assert.equal(h.posts().length,0);assert(h.nodes.get('servoCenterHint').textContent.includes('超出安全范围'));
});

test('current position outside saved range blocks return before a request is sent',async()=>{
  const h=harness(),s=maintenanceFixture();s.maintenance.servo.position=2700;h.set(s);h.nodes.get('servoSupportConfirmed').checked=true;h.eval('renderMaintenance()');
  assert.equal(h.nodes.get('servoCenter').disabled,true);assert(h.nodes.get('servoCenterHint').textContent.includes('当前位置 2700 超出'));await h.nodes.get('servoCenter').onclick();assert.equal(h.posts().length,0);
});

test('scan table shows recorded positions and selection reads once; stable poll preserves button DOM',async()=>{
  const h=harness(),s=maintenanceFixture();s.maintenance.results=[{id:12,position:2077,torque:0,online:true,error:0,state:0},{id:34,position:2090,torque:0,online:true,error:0,state:0}];h.set(s);
  const rows=h.nodes.get('servoScanRows').children;assert.equal(rows.length,2);assert.equal(rows[0].children[1].textContent,'2077');assert(rows[0].children[2].textContent.includes('°'));
  const button=rows[0].children[0].children[0];h.eval('render()');assert.equal(h.nodes.get('servoScanRows').children[0].children[0].children[0],button);
  h.post(async(url,options)=>{assert.deepEqual(JSON.parse(options.body),{action:'read',generation:101,id:12});const next=clone(s);next.maintenance.servo={id:12,position:2078,torque:0,online:true,error:0,state:0};return response(next);});
  await button.onclick();assert.equal(h.posts().length,1);assert.equal(h.nodes.get('servoQueryId').value,'12');assert.equal(h.nodes.get('servoSelectedPosition').textContent,'2078');
  h.age(501);assert.equal(h.nodes.get('servoSelectedPosition').textContent,'—');assert.equal(h.nodes.get('servoScanRows').children[0].children[1].textContent,'2077');assert(h.nodes.get('servoScanResult').textContent.includes('不代表实时状态'));
});

test('operation log keeps results as text and never turns them into markup',()=>{
  const h=harness(),s=maintenanceFixture();h.set(s);s.maintenance.message='<script>bad()</script>';h.set(s);
  const lines=h.nodes.get('servoLogs').children;assert(lines.length>=2);assert.equal(lines[0].children[1].textContent,'<script>bad()</script>');
  const count=lines.length;h.eval('render()');assert.equal(h.nodes.get('servoLogs').children.length,count);
});

function recheckFixture(generation=101){
  const s=jointFixture('all-complete',generation);s.session.status='fault';s.session.error='历史舵机电压报警';s.session.recovery_allowed=false;s.session.recovery_blocked_reason='舵机反馈尚需重新核对';s.session.recheck_allowed=true;s.session.recheck_blocked_reason=null;s.session.feedback_recheck=null;s.feedback=null;return s;
}

function zeroFixture(stage='idle',generation=101){
  const s=jointFixture('all-complete',generation);s.feedback.last_error=0;s.feedback.positions[2]=1815;s.calibration.joints[2].zero=2032;
  s.joint_zero={id:stage==='idle'?null:12,stage,busy:stage==='sampling',old_zero:stage==='idle'?null:2032,candidate_zero:['ready','saved'].includes(stage)?1815:null,sample_count:stage==='sampling'?4:18,progress:stage==='sampling'?.2:1,error:null,message:''};return s;
}
function supportedZero(h){h.nodes.get('zeroSupportConfirmed').checked=true;h.nodes.get('zeroReferenceConfirmed').checked=true;h.eval('render()');}

test('mechanical zero page restores a visible sidebar and 15 selectable saved and live readings without connecting',()=>{
  const h=harness('http:','#joint-zeros');h.set(zeroFixture());
  assert.equal(h.eval('workspaceView'),'joint-zeros');assert.equal(h.nodes.get('joint-zeros').hidden,false);assert.equal(h.nodes.get('joint-steps').hidden,true);assert.equal(h.nodes.get('zeroRows').children.length,15);
  const row=h.nodes.get('zeroRows').children[2];assert.equal(row.children[1].textContent,'1815');assert.equal(row.children[2].textContent,'2032');row.children[4].children[0].onclick();
  assert.equal(h.eval('selectedJointId'),12);assert(h.nodes.get('zeroSelectedLabel').textContent.includes('只更新 ID 12'));assert.equal(h.posts().length,0);
  h.navigate('#joint-steps');assert.equal(h.eval('selectedJointId'),12);assert.equal(h.posts().length,0);
});

test('joint calibration shows the same zero editor beside readings and navigation never duplicates controls or sends requests',()=>{
  const h=harness('http:','#joint-steps');h.set(zeroFixture());h.eval('chooseJoint(12)');const editor=h.nodes.get('zeroEditor'),capture=h.nodes.get('zeroCapture');
  assert.equal(editor.parentElement,h.nodes.get('jointInlineZeroSlot'));assert.equal(h.nodes.get('zeroCapture').textContent,'记零点');assert(h.nodes.get('zeroEditorTitle').textContent.includes('ID 12'));assert(h.nodes.get('zeroReferenceLabel').textContent.includes('左图灰色'));assert.equal(h.nodes.get('zeroOldBox').hidden,true);assert.equal(h.nodes.get('zeroLiveBox').hidden,true);
  h.navigate('#joint-zeros');assert.equal(editor.parentElement,h.nodes.get('zeroEditorHome'));assert.equal(h.nodes.get('jointInlineZeroSlot').children.length,0);assert.equal(h.nodes.get('zeroOldBox').hidden,false);
  h.navigate('#joint-steps');h.navigate('#joint-steps');assert.equal(h.nodes.get('jointInlineZeroSlot').children.filter(node=>node===editor).length,1);assert.equal(h.nodes.get('zeroCapture'),capture);assert.equal(h.eval('selectedJointId'),12);assert.equal(h.posts().length,0);
});

test('inline zero capture reuses joint support and makes one selectedjoint preview request without visiting another page',async()=>{
  const h=harness('http:','#joint-steps'),s=zeroFixture();h.set(s);await h.settleOld(s);h.eval('chooseJoint(12)');supportedJoint(h);h.nodes.get('zeroReferenceConfirmed').checked=true;h.eval('render()');
  assert.equal(h.nodes.get('zeroSupportConfirmed').checked,true);assert.equal(h.nodes.get('zeroCapture').disabled,false);assert.equal(h.nodes.get('zeroSave').disabled,true);
  h.post(async(url,options)=>{assert.equal(url,'/api/joint-zero');assert.deepEqual(JSON.parse(options.body),{generation:101,id:12,action:'capture',support_confirmed:true,zero_reference_confirmed:true});const next=zeroFixture('sampling');h.set(next);return response(next);});
  await h.nodes.get('zeroCapture').onclick();assert.equal(h.posts().length,1);assert.equal(h.eval('workspaceView'),'joint-steps');assert.equal(h.eval('snapshot.calibration.joints[2].zero'),2032);assert.equal(h.nodes.get('zeroSave').disabled,true);
});

test('inline zero offers both confirmations beside capture and shares support in both directions',async()=>{
  const h=harness('http:','#joint-steps'),s=zeroFixture();h.set(s);await h.settleOld(s);h.eval('chooseJoint(12)');
  const editorStart=html.indexOf('id="zeroEditor"'),editorEnd=html.indexOf('id="zeroGate"');
  assert(html.indexOf('id="zeroSupportConfirmed"')>editorStart&&html.indexOf('id="zeroSupportConfirmed"')<editorEnd);
  assert.equal([...html.matchAll(/id="zeroSupportConfirmed"/g)].length,1);
  h.nodes.get('zeroReferenceConfirmed').checked=true;h.nodes.get('zeroReferenceConfirmed').onchange();
  assert.equal(h.nodes.get('zeroCapture').disabled,true);assert(h.nodes.get('zeroResult').textContent.includes('请勾选本区'));
  assert(h.nodes.get('zeroCapture').title.includes('机身已由垫块'));
  h.nodes.get('zeroSupportConfirmed').checked=true;h.nodes.get('zeroSupportConfirmed').onchange();
  assert.equal(h.nodes.get('jointSupportConfirmed').checked,true);assert.equal(h.nodes.get('zeroCapture').disabled,false);
  h.nodes.get('jointSupportConfirmed').checked=false;h.nodes.get('jointSupportConfirmed').onchange();
  assert.equal(h.nodes.get('zeroSupportConfirmed').checked,false);assert.equal(h.nodes.get('zeroCapture').disabled,true);
  h.nodes.get('zeroSupportConfirmed').checked=true;h.nodes.get('zeroSupportConfirmed').onchange();
  h.post(async(url,options)=>{assert.equal(url,'/api/joint-zero');const body=JSON.parse(options.body);assert.equal(body.support_confirmed,true);assert.equal(body.zero_reference_confirmed,true);return response(zeroFixture('sampling'));});
  await h.nodes.get('zeroCapture').onclick();assert.equal(h.posts().length,1);
});

test('inline confirmation survives display expiry while capture stays blocked; a new generation clears it',()=>{
  const h=harness('http:','#joint-steps');h.set(zeroFixture());h.nodes.get('zeroReferenceConfirmed').checked=true;
  h.nodes.get('zeroSupportConfirmed').checked=true;h.nodes.get('zeroSupportConfirmed').onchange();assert.equal(h.nodes.get('zeroCapture').disabled,false);
  h.age(501);for(const id of ['zeroSupportConfirmed','jointSupportConfirmed','zeroReferenceConfirmed'])assert.equal(h.nodes.get(id).checked,true);assert.equal(h.nodes.get('zeroCapture').disabled,true);
  h.set(zeroFixture('idle',102));assert.equal(h.nodes.get('zeroSupportConfirmed').checked,false);assert.equal(h.nodes.get('jointSupportConfirmed').checked,false);assert.equal(h.nodes.get('zeroCapture').disabled,true);
});

test('inline preview requires explicit save with joint support and preserves unrelated calibration',async()=>{
  const h=harness('http:','#joint-steps'),s=zeroFixture('ready');h.set(s);await h.settleOld(s);supportedJoint(h);h.nodes.get('zeroReferenceConfirmed').checked=true;h.eval('render()');
  assert.equal(h.nodes.get('zeroReadings').hidden,false);assert.equal(h.nodes.get('zeroSave').disabled,false);jointLocked(h);const before=clone(s.calibration);
  h.post(async(url,options)=>{assert.deepEqual(JSON.parse(options.body),{generation:101,id:12,action:'save',support_confirmed:true,zero_reference_confirmed:true});const next=zeroFixture('saved');next.calibration.joints[2]={...next.calibration.joints[2],zero:1815,min:null,max:null,direction_confirmed:false};h.set(next);return response(next);});
  await h.nodes.get('zeroSave').onclick();const saved=clone(h.eval('snapshot.calibration'));assert.equal(saved.joints[2].zero,1815);assert.deepEqual(saved.joints.filter(j=>j.id!==12),before.joints.filter(j=>j.id!==12));assert.deepEqual(saved.imu,before.imu);assert.equal(h.posts().length,1);assert.equal(h.nodes.get('zeroReferenceConfirmed').checked,false);
});

test('inline zero confirmation clears when changing joints and unfinished direction steps remain intact until explicit cancellation',async()=>{
  const h=harness('http:','#joint-steps');h.set(zeroFixture());h.eval('chooseJoint(12)');supportedJoint(h);h.nodes.get('zeroReferenceConfirmed').checked=true;h.eval('render()');h.eval('chooseJoint(13)');assert.equal(h.nodes.get('zeroReferenceConfirmed').checked,false);assert(h.nodes.get('zeroEditorTitle').textContent.includes('ID 13'));
  const s=zeroFixture();s.joint_wizard={...jointFixture('range-retry').joint_wizard};h.set(s);supportedJoint(h);h.nodes.get('zeroReferenceConfirmed').checked=true;h.eval('render()');const before=JSON.stringify(h.eval('snapshot.joint_wizard'));
  assert.equal(h.nodes.get('zeroCapture').disabled,true);assert.equal(h.nodes.get('zeroCancelJointDraft').hidden,false);await h.nodes.get('zeroCapture').onclick();assert.equal(h.posts().length,0);assert.equal(JSON.stringify(h.eval('snapshot.joint_wizard')),before);
});

test('zero capture needs both confirmations and creates only a candidate preview through one exact request',async()=>{
  const h=harness('http:','#joint-zeros'),s=zeroFixture();h.set(s);await h.settleOld(s);h.eval('chooseJoint(12)');
  await h.nodes.get('zeroCapture').onclick();assert.equal(h.posts().length,0);h.nodes.get('zeroSupportConfirmed').checked=true;h.eval('render()');assert.equal(h.nodes.get('zeroCapture').disabled,true);
  supportedZero(h);assert.equal(h.nodes.get('zeroCapture').disabled,false);assert.equal(h.nodes.get('zeroSave').disabled,true);
  const pending=deferred();h.post(async(url,options)=>{assert.equal(url,'/api/joint-zero');assert.deepEqual(JSON.parse(options.body),{generation:101,id:12,action:'capture',support_confirmed:true,zero_reference_confirmed:true});return pending.promise;});
  const capture=h.nodes.get('zeroCapture').onclick();await h.nodes.get('zeroCapture').onclick();assert.equal(h.posts().length,1);h.set(zeroFixture('sampling'));pending.resolve(response(zeroFixture('sampling')));await capture;
  assert.equal(h.eval('snapshot.calibration.joints[2].zero'),2032);assert.equal(h.nodes.get('zeroSave').disabled,true);assert.equal(h.nodes.get('zeroJointSelect').disabled,true);
  h.set(zeroFixture('ready'));assert.equal(h.nodes.get('zeroCandidateReading').textContent,'1815');assert.equal(h.nodes.get('zeroOldReading').textContent,'2032');assert(h.nodes.get('zeroResult').textContent.includes('尚未修改'));jointLocked(h);assert.equal(h.nodes.get('zeroSave').disabled,false);
});

test('zero save updates only selected record, clears its completion and preserves other joints and IMU',async()=>{
  const h=harness('http:','#joint-zeros'),s=zeroFixture('ready');h.set(s);await h.settleOld(s);supportedZero(h);const old=JSON.stringify(s.calibration);
  h.post(async(url,options)=>{assert.equal(url,'/api/joint-zero');assert.deepEqual(JSON.parse(options.body),{generation:101,id:12,action:'save',support_confirmed:true,zero_reference_confirmed:true});const saved=zeroFixture('saved');saved.calibration.joints[2]={...saved.calibration.joints[2],zero:1815,min:null,max:null,direction_confirmed:false};saved.joint_zero.message='12 号零点已保存，其他关节和 IMU 保留';h.set(saved);return response(saved);});
  await h.nodes.get('zeroSave').onclick();const updated=h.eval('snapshot.calibration');assert.equal(updated.joints[2].zero,1815);assert.equal(updated.joints[2].direction_confirmed,false);assert.equal(updated.joints[2].min,null);
  const before=JSON.parse(old);for(let i=0;i<15;i++)if(i!==2)assert.equal(JSON.stringify(updated.joints[i]),JSON.stringify(before.joints[i]));assert.equal(JSON.stringify(updated.imu),JSON.stringify(before.imu));assert.equal(h.nodes.get('zeroReferenceConfirmed').checked,false);assert.equal(h.posts().length,1);
});

test('direction range draft must be explicitly cancelled before zero recapture without dropping saved configuration',async()=>{
  const h=harness('http:','#joint-zeros'),s=zeroFixture();s.joint_wizard={...jointFixture('range-retry').joint_wizard};h.set(s);await h.settleOld(s);supportedZero(h);
  assert.equal(h.nodes.get('zeroCapture').disabled,true);assert.equal(h.nodes.get('zeroCancelJointDraft').hidden,false);assert(h.nodes.get('zeroGate').textContent.includes('先点击'));
  const saved=JSON.stringify(s.calibration);h.post(async(url,options)=>{assert.equal(url,'/api/joint-calibration');assert.equal(JSON.parse(options.body).action,'cancel');const next=zeroFixture();next.joint_wizard.stage='cancelled';h.set(next);return response(next);});
  await h.nodes.get('zeroCancelJointDraft').onclick();assert.equal(JSON.stringify(h.eval('snapshot.calibration')),saved);assert.equal(h.nodes.get('zeroCapture').disabled,false);assert.equal(h.posts().length,1);
});

test('stale faulty moved or unconfirmed zero preview cannot be saved and stale encoder text disappears',async()=>{
  const h=harness('http:','#joint-zeros');
  for(const mutate of [s=>s.feedback.torque_mask=4,s=>s.feedback.fault_mask=4,s=>s.feedback.online_mask=0,s=>s.feedback.positions[2]=1822,s=>s.feedback.last_error=1,s=>s.session.status='closed',s=>delete s.joint_zero]){
    const s=zeroFixture('ready');mutate(s);h.set(s);supportedZero(h);assert.equal(h.nodes.get('zeroSave').disabled,true);await h.nodes.get('zeroSave').onclick();
  }
  h.set(zeroFixture('ready'));supportedZero(h);h.age(501);assert.equal(h.nodes.get('zeroLiveReading').textContent,'—');assert.equal(h.nodes.get('zeroSupportConfirmed').checked,true);assert.equal(h.nodes.get('zeroSave').disabled,true);assert.equal(h.posts().length,0);
});

test('zero cancellation stays available after loss of feedback and never needs physical confirmations',async()=>{
  const h=harness('http:','#joint-zeros'),s=zeroFixture('ready');s.session.status='fault';s.feedback=null;h.set(s);await h.settleOld(s);assert.equal(h.nodes.get('zeroCancel').disabled,false);
  h.post(async(url,options)=>{assert.equal(url,'/api/joint-zero');assert.deepEqual(JSON.parse(options.body),{generation:101,id:12,action:'cancel',support_confirmed:false,zero_reference_confirmed:false});const next=zeroFixture('cancelled');h.set(next);return response(next);});
  await h.nodes.get('zeroCancel').onclick();assert.equal(h.posts().length,1);
});

test('zero request timeout never retries or saves and an old generation response cannot roll back state',async()=>{
  const h=harness('http:','#joint-zeros');h.set(zeroFixture());h.eval('chooseJoint(12)');supportedZero(h);h.post(async()=>{const e=new Error('timeout');e.name='AbortError';throw e;});await h.nodes.get('zeroCapture').onclick();assert.equal(h.posts().length,1);assert(h.nodes.get('zeroResult').textContent.includes('不会自动'));
  h.set(zeroFixture());supportedZero(h);const pending=deferred();h.post(()=>pending.promise);const capture=h.nodes.get('zeroCapture').onclick();h.set(zeroFixture('idle',102));pending.resolve(response(zeroFixture('ready',101)));await capture;assert.equal(h.eval('snapshot.session.generation'),102);assert.equal(h.eval('snapshot.joint_zero.stage'),'idle');assert.equal(h.posts().length,2);
});

test('file origin cannot use mechanical zero actions with an injected ready candidate',async()=>{
  const h=harness('file:','#joint-zeros');h.set(zeroFixture('ready'));supportedZero(h);for(const action of ['capture','save','cancel'])await h.eval('runZero('+JSON.stringify(action)+')');assert.equal(h.calls.length,0);
});

test('historical servo hold has a self-service IMU recheck button and no automatic connection',async()=>{
  const h=harness('http:','#imu-steps'),s=recheckFixture();h.set(s);await h.settleOld(s);await h.eval('refresh()');
  assert.equal(h.posts().length,0);assert.equal(h.nodes.get('recoverFeedback').disabled,false);assert.equal(h.nodes.get('recoverFeedback').textContent,'重新检查连接与 IMU');assert(h.nodes.get('recoveryStatus').textContent.includes('URT2 电池断开'));
  h.post(async(url,options)=>{assert.equal(url,'/api/recheck-feedback');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose:'imu'});const next=recheckFixture(102);next.session.status='waiting';next.session.recheck_allowed=false;next.session.recovery_pending=true;next.session.feedback_recheck={purpose:'imu',phase:'checking'};return response(next);});
  await h.nodes.get('recoverFeedback').onclick();assert.equal(h.posts().length,1);locked(h);jointLocked(h);assert(h.nodes.get('recoveryStatus').textContent.includes('连续有效数据'));
});

test('joint view requests powered joint recheck rather than accepting IMU-only success',async()=>{
  const h=harness('http:','#joint-steps');h.set(recheckFixture());
  assert.equal(h.nodes.get('recoverFeedback').textContent,'重新检查连接与舵机');
  h.post(async(url,options)=>{assert.equal(url,'/api/recheck-feedback');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose:'joints'});return response(recheckFixture(102));});
  await h.nodes.get('recoverFeedback').onclick();assert.equal(h.posts().length,1);
  const imu=jointFixture('all-complete',103);imu.feedback.online_mask=32767;imu.session.feedback_recheck={purpose:'imu',phase:'passed'};imu.session.recovery_allowed=false;h.set(imu);supportedJoint(h);
  jointLocked(h);assert(h.eval('jointGateReason()').includes('只通过了 IMU'));
});

test('all calibration pages expose an explicit same-page feedback recheck with the correct purpose',async()=>{
  for(const [hash,button,purpose] of [
    ['#device-status','recoverFeedbackNearFault','joints'],
    ['#joint-zeros','recoverFeedbackInZeros','joints'],
    ['#joint-steps','recoverFeedbackInJoints','joints'],
    ['#imu-steps','recoverFeedbackInWizard','imu']
  ]){
    const h=harness('http:',hash),s=recheckFixture();h.set(s);
    const label=purpose==='imu'?'重新检查连接与 IMU':'重新检查连接与舵机';
    assert.equal(h.nodes.get('recoverFeedback').hidden,false);
    assert.equal(h.nodes.get('recoveryPanel').hidden,false);
    assert.equal(h.nodes.get(button).disabled,false);assert.equal(h.nodes.get(button).textContent,label);
    assert.equal(h.posts().length,0);
    const pending=deferred();h.post((url,options)=>{
      assert.equal(url,'/api/recheck-feedback');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose});return pending.promise;
    });
    const click=h.nodes.get(button).onclick();await h.nodes.get('recoverFeedback').onclick();
    assert.equal(h.posts().length,1);jointLocked(h);locked(h);
    const checking=clone(s);checking.session.status='waiting';checking.session.recovery_pending=true;checking.session.recheck_allowed=false;checking.session.feedback_recheck={purpose,phase:'checking'};
    pending.resolve(response(checking));await click;
    assert.equal(h.nodes.get(button).disabled,true);assert.equal(h.posts().length,1);
  }
});

test('leaving motion updates retry labels and IMU power instructions immediately without polling or requests',()=>{
  const h=harness('http:','#motion-models');h.set(motionRecheckFixture());
  assert.equal(h.nodes.get('recoveryPanel').hidden,true);const calls=h.calls.length;
  h.navigate('#imu-steps');
  assert.equal(h.nodes.get('recoverFeedback').hidden,false);assert.equal(h.nodes.get('recoveryPanel').hidden,false);
  assert.equal(h.nodes.get('recoverFeedbackInWizard').textContent,'重新检查连接与 IMU');
  assert(h.nodes.get('recoveryStatus').textContent.includes('URT2 电池断开'));
  h.navigate('#joint-zeros');
  assert.equal(h.nodes.get('recoverFeedbackInZeros').textContent,'重新检查连接与舵机');
  assert(h.nodes.get('recoveryStatus').textContent.includes('舵机供电正常'));
  assert(!h.nodes.get('recoveryStatus').textContent.includes('URT2 电池断开'));
  assert.equal(h.calls.length,calls);assert.equal(h.posts().length,0);
});

test('checking states do not expose live values or enable samples; passing IMU requires new physical confirmations',()=>{
  const h=harness('http:','#imu-steps'),checking=jointFixture('all-complete',102);checking.session.status='waiting';checking.session.feedback_recheck={purpose:'imu',phase:'checking'};checking.session.recovery_pending=false;checking.feedback.online_mask=0;h.set(checking);
  assert.equal(h.nodes.get('accZ').textContent,'—');locked(h);assert.equal(h.nodes.get('recoverFeedback').disabled,true);
  const passed=clone(checking);passed.session.status='active';passed.session.feedback_recheck.phase='passed';passed.session.recovery_allowed=false;passed.session.recovery_blocked_reason='带电舵机仍待复查';h.set(passed);
  assert(h.nodes.get('recoveryStatus').textContent.includes('IMU 采集已恢复'));assert(h.nodes.get('recoveryStatus').textContent.includes('舵机供电状态仍需单独复查'));locked(h);
  h.nodes.get('mountedConfirmed').checked=true;h.nodes.get('supportConfirmed').checked=true;h.eval('render()');assert.equal(h.nodes.get('wizardLevel').disabled,false);jointLocked(h);
});

test('hard storage identity or closure failure keeps all recovery entries disabled without a POST',async()=>{
  const h=harness('http:','#imu-steps');
  for(const reason of ['新的 SD 卡读取错误','板卡身份已改变','原串口关闭尚无证明']){
    const s=recheckFixture();s.session.recheck_allowed=false;s.session.recheck_blocked_reason=reason;h.set(s);assert.equal(h.nodes.get('recoverFeedback').disabled,true);assert(h.nodes.get('recoveryStatus').textContent.includes(reason));await h.nodes.get('recoverFeedback').onclick();
  }
  assert.equal(h.posts().length,0);
});

test('contradictory recheck permission cannot bypass a reported closure restriction',async()=>{
  const h=harness('http:','#imu-steps'),s=recheckFixture();s.session.recheck_blocked_reason='原串口关闭尚无证明';h.set(s);
  assert.equal(h.nodes.get('recoverFeedback').disabled,true);await h.nodes.get('recoverFeedback').onclick();assert.equal(h.posts().length,0);
});

test('a recheck timeout or repeated click cannot create an automatic second attempt',async()=>{
  const h=harness('http:','#imu-steps');h.set(recheckFixture());const pending=deferred();h.post(()=>pending.promise);
  const first=h.nodes.get('recoverFeedback').onclick();await h.nodes.get('recoverFeedback').onclick();assert.equal(h.posts().length,1);
  pending.resolve(response({error:'实际电压报警仍存在'},409));await first;assert(h.nodes.get('recoveryStatus').textContent.includes('实际电压报警仍存在'));assert.equal(h.posts().length,1);
  h.set(recheckFixture());h.post(async()=>{const error=new Error('timeout');error.name='AbortError';throw error;});await h.nodes.get('recoverFeedback').onclick();assert(h.nodes.get('recoveryStatus').textContent.includes('不会自动重试'));assert.equal(h.posts().length,2);
});

test('file origin cannot start recheck even if a synthetic permission snapshot is supplied',async()=>{
  const h=harness('file:','#imu-steps');h.set(recheckFixture());await h.nodes.get('recoverFeedback').onclick();assert.equal(h.calls.length,0);
});

test('file origin cannot use maintenance even with injected compatible snapshot',async()=>{
  const h=harness('file:');h.set(maintenanceFixture());
  for(const action of ['enter','review','read','scan','rename','center','release','exit'])await h.eval('runMaintenance('+JSON.stringify(action)+')');
  assert.equal(h.calls.length,0);
});

function disconnectedFixture(generation=101){
  const s=recheckFixture(generation);s.session.error='连续 0.6 秒未收到新反馈，采集已暂停';
  s.session.recheck_allowed=false;s.session.recheck_blocked_reason='旧串口关闭尚未确认';
  s.session.recovery_recheck={category:'closure',technical_reason:'真机反馈连接已结束，未收到完整关闭报告',message:'旧串口关闭尚未确认'};
  s.session.connection_check={allowed:true,blocked_reason:null,phase:'idle',error:null,message:'可检查连接'};
  return s;
}

test('lost SSH stream keeps first stop cause visible and offers a same-page connection check without automatic requests',async()=>{
  const h=harness('http:','#joint-steps'),s=disconnectedFixture();h.set(s);await h.settleOld(s);await h.eval('refresh()');
  assert.equal(h.posts().length,0);assert.equal(h.nodes.get('sessionAlert').textContent,s.session.error);
  assert.equal(h.nodes.get('recoverFeedback').textContent,'重新检查连接并继续');assert.equal(h.nodes.get('recoverFeedback').disabled,false);
  assert(h.nodes.get('recoveryStatus').textContent.includes('没有收到旧串口的关闭确认'));
  assert(!h.nodes.get('recoveryStatus').textContent.includes('身份检查未通过'));
  assert(h.nodes.get('recoveryHelp').textContent.includes('不需要在聊天里重开'));
  h.nodes.get('connectionStatusLink').onclick();assert.equal(h.nodes.get('recoveryPanel').lastScroll.block,'center');assert.equal(h.nodes.get('recoverFeedbackNearFault').focused,true);
  jointLocked(h);locked(h);
});

test('connection checks on every calibration page send the exact generation and current purpose once',async()=>{
  for(const [hash,purpose] of [['#device-status','joints'],['#joint-zeros','joints'],['#joint-steps','joints'],['#imu-steps','imu']]){
    const h=harness('http:',hash),s=disconnectedFixture();h.set(s);const pending=deferred();h.post((url,options)=>{
      assert.equal(url,'/api/check-connection');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose});return pending.promise;
    });
    const click=h.nodes.get('recoverFeedback').onclick();await h.nodes.get('recoverFeedbackNearFault').onclick();assert.equal(h.posts().length,1);jointLocked(h);locked(h);
    const checking=clone(s);checking.session.connection_check.phase='checking';checking.session.connection_check.allowed=false;checking.session.recovery_pending=true;
    pending.resolve(response(checking));await click;assert(h.nodes.get('recoveryStatus').textContent.includes('旧串口占用'));assert.equal(h.nodes.get('recoverFeedback').disabled,true);
    h.set(checking);assert.equal(h.eval('recoveryPhase'),'checking');jointLocked(h);locked(h);
  }
});

test('failed connection check exposes actual holder or trust failure without retry or live samples',async()=>{
  const h=harness('http:','#joint-steps');h.set(disconnectedFixture());
  const failed=disconnectedFixture();failed.session.connection_check={allowed:false,blocked_reason:'新的 SD 卡读取错误',phase:'failed',error:'新的 SD 卡读取错误',message:'检查未通过'};
  failed.session.recovery_recheck.category='trust';failed.session.recheck_blocked_reason='新的 SD 卡读取错误';
  h.post(async()=>response(failed));await h.nodes.get('recoverFeedback').onclick();
  assert(h.nodes.get('recoveryStatus').textContent.includes('新的 SD 卡读取错误'));assert.equal(h.nodes.get('recoverFeedback').disabled,true);assert.equal(h.posts().length,1);jointLocked(h);
  h.set(failed);await h.settleOld(failed);h.navigate('#imu-steps');await h.eval('refresh()');assert.equal(h.posts().length,1);locked(h);
});

test('missing connection-check permission or a reported restriction cannot be bypassed by a trust retry',async()=>{
  const h=harness('http:','#joint-steps');
  for(const mutate of [s=>{s.session.recovery_recheck.category='trust';s.session.connection_check.allowed=false;},s=>{s.session.connection_check.blocked_reason='旧读取进程仍在运行';}]){
    const s=disconnectedFixture();mutate(s);h.set(s);assert.equal(h.nodes.get('recoverFeedback').disabled,true);await h.nodes.get('recoverFeedback').onclick();
  }
  assert.equal(h.posts().length,0);
});

test('blocked identity check names the lock and safely exposes its exact reason',async()=>{
  const h=harness('http:','#device-status'),s=disconnectedFixture();
  s.session.recovery_recheck={category:'trust',message:'连接检查未通过',technical_reason:'Connection-check UART node proof differs'};
  s.session.connection_check={allowed:false,phase:'failed',error:'串口设备信息校验未通过',technical_reason:'Connection-check UART node proof differs <img onerror=bad>'};
  s.session.recovery_allowed=false;s.session.recheck_allowed=false;s.session.recovery_blocked_reason='连接检查未通过';
  h.set(s);
  for(const id of ['recoverFeedback','recoverFeedbackNearFault']){assert.equal(h.nodes.get(id).textContent,'需核对连接');assert.equal(h.nodes.get(id).disabled,true);}
  assert.equal(h.nodes.get('recoveryDetails').hidden,false);assert.equal(h.nodes.get('recoveryTechnicalReason').textContent,s.session.connection_check.technical_reason);
  assert(h.nodes.get('recoveryHelp').textContent.includes('入口已锁定'));
  await h.nodes.get('recoverFeedback').onclick();assert.equal(h.posts().length,0);
});

function usbKernelEvidence(){return {new_fault_count:1,fault_details_truncated:false,faults:[{kernel_line_number:936,line:'[  386.045233] usb 1-1.4: USB disconnect, device number 4',line_truncated:false,classifier_reason:'Kernel reported a USB disconnect/reset/error',context:[{kernel_line_number:937,line:'[  386.045990] ch341-uart ttyUSB0: disconnected <img onerror=bad>',line_truncated:false}]}]};}
function kernelCheckFixture(evidence=usbKernelEvidence()){
  const s=disconnectedFixture(),raw='New kernel fault; classifier result requires source review: '+JSON.stringify(evidence);
  s.session.error=raw;s.session.error_details=raw;s.session.recovery_recheck={category:'trust',message:'连接检查未通过',technical_reason:raw};
  s.session.connection_check={allowed:false,phase:'failed',blocked_reason:raw,error:raw,technical_reason:raw};
  s.session.recovery_allowed=false;s.session.recheck_allowed=false;s.session.recovery_blocked_reason=raw;s.session.recheck_blocked_reason=raw;
  return s;
}

test('recorded USB fault is concise and historical while exact kernel evidence stays collapsed',async()=>{
  const h=harness('http:','#device-status'),s=kernelCheckFixture();h.set(s);await h.settleOld(s);await h.eval('refresh()');
  for(const id of ['sessionAlert','recoveryStatus','recoveryWizardStatus','jointRecoveryStatus']){
    const message=h.nodes.get(id).textContent;assert(message.includes('USB 连接曾中断或复位'));assert(message.includes('当前连接检查尚未通过'));
    assert(!message.includes('kernel_line_number'));assert(!message.includes('device number 4'));assert(!message.includes('现在正在断开'));assert(message.length<180);
  }
  assert.equal(h.nodes.get('recoveryDetails').hidden,false);assert.equal(h.nodes.get('recoveryDetails').getAttribute('open'),null);
  assert.equal(h.nodes.get('recoveryTechnicalReason').textContent,s.session.connection_check.technical_reason);
  for(const id of ['recoverFeedback','recoverFeedbackNearFault']){assert.equal(h.nodes.get(id).disabled,true);assert(!h.nodes.get(id).title.includes('kernel_line_number'));}
  locked(h);jointLocked(h);await h.nodes.get('recoverFeedback').onclick();assert.equal(h.posts().length,0);
});

test('motion page exposes exact connection failure details without navigation or hardware requests',()=>{
  const h=harness('http:','#motion-models'),s=kernelCheckFixture();h.set(s);
  assert.equal(h.nodes.get('recoveryPanel').hidden,false);
  assert.equal(h.nodes.get('recoveryDetails').hidden,false);
  assert.equal(h.nodes.get('recoveryDetails').getAttribute('open'),null);
  assert.equal(h.nodes.get('recoveryTechnicalReason').textContent,s.session.connection_check.technical_reason);
  assert(h.nodes.get('recoveryStatus').textContent.includes('展开「查看具体检查原因」'));
  assert(!h.nodes.get('recoveryStatus').textContent.includes('见下方详情'));
  assert.equal(h.nodes.get('recoverFeedbackNearFault').hidden,true);
  h.nodes.get('connectionStatusLink').onclick();
  assert.equal(h.nodes.get('recoveryDetails').open,true);
  assert.equal(h.nodes.get('recoveryDetailsSummary').focused,true);
  assert.equal(h.eval('workspaceView'),'motion-models');
  h.age(200);
  assert.equal(h.nodes.get('recoveryPanel').hidden,false);
  assert.equal(h.nodes.get('recoveryDetails').open,true);
  h.navigate('#device-status');
  assert.equal(h.nodes.get('recoverFeedbackNearFault').hidden,false);
  assert.equal(h.nodes.get('recoveryPanel').hidden,false);
  h.navigate('#motion-models');
  h.set(motionRecheckFixture());
  assert.equal(h.nodes.get('recoveryPanel').hidden,true);
  assert.equal(h.posts().length,0);
});

test('structured kernel details summarize the fault_text message and retain the whole object safely',async()=>{
  const h=harness('http:','#device-status'),s=kernelCheckFixture(),details={new_kernel_faults:usbKernelEvidence()};
  s.session.error='板端报告新的异常，采集未启动或已停止。';s.session.error_details=details;s.session.recovery_recheck.technical_reason=null;
  s.session.connection_check.technical_reason=details;s.session.connection_check.error=s.session.error;
  h.set(s);assert(h.nodes.get('sessionAlert').textContent.includes('USB 连接曾中断或复位'));assert(h.nodes.get('recoveryStatus').textContent.includes('USB 连接曾中断或复位'));
  assert.equal(h.nodes.get('recoveryTechnicalReason').textContent,JSON.stringify(details,null,2));assert(!h.nodes.get('recoveryStatus').textContent.includes('[object Object]'));
  assert.equal(h.nodes.get('recoverFeedback').disabled,true);assert.equal(h.posts().length,0);
});

test('mixed incomplete or malformed kernel evidence never claims USB is the only fault',async()=>{
  const mixed=usbKernelEvidence();mixed.faults.push({classifier_reason:'Kernel reported an SD card I/O error',line:'mmc: error -110'});mixed.new_fault_count=2;
  const truncated=usbKernelEvidence();truncated.fault_details_truncated=true;
  const missing=usbKernelEvidence();delete missing.new_fault_count;
  for(const evidence of [mixed,truncated,missing]){
    const h=harness('http:','#device-status'),s=kernelCheckFixture(evidence);h.set(s);
    assert(h.nodes.get('recoveryStatus').textContent.includes('板端日志记录了异常'));assert(!h.nodes.get('recoveryStatus').textContent.includes('USB 连接曾中断'));
    assert.equal(h.nodes.get('recoveryTechnicalReason').textContent,s.session.connection_check.technical_reason);assert.equal(h.nodes.get('recoverFeedback').disabled,true);assert.equal(h.posts().length,0);
  }
  const h=harness('http:','#device-status'),s=kernelCheckFixture();s.session.connection_check.error=s.session.connection_check.technical_reason='New kernel fault; classifier result requires source review: {broken';
  h.set(s);assert(!h.nodes.get('recoveryStatus').textContent.includes('{broken'));assert(h.nodes.get('recoveryTechnicalReason').textContent.endsWith('{broken'));assert.equal(h.posts().length,0);
});

test('ordinary transport failure keeps explicit connection check available',async()=>{
  const h=harness('http:','#device-status'),s=disconnectedFixture();
  s.session.connection_check.phase='failed';s.session.connection_check.error='X5 USB 地址没有响应';s.session.connection_check.technical_reason='Host is down';
  h.set(s);assert.equal(h.nodes.get('recoverFeedback').disabled,false);assert.equal(h.nodes.get('recoverFeedback').textContent,'重新检查连接并继续');
  assert.equal(h.nodes.get('recoveryTechnicalReason').textContent,'Host is down');assert.equal(h.posts().length,0);
});

test('connection-check response loss never retries and late actual success remains visible',async()=>{
  const h=harness('http:','#joint-steps');h.set(disconnectedFixture());h.post(async()=>{const error=new Error('timeout');error.name='AbortError';throw error;});
  await h.nodes.get('recoverFeedback').onclick();assert.equal(h.posts().length,1);assert(h.nodes.get('recoveryStatus').textContent.includes('不会自动重试'));
  const live=jointFixture('all-complete',102);live.session.recovery_pending=false;live.session.feedback_recheck={purpose:'joints',phase:'passed'};live.session.connection_check={phase:'passed',allowed:false,error:null};h.set(live);
  assert(h.nodes.get('recoveryStatus').textContent.includes('关节采集已恢复'));assert.equal(h.nodes.get('jointDirectionStart').disabled,true);assert.equal(h.posts().length,1);
});

test('file origin cannot request a connection check even with an injected eligible closure snapshot',async()=>{
  const h=harness('file:','#joint-steps');h.set(disconnectedFixture());await h.nodes.get('recoverFeedback').onclick();assert.equal(h.calls.length,0);
});

function motionFixture(mode='off', nativeGeneration=3) {
  const s=jointFixture('all-complete');s.session.status='closed';s.session.error=null;s.feedback=null;
  s.motion={phase:'ready',host:'X5',owns_connection:true,feedback_age_ms:0,error:null,deployment:{archive_sha256:'bundle-one',calibration_sha256:'latest-saved-hash',model_hashes:{walk:'walk-hash',getup:'getup-hash',pick:'pick-hash',roulade:'roll-hash'}},hardware:{generation:nativeGeneration,connected:true,age_ms:5,mode,calibrated:true,hold_ready:false,start_blocked_reason:null,release_confirmed:mode==='off',online_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],fault_ids:[],torque_ids:mode==='off'?[]:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],rawdata:s.calibration.joints.map(j=>({id:j.id,position:j.zero})),last_action:null,rates:{state_hz:50,control_hz:mode==='off'?0:50},error:null}};
  s.motion.hardware.target_validation_protocol=1;
  s.motion.hardware.policy_metadata_protocol=1;
  s.motion.hardware.stand_target_deg=[0,-.087,-.349,-.005,.349,0,.087,.349,.005,-.349,.349,.349,0,0,0].map(v=>v*180/Math.PI);
  s.motion.hardware.command={forward:0,side:0,yaw:0};
  s.motion.deployment.started_monotonic=1;
  Object.assign(s.motion.hardware,{imu_ok:true,imu_age_ms:5,imu:{gravity:[0,0,-1]}});
  return s;
}
function motionRecheckFixture(generation=101){
  const s=recheckFixture(generation);
  s.session.recovery_recheck={category:'servo',message:'舵机电压故障尚需复查'};
  s.motion={phase:'idle',owns_connection:false,hardware:null,deployment:null,error:null};
  return s;
}
test('motion fault offers a powered feedback recheck on the same page and never deploys or moves automatically',async()=>{
  const h=harness('http:','#motion-models'),held=motionRecheckFixture();h.set(held);
  assert.equal(h.nodes.get('motionRecheck').disabled,false);assert.equal(h.nodes.get('motionConnect').disabled,true);
  assert(h.nodes.get('motionRecoveryStatus').textContent.includes('电压报警'));
  assert(h.nodes.get('motionRecoveryStatus').textContent.includes('不会开启扭矩'));
  await h.nodes.get('motionConnect').onclick();assert.equal(h.posts().length,0);
  const pending=deferred();h.post((url,options)=>{
    assert.equal(url,'/api/recheck-feedback');assert.equal(options.headers['X-CSRF-Token'],'token-101');
    assert.deepEqual(JSON.parse(options.body),{generation:101,purpose:'joints'});return pending.promise;
  });
  const first=h.nodes.get('motionRecheck').onclick();await h.nodes.get('motionRecheck').onclick();
  assert.equal(h.posts().length,1);assert.equal(h.nodes.get('motionRecheck').disabled,true);
  const checking=motionRecheckFixture(102);checking.session.status='waiting';checking.session.recovery_pending=true;checking.session.recheck_allowed=false;checking.session.feedback_recheck={purpose:'joints',phase:'checking'};
  pending.resolve(response(checking));await first;
  assert.equal(h.nodes.get('motionConnect').disabled,true);assert.equal(h.nodes.get('motionStand').disabled,true);
  const passed=jointFixture('all-complete',102);passed.session.recovery_recheck=null;passed.session.feedback_recheck={purpose:'joints',phase:'passed'};passed.motion=held.motion;
  h.set(passed);assert(h.nodes.get('motionRecoveryStatus').textContent.includes('复查通过'));
  assert.equal(h.nodes.get('motionConnect').disabled,false);assert.equal(h.posts().length,1);
  assert.equal(h.nodes.get('motionStand').disabled,true);
  h.post(async()=>response(passed));await h.nodes.get('motionConnect').onclick();
  assert.equal(h.posts().length,2);assert.equal(h.posts()[1].url,'/api/motion');
  assert.equal(JSON.parse(h.posts()[1].options.body).action,'connect');
});
test('motion recheck retains the voltage fault and exposes refusal or timeout without retrying',async()=>{
  const h=harness('http:','#motion-models');h.set(motionRecheckFixture());
  h.post(async()=>response({error:'15 个舵机仍有电压报警'},409));await h.nodes.get('motionRecheck').onclick();
  assert(h.nodes.get('motionRecoveryStatus').textContent.includes('仍有电压报警'));
  assert.equal(h.nodes.get('motionConnect').disabled,true);assert.equal(h.posts().length,1);
  h.set(motionRecheckFixture());h.post(async()=>{const e=new Error('timeout');e.name='AbortError';throw e;});
  await h.nodes.get('motionRecheck').onclick();assert(h.nodes.get('motionRecoveryStatus').textContent.includes('不会自动重试'));
  assert.equal(h.posts().length,2);assert.equal(h.nodes.get('motionStand').disabled,true);
});
test('motion page cannot override a trust or closure restriction even with contradictory recheck permission',async()=>{
  const h=harness('http:','#motion-models');
  for(const reason of ['板卡启动身份待核对','新的 SD 卡读取错误','旧串口关闭尚无证明']){
    const s=motionRecheckFixture();s.session.recheck_blocked_reason=reason;h.set(s);
    assert.equal(h.nodes.get('motionRecheck').disabled,true);assert.equal(h.nodes.get('motionConnect').disabled,true);
    assert(h.nodes.get('motionRecoveryStatus').textContent.includes(reason));await h.nodes.get('motionRecheck').onclick();await h.nodes.get('motionConnect').onclick();
  }
  assert.equal(h.posts().length,0);
});
test('motion page uses a metadata connection check when closure needs reconciliation',async()=>{
  const h=harness('http:','#motion-models'),s=disconnectedFixture();s.motion=motionRecheckFixture().motion;h.set(s);
  h.post(async(url,options)=>{assert.equal(url,'/api/check-connection');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose:'joints'});return response(s);});
  await h.nodes.get('motionRecheck').onclick();assert.equal(h.posts().length,1);assert.equal(h.nodes.get('motionStand').disabled,true);
});
test('failed motion metadata check remains explicitly retryable without clearing its trust hold or starting motors',async()=>{
  const h=harness('http:','#motion-models'),s=disconnectedFixture();s.motion=motionRecheckFixture().motion;
  s.session.recovery_recheck={category:'trust',message:'板端状态尚未核对',technical_reason:'New kernel fault'};
  Object.assign(s.session.connection_check,{phase:'failed',error:'X5 记录了一次 USB 断开',allowed:true,blocked_reason:null});h.set(s);
  assert.equal(h.nodes.get('motionRecheck').disabled,false);assert.equal(h.nodes.get('motionConnect').disabled,true);
  h.post(async(url,options)=>{assert.equal(url,'/api/check-connection');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose:'joints'});return response(s);});
  await h.nodes.get('motionRecheck').onclick();await h.nodes.get('motionRecheck').onclick();
  assert.equal(h.posts().length,2);assert.equal(h.nodes.get('motionConnect').disabled,true);assert.equal(h.nodes.get('motionStand').disabled,true);
});
test('motion page reuses the scoped maintenance review rather than clearing a maintenance hold locally',async()=>{
  const h=harness('http:','#motion-models'),s=maintenanceFixture(false);s.motion=motionRecheckFixture().motion;
  Object.assign(s.maintenance,{fault_hold:true,fault_review_allowed:true,fault_review_blocked_reason:null,error:'舵机电压报警'});h.set(s);
  assert.equal(h.nodes.get('motionRecheck').disabled,false);assert.equal(h.nodes.get('motionConnect').disabled,true);
  const checking=clone(s);checking.maintenance.busy=true;
  h.post(async(url,options)=>{assert.equal(url,'/api/servo-maintenance');assert.deepEqual(JSON.parse(options.body),{action:'review',generation:101});return response(checking);});
  await h.nodes.get('motionRecheck').onclick();assert.equal(h.posts().length,1);assert.equal(h.nodes.get('motionRecheck').disabled,true);
  const passed=clone(s);passed.maintenance.fault_hold=false;passed.maintenance.error=null;h.set(passed);
  assert(h.nodes.get('motionRecoveryStatus').textContent.includes('复查通过'));assert.equal(h.nodes.get('motionConnect').disabled,false);
  assert.equal(h.posts().length,1);
});
test('motion recheck never competes with an existing UART owner and file pages cannot submit it',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();s.motion.hardware.fault_ids=[10];h.set(s);
  assert.equal(h.nodes.get('motionRecheck').disabled,true);assert(h.nodes.get('motionRecoveryStatus').textContent.includes('停止并断开运控'));
  await h.nodes.get('motionRecheck').onclick();assert.equal(h.posts().length,0);assert.equal(h.nodes.get('motionStop').disabled,false);
  const file=harness('file:','#motion-models');file.set(motionRecheckFixture());await file.nodes.get('motionRecheck').onclick();assert.equal(file.calls.length,0);
});
test('claimed joint recheck success with unsafe fresh values cannot unlock motion deployment',async()=>{
  const h=harness('http:','#motion-models');
  for(const change of [{online_mask:0},{fault_mask:1},{torque_mask:1},{armed:true},{arm_cookie:1}]){
    const s=jointFixture('all-complete');s.session.feedback_recheck={purpose:'joints',phase:'passed'};s.motion=motionRecheckFixture().motion;Object.assign(s.feedback,change);h.set(s);
    assert.equal(h.nodes.get('motionConnect').disabled,true);assert(!h.nodes.get('motionRecoveryStatus').textContent.startsWith('复查通过'));
    await h.nodes.get('motionConnect').onclick();
  }
  assert.equal(h.posts().length,0);
});
test('motion deep link shows unloaded state and navigation never deploys or starts motors',async()=>{
  const h=harness('http:','#motion-models');h.set(fixture());
  assert.equal(h.nodes.get('motion-models').hidden,false);assert.equal(h.nodes.get('motionHost').textContent,'尚未部署');
  assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.nodes.get('motionWalk').disabled,true);assert.equal(h.posts().length,0);
  h.navigate('#imu-steps');h.navigate('#motion-models');assert.equal(h.posts().length,0);
});
test('loaded models show X5 actual calibration and standing requires support and live feedback',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();h.set(s);
  assert.equal(h.nodes.get('motionHost').textContent,'X5 · CPU');assert.equal(h.nodes.get('motionModelsCount').textContent,'4 / 4');
  assert.equal(h.nodes.get('motionCalibration').textContent,'15 / 15 + IMU');assert(h.nodes.get('motionDeploymentDetail').textContent.includes('latest-saved-hash'));
  assert.equal(h.nodes.get('motionStand').disabled,true);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');assert.equal(h.nodes.get('motionStand').disabled,false);
  s.motion.hardware.start_blocked_reason='ID 12 实际位置超出记录范围';h.set(s);assert.equal(h.nodes.get('motionStand').disabled,true);assert(h.nodes.get('motionStatusMessage').textContent.includes('ID 12'));
  s.motion.hardware.start_blocked_reason=null;h.set(s);h.age(501);assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.nodes.get('motionSupportConfirmed').checked,true);assert.equal(h.nodes.get('motionStop').disabled,false);
});
test('support and behavior confirmation survive timer gaps and recover without a second checkbox click',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();h.set(s);await h.settleOld(s);
  h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionSupportConfirmed').onchange();
  h.nodes.get('motionBehaviorConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').onchange();
  for(let cycle=0;cycle<4;cycle++){
    h.age(600);h.intervals[1]();
    assert.equal(h.nodes.get('motionSupportConfirmed').checked,true);
    assert.equal(h.nodes.get('motionBehaviorConfirmed').checked,true);
    assert.equal(h.nodes.get('motionSupportConfirmed').disabled,false);
    assert.equal(h.nodes.get('motionStand').disabled,true);
    await h.nodes.get('motionStand').onclick();assert.equal(h.posts().length,0,'stale callbacks cannot start motion');
    h.age(400);await h.intervals[0]();
    assert.equal(h.nodes.get('motionStand').disabled,false,'fresh poll restores eligibility without rechecking support');
  }
  h.eval("fetchError='页面状态读取超时';render()");
  assert.equal(h.nodes.get('motionSupportConfirmed').checked,true);assert.equal(h.nodes.get('motionStand').disabled,true);
  await h.nodes.get('motionStand').onclick();assert.equal(h.posts().length,0);
  h.set(s);assert.equal(h.nodes.get('motionStand').disabled,false);assert.equal(h.posts().length,0);
});
test('motion confirmation is cleared by real connection, calibration and protective stop changes',()=>{
  const changes=[
    s=>{s.motion.hardware.connected=false;},
    s=>{s.motion.phase='closed';s.motion.owns_connection=false;},
    s=>{s.motion.transport_exit={process_exited:true};},
    s=>{s.motion.connection_sequence=2;},
    s=>{s.session.generation++;},
    s=>{s.motion.deployment.started_monotonic=2;},
    s=>{s.calibration.joints[0].zero++;},
    s=>{s.motion.hardware.error='IMU 反馈超时';}
  ];
  for(const change of changes){
    const h=harness('http:','#motion-models'),s=motionFixture();h.set(s);
    h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;h.eval('renderMotion()');
    const changed=clone(s);change(changed);h.set(changed);
    assert.equal(h.nodes.get('motionSupportConfirmed').checked,false);assert.equal(h.nodes.get('motionBehaviorConfirmed').checked,false);
    assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.posts().length,0);
  }
});
test('standing request is explicit and bound to both page and native generation',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  const next=motionFixture('hold',4);h.post(async(url,options)=>response(next));
  await h.nodes.get('motionStand').onclick();const start=h.posts().filter(p=>JSON.parse(p.options.body).action==='start');
  assert.equal(start.length,1);assert.deepEqual(JSON.parse(start[0].options.body),{action:'start',page_generation:101,generation:3,control_id:h.eval('motionControlId'),mode:'hold',support_confirmed:true});
  assert.equal(start[0].url,'/api/motion');assert.equal(h.nodes.get('motionWalk').disabled,true);
  next.motion.hardware.hold_ready=true;h.set(next);assert.equal(h.nodes.get('motionWalk').disabled,false);
  assert.equal(h.nodes.get('motionMode').textContent,'关节站姿已到位 · 待核对机身姿态');
  assert(h.nodes.get('motionStatusMessage').textContent.includes('不能验证实物'));
  assert(h.nodes.get('motionStatusMessage').textContent.includes('双脚前后是否对齐'));
  h.age(501);assert(h.nodes.get('motionMode').textContent.includes('关节站姿已到位'));assert.equal(h.nodes.get('motionWalk').disabled,true);h.age(4500);assert(!h.nodes.get('motionMode').textContent.includes('关节站姿已到位'));
});
test('standing refusal remains beside buttons across refreshed status and shows original error',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();h.set(s);
  h.nodes.get('motionSupportConfirmed').checked=true;
  h.post(async()=>response({error:'下位机反馈超时（指令 ARM，错误码 4）'},409));
  await h.nodes.get('motionStand').onclick();
  assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,1);
  assert(h.nodes.get('motionActionResult').textContent.includes('错误码 4'));
  const stopped=motionFixture('off',4);
  stopped.motion.last_operation={action:'start',mode:'hold',state:'failed',error:'下位机反馈超时（指令 ARM，错误码 4）'};
  stopped.motion.hardware.inference_ms=.62;
  h.eval("motionError=''");h.set(stopped);
  assert(h.nodes.get('motionActionResult').textContent.includes('进入固定站姿失败'));
  assert(h.nodes.get('motionActionResult').textContent.includes('错误码 4'));
  assert(!h.nodes.get('motionLiveDetail').textContent.includes('推理'));
});
function connectionTimeoutFixture(){
  const s=motionFixture('off',2);s.motion.phase='closed';s.motion.owns_connection=false;s.motion.hardware=null;s.motion.connection_sequence=0;
  s.motion.last_operation={action:'disconnect',state:'complete',recorded_monotonic:10};return s;
}
function completedConnectionFixture(){
  const s=motionFixture('off',2);s.motion.connection_sequence=1;s.motion.hardware.armed=false;
  s.motion.last_operation={action:'connect',state:'complete',error:null,recorded_monotonic:11,after:{connected:true,generation:2}};return s;
}
async function abortConnectionAttempt(h){
  h.set(connectionTimeoutFixture());h.post(async()=>{assert(Array.from(h.timers.values()).some(timer=>timer.ms===20000));const error=new Error('timeout');error.name='AbortError';throw error;});
  await h.nodes.get('motionConnect').onclick();assert.equal(h.posts().length,1);assert(h.eval('motionError').includes('连接请求应答超时'));
}
test('a lost connect reply reconciles only after the matching new connection completes and all torque-off feedback is fresh',async()=>{
  const h=harness('http:','#motion-models');await abortConnectionAttempt(h);
  const next=completedConnectionFixture();h.age(1000);h.set(next);
  assert.equal(h.eval('motionError'),'');assert.equal(h.eval('motionRequestTimeout'),null);
  assert.equal(h.nodes.get('sessionAlert').hidden,true);assert.equal(h.nodes.get('motionConnect').textContent,'运控已连接');
  assert.equal(h.eval('ownsMotionControl()'),false,'reconciling a connection never claims or starts motion');
  for(let cycle=0;cycle<4;cycle++){h.age(800);h.set(next);}
  assert.equal(h.posts().length,1,'status reconciliation must not replay the POST');
});
test('old, unrelated, stale or unsafe telemetry never clears a connection timeout',async()=>{
  const mutations=[
    s=>{s.motion.connection_sequence=0;},s=>{s.motion.connection_sequence=2;},
    s=>{s.motion.last_operation.recorded_monotonic=10;},s=>{delete s.motion.last_operation.recorded_monotonic;},
    s=>{s.motion.last_operation.action='start';},s=>{s.motion.last_operation.state='pending';},
    s=>{s.motion.last_operation.after.generation=1;},s=>{s.motion.hardware.release_confirmed=false;},
    s=>{s.motion.hardware.age_ms=110;},s=>{s.motion.hardware.armed=true;},
    s=>{s.motion.hardware.online_ids=[];},s=>{s.motion.hardware.torque_ids=[10];},
    s=>{s.motion.hardware.mode='hold';},s=>{s.motion.error='真实连接异常';}
  ];
  for(const mutate of mutations){const h=harness('http:','#motion-models');await abortConnectionAttempt(h);const next=completedConnectionFixture();mutate(next);h.set(next);assert(h.eval('motionError').includes('连接请求应答超时'));assert.equal(h.posts().length,1);}
});
test('a matching failed connection replaces the uncertain timeout with its actual refusal',async()=>{
  const h=harness('http:','#motion-models');await abortConnectionAttempt(h);const next=completedConnectionFixture();
  next.motion.phase='fault';next.motion.owns_connection=false;next.motion.hardware=null;
  next.motion.last_operation={action:'connect',state:'failed',error:'ESP32 USB serial device not found',recorded_monotonic:11};h.set(next);
  assert.equal(h.eval('motionError'),'ESP32 USB serial device not found');assert.equal(h.eval('motionRequestTimeout'),null);
  assert(h.nodes.get('motionActionResult').textContent.includes('检查 COM 线'));assert.equal(h.posts().length,1);
});
test('an action timeout keeps its five-second limit and cannot be cleared by a later connect success',async()=>{
  const h=harness('http:','#motion-models');h.set(motionFixture());h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  h.post(async()=>{assert(Array.from(h.timers.values()).some(timer=>timer.ms===5000));const error=new Error('timeout');error.name='AbortError';throw error;});
  await h.nodes.get('motionStand').onclick();assert(h.eval('motionError').includes('运控请求超时'));h.set(completedConnectionFixture());
  assert(h.eval('motionError').includes('运控请求超时'));assert.equal(h.eval('ownsMotionControl()'),false);assert.equal(h.posts().length,1);
});

test('missing ESP32 connection is actionable and never enables movement or auto reconnects',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();
  s.motion.phase='fault';s.motion.owns_connection=false;s.motion.hardware=null;
  s.motion.error='ESP32 USB serial device not found; check its COM cable and X5 USB connection';
  h.set(s);
  assert(h.nodes.get('motionStatusMessage').textContent.includes('检查 COM 线'));
  assert.equal(h.nodes.get('motionConnect').disabled,false);
  assert.equal(h.nodes.get('motionStand').disabled,true);
  assert.equal(h.posts().length,0);
});
test('ambiguous USB serial identity stays visible and requires a new explicit connection',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();
  s.motion.phase='fault';s.motion.owns_connection=false;s.motion.hardware=null;
  s.motion.error='Multiple matching CH340 USB serial devices; cannot identify ESP32 uniquely';
  h.set(s);
  assert(h.nodes.get('motionActionResult').textContent.includes('多个同型号 USB 串口'));
  assert.equal(h.nodes.get('motionStand').disabled,true);
  assert.equal(h.posts().length,0);
});
test('exited motion pipe disables stop and exposes an explicit metadata disconnect check',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();
  s.motion.phase='fault';s.motion.owns_connection=true;s.motion.hardware=null;
  s.motion.error='Maintenance SSH output closed';
  s.motion.transport_exit={process_exited:true,ssh_exit_code:255,remote_closure_inferred:false};
  h.set(s);
  assert.equal(h.nodes.get('motionStop').disabled,true);
  assert(h.nodes.get('motionStop').textContent.includes('连接已断开'));
  assert.equal(h.nodes.get('motionDisconnect').disabled,false);
  assert.equal(h.nodes.get('motionDisconnect').textContent,'检查断开并释放连接');
  assert(h.nodes.get('motionStatusMessage').textContent.includes('尚未收到完整关闭应答'));
  assert.equal(h.nodes.get('motionConnect').disabled,true);
  assert.equal(h.nodes.get('motionStand').disabled,true);
  assert.equal(h.posts().length,0);
});
test('calibration request locks standing button while stop remains available',()=>{
  const h=harness('http:','#motion-models');h.set(motionFixture());
  h.nodes.get('motionSupportConfirmed').checked=true;
  h.eval('requestBusy=true;renderMotion()');
  assert.equal(h.nodes.get('motionStand').disabled,true);
  assert.equal(h.nodes.get('motionStop').disabled,false);
});
test('motion navigation sends STOP and old native status cannot restore moving state',async()=>{
  const h=harness('http:','#device-status'),running=motionFixture('walk',4);h.set(running);
  h.ownMotion();
  h.post(async()=>response(motionFixture('off',5)));h.navigate('#motion-models');await tick();h.navigate('#device-status');await tick();
  assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='stop').length,1);
  h.set(running);assert.equal(h.eval('snapshot.motion.hardware.generation'),5);assert.equal(h.eval('snapshot.motion.hardware.mode'),'off');
});
test('press and release queues zero command even while first command is pending',async()=>{
  const h=harness('http:','#motion-models'),running=motionFixture('walk',4);h.set(running);const first=deferred();let commands=0;
  h.ownMotion();await tick();
  h.post(async(url,options)=>{if(JSON.parse(options.body).action==='command'&&++commands===1)return first.promise;return response(running);});
  const started=h.nodes.get('motionForward').onpointerdown({pointerId:1,preventDefault(){}});h.nodes.get('motionForward').onpointerup();first.resolve(response(running));await started;
  const bodies=h.posts().map(p=>JSON.parse(p.options.body)).filter(b=>b.action==='command');assert.equal(bodies.length,2);assert.equal(bodies[0].forward,.1);assert.equal(bodies[1].forward,0);
});
test('file page and expired UART feedback never start motion or heartbeat',async()=>{
  const h=harness('file:','#motion-models');h.set(motionFixture());await h.nodes.get('motionStand').onclick();await h.nodes.get('motionConnect').onclick();assert.equal(h.calls.length,0);
  const live=harness('http:','#motion-models'),s=motionFixture('walk');s.motion.hardware.age_ms=200;live.set(s);assert.equal(live.nodes.get('motionWalk').disabled,true);assert.equal(live.nodes.get('motionForward').disabled,true);assert.equal(live.posts().length,0);
});

function behaviorFixture(mode='hold', generation=3) {
  const s=motionFixture(mode,generation);Object.assign(s.motion.hardware,{behavior_protocol:1,hold_ready:mode==='hold',active_model:mode==='hold'?'walk':mode,model_start_blocked_reasons:{walk:null,getup:null,pick:null,roulade:null},behavior_seconds_remaining:1.5});return s;
}
test('completed stop keeps a labeled historical release proof through polling gaps and never invents unconfirmed release',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('off',5);
  s.motion.last_operation={action:'stop',state:'complete',error:null};h.set(s);
  assert.equal(h.nodes.get('motionActionResult').hidden,false);
  assert(h.nodes.get('motionActionResult').textContent.includes('回读确认 15 个舵机扭矩全部关闭'));
  assert(h.nodes.get('motionActionResult').textContent.includes('不会主动把关节摆回'));
  s.motion.hardware.release_confirmed=false;h.set(s);assert.equal(h.nodes.get('motionActionResult').hidden,true);
  s.motion.hardware.release_confirmed=true;h.set(s);const recorded=h.nodes.get('motionActionResult').textContent;h.age(501);assert.equal(h.nodes.get('motionActionResult').hidden,false);assert.equal(h.nodes.get('motionActionResult').textContent,recorded);assert(recorded.includes('最近回读'));assert(recorded.includes('反馈时效见下方'));assert(h.nodes.get('motionFeedbackAge').textContent.includes('0.5 秒'));
});
test('manual stop, hidden dynamic motion, page closure and workspace exit still stop without restarting',async()=>{
  for(const source of ['button','page-hidden','pagehide','workspace-leave']){
    const h=harness('http:','#motion-models');h.set(motionFixture(source==='page-hidden'?'walk':'hold'));h.ownMotion();await tick();
    h.post(async()=>response(motionFixture('off',4)));
    if(source==='button')await h.nodes.get('motionStop').onclick();
    else if(source==='workspace-leave')h.navigate('#joint-steps');
    else if(source==='page-hidden'){h.eval('document.hidden=true');h.emit('visibilitychange');}
    else h.emit('pagehide');
    await tick();
    const requests=h.posts().map(p=>JSON.parse(p.options.body));
    const stops=requests.filter(p=>p.action==='stop');
    assert.equal(stops.length,1,source);assert.equal(stops[0].stop_source,source);
    assert(!requests.some(p=>p.action==='start'),source);
  }
});
test('owned standing survives focus loss and keeps heartbeats while hidden',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('hold');h.set(s);h.ownMotion();await tick();
  h.emit('blur');h.eval('document.hidden=true');h.emit('visibilitychange');await tick();
  for(let i=0;i<3;i++){h.age(1000);h.set(s);await tick();}
  const requests=h.posts().map(p=>JSON.parse(p.options.body));
  assert.equal(requests.filter(p=>p.action==='stop'||p.action==='start').length,0);
  const hidden=requests.filter(p=>p.action==='heartbeat'&&p.background===true);
  assert(hidden.length>=3);assert(hidden.every(p=>p.generation===3&&p.page_generation===101));
  h.eval('document.hidden=false');h.emit('visibilitychange');await tick();
  assert.equal(JSON.parse(h.posts().at(-1).options.body).background,false);
});
test('hiding during a pending foreground heartbeat immediately hands over to background hold heartbeat',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('hold');h.set(s);
  const pending=deferred();let count=0;
  h.post(async()=>++count===1?pending.promise:response(s));h.ownMotion();
  h.eval('document.hidden=true');h.emit('visibilitychange');pending.resolve(response(s));await tick();await tick();
  const heartbeats=h.posts().map(p=>JSON.parse(p.options.body)).filter(p=>p.action==='heartbeat');
  assert.deepEqual(heartbeats.map(p=>p.background),[false,true]);
  assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='stop').length,0);
});
test('passive and superseded motion pages cannot send heartbeats or automatic stops but manual stop still works',async()=>{
  for(const superseded of [false,true]){
    const h=harness('http:','#motion-models');h.set(motionFixture('hold'));
    if(superseded){h.ownMotion();await tick();h.set(motionFixture('hold',4));}
    const before=h.posts().length;
    h.emit('blur');h.eval('document.hidden=true');h.emit('visibilitychange');h.emit('pagehide');h.navigate('#joint-steps');await h.eval('motionHeartbeat()');await tick();
    assert.equal(h.posts().length,before);
    await h.nodes.get('motionStop').onclick();
    assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='stop').length,1);
  }
});
test('focus loss clears a held walking command without releasing torque',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('walk');h.set(s);h.ownMotion();await tick();
  h.emit('blur');await tick();
  const requests=h.posts().map(p=>JSON.parse(p.options.body));
  const commands=requests.filter(p=>p.action==='command');
  assert.deepEqual(commands.map(p=>[p.forward,p.side,p.yaw]),[[0,0,0]]);
  assert(!requests.some(p=>p.action==='stop'||p.action==='start'));
});
test('first active stop cause remains visible after a redundant browser stop',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('off',5);
  s.motion.last_operation={action:'stop',state:'complete',stop_source:'page-hidden'};
  s.motion.hardware.last_stop_diagnostic={sequence:1,source:'window-blur',reason:null};h.set(s);
  assert(h.nodes.get('motionActionResult').textContent.includes('原因：运控窗口失去焦点'));
  assert(!h.nodes.get('motionActionResult').textContent.includes('页面被隐藏'));
  assert(html.includes('id="motionFocusHint">固定站姿由 X5 独立保持'));
  assert.equal(h.posts().length,0);
});
test('disconnected control process has a visible closure route and never looks connected or stop-confirmed',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('off',5);
  s.motion.hardware.connected=false;s.motion.hardware.release_confirmed=false;s.motion.hardware.error=null;
  s.motion.last_operation={action:'stop',state:'complete',error:null};h.set(s);
  assert(h.nodes.get('motionStatusMessage').textContent.includes('停止并断开运控'));
  assert.equal(h.nodes.get('motionConnect').textContent,'连接待关闭确认');
  assert.equal(h.nodes.get('motionConnect').disabled,true);
  assert.equal(h.nodes.get('motionDisconnect').disabled,false);
  assert(!h.nodes.get('motionActionResult').textContent.includes('回读确认 15 个舵机扭矩全部关闭'));
  s.motion.phase='closed';s.motion.owns_connection=false;s.motion.closed_proof={serial_closed:true,ssh_exit_code:20};
  s.motion.error='ID 11 目标超过已记录的机械范围';h.set(s);
  assert.equal(h.nodes.get('motionConnect').disabled,false);
  assert.equal(h.nodes.get('motionConnect').textContent,'连接并载入模型');
  assert(h.nodes.get('motionStatusMessage').textContent.includes('ID 11'));
  assert.equal(h.nodes.get('motionStand').disabled,true);
});
test('late fresh torque-off STATE confirms the safe state after STOP reply timeout without replaying the request',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('off',5);
  s.motion.last_operation={action:'stop',state:'failed',error:'运控应答超时，指令不会重发'};
  h.eval("motionError='运控请求超时；不会重发'");h.set(s);
  assert(h.nodes.get('motionActionResult').textContent.includes('随后真实反馈已确认停止'));
  assert.equal(h.nodes.get('motionActionResult').classList.contains('error'),false);
  assert.equal(h.eval('snapshot.motion.last_operation.state'),'failed');assert.equal(h.posts().length,0);
  s.motion.hardware.release_confirmed=false;h.set(s);
  assert.equal(h.nodes.get('motionActionResult').classList.contains('error'),true);
  assert(!h.nodes.get('motionActionResult').textContent.includes('已确认停止'));
  s.motion.hardware.release_confirmed=true;h.set(s);h.age(501);
  assert(h.nodes.get('motionActionResult').textContent.includes('反馈时效见下方'));
  assert(h.nodes.get('motionActionResult').textContent.includes('随后真实反馈已确认停止'));
});
test('all behavior buttons require live protocol and per-model readiness plus physical confirmations',async()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture();h.set(s);
  assert.equal(h.nodes.get('motionPick').disabled,true);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionPick').disabled,false);assert.equal(h.nodes.get('motionGetup').disabled,true);assert.equal(h.nodes.get('motionRoulade').disabled,true);
  await h.nodes.get('motionRoulade').onclick();assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,0);
  h.nodes.get('motionBehaviorConfirmed').checked=true;h.eval('renderMotion()');assert.equal(h.nodes.get('motionRoulade').disabled,false);
  s.motion.hardware.model_start_blocked_reasons.roulade='先进入站姿';h.set(s);assert.equal(h.nodes.get('motionRoulade').disabled,true);
  s.motion.hardware.model_start_blocked_reasons.roulade=null;s.motion.hardware.behavior_protocol=undefined;h.set(s);assert.equal(h.nodes.get('motionPick').disabled,true);
  s.motion.hardware.behavior_protocol=1;h.set(s);h.age(501);assert.equal(h.nodes.get('motionPick').disabled,true);assert.equal(h.nodes.get('motionBehaviorConfirmed').checked,true);
});
test('each new explicit model button sends corresponding mode and both generation pins once',async()=>{
  for(const [id,mode] of [['motionGetup','getup'],['motionPick','pick'],['motionRoulade','roulade']]){
    const h=harness('http:','#motion-models');h.set(behaviorFixture());
    h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;h.eval('renderMotion()');
    h.post(async()=>response(behaviorFixture(mode,4)));await h.nodes.get(id).onclick();
    const requests=h.posts().map(p=>JSON.parse(p.options.body)).filter(b=>b.action==='start');
    assert.deepEqual(requests,[{action:'start',page_generation:101,generation:3,control_id:h.eval('motionControlId'),mode,support_confirmed:true,behavior_confirmed:true}]);
    assert(h.nodes.get('motionMode').textContent.includes({getup:'起身',pick:'俯身',roulade:'翻滚'}[mode]));
    assert(h.nodes.get('motionBehaviorHint').textContent.includes('剩余约 1.5 秒'));
  }
});
test('preview selector requests the selected model without support or motor action',async()=>{
  for(const name of ['walk','getup','pick','roulade']){
    const h=harness('http:','#motion-models');h.set(behaviorFixture('off'));h.nodes.get('motionPreviewModel').value=name;
    const next=behaviorFixture('shadow',4);next.motion.hardware.active_model=name;h.post(async()=>response(next));
    await h.nodes.get('motionShadow').onclick();const bodies=h.posts().map(p=>JSON.parse(p.options.body)).filter(b=>b.action==='start');
    assert.deepEqual(bodies,[{action:'start',page_generation:101,generation:3,control_id:h.eval('motionControlId'),mode:'shadow',preview_model:name}]);
    assert.equal(h.nodes.get('motionGetup').disabled,true);
  }
});
test('leaving any new model mode sends STOP and never restarts after stale status',async()=>{
  for(const mode of ['getup','pick','roulade']){
    const h=harness('http:','#device-status'),running=behaviorFixture(mode,4);h.set(running);
    h.ownMotion();
    h.post(async()=>response(behaviorFixture('off',5)));h.navigate('#motion-models');await tick();h.navigate('#device-status');await tick();
    assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='stop').length,1);
    h.set(running);assert.equal(h.eval('snapshot.motion.hardware.mode'),'off');
    assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,0);
  }
});

function recordedGetupRangeFaultFixture() {
  // Minimal replay of run 15 motion-5ab08a2b9bd47883: START 1 -> 2 succeeded,
  // then ID 30's model target failed the saved range and CLOSED reported 8/off.
  const s=behaviorFixture('off',8),reason='ID 30 目标超过已记录的机械范围';
  s.motion.phase='closed';s.motion.owns_connection=false;s.motion.error=reason;
  Object.assign(s.motion.hardware,{connected:false,release_confirmed:false,error:null,
    last_stop_diagnostic:{sequence:1,reason,source:null,before:{generation:2,mode:'getup',connected:true}}});
  s.motion.closed_proof={serial_closed:true,release_confirmed:true,ssh_exit_code:20};
  s.motion.transport_exit={process_exited:true,ssh_exit_code:20,original_error:reason,closed_record_received:true,remote_closure_inferred:false};
  s.motion.last_operation={action:'start',mode:'getup',generation:1,state:'complete',error:null,
    before:{generation:1,mode:'off',connected:true},after:{generation:2,mode:'getup',connected:true,error:null}};
  return s;
}

function recordedHoldTiltStopFixture(){
  // Run 15 motion-c2e09dddb6ede620, last-stop sequence 2. HOLD generation 4
  // still had all 15 torques and 319.2ms heartbeat time before the 40deg gate.
  const s=behaviorFixture('off',5),reason='机身倾角超过 40°，请先支撑并扶正';
  s.motion.hardware.error=reason;
  s.motion.hardware.last_stop_diagnostic={sequence:2,reason,source:null,before:{generation:4,mode:'hold',connected:true,armed:true,torque_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],heartbeat_remaining_ms:319.2,error:null}};
  s.motion.last_operation={action:'start',mode:'hold',generation:3,state:'complete',error:null,after:{generation:4,mode:'hold',connected:true,error:null}};
  return s;
}

test('recorded ready off tilt protection outranks an old heartbeat state-changed error',()=>{
  const h=harness('http:','#motion-models');h.set(behaviorFixture('hold',4));
  h.eval("motionError='Motion state changed; refresh before starting'");h.set(recordedHoldTiltStopFixture());
  for(const id of ['motionActionResult','motionStatusMessage','sessionAlert']){
    const text=h.nodes.get(id).textContent;
    assert(text.includes('保护停止：机身倾角超过 40°，请先支撑并扶正'),id);
    assert(text.includes('展开「查看停止原因与反馈详情」'),id);
    assert(h.nodes.get('motionTorqueState').textContent.includes('最近回读：15 个扭矩已全关'));
    assert(text.includes('请继续托稳机身'),id);
    assert(!text.includes('旧请求')&&!text.includes('Motion state')&&!text.includes('失败：'),id);
  }
  assert.equal(h.nodes.get('motionMode').textContent,'停止 · 扭矩关闭');
  assert(h.nodes.get('motionLiveDetail').textContent.includes('舵机在线 15/15；故障 0；扭矩开启 0/15'));
  assert.equal(h.nodes.get('motionStop').disabled,false);assert.equal(h.nodes.get('motionDisconnect').disabled,false);
  assert.equal(h.posts().length,0);
});

test('a current-generation stop diagnostic outranks an old rejected walk operation',()=>{
  const h=harness('http:','#motion-models'),s=recordedHoldTiltStopFixture();
  s.motion.hardware.error=null;
  s.motion.last_operation={action:'start',mode:'walk',state:'failed',error:'Motion state changed; refresh before starting'};
  h.set(s);
  assert(h.nodes.get('motionActionResult').textContent.includes('保护停止：机身倾角超过 40°'));
  assert(!h.nodes.get('motionActionResult').textContent.includes('启动站立平衡失败'));
  assert(!h.nodes.get('motionStatusMessage').textContent.includes('旧请求'));
  assert.equal(h.posts().length,0);
});

test('a new start refusal in the current stopped generation remains visible beside the protection reason',()=>{
  const reason='动作开始前机身需要保持水平（倾角小于 15°）';
  for(const generation of [undefined,4,'5',5]){
    const h=harness('http:','#motion-models'),s=recordedHoldTiltStopFixture();
    s.motion.last_operation={action:'start',mode:'walk',generation,state:'failed',error:reason};h.set(s);
    const text=h.nodes.get('motionActionResult').textContent;
    assert(text.includes('保护停止：机身倾角超过 40°'));
    assert(text.includes('展开「查看停止原因与反馈详情」'));
    assert(h.nodes.get('motionTorqueState').textContent.includes('最近回读：15 个扭矩已全关'));
    assert.equal(text.includes('本次启动站立平衡未执行：'+reason),generation===5);
    assert.equal(h.nodes.get('motionStatusMessage').textContent,text);
    assert(!text.includes('启动站立平衡失败：'));assert.equal(h.posts().length,0);
  }
});

test('protective stop details open beside the banner and retain actual torque and feedback age without requests',()=>{
  const h=harness('http:','#motion-models'),s=recordedHoldTiltStopFixture();
  s.motion.hardware.release_confirmed=false;h.set(s);
  assert.equal(h.nodes.get('motionStopDetails').hidden,false);
  assert(h.nodes.get('motionStopReason').textContent.includes('机身倾角超过 40°'));
  assert(h.nodes.get('motionStopTorqueState').textContent.includes('扭矩关闭尚未确认'));
  assert.equal(h.nodes.get('connectionStatusLink').disabled,false);
  h.nodes.get('connectionStatusLink').onclick();
  assert.equal(h.nodes.get('motionStopDetails').open,true);
  assert.equal(h.nodes.get('motionStopDetailsSummary').focused,true);
  assert.equal(h.eval('workspaceView'),'motion-models');
  s.motion.hardware.release_confirmed=true;h.set(s);h.age(5500);
  assert(h.nodes.get('motionStopTorqueState').textContent.includes('最近回读：15 个扭矩已全关'));
  assert(h.nodes.get('motionStopFeedbackAge').textContent.includes('5.5 秒'));
  assert(h.nodes.get('motionStopFeedbackState').textContent.includes('反馈延迟'));
  assert.equal(h.nodes.get('motionStopDetails').open,true);
  h.navigate('#device-status');assert.equal(h.nodes.get('motionStopDetails').hidden,true);
  h.navigate('#motion-models');h.set(behaviorFixture('hold',6));
  assert.equal(h.nodes.get('motionStopDetails').hidden,true);
  assert.equal(h.posts().length,0);
});

test('protective reason stays stable while a separate row labels historical torque proof and feedback age',()=>{
  const h=harness('http:','#motion-models'),s=recordedHoldTiltStopFixture();s.motion.hardware.release_confirmed=false;h.set(s);
  const reason=h.nodes.get('motionActionResult').textContent;
  assert(h.nodes.get('motionTorqueState').textContent.includes('扭矩关闭尚未确认'));
  s.motion.hardware.release_confirmed=true;h.set(s);h.age(501);
  assert.equal(h.nodes.get('motionActionResult').textContent,reason);
  assert(reason.includes('保护停止：机身倾角超过 40°'));
  assert(!reason.includes('通信仍正常')&&!reason.includes('已回读确认 15'));
  assert(h.nodes.get('motionTorqueState').textContent.includes('最近回读：15 个扭矩已全关'));
  assert(h.nodes.get('motionFeedbackAge').textContent.includes('0.5 秒'));
  h.age(5000);assert(h.nodes.get('motionFeedbackState').textContent.includes('反馈延迟'));
  assert.equal(h.nodes.get('motionActionResult').textContent,reason);
  assert.equal(h.posts().length,0);
});

test('an earlier protective diagnostic cannot cover a later action or genuine request refusal',()=>{
  const h=harness('http:','#motion-models'),old=recordedHoldTiltStopFixture();h.set(old);
  const later=behaviorFixture('hold',6);later.motion.hardware.last_stop_diagnostic=old.motion.hardware.last_stop_diagnostic;h.set(later);
  assert(!h.nodes.get('motionActionResult').textContent.includes('保护停止'));
  assert(!h.nodes.get('motionStatusMessage').textContent.includes('机身倾角超过 40'));
  const stopped=behaviorFixture('off',7);stopped.motion.hardware.last_stop_diagnostic=old.motion.hardware.last_stop_diagnostic;
  stopped.motion.last_operation={action:'start',mode:'walk',state:'failed',error:'Motion state changed; refresh before starting'};h.set(stopped);
  assert(h.nodes.get('motionActionResult').textContent.includes('启动站立平衡失败：运控状态已变化'));
  assert(h.nodes.get('motionActionResult').textContent.includes('这条旧请求未执行'));
  assert(!h.nodes.get('motionActionResult').textContent.includes('机身倾角超过 40'));
  assert.equal(h.posts().length,0);
});

test('recorded ID30 getup termination outranks an old heartbeat state-changed error',()=>{
  const h=harness('http:','#motion-models');h.set(behaviorFixture('getup',2));
  h.eval("motionError='Motion state changed; refresh before starting'");
  h.set(recordedGetupRangeFaultFixture());
  for(const id of ['motionActionResult','motionStatusMessage','sessionAlert']){
    assert(h.nodes.get(id).textContent.includes('ID 30 目标超过已记录的机械范围'),id);
    assert(!h.nodes.get(id).textContent.includes('旧请求'),id);
    assert(!h.nodes.get(id).textContent.includes('Motion state'),id);
  }
  assert.equal(h.nodes.get('motionConnect').textContent,'连接并载入模型');
  assert.equal(h.nodes.get('motionGetup').disabled,true);assert.equal(h.posts().length,0);
});

test('a real stale start rejection remains visible in Chinese while the connection is ready',()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('off',3);
  s.motion.last_operation={action:'start',mode:'getup',state:'failed',error:'Motion state changed; refresh before starting'};
  h.set(s);
  assert(h.nodes.get('motionActionResult').textContent.includes('起身失败：运控状态已变化'));
  assert(h.nodes.get('motionActionResult').textContent.includes('这条旧请求未执行'));
  assert(!h.nodes.get('motionActionResult').textContent.includes('Motion state'));
  assert.equal(h.posts().length,0);
});

test('hold to getup transition sends one start without an old-generation heartbeat',async()=>{
  const h=harness('http:','#motion-models'),pending=deferred(),next=behaviorFixture('getup',4);
  h.set(behaviorFixture('hold',3));h.eval('claimMotionControl(snapshot)');
  h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;
  h.post((url,options)=>JSON.parse(options.body).action==='start'?pending.promise:Promise.resolve(response({})));
  const start=h.nodes.get('motionGetup').onclick();await h.nodes.get('motionGetup').onclick();
  assert.equal(h.posts().length,1);assert.equal(JSON.parse(h.posts()[0].options.body).action,'start');
  await h.eval('motionHeartbeat()');assert.equal(h.posts().length,1);
  pending.resolve(response(next));await start;await tick();
  const bodies=h.posts().map(call=>JSON.parse(call.options.body));
  assert.equal(bodies.filter(body=>body.action==='start').length,1);
  assert(bodies.filter(body=>body.action==='heartbeat').every(body=>body.generation===4));
});

test('owned dynamic heartbeat continues through display polling delay without authorizing a start',async()=>{
  const h=harness('http:','#motion-models'),running=motionFixture('walk',4);
  h.set(running);h.eval('claimMotionControl(snapshot)');
  h.post(async()=>response({heartbeat_accepted:true,generation:4}));
  h.age(600);await tick();
  assert.equal(h.eval('motionFresh()'),false,'old display feedback must remain stale');
  assert.equal(h.nodes.get('motionForward').disabled,true,'display delay cannot authorize a velocity command');
  assert.equal(h.posts().length,1);
  assert.equal(JSON.parse(h.posts()[0].options.body).action,'heartbeat');
  const delayed=clone(running);delayed.motion.last_hardware=clone(delayed.motion.hardware);delayed.motion.last_hardware.age_ms=800;delayed.motion.hardware=null;
  h.set(delayed);h.age(100);await tick();
  assert.equal(h.eval('ownsMotionControl()'),true,'same connection/generation retains the page lease');
  assert.equal(h.posts().length,2,'missing display projection must not suppress the page heartbeat');
  h.eval("fetchError='display poll failed'");h.age(100);await tick();
  assert.equal(h.posts().length,3,'display HTTP failure does not certify a UART fault');
  assert(h.posts().every(call=>JSON.parse(call.options.body).action==='heartbeat'));
});

test('heartbeat display continuity cannot renew after connection, generation or lifecycle changes',async()=>{
  for(const change of ['generation','connection','closed','transport','off','hidden','workspace']){
    const h=harness('http:','#motion-models'),running=motionFixture('walk',4);
    running.motion.connection_sequence=2;h.set(running);h.eval('claimMotionControl(snapshot)');
    const next=clone(running);
    if(change==='generation')next.motion.hardware.generation++;
    if(change==='connection')next.motion.connection_sequence++;
    if(change==='closed'){next.motion.phase='closed';next.motion.owns_connection=false;}
    if(change==='transport')next.motion.transport_exit={process_exited:true};
    if(change==='off')next.motion.hardware.mode='off';
    if(change==='hidden')h.eval('document.hidden=true');
    if(change==='workspace')h.eval("workspaceView='device-status'");
    h.set(next);
    await h.eval('motionHeartbeat()');
    assert.equal(h.posts().length,0,change+' must not renew a page lease');
  }
});

test('a lost heartbeat response releases its slot before the400ms backend lease',async()=>{
  const h=harness('http:','#motion-models');h.set(motionFixture('walk',4));h.eval('claimMotionControl(snapshot)');
  let requests=0;
  h.post((url,options)=>{
    assert.equal(JSON.parse(options.body).action,'heartbeat');requests++;
    if(requests===1)return new Promise((resolve,reject)=>options.signal.addEventListener('abort',()=>reject(new DOMException('timeout','AbortError')),{once:true}));
    return Promise.resolve(response({heartbeat_accepted:true,generation:4}));
  });
  const pending=h.eval('motionHeartbeat()');
  const timeout=[...h.timers.values()].find(timer=>timer.ms===180);
  assert(timeout,'dynamic heartbeat response budget is180ms, not1000ms');
  h.age(180);timeout.callback();await pending;
  assert.equal(h.eval('motionHeartbeatBusy'),false);
  assert.notEqual(h.eval('motionLastHeartbeat'),Infinity,'transport timeout does not permanently disable renewals');
  assert.equal(h.eval('motionError'),'','no invented terminal motor error from a lost HTTP response');
  h.age(20);await tick();
  assert.equal(requests,2,'the ordinary200ms tick can renew while the400ms backend lease remains valid');
  assert.equal(h.eval('motionHeartbeatTransientError'),'');
  assert(h.posts().every(call=>JSON.parse(call.options.body).action==='heartbeat'),'no START or velocity retry');
});

test('an explicit heartbeat rejection remains terminal after the response-timeout fix',async()=>{
  const h=harness('http:','#motion-models');h.set(motionFixture('walk',4));h.eval('claimMotionControl(snapshot)');
  h.post(async()=>response({error:'control owner changed'},409));
  await h.eval('motionHeartbeat()');
  assert.equal(h.eval('motionLastHeartbeat'),Infinity);
  assert.equal(h.eval('motionError'),'control owner changed');
  h.age(200);await tick();assert.equal(h.posts().length,1);
});

test('transport heartbeat failures cannot renew after OFF or ownership changes',async()=>{
  for(const mode of ['off','generation']){
    const h=harness('http:','#motion-models');h.set(motionFixture('walk',4));h.eval('claimMotionControl(snapshot)');
    let reject;h.post(()=>new Promise((resolve,no)=>{reject=no;}));const pending=h.eval('motionHeartbeat()');
    h.set(motionFixture(mode==='off'?'off':'walk',mode==='off'?5:6));
    reject(new TypeError('lost response'));await pending;h.age(200);await tick();
    assert.equal(h.posts().length,1,mode+' cannot be revived');
  }
});

test('a late old heartbeat failure cannot replace the recorded terminal fault',async()=>{
  const h=harness('http:','#motion-models'),pending=deferred();h.set(behaviorFixture('getup',2));
  h.eval('claimMotionControl(snapshot)');h.post(()=>pending.promise);
  const heartbeat=h.eval('motionHeartbeat()');assert.equal(h.posts().length,1);
  h.set(recordedGetupRangeFaultFixture());
  pending.resolve(response({error:'Motion state changed; refresh before starting'},409));await heartbeat;
  assert.equal(h.eval('motionError'),'');
  assert(h.nodes.get('motionStatusMessage').textContent.includes('ID 30 目标超过已记录的机械范围'));
  assert.equal(h.posts().length,1);
});

test('a hold heartbeat already in flight cannot report failure during a pending getup transition',async()=>{
  const h=harness('http:','#motion-models'),heartbeatReply=deferred(),startReply=deferred();
  h.set(behaviorFixture('hold',3));h.eval('claimMotionControl(snapshot)');
  h.post((url,options)=>JSON.parse(options.body).action==='start'?startReply.promise:heartbeatReply.promise);
  const heartbeat=h.eval('motionHeartbeat()');
  h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;
  const start=h.nodes.get('motionGetup').onclick();
  heartbeatReply.resolve(response({error:'Motion state changed; refresh before starting'},409));await heartbeat;
  assert.equal(h.eval('motionError'),'');assert.equal(h.posts().length,2);
  h.post(async()=>response({}));startReply.resolve(response(behaviorFixture('getup',4)));await start;
  assert.equal(h.eval('motionError'),'');
  assert.equal(h.posts().filter(call=>JSON.parse(call.options.body).action==='start').length,1);
});

test('late heartbeat error from another deployment or action cannot poison current control',async()=>{
  for(const change of ['deployment','action']){
    const h=harness('http:','#motion-models'),pending=deferred(),old=behaviorFixture('hold',3);
    old.motion.deployment.started_monotonic=1;h.set(old);h.eval('claimMotionControl(snapshot)');
    h.post(()=>pending.promise);const heartbeat=h.eval('motionHeartbeat()');
    const next=behaviorFixture(change==='action'?'getup':'hold',change==='action'?4:3);
    next.motion.deployment.started_monotonic=change==='deployment'?2:1;
    h.set(next);h.eval('claimMotionControl(snapshot)');
    pending.resolve(response({error:'Motion state changed; refresh before starting'},409));await heartbeat;
    assert.equal(h.eval('motionError'),'');assert.equal(h.posts().length,1);
    assert(!h.nodes.get('motionActionResult').textContent.includes('旧请求'));
  }
});

test('STOP remains available during pending getup and its newer proof defeats a late start reply',async()=>{
  const h=harness('http:','#motion-models'),pending=deferred();h.set(behaviorFixture('hold',3));
  h.eval('claimMotionControl(snapshot)');h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;
  const stopped=behaviorFixture('off',5);stopped.motion.last_operation={action:'stop',state:'complete',error:null};
  h.post((url,options)=>JSON.parse(options.body).action==='start'?pending.promise:Promise.resolve(response(stopped)));
  const start=h.nodes.get('motionGetup').onclick();assert.equal(h.nodes.get('motionStop').disabled,false);
  await h.nodes.get('motionStop').onclick();
  pending.resolve(response(behaviorFixture('getup',4)));await start;
  assert.equal(h.eval('snapshot.motion.hardware.generation'),5);assert.equal(h.eval('snapshot.motion.hardware.mode'),'off');
  assert(h.nodes.get('motionActionResult').textContent.includes('回读确认 15 个舵机扭矩全部关闭'));
  assert.equal(h.posts().filter(call=>JSON.parse(call.options.body).action==='stop').length,1);
});

test('old target-validation controller cannot start models but still offers STOP and disconnect',async()=>{
  for(const marker of [undefined,0,true,'1']){
    const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);
    s.motion.hardware.target_validation_protocol=marker;h.set(s);
    h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;h.eval('renderMotion()');
    for(const id of ['motionWalk','motionGetup','motionPick','motionRoulade']){
      assert.equal(h.nodes.get(id).disabled,true,id);await h.nodes.get(id).onclick();
      assert(h.nodes.get(id).title.includes('控制程序已更新'),id);
    }
    assert.equal(h.posts().length,0);
    assert.equal(h.nodes.get('motionStop').disabled,false);assert.equal(h.nodes.get('motionDisconnect').disabled,false);
    assert(h.nodes.get('motionBehaviorHint').textContent.includes('控制程序已更新，请停止并断开运控，再连接并载入模型'));
    const stopped=behaviorFixture('off',4);stopped.motion.hardware.target_validation_protocol=marker;
    const closed=behaviorFixture('off',5);closed.motion.phase='closed';closed.motion.owns_connection=false;closed.motion.hardware.target_validation_protocol=marker;
    h.post(async(url,options)=>response(JSON.parse(options.body).action==='disconnect'?closed:stopped));await h.nodes.get('motionStop').onclick();
    assert.equal(h.posts().length,1);assert.equal(JSON.parse(h.posts()[0].options.body).action,'stop');
    await h.nodes.get('motionDisconnect').onclick();assert.equal(h.posts().length,2);assert.equal(JSON.parse(h.posts()[1].options.body).action,'disconnect');
  }
});

test('target-validation marker keeps stand and read-only preview available on an old controller',async()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('off',3);
  delete s.motion.hardware.target_validation_protocol;h.set(s);
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionStand').disabled,false);assert.equal(h.nodes.get('motionShadow').disabled,false);
  assert.equal(h.nodes.get('motionStop').disabled,false);assert.equal(h.nodes.get('motionDisconnect').disabled,false);
  h.post(async()=>response(behaviorFixture('shadow',4)));await h.nodes.get('motionShadow').onclick();
  const starts=h.posts().map(call=>JSON.parse(call.options.body)).filter(body=>body.action==='start');
  assert.equal(starts.length,1);assert.equal(starts[0].mode,'shadow');
});

test('recorded 28.5 degree hold blocks walk before POST and shows current IMU posture',async()=>{
  // Run 15 motion-c2e09dddb6ede620 / operation-860e365396b155319a3cd186:
  // joint targets were reached, but the body had a 28.538 degree total tilt.
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);
  const reason='动作开始前机身需要保持水平（倾角小于 15°）';
  s.motion.hardware.imu.gravity=[0.476690588,0.031692234,-0.878499679];
  s.motion.hardware.model_start_blocked_reasons.walk=reason;
  assert.equal(s.motion.hardware.start_blocked_reason,null);
  h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionWalk').disabled,true);assert.equal(h.nodes.get('motionWalk').title,reason);
  assert(h.nodes.get('motionWalkGateStatus').textContent.includes(reason));
  assert.equal(h.nodes.get('motionImuTilt').textContent,'28.5°');
  assert.equal(h.nodes.get('motionImuPitch').textContent,'28.5°');
  assert.equal(h.nodes.get('motionImuRoll').textContent,'-2.1°');
  assert.equal(h.nodes.get('motionMode').textContent,'关节站姿已到位 · 待核对机身姿态');
  await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
  assert.equal(h.nodes.get('motionPick').disabled,false);
});

test('a fresh under-15-degree hold retains the ordinary explicit walk gate',async()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3),angle=10*Math.PI/180;
  s.motion.hardware.imu.gravity=[Math.sin(angle),0,-Math.cos(angle)];h.set(s);
  assert.equal(h.nodes.get('motionImuTilt').textContent,'10.0°');
  assert.equal(h.nodes.get('motionWalk').disabled,true);
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionWalk').disabled,false);assert.equal(h.nodes.get('motionWalk').title,'');
  assert(h.nodes.get('motionWalkGateStatus').textContent.includes('符合小于 15°'));
  const next=behaviorFixture('walk',4);next.motion.hardware.imu.gravity=s.motion.hardware.imu.gravity;
  h.post(async()=>response(next));await h.nodes.get('motionWalk').onclick();
  const starts=h.posts().map(call=>JSON.parse(call.options.body)).filter(body=>body.action==='start');
  assert.equal(starts.length,1);assert.equal(starts[0].mode,'walk');
  assert.equal(starts[0].generation,3);assert.equal(starts[0].page_generation,101);
});

test('diagonal tilt uses the total acos angle rather than either smaller component',async()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3),angle=20*Math.PI/180;
  s.motion.hardware.imu.gravity=[Math.sin(angle)/Math.sqrt(2),Math.sin(angle)/Math.sqrt(2),-Math.cos(angle)];
  h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionImuTilt').textContent,'20.0°');
  assert.equal(h.nodes.get('motionImuPitch').textContent,'14.4°');
  assert.equal(h.nodes.get('motionImuRoll').textContent,'-14.4°');
  assert.equal(h.nodes.get('motionWalk').disabled,true);
  assert(h.nodes.get('motionWalk').title.includes('倾角小于 15°'));
  await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
});

test('expired motion feedback retains labeled posture readings and never sends a walk request',async()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);h.set(s);
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionImuTilt').textContent,'0.0°');h.age(501);
  for(const id of ['motionImuTilt','motionImuPitch','motionImuRoll'])assert.equal(h.nodes.get(id).textContent,'0.0°',id);
  assert(h.nodes.get('motionImuFreshness').textContent.includes('保留上次读数'));
  assert.equal(h.nodes.get('motionWalk').disabled,true);
  await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
});

test('IMU start freshness stays strictly below 100ms while display labels the last received frame',async()=>{
  for(const [imuAge,feedbackAge,browserAge] of [[100,0,0],[5,95,0],[5,90,5],[99,0,1]]){
    const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);
    s.motion.hardware.imu_age_ms=imuAge;s.motion.feedback_age_ms=feedbackAge;h.set(s);if(browserAge)h.age(browserAge);
    assert.equal(h.eval('motionFresh()'),true);
    assert.equal(h.eval('motionImuAngles()'),null);
    if(imuAge>=100){
      for(const id of ['motionImuTilt','motionImuPitch','motionImuRoll'])assert.equal(h.nodes.get(id).textContent,'等待读数',id);
      assert(h.nodes.get('motionImuFreshness').textContent.includes('尚未收到有效读数'));
    }else{
      assert.equal(h.nodes.get('motionImuTilt').textContent,'0.0°');
      assert(h.nodes.get('motionImuFreshness').textContent.includes('最近收到的'));
    }
    h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
    assert.equal(h.nodes.get('motionWalk').disabled,true);
    assert(h.nodes.get('motionWalkGateStatus').textContent.includes('100 毫秒内'));
    await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
  }
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);
  s.motion.hardware.imu_age_ms=5;s.motion.feedback_age_ms=90;h.set(s);h.age(4);
  assert.equal(h.nodes.get('motionImuTilt').textContent,'0.0°');
  assert.equal(h.posts().length,0);
});

test('250ms polling retains the displayed 24.5 degree frame without unlocking stale or legacy starts',async()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',6),tilt=24.5*Math.PI/180;
  s.motion.hardware.imu.gravity=[Math.sin(tilt),0,-Math.cos(tilt)];
  s.motion.hardware.imu_age_ms=13;s.motion.hardware.age_ms=13;
  delete s.motion.hardware.policy_metadata_protocol;delete s.motion.hardware.stand_target_deg;
  s.motion.hardware.model_start_blocked_reasons.walk='动作开始前机身需要保持水平（倾角小于 15°）';
  h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  for(const gap of [100,100,50]){
    h.age(gap);assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');
    assert.equal(h.nodes.get('motionImuPitch').textContent,'24.5°');
    assert.equal(h.nodes.get('motionImuRoll').textContent,'0.0°');
    assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();
  }
  assert(h.nodes.get('motionWalkGateStatus').textContent.includes('停止并断开运控'));
  assert(h.nodes.get('motionWalkGateStatus').textContent.includes('倾角小于 15°'));
  assert(h.nodes.get('motionImuFreshness').textContent.includes('0.3 秒'));
  assert.equal(h.nodes.get('motionStop').disabled,false);assert.equal(h.posts().length,0);
  h.set(s);assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');
  h.age(487);assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');assert.equal(h.nodes.get('motionImuPitch').textContent,'24.5°');assert.equal(h.nodes.get('motionImuRoll').textContent,'0.0°');
  assert(h.nodes.get('motionImuFreshness').textContent.includes('保留上次读数'));
  assert.equal(h.nodes.get('motionStop').disabled,false);assert.equal(h.posts().length,0);
});

test('missing or invalid IMU data is never described as a current posture',()=>{
  const mutations=[s=>delete s.motion.hardware.imu_age_ms,s=>s.motion.hardware.imu_age_ms='5',
    s=>s.motion.hardware.imu_age_ms=-1,s=>s.motion.hardware.imu_age_ms=Infinity,
    s=>s.motion.hardware.imu_ok=false,s=>s.motion.hardware.imu.gravity=[0,0,0],
    s=>s.motion.hardware.imu.gravity=[0,0,NaN],s=>delete s.motion.hardware.imu];
  for(const mutate of mutations){
    const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);mutate(s);h.set(s);
    for(const id of ['motionImuTilt','motionImuPitch','motionImuRoll'])assert.equal(h.nodes.get(id).textContent,'等待读数',id);
    assert(h.nodes.get('motionImuFreshness').textContent.includes('实时 IMU 姿态不可用'));
    assert(!h.nodes.get('motionWalkGateStatus').textContent.includes('符合小于 15°'));
    assert.equal(h.posts().length,0);
  }
});

test('same archive with a newer deployment accepts a lower native generation and drops old stop evidence',()=>{
  const h=harness('http:','#motion-models'),old=recordedHoldTiltStopFixture(),next=behaviorFixture('off',1);
  h.set(old);h.eval("motionError='Motion state changed; refresh before starting'");
  next.motion.deployment.started_monotonic=2;h.set(next);
  assert.equal(h.eval('snapshot.motion.deployment.started_monotonic'),2);
  assert.equal(h.eval('snapshot.motion.hardware.generation'),1);
  assert.equal(h.eval('motionError'),'');
  assert(!h.nodes.get('motionStatusMessage').textContent.includes('机身倾角超过 40'));
  assert(!h.nodes.get('motionActionResult').textContent.includes('保护停止'));
  h.set(old);
  assert.equal(h.eval('snapshot.motion.deployment.started_monotonic'),2);
  assert.equal(h.eval('snapshot.motion.hardware.generation'),1);
  assert(!h.nodes.get('motionActionResult').textContent.includes('保护停止'));
  assert.equal(h.posts().length,0);
});

test('a new deployment with the same hash and native generation cannot reuse the old page owner',async()=>{
  const h=harness('http:','#motion-models'),old=behaviorFixture('hold',3),next=behaviorFixture('hold',3);
  h.set(old);h.eval('claimMotionControl(snapshot)');next.motion.deployment.started_monotonic=2;h.set(next);
  assert.equal(h.eval('ownsMotionControl()'),false);
  await h.eval('motionHeartbeat()');h.emit('blur');h.eval('document.hidden=true');h.emit('visibilitychange');h.emit('pagehide');h.navigate('#joint-steps');await tick();
  assert.equal(h.posts().length,0);
  assert.equal(h.nodes.get('motionStop').disabled,false);
});

test('explicit connect accepts its deployment reset or newer same-archive low-generation reply',async()=>{
  for(const pendingDeployment of [false,true]){
    const h=harness('http:','#motion-models'),old=recordedHoldTiltStopFixture(),next=behaviorFixture('off',1);
    old.motion.phase='closed';old.motion.owns_connection=false;old.motion.hardware.connected=false;
    next.motion.deployment.started_monotonic=2;h.set(old);
    const connecting=clone(next);connecting.motion={phase:'deploying',owns_connection:false,hardware:null,deployment:null,error:null};
    h.post(async()=>response(pendingDeployment?connecting:next));await h.nodes.get('motionConnect').onclick();
    if(pendingDeployment){
      assert.equal(h.eval('snapshot.motion.phase'),'deploying');assert.equal(h.eval('snapshot.motion.deployment'),null);
      assert.equal(h.nodes.get('motionMode').textContent,'正在部署');h.set(next);
    }
    assert.equal(h.eval('snapshot.motion.deployment.started_monotonic'),2);
    assert.equal(h.eval('snapshot.motion.hardware.generation'),1);
    assert(!h.nodes.get('motionStatusMessage').textContent.includes('机身倾角超过 40'));
    assert.equal(h.posts().length,1);assert.equal(JSON.parse(h.posts()[0].options.body).action,'connect');
  }
});

test('late start replies and errors from an older same-archive deployment cannot replace current state',async()=>{
  for(const refused of [false,true]){
    const h=harness('http:','#motion-models'),old=behaviorFixture('hold',3),pending=deferred(),current=behaviorFixture('hold',3);
    h.set(old);h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;
    h.post(()=>pending.promise);const start=h.nodes.get('motionGetup').onclick();
    current.motion.deployment.started_monotonic=2;h.set(current);
    pending.resolve(refused?response({error:'Motion state changed; refresh before starting'},409):response(behaviorFixture('getup',4)));await start;
    assert.equal(h.eval('snapshot.motion.deployment.started_monotonic'),2);
    assert.equal(h.eval('snapshot.motion.hardware.generation'),3);assert.equal(h.eval('snapshot.motion.hardware.mode'),'hold');
    assert.equal(h.eval('ownsMotionControl()'),false);assert.equal(h.eval('motionError'),'');
    assert(!h.nodes.get('motionActionResult').textContent.includes('旧请求'));
    assert.equal(h.posts().length,1);assert.equal(JSON.parse(h.posts()[0].options.body).action,'start');
  }
});

test('fixed stance and zero-command balance are distinct explicit actions',async()=>{
  assert(html.includes('缓慢进入固定站姿'));
  assert(html.includes('启动站立平衡'));
  assert(html.includes('固定站姿只保持关节角度'));
  const h=harness('http:','#motion-models'),held=behaviorFixture('hold',3);h.set(held);
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.posts().length,0);assert.equal(h.nodes.get('motionWalk').disabled,false);
  const running=behaviorFixture('walk',4);h.post(async()=>response(running));await h.nodes.get('motionWalk').onclick();
  const starts=h.posts().map(p=>JSON.parse(p.options.body)).filter(p=>p.action==='start');
  assert.equal(starts.length,1);assert.equal(starts[0].mode,'walk');assert.equal(starts[0].support_confirmed,true);
  assert.equal(h.nodes.get('motionMode').textContent,'站立平衡模型运行');
  running.motion.hardware.command={forward:.1,side:0,yaw:0};h.set(running);
  assert.equal(h.nodes.get('motionMode').textContent,'平衡与行走模型运行');
  assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='start').length,1);
});

test('old policy metadata contract blocks models while STOP and fixed stance remain available',async()=>{
  for(const marker of [undefined,0,true,'1']){
    const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);s.motion.hardware.policy_metadata_protocol=marker;h.set(s);
    h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;h.eval('renderMotion()');
    for(const id of ['motionWalk','motionGetup','motionPick','motionRoulade']){
      assert.equal(h.nodes.get(id).disabled,true,id);await h.nodes.get(id).onclick();
    }
    assert.equal(h.posts().length,0);assert.equal(h.nodes.get('motionStop').disabled,false);assert.equal(h.nodes.get('motionDisconnect').disabled,false);
    s.motion.hardware.mode='off';s.motion.hardware.generation=4;delete s.motion.hardware.stand_target_deg;h.set(s);
    h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');assert.equal(h.nodes.get('motionStand').disabled,false);
  }
});

test('stance table uses the current worker targets without changing saved calibration',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('off',3),before=JSON.stringify(s.calibration);h.set(s);
  const joint=s.calibration.joints.find(j=>j.id===12),angle=s.motion.hardware.stand_target_deg[2];
  const expected=Math.round(joint.zero+angle/360*4095*joint.sign);
  assert.equal(h.eval('motionRows.get(12)[3].textContent'),expected);
  const legacy=Math.round(joint.zero-23/360*4095*joint.sign);assert.notEqual(expected,legacy);
  s.motion.hardware.stand_target_deg[2]=-18;h.set(s);
  assert.equal(h.eval('motionRows.get(12)[3].textContent'),Math.round(joint.zero-18/360*4095*joint.sign));
  assert.equal(h.eval('JSON.stringify(snapshot.calibration)'),before);assert.equal(h.posts().length,0);
});

test('neck calibration and fixed stance fallback use saved zero instead of colliding model home',()=>{
  const h=harness('http:','#joint-steps'),s=jointFixture('all-complete');
  const neck=s.calibration.joints.find(j=>j.id===30);
  Object.assign(neck,{zero:1977,sign:-1,min:1814,max:2770,direction_confirmed:true});
  Object.assign(s.joint_wizard,{id:30,inferred_sign:-1});
  const before=JSON.stringify(s.calibration);
  h.set(s);h.eval('selectedJointId=30;render()');
  assert.equal(h.eval('jointSaved(30)'),true);
  assert(h.nodes.get('jointRangeGuide').textContent.includes('固定站姿参照 1977'));
  assert(h.nodes.get('jointRangeGuide').textContent.includes('头部姿态不变'));
  assert.equal(h.eval('jointDefaultDegrees[10]'),0);
  assert.equal(h.eval('motionStandDegrees({})[10]'),0);
  assert.equal(h.eval('JSON.stringify(snapshot.calibration)'),before);
  assert.equal(h.posts().length,0);
});

test('invalid new stance targets never fall back to a guessed 23-degree pose',async()=>{
  for(const invalid of [undefined,[0,0,0],Array(15).fill('0')]){
    const h=harness('http:','#motion-models'),s=behaviorFixture('off',3);s.motion.hardware.stand_target_deg=invalid;h.set(s);
    h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
    assert.equal(h.eval('motionRows.get(12)[3].textContent'),'待采集');assert.equal(h.nodes.get('motionStand').disabled,true);await h.nodes.get('motionStand').onclick();assert.equal(h.posts().length,0);
    const held=behaviorFixture('hold',4);held.motion.hardware.stand_target_deg=invalid;h.set(held);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
    assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
    assert.equal(h.nodes.get('motionStop').disabled,false);
  }
});

function nativeImuFixture(status='active',generation=101){
  const s=jointFixture('all-complete',generation);
  s.native_access={full_workbench:true,page_generation:77};s.capabilities={imu_calibration:true};
  Object.assign(s.session,{status,error:null,error_details:null,collection_scope:status==='closed'?'joints':'imu',received_age_ms:0,recovery_allowed:status==='closed',recovery_blocked_reason:null,recheck_allowed:false,recheck_blocked_reason:null,recovery_pending:false});
  s.motion={phase:'fault',owns_connection:false,error:'连接后全部舵机扭矩关闭尚未确认',hardware:null,deployment:{archive_sha256:'native-bundle',started_monotonic:1,model_hashes:{walk:'w',getup:'g',pick:'p',roulade:'r'}}};
  Object.assign(s.feedback,{scope:'imu',joint_feedback_valid:false,online_mask:0,fault_mask:32767,torque_mask:0,armed:false,arm_cookie:0,imu_ok:true,last_error:0});
  s.body_imu={pitch_deg:99,roll_deg:-99,attitude_protocol:1,source:'calibration-feedback',collection_scope:'imu',imu_age_ms:.01,sample_us:s.feedback.sample_us,imu_us:s.feedback.imu_us,attitude:{protocol:1,tilt_deg:24.5,pitch_deg:-24.5,roll_deg:.4}};
  if(status!=='active'){s.feedback=null;s.body_imu=null;}
  return s;
}

test('diagnostic-only startup shows the reconnect route without a false hardware error',()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture('closed');
  s.motion.error='已明确选择仅启动诊断网页；尚未执行完整硬件连接检查，尚未启动舵机操作和采集。';
  s.motion.closed_proof={no_uart_opened:true};h.set(s);
  assert.equal(h.nodes.get('sessionAlert').hidden,true);assert.equal(h.nodes.get('motionConnect').disabled,false);
  assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.nodes.get('motionWalk').disabled,true);assert.equal(h.posts().length,0);
  s.motion.error='USB connection failed';h.set(s);assert.equal(h.nodes.get('sessionAlert').hidden,false);
  assert(h.nodes.get('sessionAlert').textContent.includes('USB connection failed'));
});

test('native startup and maintenance holds allow one explicit backend-approved review and subsequent retries',async()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture('closed');
  s.session.status='fault';s.session.recovery_recheck={category:'trust',message:'启动检查尚未通过'};
  s.maintenance={phase:'closed',fault_hold:true,fault_review_allowed:true,fault_review_blocked_reason:null,owns_connection:false,busy:false,error:'舵机 ID 10 未响应'};
  h.set(s);await h.settleOld(s);await h.eval('refresh()');
  assert.equal(h.nodes.get('motionRecheck').disabled,false);assert.equal(h.nodes.get('motionConnect').disabled,true);
  assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.nodes.get('motionWalk').disabled,true);
  h.post(async(url,options)=>{assert.equal(url,'/api/servo-maintenance');assert.deepEqual(JSON.parse(options.body),{action:'review',generation:101});return response(s);});
  await h.nodes.get('motionRecheck').onclick();assert.equal(h.posts().length,1);assert.equal(h.nodes.get('motionRecheck').disabled,false);
  s.maintenance.fault_review_allowed=false;h.set(s);assert.equal(h.nodes.get('motionRecheck').disabled,true);
  s.maintenance.fault_review_allowed=true;s.native_access.full_workbench=false;h.set(s);assert.equal(h.nodes.get('motionRecheck').disabled,true);
});

test('battery-off native IMU angles display independently of old motion errors without unlocking any movement',async()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture(),before=JSON.stringify(s.calibration);h.set(s);await h.settleOld(s);await h.eval('refresh()');
  assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');assert.equal(h.nodes.get('motionImuPitch').textContent,'-24.5°');assert.equal(h.nodes.get('motionImuRoll').textContent,'0.4°');
  assert(h.nodes.get('motionImuFreshness').textContent.includes('IMU 独立读取 · 不动作'));assert(h.nodes.get('motionImuFreshness').textContent.includes('舵机状态未确认'));
  assert.equal(h.eval('motionImuAngles()'),null);assert.equal(h.eval('motionFresh()'),false);
  for(const id of ['motionShadow','motionStand','motionWalk','motionGetup','motionPick','motionRoulade'])assert.equal(h.nodes.get(id).disabled,true);
  assert.equal(h.nodes.get('motionImuRead').disabled,true);assert.equal(h.nodes.get('motionImuEnd').disabled,false);assert.equal(h.posts().length,0);assert.equal(h.eval('JSON.stringify(snapshot.calibration)'),before);
});

test('partial and online servo masks do not hide IMU angles or authorize movement',()=>{
  for(const [online,fault] of [[2,32765],[32767,0],[32767,1],[0,0]]){
    const h=harness('http:','#motion-models'),s=nativeImuFixture();s.feedback.online_mask=online;s.feedback.fault_mask=fault;h.set(s);
    assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');assert.equal(h.nodes.get('motionImuPitch').textContent,'-24.5°');assert.equal(h.nodes.get('motionImuRoll').textContent,'0.4°');
    assert.equal(h.eval('snapshot.feedback.online_mask'),online);assert.equal(h.eval('snapshot.feedback.fault_mask'),fault);assert.equal(h.eval('motionImuAngles()'),null);
    for(const id of ['motionShadow','motionStand','motionWalk','motionGetup','motionPick','motionRoulade'])assert.equal(h.nodes.get(id).disabled,true);
    assert.equal(h.posts().length,0);
  }
});

test('independent IMU display hides expired mismatched uncalibrated or invalid protocol readings',()=>{
  const invalid=[s=>{s.native_access.full_workbench=false;},s=>{s.capabilities.imu_calibration=false;},s=>{s.session.collection_scope='joints';},s=>{s.feedback.scope='joints';},s=>{s.feedback.joint_feedback_valid=true;},s=>{s.calibration.imu.calibrated=false;},s=>{s.body_imu.attitude_protocol=0;},s=>{s.body_imu.attitude.protocol=0;},s=>{s.body_imu.attitude.tilt_deg=null;},s=>{s.body_imu.attitude.pitch_deg=181;},s=>{s.body_imu.imu_age_ms=500;},s=>{s.body_imu.imu_age_ms=-1;},s=>{s.body_imu.sample_us++;},s=>{delete s.body_imu.imu_us;},s=>{s.body_imu.source='historical';},s=>{s.feedback.imu_ok=false;},s=>{s.feedback.online_mask=false;},s=>{s.feedback.online_mask='1';},s=>{s.feedback.online_mask=-1;},s=>{s.feedback.online_mask=32768;},s=>{s.feedback.fault_mask=true;},s=>{s.feedback.fault_mask=-1;},s=>{s.feedback.fault_mask=32768;},s=>{s.feedback.torque_mask=1;},s=>{s.feedback.armed=true;},s=>{s.feedback.arm_cookie=1;},s=>{s.feedback.last_error=1;},s=>{s.feedback.accel_ms2=[0,0,0];},s=>{s.session.received_age_ms=500;}];
  for(const mutate of invalid){const h=harness('http:','#motion-models'),s=nativeImuFixture();mutate(s);h.set(s);for(const id of ['motionImuTilt','motionImuPitch','motionImuRoll'])assert.equal(h.nodes.get(id).textContent,'等待读数');assert.equal(h.posts().length,0);}
  const h=harness('http:','#motion-models'),s=nativeImuFixture();s.session.received_age_ms=200;s.body_imu.imu_age_ms=50;h.set(s);h.age(249);assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');h.age(1);assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');assert(h.nodes.get('motionImuFreshness').textContent.includes('保留上次读数'));assert.equal(h.posts().length,0);
});

test('IMU-only display matches the backend signed timestamp boundary without changing motion freshness',()=>{
  for(const [delta,age,visible] of [[-500000,0,true],[-500001,0,false],[500000,500,false]]){
    const h=harness('http:','#motion-models'),s=nativeImuFixture();s.feedback.sample_us=600000;s.feedback.imu_us=s.feedback.sample_us-delta;s.body_imu.sample_us=s.feedback.sample_us;s.body_imu.imu_us=s.feedback.imu_us;s.body_imu.imu_age_ms=age;h.set(s);
    assert.equal(h.nodes.get('motionImuTilt').textContent,visible?'24.5°':'等待读数');assert.equal(h.eval('motionImuAngles()'),null);assert.equal(h.nodes.get('motionWalk').disabled,true);assert.equal(h.posts().length,0);
  }
});

test('same-card IMU read submits one explicit purpose imu request without connecting models or sampling calibration',async()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture('closed'),before=JSON.stringify(s.calibration);h.set(s);await h.settleOld(s);await h.eval('refresh()');
  assert.equal(h.posts().length,0);assert.equal(h.nodes.get('motionImuRead').disabled,false);assert.equal(h.nodes.get('motionImuEnd').disabled,true);
  const pending=deferred();h.post((url,options)=>{assert.equal(url,'/api/recover-feedback');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose:'imu'});assert.equal(options.headers['X-CSRF-Token'],'token-101');return pending.promise;});
  const first=h.nodes.get('motionImuRead').onclick();await h.nodes.get('motionImuRead').onclick();assert.equal(h.posts().length,1);
  const waiting=nativeImuFixture('waiting',102);waiting.session.recovery_pending=true;waiting.session.recheck_allowed=false;waiting.session.feedback_recheck={purpose:'imu',phase:'checking'};pending.resolve(response(waiting));await first;
  assert.equal(h.nodes.get('motionImuRead').disabled,true);assert(h.nodes.get('motionImuReadStatus').textContent.includes('正在连接 IMU'));assert.equal(h.nodes.get('motionImuTilt').textContent,'等待读数');
  h.set(nativeImuFixture('active',102));assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');assert.equal(h.posts().length,1);assert.equal(h.eval('JSON.stringify(snapshot.calibration)'),before);
});

test('backend-approved independent IMU read survives servo holds and submits only a purpose-bound connection check',async()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture('closed');
  Object.assign(s.session,{status:'fault',recovery_allowed:false,recheck_allowed:false,imu_read_allowed:true,imu_read_blocked_reason:null,imu_read_requires_connection_check:true,recovery_recheck:{category:'trust',technical_reason:'USB disconnected'},connection_check:{phase:'failed',allowed:false,error:'USB disconnected'}});
  s.maintenance={phase:'closed',owns_connection:false,fault_hold:true,fault_review_allowed:true,busy:false,error:'ID 10 未响应'};
  h.set(s);await h.settleOld(s);assert.equal(h.nodes.get('motionImuRead').disabled,false);
  assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.nodes.get('motionWalk').disabled,true);
  h.post(async(url,options)=>{assert.equal(url,'/api/check-connection');assert.deepEqual(JSON.parse(options.body),{generation:101,purpose:'imu'});return response(s);});
  await h.nodes.get('motionImuRead').onclick();assert.equal(h.posts().length,1);
  s.session.imu_read_allowed=false;s.session.imu_read_blocked_reason='原连接缺少匹配的串口关闭证明';h.set(s);
  assert.equal(h.nodes.get('motionImuRead').disabled,true);assert(h.nodes.get('motionImuReadStatus').textContent.includes('关闭证明'));
  await h.nodes.get('motionImuRead').onclick();assert.equal(h.posts().length,1);
});

test('same-card IMU stop uses only end-feedback and never closes another UART owner',async()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture();h.set(s);await h.settleOld(s);h.post(async(url,options)=>{assert.equal(url,'/api/end-feedback');assert.deepEqual(JSON.parse(options.body),{generation:101});const ended=nativeImuFixture('closed');h.set(ended);return response(ended);});
  await h.nodes.get('motionImuEnd').onclick();assert.equal(h.posts().length,1);assert.equal(h.nodes.get('motionImuTilt').textContent,'24.5°');assert(h.nodes.get('motionImuFreshness').textContent.includes('保留上次读数'));assert.equal(h.nodes.get('motionImuEnd').disabled,true);
  for(const [mutate,hint] of [[s=>{s.session.status='active';s.session.collection_scope='joints';},'结束采集'],[s=>{s.maintenance={owns_connection:true};},'舵机工作台'],[s=>{s.motion.owns_connection=true;},'停止并断开运控'],[s=>{s.motion.hardware={connected:true};},'停止并断开运控'],[s=>{s.motion.phase='deploying';},'停止并断开运控']]){
    const blocked=harness('http:','#motion-models'),current=nativeImuFixture('closed');mutate(current);blocked.set(current);assert.equal(blocked.nodes.get('motionImuRead').disabled,true);assert(blocked.nodes.get('motionImuReadStatus').textContent.includes(hint));assert.equal(blocked.nodes.get('motionImuEnd').disabled,true);await blocked.nodes.get('motionImuRead').onclick();await blocked.nodes.get('motionImuEnd').onclick();assert.equal(blocked.posts().length,0);
  }
});

test('independent IMU read failure never retries and unsupported native or file pages cannot request it',async()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture('closed');h.set(s);h.post(async()=>response({error:'当前 USB 检查未通过'},409));await h.nodes.get('motionImuRead').onclick();assert.equal(h.posts().length,1);assert(h.nodes.get('motionImuReadStatus').textContent.includes('当前 USB 检查未通过'));h.age(600);assert.equal(h.posts().length,1);
  for(const protocol of ['file:','http:']){const blocked=harness(protocol,'#motion-models'),current=nativeImuFixture('closed');if(protocol==='http:')current.native_access.full_workbench=false;blocked.set(current);assert.equal(blocked.nodes.get('motionImuRead').disabled,true);await blocked.nodes.get('motionImuRead').onclick();await blocked.nodes.get('motionImuEnd').onclick();assert.equal(blocked.posts().length,0);}
});

test('voltage collection is one explicit scoped request; refresh and navigation never collect',async()=>{
  const h=harness('http:','#servo-settings'),s=maintenanceFixture(false);h.set(s);await h.settleOld(s);
  assert.equal(h.nodes.get('servoVoltages').disabled,false);assert.equal(h.nodes.get('servoVoltageRows').children.length,15);
  await h.eval('refresh()');h.navigate('#device-status');h.navigate('#servo-settings');assert.equal(h.posts().length,0);
  const pending=deferred();h.post((url,options)=>{assert.equal(url,'/api/servo-maintenance');assert.deepEqual(JSON.parse(options.body),{action:'voltages',generation:101});return pending.promise;});
  const first=h.nodes.get('servoVoltages').onclick();await h.nodes.get('servoVoltages').onclick();assert.equal(h.posts().length,1);
  const next=maintenanceFixture();Object.assign(next.maintenance,{busy:true,action:'voltages'});pending.resolve(response(next));await first;
  assert.equal(h.nodes.get('servoVoltages').disabled,true);assert(h.nodes.get('servoVoltageSummary').textContent.includes('正在采集'));
});

test('voltage table distinguishes unknown, errors and genuine zero; samples age instead of becoming live',()=>{
  const h=harness(),s=maintenanceFixture();Object.assign(s.maintenance,{voltage_supported:true,voltage_results:[
    {id:10,state:0,error:0,raw:74,voltage_v:7.4,age_ms:500},
    {id:11,state:1,error:0,raw:null,voltage_v:null,age_ms:400},
    {id:12,state:2,error:1,raw:null,voltage_v:null,age_ms:300},
    {id:13,state:3,error:0,raw:null,voltage_v:null,age_ms:200},
    {id:14,state:0,error:0,raw:0,voltage_v:0,age_ms:100}]});h.set(s);
  const rows=()=>h.nodes.get('servoVoltageRows').children;
  assert.equal(rows()[0].children[1].textContent,'7.4 V');assert(rows()[0].children[2].textContent.includes('采样记录'));
  for(const i of [1,2,3])assert.equal(rows()[i].children[1].textContent,'—');
  assert(rows()[1].children[2].textContent.includes('未响应'));assert(rows()[2].children[2].textContent.includes('报错'));assert(rows()[3].children[2].textContent.includes('应答无效'));
  assert.equal(rows()[4].children[1].textContent,'0.0 V');assert.equal(rows()[5].children[2].textContent,'未采集');
  h.age(60500);assert(rows()[0].children[3].textContent.includes('61 秒前'));assert.equal(rows()[0].children[1].textContent,'7.4 V');assert.equal(h.posts().length,0);
});

test('voltage capability refusal and malformed values never invent voltages or enable blocked collection',async()=>{
  const h=harness(),s=maintenanceFixture();s.maintenance.voltage_supported=false;h.set(s);assert(h.nodes.get('servoVoltageSummary').textContent.includes('需要更新固件'));
  s.maintenance.voltage_supported=true;s.maintenance.voltage_results=[{id:10,state:0,error:0,raw:74,voltage_v:8.4,age_ms:0}];h.set(s);
  assert.equal(h.nodes.get('servoVoltageRows').children[0].children[1].textContent,'—');
  s.maintenance.voltage_results=[{id:10,state:0,error:0,raw:74,voltage_v:7.4,age_ms:0},{id:10,state:0,error:0,raw:74,voltage_v:7.4,age_ms:0}];h.set(s);
  assert.equal(h.nodes.get('servoVoltageRows').children[0].children[1].textContent,'—');assert(h.nodes.get('servoVoltageSummary').textContent.includes('没有有效'));
  const held=maintenanceFixture(false);held.maintenance.blocked_reason='运控正在使用连接';h.set(held);assert.equal(h.nodes.get('servoVoltages').disabled,true);await h.nodes.get('servoVoltages').onclick();assert.equal(h.posts().length,0);
  held.maintenance.blocked_reason=null;held.session.status='active';h.set(held);assert.equal(h.nodes.get('servoVoltages').disabled,true);
});

test('slow feedback polls never turn an established connection into a closing connection or reflow recovery controls',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();h.set(s);
  for(let repeat=0;repeat<4;repeat++){
    h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
    assert.equal(h.nodes.get('motionStand').disabled,false);
    for(const delay of [100,400,100,900]){
      h.age(delay);h.intervals[1]();
      assert.equal(h.nodes.get('motionConnect').textContent,'运控已连接');
      assert.equal(h.nodes.get('motionRecoveryControls').hidden,true);
      assert.equal(h.nodes.get('motionStatusTitle').textContent,'X5 运控已连接');
    }
    assert.equal(h.eval('motionFresh()'),false);
    assert.equal(h.nodes.get('motionStand').disabled,true);
    assert.equal(h.nodes.get('motionWalk').disabled,true);
    assert.equal(h.nodes.get('motionImuTilt').textContent,'0.0°');
    assert(h.nodes.get('motionStatusMessage').textContent.includes('模型和已保存校准已载入'));
    assert.equal(h.nodes.get('motionFeedbackState').textContent,'反馈自动更新');
    assert.equal(h.nodes.get('motionStop').disabled,false);
    assert.equal(h.nodes.get('motionDisconnect').disabled,false);
  }
  h.eval("fetchError='页面状态读取超时';render()");
  assert.equal(h.nodes.get('motionConnect').textContent,'运控已连接');
  assert.equal(h.nodes.get('motionRecoveryControls').hidden,true);
  assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.posts().length,0);
});

test('connection presentation follows explicit connection and fault transitions while stale source frames still block actions',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();
  s.motion.hardware.age_ms=150;h.set(s);
  assert.equal(h.eval('motionFresh()'),false);assert.equal(h.nodes.get('motionConnect').textContent,'运控已连接');
  assert.equal(h.nodes.get('motionRecoveryControls').hidden,true);assert.equal(h.nodes.get('motionStand').disabled,true);
  s.motion.last_hardware=clone(s.motion.hardware);s.motion.hardware=null;
  // The native adapter accepts last_hardware when its current frame expires;
  // this base-page presentation test does not exercise its generation merger.
  h.eval('snapshot.motion.hardware=null');h.set(s);
  assert.equal(h.nodes.get('motionConnect').textContent,'运控已连接');assert.equal(h.nodes.get('motionRecoveryControls').hidden,true);
  s.motion.last_hardware.connected=false;h.set(s);
  assert.equal(h.nodes.get('motionConnect').textContent,'连接待关闭确认');assert.equal(h.nodes.get('motionRecoveryControls').hidden,false);
  s.motion.hardware=clone(s.motion.last_hardware);s.motion.hardware.connected=true;s.motion.hardware.fault_ids=[11];h.set(s);
  assert.equal(h.nodes.get('motionRecoveryControls').hidden,false);
  s.motion.hardware.fault_ids=[];s.motion.transport_exit={process_exited:true};h.set(s);
  assert.equal(h.nodes.get('motionConnect').textContent,'连接待关闭确认');assert.equal(h.nodes.get('motionRecoveryControls').hidden,false);
  s.motion.phase='closed';s.motion.owns_connection=false;h.set(s);
  assert.equal(h.nodes.get('motionConnect').textContent,'连接并载入模型');assert.equal(h.nodes.get('motionRecoveryControls').hidden,false);
  assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.posts().length,0);
});

test('both native pages keep connection and loaded calibration stable through slow polls across all workspaces',async()=>{
  for(const full of [true,false]){
    const nativeHTML=execFileSync('python3',['-B','-c',
      'import sys; sys.path.insert(0,sys.argv[1]); from native_workbench import native_page; sys.stdout.buffer.write(native_page(sys.stdin.read(),authentication_required=False,full_workbench=sys.argv[2]=="true"))',
      path.join(__dirname,'../x5_policy'),String(full)],{input:html,encoding:'utf8'});
    const h=harness('http:','#motion-models',nativeHTML),s=motionFixture();h.set(s);await h.settleOld(s);
    for(const route of ['device-status','servo-settings','joint-zeros','joint-steps','imu-steps','motion-models']){
      h.navigate('#'+route);
      for(let repeat=0;repeat<3;repeat++){
        await h.eval('refresh()');h.age(1500);h.intervals[1]();h.eval('nativeOriginalRender()');
        assert.equal(h.nodes.get('motionConnect').textContent,'运控已连接',route);
        assert.equal(h.nodes.get('motionRecoveryControls').hidden,true,route);
        assert.equal(h.nodes.get('motionCalibration').textContent,'15 / 15 + IMU',route);
        assert.equal(h.nodes.get('motionStand').disabled,true);
        assert.equal(h.nodes.get('motionRecheck').disabled,true);
      }
    }
    s.motion.hardware.connected=false;h.set(s);
    assert.equal(h.nodes.get('motionRecoveryControls').hidden,false);
    assert.equal(h.nodes.get('motionCalibration').textContent,'已载入 · 连接待复查');
    assert.equal(h.posts().length,0);
  }
});

test('IMU and actual joint positions remain visible between slow polls and update only from received measurements',()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3),before=JSON.stringify(s.calibration);
  for(const [degrees,position] of [[5,2016],[9,2024],[-3,1998]]){
    const angle=degrees*Math.PI/180;s.motion.hardware.imu.gravity=[Math.sin(angle),0,-Math.cos(angle)];
    s.motion.hardware.rawdata[0].position=position;h.set(s);
    for(const gap of [0,200,400,1400]){
      h.age(gap);h.intervals[1]();
      assert.equal(h.nodes.get('motionImuPitch').textContent,degrees.toFixed(1)+'°');
      assert.equal(h.eval('motionRows.get(10)[1].textContent'),position);
      assert(!h.nodes.get('motionLiveDetail').textContent.includes('等待新鲜的运控反馈。'));
    }
    assert(h.nodes.get('motionImuFreshness').textContent.includes('保留上次读数'));
    assert(h.eval('motionRows.get(10)[5].textContent').startsWith('最近读数'));
    assert.equal(h.nodes.get('motionWalk').disabled,true);
    assert.equal(h.eval('motionImuAngles()'),null);
  }
  h.eval("fetchError='页面状态读取中断';render()");h.age(5000);
  assert.equal(h.nodes.get('motionImuPitch').textContent,'-3.0°');
  assert.equal(h.eval('motionRows.get(10)[1].textContent'),1998);
  assert(h.nodes.get('motionFeedbackState').textContent.includes('通信中断'));
  assert.equal(h.eval('JSON.stringify(snapshot.calibration)'),before);assert.equal(h.posts().length,0);
});

test('slow polling changes ages without alternating protective text, table labels or replacing unchanged values',()=>{
  const h=harness('http:','#motion-models'),s=recordedHoldTiltStopFixture();h.set(s);
  const watched=['motionActionResult','motionStatusMessage','sessionAlert','motionLiveDetail','motionMode','motionFeedbackState','connectionLabel'];
  const baseline=watched.map(id=>h.nodes.get(id).textContent),cell=h.eval('motionRows.get(10)[1]'),check=h.eval('motionRows.get(10)[5]'),angle=h.nodes.get('motionImuPitch');
  const baselineCheck=check.textContent;let writes=0;
  for(const node of [cell,check,angle,h.nodes.get('motionActionResult')]){
    let value=node.textContent;Object.defineProperty(node,'textContent',{get(){return value;},set(next){writes++;value=next;},configurable:true});
  }
  for(let cycle=0;cycle<4;cycle++){
    for(const ms of [100,450,650,650]){
      h.age(ms);h.intervals[1]();
      assert.deepEqual(watched.map(id=>h.nodes.get(id).textContent),baseline);
      assert.equal(check.textContent,baselineCheck);
    }
    assert.equal(h.nodes.get('motionStand').disabled,true);h.set(s);
  }
  assert.equal(writes,0,'same measurement must not repeatedly replace DOM text');
  s.motion.hardware.rawdata[0].position=2050;s.motion.hardware.imu.gravity=[.1,0,-.995];h.set(s);
  assert.equal(cell.textContent,2050);assert.notEqual(angle.textContent,'0.0°');assert(writes>=2);
  h.age(5000);assert(h.nodes.get('motionFeedbackState').textContent.includes('反馈延迟'));
  assert.equal(h.nodes.get('motionActionResult').textContent,baseline[0]);assert.equal(cell.textContent,2050);
  assert.equal(h.eval('motionFresh()'),false);assert.equal(h.posts().length,0);
});

test('offline servo positions are retained without fabricating new samples or resetting their age',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();h.set(s);const before=h.eval('motionDisplayJoints.get(10).receivedAt');
  h.age(1600);s.motion.hardware.online_ids=[];s.motion.hardware.release_confirmed=false;s.motion.hardware.rawdata[0].position=2222;h.set(s);
  assert.equal(h.eval('motionRows.get(10)[1].textContent'),2048);
  assert.equal(h.eval('motionDisplayJoints.get(10).receivedAt'),before);
  assert(h.eval('motionRows.get(10)[5].textContent').startsWith('上次读数'));
  assert(h.nodes.get('motionLiveDetail').textContent.includes('舵机在线 0/15'));
  assert(h.nodes.get('motionTorqueState').textContent.includes('尚未确认'));
  assert.equal(h.posts().length,0);
});

test('missing individual servo or invalid IMU frames preserve labeled last readings without presenting them as fresh',()=>{
  const h=harness('http:','#motion-models'),s=behaviorFixture('hold',3);h.set(s);
  const previous=h.eval('motionRows.get(10)[1].textContent');
  s.motion.hardware.imu_ok=false;s.motion.hardware.rawdata[0].position=null;
  s.motion.hardware.online_ids=s.motion.hardware.online_ids.filter(id=>id!==10);h.set(s);
  assert.equal(h.nodes.get('motionImuTilt').textContent,'0.0°');
  assert(h.nodes.get('motionImuFreshness').textContent.includes('保留上次读数'));
  assert.equal(h.eval('motionRows.get(10)[1].textContent'),previous);
  assert(h.eval('motionRows.get(10)[5].textContent').startsWith('上次读数'));
  assert.equal(h.eval('motionRows.get(10)[5].dataset.state'),'stale');
  assert.equal(h.eval('motionImuAngles()'),null);assert.equal(h.nodes.get('motionWalk').disabled,true);
  const changed=behaviorFixture('hold',3);changed.motion.connection_sequence=2;
  changed.motion.hardware.imu_ok=false;changed.motion.hardware.rawdata=[];h.set(changed);
  assert.equal(h.nodes.get('motionImuTilt').textContent,'等待读数');
  assert.equal(h.eval('motionRows.get(10)[1].textContent'),'等待读数');
  assert.equal(h.posts().length,0);
});

test('independent IMU display retains its last frame after a gap and resumes with the next valid sample',()=>{
  const h=harness('http:','#motion-models'),s=nativeImuFixture();h.set(s);h.age(2000);
  assert.equal(h.nodes.get('motionImuPitch').textContent,'-24.5°');
  assert(h.nodes.get('motionImuFreshness').textContent.includes('保留上次读数'));
  s.body_imu.attitude.pitch_deg=8.3;s.body_imu.attitude.tilt_deg=8.4;h.set(s);
  assert.equal(h.nodes.get('motionImuPitch').textContent,'8.3°');
  assert(h.nodes.get('motionImuFreshness').textContent.includes('IMU 独立读取'));
  assert.equal(h.nodes.get('motionWalk').disabled,true);assert.equal(h.eval('motionImuAngles()'),null);
  assert.equal(h.posts().length,0);
});

test('native connection caption stays identical through captured timers, polling and all six workspaces',async()=>{
  for(const full of [true,false]){
    const nativeHTML=execFileSync('python3',['-B','-c',
      'import sys; sys.path.insert(0,sys.argv[1]); from native_workbench import native_page; sys.stdout.buffer.write(native_page(sys.stdin.read(),authentication_required=False,full_workbench=sys.argv[2]=="true"))',
      path.join(__dirname,'../x5_policy'),String(full)],{input:html,encoding:'utf8'});
    const h=harness('http:','#device-status',nativeHTML),s=nativeImuFixture('closed');
    s.connection_target={kind:'wifi',host:'192.168.1.11',page_url:'http://192.168.1.11:8767/',native:true};
    s.native_access.full_workbench=full;
    const caption=h.nodes.get('x5ConnectionTarget'),writes=[];let value=caption.textContent;
    Object.defineProperty(caption,'textContent',{get:()=>value,set:next=>{value=next;writes.push(next);}});
    const expected='X5 独立工作台 · 192.168.1.11:8767；'+(full?'采集、维护和运控均在 X5 运行。':'已保存校准只读。');
    h.set(s);await h.settleOld(s);
    for(const route of ['device-status','servo-settings','joint-zeros','joint-steps','imu-steps','motion-models']){
      h.navigate('#'+route);
      for(let repeat=0;repeat<3;repeat++){
        h.intervals[1]();await h.eval('refresh()');h.eval('nativeOriginalRender()');
        assert.equal(caption.hidden,false);assert.equal(caption.textContent,expected,route);
      }
    }
    assert(writes.every(text=>text===expected),'No intermediate caption may drop the port or change the native description');
    assert.equal(h.posts().length,0);
  }
});

test('laptop connection captions retain USB and Wi-Fi destinations and hide missing targets',()=>{
  const h=harness(),s=fixture('closed');
  for(const [kind,label,host] of [['usb','USB 闪联','192.168.128.10'],['wifi','Wi-Fi','192.168.1.11']]){
    s.connection_target={kind,host,native:false};h.set(s);
    assert.equal(h.nodes.get('x5ConnectionTarget').textContent,'X5 连接：'+label+' · '+host+'；网页由笔记本提供。');
    assert.equal(h.nodes.get('x5ConnectionTarget').hidden,false);
  }
  delete s.connection_target;h.set(s);assert.equal(h.nodes.get('x5ConnectionTarget').hidden,true);
  assert.equal(h.nodes.get('x5ConnectionTarget').textContent,'');assert.equal(h.posts().length,0);
});

test('live voltage updates in-place and reports real low/high rails without changing support or starting motion',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();
  s.motion.hardware.voltage_supported=true;
  s.motion.hardware.motor_voltages=[{id:10,volts:7.0,age_ms:200,fresh:true},{id:11,volts:7.1,age_ms:300,fresh:true}];
  h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;
  const cell=h.eval('motionRows.get(10)[6]');
  assert.equal(cell.textContent,'7.0 V');assert.equal(cell.dataset.state,'good');
  assert(h.nodes.get('motionVoltageSummary').textContent.includes('7.0–7.1 V'));
  let writes=0,value=cell.textContent;Object.defineProperty(cell,'textContent',{get:()=>value,set:next=>{writes++;value=next;}});
  for(let n=0;n<3;n++){h.age(100);h.intervals[1]();}
  assert.equal(writes,0);assert.equal(h.nodes.get('motionSupportConfirmed').checked,true);
  s.motion.hardware.motor_voltages[0].volts=3.9;h.set(s);
  assert.equal(cell.textContent,'3.9 V');assert.equal(cell.dataset.state,'bad');
  assert(h.nodes.get('motionVoltageSummary').textContent.includes('供电电压不足：ID 10'));
  s.motion.hardware.motor_voltages[1].volts=8.5;h.set(s);
  assert(h.nodes.get('motionVoltageSummary').textContent.includes('供电电压过高：ID 11'));
  assert.equal(h.posts().length,0);
});

test('voltage gaps retain dated values and never display missing power as zero volts',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture();
  s.motion.hardware.voltage_supported=true;
  s.motion.hardware.motor_voltages=[{id:10,volts:7.0,age_ms:200,fresh:true}];h.set(s);
  h.age(1100);h.intervals[1]();
  assert.equal(h.eval('motionRows.get(10)[6].textContent'),'7.0 V（上次）');
  assert.equal(h.eval('motionRows.get(10)[6].dataset.state'),'stale');
  assert(h.nodes.get('motionVoltageSummary').textContent.includes('当前电压未知'));
  s.motion.hardware.motor_voltages=[{id:10,volts:null,age_ms:null,fresh:false}];h.set(s);
  assert.equal(h.eval('motionRows.get(10)[6].textContent'),'7.0 V（上次）');
  assert.equal(h.eval('motionRows.get(11)[6].textContent'),'电压未知');
  assert(!h.nodes.get('motionVoltageSummary').textContent.includes('0.0 V'));
  s.motion.connection_sequence=2;h.set(s);
  assert.equal(h.eval('motionRows.get(10)[6].textContent'),'电压未知');
  s.motion.hardware.voltage_supported=false;h.set(s);
  assert(h.nodes.get('motionVoltageSummary').textContent.includes('待固件更新'));
  assert.equal(h.posts().length,0);
});

test('motion voltage table fits its container and wraps cells instead of adding a horizontal scrollbar',()=>{
  assert(html.includes('class="table-wrap motion-table-wrap"'));
  assert(html.includes('.motion-table-wrap{overflow:visible;min-width:0}'));
  assert(html.includes('#motionJointTable{table-layout:fixed;min-width:0;width:100%;white-space:normal}'));
  assert(html.includes('white-space:normal;overflow-wrap:anywhere;font-variant-numeric:tabular-nums'));
  assert(!html.includes('#motionJointTable{table-layout:fixed;min-width:760px}'));
  const h=harness();assert.equal(h.eval('motionRows.get(10).length'),7);
});

test('X5-owned fixed stance survives browser timers, hiding, page closure and all workspace changes without heartbeats',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('hold',4);
  s.motion.hardware.hold_control_protocol=2;s.motion.hardware.page_heartbeat_required=false;
  h.set(s);h.ownMotion();await tick();
  for(let i=0;i<5;i++){h.age(600);h.intervals[1]();}
  h.emit('blur');h.eval('document.hidden=true');h.emit('visibilitychange');h.emit('pagehide');
  for(const route of ['device-status','servo-settings','joint-zeros','joint-steps','imu-steps','motion-models'])h.navigate('#'+route);
  await h.eval('motionHeartbeat()');await tick();
  assert.equal(h.posts().length,0,'Browser lifecycle must neither renew nor stop X5 fixed stance');
  h.post(async()=>response(motionFixture('off',5)));await h.nodes.get('motionStop').onclick();
  const bodies=h.posts().map(p=>JSON.parse(p.options.body));assert.equal(bodies.length,1);
  assert.equal(bodies[0].action,'stop');assert.equal(bodies[0].stop_source,'button');
});

test('a reloaded passive page can manually stop an independently held stance without taking control',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('hold',4);s.motion.hardware.hold_control_protocol=2;
  h.set(s);assert.equal(h.eval('ownsMotionControl()'),false);h.emit('pagehide');await tick();
  assert.equal(h.posts().length,0);assert.equal(h.nodes.get('motionStop').disabled,false);
  h.post(async()=>response(motionFixture('off',5)));await h.nodes.get('motionStop').onclick();
  assert.equal(JSON.parse(h.posts()[0].options.body).action,'stop');
});

test('captured head walk preview is labeled without unlocking restricted physical models',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('off',4);
  Object.assign(s.motion.hardware,{head_clearance_protocol:1,head_clearance_required:true,
    walk_preview_head_protocol:1,hold_head_deg:{30:-5,31:-9,32:-3,33:10},
    walk_preview_head_command_rad:[-.44,-.51,-.05,.17]});
  h.nodes.get('motionPreviewModel').value='walk';h.set(s);
  assert.equal(h.nodes.get('motionPreviewHeadStatus').hidden,false);
  assert(h.nodes.get('motionPreviewHeadStatus').textContent.includes('已记录的头颈期望姿态'));
  assert.equal(h.nodes.get('motionWalk').disabled,true);
  h.nodes.get('motionPreviewModel').value='getup';h.nodes.get('motionPreviewModel').onchange();
  assert.equal(h.nodes.get('motionPreviewHeadStatus').hidden,true);
  s.motion.hardware.hold_head_deg=null;h.nodes.get('motionPreviewModel').value='walk';h.set(s);
  assert(h.nodes.get('motionPreviewHeadStatus').textContent.includes('模型原始头部指令'));
  delete s.motion.hardware.walk_preview_head_protocol;h.set(s);
  assert.equal(h.nodes.get('motionPreviewHeadStatus').hidden,true);
  assert.equal(h.posts().length,0);
});

function fixedHeadFixture(mode='hold',captured=true,generation=3){
  const s=behaviorFixture(mode,generation);
  Object.assign(s.motion.hardware,{head_clearance_protocol:1,head_clearance_required:true,
    fixed_head_walk_protocol:1,fixed_head_walk_ready:captured,fixed_head_walk_zero_velocity_only:true,
    hold_head_deg:captured?{30:-5,31:-9,32:-3,33:10}:null,
    armed:mode!=='off',firmware_error:0});
  return s;
}
test('fixed-head balance requires capture then an explicitly ready supported hold',async()=>{
  const h=harness('http:','#motion-models'),uncaptured=fixedHeadFixture('off',false);
  h.set(uncaptured);h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionHeadClearanceConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionHeadCapture').disabled,false);
  assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
  const captured=fixedHeadFixture('off',true,4);h.post(async()=>response(captured));await h.nodes.get('motionHeadCapture').onclick();
  assert.equal(h.posts().length,1);assert.equal(JSON.parse(h.posts()[0].options.body).action,'capture-head-clearance');
  assert(h.nodes.get('motionHeadClearanceStatus').textContent.includes('固定头颈零速平衡'));
  assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,1,'capturing does not start a model');
  const hold=fixedHeadFixture('hold',true,5);hold.motion.hardware.hold_ready=false;h.set(hold);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,1,'transitioning hold cannot start balance');
  hold.motion.hardware.hold_ready=true;h.set(hold);assert.equal(h.nodes.get('motionWalk').disabled,false);
  assert.equal(h.nodes.get('motionWalk').textContent,'启动固定头颈零速平衡');
  const running=fixedHeadFixture('walk',true,6);h.post(async()=>response(running));await h.nodes.get('motionWalk').onclick();
  const starts=h.posts().map(p=>JSON.parse(p.options.body)).filter(p=>p.action==='start');
  assert.equal(starts.length,1);assert.equal(starts[0].mode,'walk');assert.equal(starts[0].generation,5);assert.equal(starts[0].support_confirmed,true);
  assert.equal(h.nodes.get('motionMode').textContent,'固定头颈零速平衡运行');
});
test('fixed-head walk contract strictly checks readiness, captured angles and both model protocols',async()=>{
  const variants=[
    {fixed_head_walk_protocol:undefined},{fixed_head_walk_protocol:true},{fixed_head_walk_protocol:2},
    {fixed_head_walk_ready:false},{fixed_head_walk_ready:'true'},
    {fixed_head_walk_zero_velocity_only:false},{fixed_head_walk_zero_velocity_only:1},
    {head_clearance_protocol:undefined},{hold_head_deg:null},{hold_head_deg:{30:-5,31:-9,32:-3}},
    {target_validation_protocol:0},{policy_metadata_protocol:0},{stand_target_deg:Array(14).fill(0)}
  ];
  for(const changes of variants){
    const h=harness('http:','#motion-models'),s=fixedHeadFixture();Object.assign(s.motion.hardware,changes);h.set(s);
    h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
    assert.equal(h.nodes.get('motionWalk').disabled,true,JSON.stringify(changes));
    await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
    assert.equal(h.nodes.get('motionStop').disabled,false);
  }
});
test('fixed-head balance keeps support, tilt, freshness and backend refusal gates',async()=>{
  const cases=[
    s=>{s.motion.hardware.hold_ready=false;},
    s=>{s.motion.hardware.age_ms=100;},
    s=>{s.motion.hardware.imu_age_ms=100;},
    s=>{const a=16*Math.PI/180;s.motion.hardware.imu.gravity=[Math.sin(a),0,-Math.cos(a)];},
    s=>{s.motion.hardware.model_start_blocked_reasons.walk='ID 12 实际位置超出记录范围';}
  ];
  for(const change of cases){
    const h=harness('http:','#motion-models'),s=fixedHeadFixture();change(s);h.set(s);
    h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
    assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
  }
  const h=harness('http:','#motion-models'),s=fixedHeadFixture();h.set(s);
  assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');assert.equal(h.nodes.get('motionWalk').disabled,false);
  h.age(101);assert.equal(h.nodes.get('motionWalk').disabled,true);await h.nodes.get('motionWalk').onclick();assert.equal(h.posts().length,0);
});
test('fixed-head walk readiness never unlocks getup, pick or roulade',async()=>{
  const h=harness('http:','#motion-models');h.set(fixedHeadFixture());
  h.nodes.get('motionSupportConfirmed').checked=true;h.nodes.get('motionBehaviorConfirmed').checked=true;h.eval('renderMotion()');
  assert.equal(h.nodes.get('motionWalk').disabled,false);
  for(const [id,model] of [['motionGetup','getup'],['motionPick','pick'],['motionRoulade','roulade']]){
    assert.equal(h.eval('motionModelContractReady('+JSON.stringify(model)+')'),false);
    assert.equal(h.nodes.get(id).disabled,true);assert(h.nodes.get(id).title.includes('其他模型动作'));
    await h.nodes.get(id).onclick();
  }
  assert(h.nodes.get('motionBehaviorHint').textContent.includes('机械间隙尚未验证'));assert.equal(h.posts().length,0);
});
test('fixed-head live walk hides movement and rejects every nonzero command path',async()=>{
  const h=harness('http:','#motion-models'),s=fixedHeadFixture('walk');h.set(s);h.ownMotion();await tick();
  h.post(async()=>response(s));assert.equal(h.nodes.get('motionDrive').hidden,true);
  for(const id of ['motionForward','motionBackward','motionLeft','motionRight']){
    assert.equal(h.nodes.get(id).hidden,true);assert.equal(h.nodes.get(id).disabled,true);
    await h.nodes.get(id).onpointerdown({pointerId:1,preventDefault(){}});
  }
  for(const values of [[.1,0,0],[-.1,0,0],[0,.1,0],[0,0,.3],[0,0,-.3],[1e-9,0,0]]){
    await h.eval('motionDriveCommand('+JSON.stringify(values)+')');
    const [forward,side,yaw]=values;await h.eval('runMotion("command",'+JSON.stringify({forward,side,yaw})+')');
  }
  const commands=()=>h.posts().map(p=>JSON.parse(p.options.body)).filter(p=>p.action==='command');
  assert.equal(commands().length,0);
  await h.eval('motionDriveCommand([0,0,0])');assert.equal(commands().length,1);
  assert.deepEqual([commands()[0].forward,commands()[0].side,commands()[0].yaw],[0,0,0]);
  s.motion.hardware.fixed_head_walk_ready=false;h.set(s);await h.eval('motionDriveCommand([.1,0,0])');assert.equal(commands().length,1,'lost readiness cannot expose movement');
  h.age(501);await h.eval('motionDriveCommand([0,0,0])');assert.equal(commands().length,1,'stale feedback cannot submit a drive command');
});
test('fixed-head restriction preserves non-actuating shadow commands and ordinary walk controls',async()=>{
  const preview=harness('http:','#motion-models'),shadow=fixedHeadFixture('shadow');preview.set(shadow);preview.post(async()=>response(shadow));
  await preview.eval('runMotion("command",{forward:.1,side:0,yaw:0})');
  assert.equal(preview.posts().filter(p=>JSON.parse(p.options.body).action==='command').length,1);
  assert.equal(preview.nodes.get('motionDrive').hidden,true);
  const h=harness('http:','#motion-models');h.set(fixedHeadFixture('walk'));h.ownMotion();await tick();
  h.set(behaviorFixture('walk'));assert.equal(h.nodes.get('motionDrive').hidden,false);
  for(const id of ['motionForward','motionBackward','motionLeft','motionRight']){assert.equal(h.nodes.get(id).hidden,false);assert.equal(h.nodes.get(id).disabled,false);}
});

function receiptFixture(h,action,mode,generation,changes={}){
  return {action_receipt_protocol:1,action,mode,generation,connected:true,release_confirmed:action==='stop',
    page_generation:101,connection_sequence:1,operation_epoch:1,
    control_owner:action==='start'?{id:h.eval('motionControlId'),generation}:null,...changes};
}
function receiptHoldFixture(){
  const s=fixedHeadFixture('hold',true,4);s.motion.connection_sequence=1;s.motion.hardware.hold_control_protocol=2;return s;
}
test('start receipt immediately renews its new lease without refreshing sensor data',async()=>{
  const h=harness('http:','#motion-models'),held=receiptHoldFixture();h.set(held);h.ownMotion();
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');h.age(40);
  const received=h.eval('receivedAt'),sensor=JSON.stringify(h.eval('snapshot.motion.hardware'));
  h.post(async(url,options)=>{const body=JSON.parse(options.body);return response(body.action==='start'?receiptFixture(h,'start','walk',5):{heartbeat_accepted:true,page_generation:101,generation:5});});
  await h.nodes.get('motionWalk').onclick();await tick();
  assert.equal(h.eval('receivedAt'),received);assert.equal(JSON.stringify(h.eval('snapshot.motion.hardware')),sensor);
  assert.equal(h.eval('motionControlHardware().generation'),5);assert.equal(h.eval('motionControlHardware().mode'),'walk');
  const requests=h.posts().map(p=>JSON.parse(p.options.body));
  assert.equal(requests.filter(p=>p.action==='start').length,1);
  const beats=requests.filter(p=>p.action==='heartbeat');assert.equal(beats.length,1);assert.equal(beats[0].generation,5);
  assert.equal(h.nodes.get('motionWalk').disabled,true);assert(h.nodes.get('motionMode').textContent.includes('等待新反馈'));
  h.age(20);h.set(held);assert.equal(h.eval('receivedAt'),received,'a GET from before the ACK cannot refresh old sensors');
  const running=fixedHeadFixture('walk',true,5);running.motion.connection_sequence=1;h.set(running);
  assert.equal(h.eval('motionReceipt'),null);assert.equal(h.eval('snapshot.motion.hardware.mode'),'walk');assert.equal(h.eval('ownsMotionControl()'),true);
});
test('stop receipt preserves real release proof without inventing fresh torque or IMU samples',async()=>{
  const h=harness('http:','#motion-models'),s=receiptHoldFixture();h.set(s);h.ownMotion();h.age(60);
  const received=h.eval('receivedAt'),sensor=JSON.stringify(h.eval('snapshot.motion.hardware'));
  h.post(async()=>response(receiptFixture(h,'stop','off',5)));await h.nodes.get('motionStop').onclick();
  assert.equal(h.eval('receivedAt'),received);assert.equal(JSON.stringify(h.eval('snapshot.motion.hardware')),sensor);
  assert.equal(h.eval('motionControl'),null);assert.equal(h.eval('motionReceipt.release_confirmed'),true);
  assert(h.nodes.get('motionActionResult').textContent.includes('真实回读已确认 15 个舵机扭矩全部关闭'));
  assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='heartbeat').length,0);
  assert.equal(h.nodes.get('motionWalk').disabled,true);
});
test('hiding a newly acknowledged walk stops its receipt generation before the next sensor GET',async()=>{
  const h=harness('http:','#motion-models');h.set(receiptHoldFixture());h.ownMotion();
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  h.post(async(url,options)=>{const body=JSON.parse(options.body);return response(body.action==='start'?receiptFixture(h,'start','walk',5):{heartbeat_accepted:true,generation:5});});
  await h.nodes.get('motionWalk').onclick();await tick();
  h.post(async()=>response(receiptFixture(h,'stop','off',6,{operation_epoch:2})));
  h.eval('document.hidden=true');h.emit('visibilitychange');await tick();
  const stops=h.posts().map(p=>JSON.parse(p.options.body)).filter(p=>p.action==='stop');
  assert.equal(stops.length,1);assert.equal(stops[0].generation,5);assert.equal(stops[0].stop_source,'page-hidden');
  assert.equal(h.eval('motionReceipt.mode'),'off');assert.equal(h.eval('motionControl'),null);
});
test('a late start receipt cannot override a newer stop receipt',async()=>{
  const h=harness('http:','#motion-models'),pending=deferred();h.set(receiptHoldFixture());h.ownMotion();
  h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  h.post(async(url,options)=>JSON.parse(options.body).action==='start'?pending.promise:response(receiptFixture(h,'stop','off',6,{operation_epoch:2})));
  const start=h.nodes.get('motionWalk').onclick();await h.nodes.get('motionStop').onclick();
  pending.resolve(response(receiptFixture(h,'start','walk',5)));await start;
  assert.equal(h.eval('motionControl'),null);assert.equal(h.eval('motionReceipt.mode'),'off');assert.equal(h.eval('motionReceipt.generation'),6);
  assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='heartbeat').length,0);
  assert(h.nodes.get('motionActionResult').textContent.includes('真实回读已确认'));
});
test('a late start receipt cannot claim a newer GET generation or another connection',async()=>{
  for(const reconnect of [false,true]){
    const h=harness('http:','#motion-models'),pending=deferred();h.set(receiptHoldFixture());h.ownMotion();
    h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');h.post(()=>pending.promise);
    const start=h.nodes.get('motionWalk').onclick();
    const next=fixedHeadFixture('off',true,reconnect?4:6);next.motion.connection_sequence=reconnect?2:1;h.set(next);
    pending.resolve(response(receiptFixture(h,'start','walk',5)));await start;
    assert.equal(h.eval('motionReceipt'),null);assert.equal(h.eval('ownsMotionControl()'),false);
    assert.equal(h.eval('snapshot.motion.hardware.mode'),'off');assert.equal(h.posts().length,1);
  }
});
test('a receipt without verified release or matching owner never creates a control lease',async()=>{
  for(const [action,changes] of [['stop',{release_confirmed:false}],['start',{control_owner:{id:'0'.repeat(32),generation:5}}]]){
    const h=harness('http:','#motion-models');h.set(receiptHoldFixture());h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
    h.post(async()=>response(receiptFixture(h,action,action==='stop'?'off':'walk',5,changes)));
    await h.nodes.get(action==='stop'?'motionStop':'motionWalk').onclick();
    assert.equal(h.eval('motionReceipt'),null);assert.equal(h.eval('motionControl'),null);
    assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='heartbeat').length,0);
    assert(!h.nodes.get('motionActionResult').textContent.includes('真实回读已确认'));
  }
});

function receiptCaptureFixture(h,generation=5,changes={}){
  return receiptFixture(h,'capture-head-clearance','off',generation,{release_confirmed:true,
    captured_head_deg:{30:-4,31:-17,32:-10,33:-3},...changes});
}
function captureOffFixture(){
  const s=fixedHeadFixture('off',false,4);s.motion.connection_sequence=1;return s;
}
test('head capture receipt records no fresh sensor frame or lease and awaits GET before opening controls',async()=>{
  const h=harness('http:','#motion-models'),s=captureOffFixture();h.set(s);
  h.nodes.get('motionHeadClearanceConfirmed').checked=true;h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');h.age(40);
  const received=h.eval('receivedAt'),sensor=JSON.stringify(h.eval('snapshot.motion.hardware'));
  h.post(async()=>response(receiptCaptureFixture(h)));await h.nodes.get('motionHeadCapture').onclick();await tick();
  assert.equal(h.eval('receivedAt'),received);assert.equal(JSON.stringify(h.eval('snapshot.motion.hardware')),sensor);
  assert.equal(h.eval('motionControl'),null);assert.equal(h.eval('motionControlHardware().mode'),'off');
  assert.equal(h.eval('motionControlHardware().generation'),5);assert.equal(h.posts().length,1,'capture sends no heartbeat');
  assert.equal(h.nodes.get('motionHeadCapture').disabled,true);assert.equal(h.nodes.get('motionStand').disabled,true);assert.equal(h.nodes.get('motionWalk').disabled,true);
  assert(h.nodes.get('motionMode').textContent.includes('头颈记录已确认'));
  assert(h.nodes.get('motionHeadClearanceStatus').textContent.includes('尚未记录'),'displayed configuration also awaits real GET');
  h.age(20);h.set(s);assert.equal(h.eval('receivedAt'),received,'old GET cannot refresh pre-capture sensors');
  const captured=fixedHeadFixture('off',true,5);captured.motion.connection_sequence=1;captured.motion.hardware.hold_head_deg={30:-4,31:-17,32:-10,33:-3};h.set(captured);
  assert.equal(h.eval('motionReceipt'),null);assert.equal(h.nodes.get('motionStand').disabled,false);
  assert(h.nodes.get('motionHeadClearanceStatus').textContent.includes('ID 31：-17.0°'));
  assert.equal(h.nodes.get('motionWalk').disabled,true,'capture does not bypass ready HOLD');
});
test('late head capture receipt cannot replace a STOP or a new connection',async()=>{
  for(const reconnect of [false,true]){
    const h=harness('http:','#motion-models'),pending=deferred();h.set(captureOffFixture());h.nodes.get('motionHeadClearanceConfirmed').checked=true;h.eval('renderMotion()');
    h.post(async(url,options)=>JSON.parse(options.body).action==='capture-head-clearance'?pending.promise:response(receiptFixture(h,'stop','off',6,{operation_epoch:2})));
    const capture=h.nodes.get('motionHeadCapture').onclick();
    if(reconnect){const next=captureOffFixture();next.motion.connection_sequence=2;h.set(next);}else await h.nodes.get('motionStop').onclick();
    pending.resolve(response(receiptCaptureFixture(h)));await capture;
    assert.equal(h.eval('motionControl'),null);assert.equal(h.eval('snapshot.motion.hardware.hold_head_deg'),null);
    if(reconnect)assert.equal(h.eval('motionReceipt'),null);else assert.equal(h.eval('motionReceipt.action'),'stop');
    assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='heartbeat').length,0);
  }
});
test('head capture receipt requires four finite angles and confirmed OFF release',async()=>{
  for(const changes of [{captured_head_deg:{30:0,31:0,32:0}},{captured_head_deg:{30:0,31:'bad',32:0,33:0}},
    {captured_head_deg:{30:0,31:null,32:0,33:0}},{release_confirmed:false},{mode:'hold'},
    {control_owner:{id:'0'.repeat(32),generation:5}}]){
    const h=harness('http:','#motion-models');h.set(captureOffFixture());h.nodes.get('motionHeadClearanceConfirmed').checked=true;h.eval('renderMotion()');
    h.post(async()=>response(receiptCaptureFixture(h,5,changes)));await h.nodes.get('motionHeadCapture').onclick();
    assert.equal(h.eval('motionReceipt'),null);assert.equal(h.eval('motionControl'),null);
    assert(h.nodes.get('motionActionResult').textContent.includes('尚未确认'));assert.equal(h.posts().length,1);
  }
});

test('motion display polls once per second and idle device display at most twice per second',async()=>{
  const h=harness('http:','#motion-models'),s=fixedHeadFixture('walk',true,4);s.motion.connection_sequence=1;h.set(s);await h.settleOld(s);
  const gets=()=>h.calls.filter(c=>['/api/status','/api/live-status'].includes(c.url));
  assert.equal(gets().length,1);h.age(250);await h.intervals[0]();assert.equal(gets().length,1);
  h.age(750);await h.intervals[0]();assert.equal(gets().length,2);
  const off=fixedHeadFixture('off',true,5);off.motion.connection_sequence=1;h.set(off);
  h.age(500);await h.intervals[0]();assert.equal(gets().length,2,'OFF motion workspace also polls at 1Hz');
  h.navigate('#device-status');await h.intervals[0]();assert.equal(gets().length,3);
  h.age(250);await h.intervals[0]();assert.equal(gets().length,3);
  h.age(250);await h.intervals[0]();assert.equal(gets().length,4);assert.equal(h.posts().length,0);
});
test('calibration polling and delayed display preserve inputs; stale samples cannot be captured and faults clear confirmations',async()=>{
  for(const route of ['joint-steps','joint-zeros','imu-steps']){
    const h=harness('http:','#'+route),s=zeroFixture();s.session.received_age_ms=20;
    h.set(s);await h.settleOld(s);
    h.nodes.get('zeroSupportConfirmed').checked=true;h.nodes.get('zeroSupportConfirmed').onchange();
    h.nodes.get('zeroReferenceConfirmed').checked=true;h.nodes.get('zeroReferenceConfirmed').onchange();
    h.nodes.get('mountedConfirmed').checked=true;h.nodes.get('supportConfirmed').checked=true;
    for(let cycle=0;cycle<30;cycle++){
      h.age(100);await h.intervals[0]();h.intervals[1]();
      for(const id of ['zeroSupportConfirmed','zeroReferenceConfirmed','mountedConfirmed','supportConfirmed'])
        assert.equal(h.nodes.get(id).checked,true,route+' '+id+' cycle '+cycle);
    }
    assert.equal(h.eval('displayPollInterval()'),250);
    assert.equal(h.posts().length,0);
    h.age(501);
    for(const id of ['zeroSupportConfirmed','zeroReferenceConfirmed','mountedConfirmed','supportConfirmed'])
      assert.equal(h.nodes.get(id).checked,true,route+' stale '+id);
    assert.equal(h.nodes.get('zeroCapture').disabled,true);
    h.eval("fetchError='显示请求超时';render()");
    assert.equal(h.nodes.get('zeroSupportConfirmed').checked,true);
    assert.equal(h.nodes.get('zeroCapture').disabled,true);
    const fault=clone(s);fault.session.status='fault';h.set(fault);
    for(const id of ['zeroSupportConfirmed','zeroReferenceConfirmed','mountedConfirmed','supportConfirmed'])
      assert.equal(h.nodes.get(id).checked,false,route+' fault '+id);
  }
});
test('one display GET in flight does not block lease heartbeats or STOP',async()=>{
  const h=harness('http:','#motion-models'),s=fixedHeadFixture('walk',true,4);s.motion.connection_sequence=1;h.set(s);
  h.post(async(url,options)=>response(JSON.parse(options.body).action==='stop'?receiptFixture(h,'stop','off',5):{heartbeat_accepted:true,generation:4}));
  h.ownMotion();await tick();h.age(1500);h.intervals[0]();h.intervals[0]();await tick();
  assert.equal(h.calls.filter(c=>c.url==='/api/status').length,1,'automatic GETs share one pending request');
  assert(h.posts().some(p=>JSON.parse(p.options.body).action==='heartbeat'));
  await h.nodes.get('motionStop').onclick();assert.equal(h.eval('motionReceipt.mode'),'off');
  assert.equal(h.posts().filter(p=>JSON.parse(p.options.body).action==='stop').length,1);
  await h.settleOld(s);
});
test('paused display ages readings while heartbeat and STOP stay independent; resume performs one GET',async()=>{
  const h=harness('http:','#motion-models'),s=fixedHeadFixture('walk',true,4);s.motion.connection_sequence=1;h.set(s);await h.settleOld(s);
  h.post(async(url,options)=>response(JSON.parse(options.body).action==='stop'?receiptFixture(h,'stop','off',5):{heartbeat_accepted:true,generation:4}));
  h.ownMotion();await tick();const received=h.eval('receivedAt'),gets=h.calls.filter(c=>c.url==='/api/status').length;
  await h.nodes.get('displayUpdatesToggle').onclick();h.age(1000);await h.intervals[0]();await tick();
  assert.equal(h.eval('receivedAt'),received);assert.equal(h.calls.filter(c=>c.url==='/api/status').length,gets);
  assert.equal(h.eval('motionFresh()'),false);assert.equal(h.eval('motionImuAngles()'),null);
  assert.equal(h.nodes.get('motionStop').disabled,false);assert.equal(h.nodes.get('displayUpdatesToggle').textContent,'恢复画面更新');
  assert(h.posts().filter(p=>JSON.parse(p.options.body).action==='heartbeat').length>=2);
  await h.nodes.get('displayUpdatesToggle').onclick();assert.equal(h.calls.filter(c=>c.url==='/api/status').length,gets+1);
  assert(h.eval('receivedAt')>received);assert.equal(h.eval('motionFresh()'),true);
  await h.nodes.get('displayUpdatesToggle').onclick();await h.nodes.get('motionStop').onclick();
  assert.equal(h.eval('motionReceipt.mode'),'off');assert.equal(h.calls.filter(c=>c.url==='/api/status').length,gets+1,'STOP ACK does not bypass display pause');
});
test('hidden display sends no automatic GET and resumes once visible',async()=>{
  const h=harness('http:','#motion-models'),s=captureOffFixture();h.set(s);await h.settleOld(s);
  const gets=h.calls.filter(c=>c.url==='/api/status').length;h.eval('document.hidden=true');h.emit('visibilitychange');
  h.age(2000);await h.intervals[0]();await h.eval('refresh()');assert.equal(h.calls.filter(c=>c.url==='/api/status').length,gets);
  h.eval('document.hidden=false');h.emit('visibilitychange');await tick();assert.equal(h.calls.filter(c=>c.url==='/api/status').length,gets+1);
  assert.equal(h.posts().length,0);
});
test('pausing discards and aborts an already pending display GET without refreshing old sensors',async()=>{
  const h=harness('http:','#motion-models'),s=captureOffFixture();h.set(s);const received=h.eval('receivedAt');
  await h.nodes.get('displayUpdatesToggle').onclick();assert.equal(h.calls[0].options.signal.aborted,true);
  h.age(1000);const changed=clone(s);changed.motion.hardware.generation=7;changed.motion.hardware.imu.gravity=[1,0,0];
  await h.settleOld(changed);assert.equal(h.eval('receivedAt'),received);assert.equal(h.eval('snapshot.motion.hardware.generation'),4);
  assert.equal(h.eval('fetchError'),'');assert.equal(h.eval('motionFresh()'),false);
});
test('superseded or older connection/generation GET packages cannot rejuvenate any retained sensors',()=>{
  for(const change of [
    s=>{s.motion.snapshot_superseded=true;s.motion.hardware=null;s.motion.last_hardware=null;},
    s=>{s.motion.hardware.generation=8;},
    s=>{s.motion.last_hardware={...s.motion.hardware,generation:8};s.motion.hardware=null;},
    s=>{s.motion.connection_sequence=1;s.motion.hardware.generation=100;}
  ]){
    const h=harness('http:','#motion-models'),s=fixedHeadFixture('walk',true,9);s.motion.connection_sequence=2;s.feedback=fixture('active').feedback;h.set(s);
    const received=h.eval('receivedAt'),original=JSON.stringify(h.eval('snapshot'));h.age(1000);
    assert.equal(h.eval('motionFresh()'),false);assert.equal(h.eval('motionImuAngles()'),null);
    const old=clone(s);change(old);old.feedback.sample_us+=10000;old.feedback.accel_ms2=[1,2,3];h.set(old);
    assert.equal(h.eval('receivedAt'),received);assert.equal(JSON.stringify(h.eval('snapshot')),original);
    assert.equal(h.eval('motionFresh()'),false);assert.equal(h.eval('motionImuAngles()'),null);
    assert.equal(h.nodes.get('motionWalk').disabled,true);assert.equal(h.posts().length,0);
  }
});
test('a genuinely newer connection may reset controller generation and refresh only its new sensor package',()=>{
  const h=harness('http:','#motion-models'),s=fixedHeadFixture('walk',true,9);s.motion.connection_sequence=2;h.set(s);h.age(1000);
  const next=fixedHeadFixture('off',true,1);next.motion.connection_sequence=3;h.set(next);
  const received=h.eval('receivedAt');assert.equal(received,1000);assert.equal(h.eval('snapshot.motion.hardware.generation'),1);assert.equal(h.eval('snapshot.motion.connection_sequence'),3);
  h.age(1000);const late=clone(s);late.motion.hardware.generation=100;h.set(late);
  assert.equal(h.eval('receivedAt'),received);assert.equal(h.eval('snapshot.motion.connection_sequence'),3);assert.equal(h.eval('motionFresh()'),false);
});

test('blocked balance names the measured joint gaps without granting readiness or sending a start',async()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('hold');
  s.motion.hardware.model_start_blocked_reasons={walk:'先启动站姿，并等待实际关节接近默认姿态'};
  s.motion.hardware.rawdata.forEach((row,index)=>{row.angle_deg=s.motion.hardware.stand_target_deg[index];});
  s.motion.hardware.rawdata.find(row=>row.id===23).angle_deg-=7.2315;
  s.motion.hardware.rawdata.find(row=>row.id===11).angle_deg+=5.0726;
  h.set(s);h.nodes.get('motionSupportConfirmed').checked=true;h.eval('renderMotion()');
  const text=h.nodes.get('motionWalkGateStatus').textContent;
  assert(text.includes('右膝 ID 23 偏差 7.2°'));
  assert(text.includes('左髋侧摆 ID 11 偏差 5.1°'));
  assert.equal(h.nodes.get('motionWalk').disabled,true);
  await h.nodes.get('motionWalk').onclick();
  assert.equal(h.posts().length,0,'explanation is display-only; the backend readiness remains required');
});
test('pose explanation rejects incomplete readings and never replaces a different protection reason',()=>{
  const h=harness('http:','#motion-models'),s=motionFixture('hold');
  s.motion.hardware.model_start_blocked_reasons={walk:'先启动站姿，并等待实际关节接近默认姿态'};
  s.motion.hardware.rawdata.forEach((row,index)=>{row.angle_deg=s.motion.hardware.stand_target_deg[index];});
  for(const change of [state=>state.rawdata.pop(),state=>{state.rawdata[0].angle_deg='7';},state=>{state.rawdata[0].id=11;},state=>{state.hold_ready=true;}]){
    const next=clone(s);change(next.motion.hardware);h.set(next);
    assert.equal(h.eval('motionHoldPoseDiagnostic(snapshot.motion.hardware)'),'');
  }
  const fault=clone(s);fault.motion.hardware.model_start_blocked_reasons.walk='ID 23 实际位置超出记录范围';h.set(fault);
  assert.equal(h.eval('motionKnownWalkBlockedReason(snapshot.motion.hardware)'),'ID 23 实际位置超出记录范围');
  assert.equal(h.posts().length,0);
});

test('standing reference explains whole head path and preserves confirmations during sensor updates',()=>{
  const h=harness('http:','#motion-models'),s=fixedHeadFixture('off',true,9);
  s.motion.hardware.candidate_profile={standing_reference_restore:true,standing_reference_valid:true};
  h.set(s);
  assert.equal(h.nodes.get('motionStand').textContent,'缓慢恢复上次站姿');
  assert(h.nodes.get('motionHeadClearanceInstructions').textContent.includes('腿部和头颈'));
  assert(h.nodes.get('motionHeadClearanceLabel').textContent.includes('移动范围内'));
  h.nodes.get('motionHeadClearanceConfirmed').checked=true;
  h.nodes.get('motionSupportConfirmed').checked=true;
  const next=clone(s);next.motion.hardware.state_sample_us+=20000;h.set(next);
  assert.equal(h.nodes.get('motionHeadClearanceConfirmed').checked,true);
  assert.equal(h.nodes.get('motionSupportConfirmed').checked,true);
  assert.equal(h.posts().length,0,'rendering never captures or starts motion');
});

const testWatchdog = setTimeout(() => { process.stderr.write('Static UI tests did not finish; unresolved fake promise.\n'); process.exitCode = 1; }, 10000);
(async () => {
  for (const [name, fn] of tests) { await fn(); process.stdout.write('PASS ' + name + '\n'); }
  process.stdout.write(tests.length + ' static UI tests passed; no device or network access.\n');
})().catch(error => { process.stderr.write(error.stack + '\n'); process.exitCode = 1; }).finally(() => clearTimeout(testWatchdog));
