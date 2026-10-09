'use strict';
// Actual workspace navigation with fake DOM/fetch; no hardware or network.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../x5_recovery/feedback_calibration.html'), 'utf8');
const routes = html.match(/const workspaceRoutes=\[[\s\S]*?\n\];/)[0];
const navigation = ['renderWorkspaceNavigation', 'selectWorkspace'].map(name =>
  html.match(new RegExp('function '+name+'\\([^)]*\\)\\{[\\s\\S]*?\\n\\}'))[0]).join('\n');
const source = fs.readFileSync(path.join(__dirname, 'native_process.py'), 'utf8')
  .match(/LOG_SCRIPT = r'''([\s\S]*?)'''/)[1];

class Node {
  constructor(id) { this.id = id; this.children = []; this.listeners = {}; this.dataset = {}; this.attributes = {}; this.disabled = false; this.hidden = false; this.textContent = ''; const classes=new Set();this.classList={toggle:(name,on)=>on?classes.add(name):classes.delete(name),contains:name=>classes.has(name)}; }
  append(...items) { for(const item of items){item.parentElement=this;this.children.push(item);} }
  replaceChildren(...items) { this.children = items; }
  setAttribute(name, value) { this.attributes[name] = value; }
  addEventListener(name, fn) { this.listeners[name] = fn; }
  click() { if (!this.disabled) {this.onclick?.();this.listeners.click?.();} }
}
function harness({hash='',mode='off',owns=false,independentHold=false}={}) {
  const ids = [...html.matchAll(/\bid="([^"]+)"/g)].map(match=>match[1]).concat(['runtimeLogPanel','runtimeLogStatus','runtimeLogList','runtimeLogPage',
    'runtimeLogLatest','runtimeLogNewer','runtimeLogOlder','runtimeLogEmpty','runtimeLogToggle']);
  const nodes = new Map(ids.map(id => [id, new Node(id)]));
  const main = new Node('main'), events = {}, timers = new Map(), requests = [], answers = [], motionCalls=[];
  nodes.get('runtimeLogPanel').hidden=true;nodes.get('zeroEditorHome').append(nodes.get('zeroEditor'));
  let timerId = 0;
  const document = { hidden:false, getElementById:id=>{assert(nodes.has(id),id);return nodes.get(id);}, querySelector:selector=>selector==='.workspace main'?main:null,
    createElement:tag=>new Node(tag), addEventListener:(name,fn)=>{events[name]=fn;} };
  const context = vm.createContext({ document, el:document.getElementById, AbortController, console, location:{hash},
    window:{addEventListener:(name,fn)=>{events[name]=fn;}},
    renderRecovery(){},renderZeroCalibration(){},renderMotion(){},
    motionIndependentHold:()=>independentHold,ownsMotionControl:()=>owns,motionControlHardware:()=>({mode}),
    snapshot:{motion:{owns_connection:owns}},runMotion:(action,body)=>motionCalls.push({action,body}),
    fetch:(url,options)=>{ requests.push({url,options}); return answers.length ? answers.shift() : Promise.resolve({ok:true,json:async()=>({items:[],next_cursor:null})}); },
    setTimeout:(fn,ms)=>{const id=++timerId;timers.set(id,{fn,ms});return id;},
    clearTimeout:id=>timers.delete(id),
  });
  vm.runInContext(routes+"\nlet workspaceView='motion-models';\n"+navigation,context);
  vm.runInContext(source, context);
  const toggle = nodes.get('runtimeLogToggle');
  const answer = (items, next=null)=>answers.push(Promise.resolve({ok:true,json:async()=>({items,next_cursor:next})}));
  const drain = async()=>{for(let i=0;i<8;i++)await Promise.resolve();};
  const navigate=hash=>{context.location.hash=hash;vm.runInContext('selectWorkspace(location.hash)',context);};
  return {nodes,main,document,events,timers,requests,answers,motionCalls,toggle,answer,drain,navigate,eval:code=>vm.runInContext(code,context)};
}

(async()=>{
  const h = harness();
  assert.equal(h.requests.length, 0, 'other workspaces must not request logs');
  assert.equal(h.eval('workspaceView'),'motion-models','preserve native default workspace');
  h.answer([{event:'停止动作',state:'complete',mode:'off',time:'2026-10-08T12:00:00Z',message:'<img onerror=unsafe()>',level:'info'}], 'older-cursor');
  h.toggle.click();await h.drain();
  assert.equal(h.requests[0].url, '/api/logs/latest');
  assert.equal(h.eval('workspaceView'),'runtime-logs');
  assert.equal(h.main.dataset.runtimeLogs,'true');
  assert.equal(h.nodes.get('workspaceTitle').textContent,'运行日志');
  assert.equal(h.nodes.get('runtimeLogPanel').hidden,false);
  assert.equal(h.nodes.get('motion-models').hidden,true);
  assert.equal(h.toggle.attributes['aria-current'],'page');
  assert(h.toggle.classList.contains('active'));
  assert.equal(h.nodes.get('runtimeLogList').children[0].children[2].textContent, 'off · 完成 · <img onerror=unsafe()>');
  assert(h.nodes.get('runtimeLogList').children[0].children[0].textContent.includes('20:00:00'), 'display uses Shanghai time');
  assert([...h.timers.values()].some(timer=>timer.ms===5000));
  const requestsBefore=h.requests.length;h.eval('renderWorkspaceNavigation()');await h.drain();
  assert.equal(h.requests.length,requestsBefore,'display renders do not restart log reads');
  h.answer([{event:'连接运控',state:'complete'}]);
  h.nodes.get('runtimeLogOlder').click();await h.drain();
  assert.equal(h.requests.at(-1).url, '/api/logs/before/older-cursor');
  assert.equal(h.nodes.get('runtimeLogPage').textContent, '第2页');
  assert(![...h.timers.values()].some(timer=>timer.ms===5000), 'older pages pause refresh');
  assert(h.nodes.get('runtimeLogStatus').textContent.includes('历史页暂停刷新'));
  h.answer([]);h.nodes.get('runtimeLogNewer').click();await h.drain();
  assert.equal(h.requests.at(-1).url, '/api/logs/latest');
  assert.equal(h.nodes.get('runtimeLogPage').textContent, '第1页');
  assert.equal(h.nodes.get('runtimeLogEmpty').hidden,false);
  h.navigate('#motion-models');
  assert.equal(h.nodes.get('runtimeLogPanel').hidden, true);
  assert.equal(h.main.dataset.runtimeLogs,'false');
  assert.equal(h.nodes.get('motion-models').hidden,false);
  assert(!h.toggle.classList.contains('active'));
  assert(![...h.timers.values()].some(timer=>timer.ms===5000));

  const pending = harness();let resolve;
  pending.answers.push(new Promise(done=>{resolve=done;}));
  pending.toggle.click();const signal=pending.requests[0].options.signal;
  pending.navigate('#device-status');
  assert(signal.aborted, 'leaving cancels in-flight fetch');
  resolve({ok:true,json:async()=>({items:[{event:'late'}]})});await pending.drain();
  assert.equal(pending.nodes.get('runtimeLogList').children.length, 0, 'late reply cannot paint inactive workspace');
  assert(![...pending.timers.values()].some(timer=>timer.ms===5000));

  const hidden = harness();hidden.answer([]);hidden.toggle.click();await hidden.drain();
  hidden.document.hidden=true;hidden.events.visibilitychange();
  assert(![...hidden.timers.values()].some(timer=>timer.ms===5000));
  const before=hidden.requests.length;hidden.document.hidden=false;hidden.events.visibilitychange();await hidden.drain();
  assert.equal(hidden.requests.length, before+1, 'visible latest page resumes');
  assert(hidden.requests.every(request=>request.url.startsWith('/api/logs/')), 'log reads never send a motion command');
  assert.equal(hidden.motionCalls.length,0);
  const direct=harness({hash:'#runtime-logs'});await direct.drain();
  assert.equal(direct.eval('workspaceView'),'runtime-logs','log URL opens full workspace');
  assert.equal(direct.requests.length,1);
  const hold=harness({mode:'hold',owns:true,independentHold:true});hold.toggle.click();await hold.drain();
  assert.equal(hold.motionCalls.length,0,'independent fixed stance retains existing behavior');
  const walking=harness({mode:'walk',owns:true});walking.toggle.click();await walking.drain();
  assert.equal(walking.motionCalls.length,1,'walking retains existing workspace leave guard');
  assert.equal(walking.motionCalls[0].action,'stop');
  assert.equal(walking.motionCalls[0].body.stop_source,'workspace-leave');
  console.log('Full log workspace navigation, leave guard, pagination, cancellation, visibility and safe rendering passed.');
})().catch(error=>{console.error(error);process.exitCode=1;});
