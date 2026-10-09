#!/usr/bin/env node
// Browser-free checks of the actual inline UI script. Every DOM, timer and HTTP
// surface is mocked; this test never opens a browser or contacts hardware.
'use strict';

const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, 'index.html'), 'utf8');
const scripts = [...html.matchAll(/<script\b[^>]*>([\s\S]*?)<\/script>/g)];
assert.equal(scripts.length, 1, 'expected one inline application script');
const application = new vm.Script(scripts[0][1], {filename: 'index.html:inline'});
const ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34];

class Element {
  constructor(tag = 'div') {
    this.tagName = tag.toUpperCase();
    this.children = [];
    this.dataset = {};
    this.attributes = {};
    this.value = '0';
    this.min = '-1';
    this.max = '1';
    this.checked = false;
    this.disabled = false;
    this.hidden = false;
    this.textContent = '';
    this.listeners = new Map();
    this.classes = new Set();
    this.classList = {
      toggle: (name, force) => {
        const present = force === undefined ? !this.classes.has(name) : !!force;
        if (present) this.classes.add(name); else this.classes.delete(name);
        return present;
      },
      add: (...names) => names.forEach(name => this.classes.add(name)),
      remove: (...names) => names.forEach(name => this.classes.delete(name)),
      contains: name => this.classes.has(name),
    };
    this.style = {setProperty: (name, value) => { this.style[name] = value; }};
  }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  addEventListener(name, callback) {
    if (!this.listeners.has(name)) this.listeners.set(name, []);
    this.listeners.get(name).push(callback);
  }
  querySelector(selector) {
    const match = this.children.find(child => child.tagName?.toLowerCase() === selector);
    if (match) return match;
    const child = new Element(selector);
    this.append(child);
    return child;
  }
}

function fixture(overrides = {}) {
  return {
    connected: true, mode: 'off', generation: 7, calibrated: false,
    mechanical_reference_confirmed: false, hold_ready: false,
    armed: false, configured: false, motor_output: false,
    port: '/dev/mock-only', protocol: 1, age_ms: 5, imu_age_ms: 5,
    online_ids: ids.slice(), torque_ids: [], fault_ids: [],
    joints: ids.map(id => ({id, zero: null, min: null, max: null,
      sign: -1, direction_confirmed: false, ready: false})),
    rawdata: ids.map(id => ({id, position: 2047, angle_deg: null})),
    imu: {axes: ['x', 'y', 'z'], axes_confirmed: false, calibrated: false,
      calibrating: false, progress: 0, bias_dps: [0, 0, 0]},
    command: {forward: 0, side: 0, yaw: 0}, rates: {state_hz: 50, control_hz: 0},
    heartbeat_remaining_ms: 400, error: null,
    ...overrides,
  };
}

function harness() {
  const elements = new Map();
  for (const match of html.matchAll(/<([a-z][a-z\d]*)\b([^>]*\bid="([^"]+)"[^>]*)>/gi)) {
    const element = new Element(match[1]);
    for (const attribute of match[2].matchAll(/([\w-]+)="([^"]*)"/g)) {
      element.setAttribute(attribute[1], attribute[2]);
      if (['value', 'min', 'max'].includes(attribute[1])) element[attribute[1]] = attribute[2];
    }
    elements.set(match[3], element);
  }
  const windowListeners = new Map();
  const requests = [];
  const timers = new Map();
  let nextTimer = 0;
  let responder = () => { throw new Error('unexpected mocked HTTP request'); };
  const document = {
    hidden: false, focused: true, activeElement: null, listeners: new Map(),
    getElementById: id => {
      assert(elements.has(id), 'application references a missing DOM ID: ' + id);
      return elements.get(id);
    },
    createElement: tag => new Element(tag),
    createTextNode: text => ({textContent: String(text)}),
    hasFocus() { return this.focused; },
    addEventListener(name, callback) {
      if (!this.listeners.has(name)) this.listeners.set(name, []);
      this.listeners.get(name).push(callback);
    },
  };
  const context = vm.createContext({
    document,
    window: {addEventListener(name, callback) {
      if (!windowListeners.has(name)) windowListeners.set(name, []);
      windowListeners.get(name).push(callback);
    }},
    navigator: {clipboard: {writeText: async () => {}}},
    AbortController,
    performance: {now: () => 0},
    setTimeout(callback, ms) { const id = ++nextTimer; timers.set(id, {callback, ms}); return id; },
    setInterval(callback, ms) { const id = ++nextTimer; timers.set(id, {callback, ms, interval: true}); return id; },
    clearTimeout(id) { timers.delete(id); },
    fetch: async (url, options = {}) => {
      const request = {url, options, body: options.body ? JSON.parse(options.body) : undefined};
      requests.push(request);
      const body = await (url === '/api/status' ? {
        online: true, token: 'mock-session', models: [{id: 'walk', ready: true}],
        device: {hostname: 'mock-only'},
      } : responder(request));
      return {ok: true, status: 200, json: async () => body};
    },
  });
  application.runInContext(context);
  const hooks = vm.runInContext('({state,motion,receiveMotion,renderMotion,startMotion,stopMotion,heartbeatMotion})', context);
  return {
    hooks, elements, document, requests,
    respond(callback) { responder = callback; },
    async settle() { for (let i = 0; i < 4; ++i) await new Promise(resolve => setImmediate(resolve)); },
    dispatchWindow(name) { for (const callback of windowListeners.get(name) || []) callback(); },
    get(id) { return elements.get(id); },
  };
}

async function run() {
  const h = harness();
  await h.settle();
  assert.equal(h.hooks.state.token, 'mock-session');
  assert.equal(h.get('calibrationRows').children.length, 15);
  h.hooks.motion.tab = true;
  h.hooks.receiveMotion(fixture());
  assert.equal(h.get('shadowMotion').disabled, false, 'shadow diagnostics must work before calibration');
  assert.equal(h.get('holdMotion').disabled, true, 'uncalibrated hardware must not enable hold');
  assert.equal(h.get('walkMotion').disabled, true);
  assert.equal(h.get('supportConfirmed').checked, false);

  let remote = fixture();
  h.respond(request => {
    if (request.url === '/api/motion/start') {
      assert.equal(request.body.mode, 'shadow');
      assert.equal(request.body.generation, 7, 'start must carry the last observed server generation');
      assert.equal(request.body.support_confirmed, false);
      assert.equal(request.body.zero_reference_confirmed, false);
      remote = fixture({mode: 'shadow', generation: 8});
      return remote;
    }
    if (request.url === '/api/motion/heartbeat') return remote;
    if (request.url === '/api/motion/stop') return remote = fixture({generation: 9});
    throw new Error('unexpected request: ' + request.url);
  });
  await h.hooks.startMotion('shadow');
  assert.equal(h.hooks.motion.owned, true, 'shadow requires page ownership for its heartbeat');
  assert.equal(h.hooks.motion.status.mode, 'shadow');
  const heartbeatBefore = h.requests.filter(request => request.url.endsWith('/heartbeat')).length;
  await h.hooks.heartbeatMotion();
  assert.equal(h.requests.filter(request => request.url.endsWith('/heartbeat')).length, heartbeatBefore + 1);

  h.document.focused = false;
  h.dispatchWindow('blur');
  await h.settle();
  assert.equal(h.hooks.motion.status.mode, 'off', 'blur must stop a shadow session too');
  assert.equal(h.hooks.motion.owned, false);
  const blurStop = h.requests.filter(request => request.url.endsWith('/stop')).at(-1);
  assert.equal(blurStop.options.keepalive, true);
  assert.equal(blurStop.options.headers['X-Session-Token'], 'mock-session');

  // A deliberately reordered mocked server reply checks the client backstop.
  // The real server also rejects the stale generation before ARM.
  h.document.focused = true;
  h.hooks.receiveMotion(fixture({generation: 20}));
  let resolveStart;
  let stopCount = 0;
  h.respond(request => {
    if (request.url === '/api/motion/start') {
      assert.equal(request.body.generation, 20);
      return new Promise(resolve => { resolveStart = resolve; });
    }
    if (request.url === '/api/motion/stop') return fixture({generation: 21 + stopCount++});
    throw new Error('unexpected request: ' + request.url);
  });
  const pendingStart = h.hooks.startMotion('shadow');
  assert.equal(typeof resolveStart, 'function');
  await h.hooks.stopMotion();
  resolveStart(fixture({mode: 'shadow', generation: 21}));
  await pendingStart;
  await h.settle();
  assert.equal(stopCount, 2, 'a stale start response must send another STOP');
  assert.equal(h.hooks.motion.status.mode, 'off');
  assert.equal(h.hooks.motion.owned, false);

  // Complete calibration can open hold; walking still requires measured hold readiness.
  const calibrated = fixture({generation: 50, calibrated: true, configured: true,
    mechanical_reference_confirmed: true,
    joints: ids.map(id => ({id, zero: 2047, min: 100, max: 3990, sign: -1,
      direction_confirmed: true, ready: true})),
    imu: {axes: ['x', 'y', 'z'], axes_confirmed: true, calibrated: true,
      calibrating: false, progress: 0, bias_dps: [0, 0, 0]},
  });
  h.get('supportConfirmed').checked = true;
  h.hooks.receiveMotion(calibrated);
  assert.equal(h.get('holdMotion').disabled, false);
  h.respond(request => {
    if (request.url === '/api/motion/start') {
      assert.equal(request.body.mode, 'hold');
      assert.equal(request.body.generation, 50);
      assert.equal(request.body.support_confirmed, true);
      assert.equal(request.body.zero_reference_confirmed, true);
      return {...calibrated, mode: 'hold', generation: 51, armed: true, motor_output: true,
        torque_ids: ids.slice()};
    }
    if (request.url === '/api/motion/stop') return {...calibrated, generation: 52};
    throw new Error('unexpected request: ' + request.url);
  });
  await h.hooks.startMotion('hold');
  assert.equal(h.get('walkMotion').disabled, true, 'elapsed time alone must not open walking');
  h.hooks.receiveMotion({...h.hooks.motion.status, hold_ready: true});
  assert.equal(h.get('walkMotion').disabled, false);
  h.document.focused = false;
  h.dispatchWindow('blur');
  await h.settle();
  assert.equal(h.hooks.motion.status.mode, 'off', 'blur must stop energized hold');

  console.log('PASS UI syntax and mocked DOM: pre-calibration shadow, generation payload, shadow heartbeat, blur STOP, stale-start STOP, calibrated hold and measured walk gate');
}

run().catch(error => { console.error(error); process.exitCode = 1; });
