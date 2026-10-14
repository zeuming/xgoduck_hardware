"""Explicit X5-local ownership of existing bounded motion actions.

The input service alone opens js0. This adapter connects through the existing
workbench only on a fresh explicit action button, never arms at boot,
changes the motion envelope, or retries rejected commands.
USB input freshness is NOT radio-link liveness. Single-button commands use
the installed continuous control API; no timed trial or one-pulse gate is added.
"""
import copy
import collections
import re
import secrets
import threading
import time

import gamepad_web

PRINCIPAL = 'x5-local-gamepad'
HOLD_PRINCIPAL = 'x5-local-recovered-hold'
PERIOD = .05
BEHAVIOR_BUTTONS = {8: 'getup', 9: 'pick', 14: 'roulade'}
BEHAVIOR_LABELS = {'getup': '起身', 'pick': '俯身', 'roulade': '翻滚'}
MODEL_CONTROL_MODES = ('off', 'hold', 'walk', 'getup', 'pick', 'roulade')
OPERATION_BUTTONS = {0, 1, 3, 4, 6, 7, 8, 9, 11, 14}


class ControlMonitor:
    """Bounded request provenance, without sensor reads or lease renewal."""
    ACTIONS = {'start', 'heartbeat', 'command', 'stop', 'connect', 'disconnect',
               'capture-head-clearance', 'forward-trial', 'tracking-probe',
               'gamepad-enable', 'gamepad-disable'}

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.events = collections.deque(maxlen=64)
        self.sequence = 0
        self.sources = {name: dict(requests=0, accepted=0, rejected=0, actions={},
            heartbeat_accepted=0, last_heartbeat_at=None, heartbeat_generation=None,
            maximum_heartbeat_gap_ms=0., maximum_request_ms=0., last_rejection=None)
            for name in ('browser', 'x5_local', 'x5_runtime')}

    def call(self, source, body, callback):
        began = self.clock()
        action = body.get('action') if isinstance(body, dict) else None
        action = action if isinstance(action, str) and action in self.ACTIONS else 'other'
        generation = body.get('generation') if isinstance(body, dict) else None
        generation = generation if type(generation) is int else None
        mode = body.get('mode') if isinstance(body, dict) else None
        mode = mode if mode in ('hold', 'walk', 'off', 'shadow', 'getup', 'pick', 'roulade') else None
        error = None
        with self.lock:
            self.sources[source]['requests'] += 1
        try:
            return callback()
        except Exception as failure:
            error = str(failure)[:240]
            raise
        finally:
            ended = self.clock()
            with self.lock:
                record = self.sources[source]
                record['accepted' if error is None else 'rejected'] += 1
                result = record['actions'].setdefault(action, dict(accepted=0, rejected=0))
                result['accepted' if error is None else 'rejected'] += 1
                record['maximum_request_ms'] = max(record['maximum_request_ms'], (ended-began)*1000)
                if action == 'heartbeat' and error is None:
                    if record['heartbeat_generation'] == generation and record['last_heartbeat_at'] is not None:
                        record['maximum_heartbeat_gap_ms'] = max(record['maximum_heartbeat_gap_ms'],
                            (ended-record['last_heartbeat_at'])*1000)
                    elif record['heartbeat_generation'] != generation:
                        record['maximum_heartbeat_gap_ms'] = 0.
                    record['heartbeat_generation'] = generation
                    record['last_heartbeat_at'] = ended
                    record['heartbeat_accepted'] += 1
                if action != 'heartbeat' or error is not None:
                    self.sequence += 1
                    event = dict(sequence=self.sequence, source=source, action=action, mode=mode,
                        generation=generation, result='accepted' if error is None else 'rejected',
                        recorded_monotonic=ended, request_ms=round((ended-began)*1000, 2), error=error)
                    self.events.append(event)
                    if error is not None:
                        record['last_rejection'] = event

    def snapshot(self, *, compact=False):
        now = self.clock()
        with self.lock:
            sources = copy.deepcopy(self.sources)
            events = copy.deepcopy(list(self.events)[-8:] if compact else list(self.events))
            sequence = self.sequence
        for item in sources.values():
            at = item.pop('last_heartbeat_at')
            item['heartbeat_age_ms'] = None if at is None else round((now-at)*1000, 2)
        return dict(protocol=1, sources=sources, recent_requests=events, sequence=sequence,
                    sampled_monotonic=now, scope='accepted_control_requests_not_motor_execution',
                    browser_emergency_stop_allowed=True, get_requests_do_not_renew_control=True)


class GamepadControl:
    def __init__(self, workbench, *, reader=gamepad_web.snapshot, clock=time.monotonic, monitor=None, standalone=True, shared_buttons=True):
        self.wb, self.reader, self.clock = workbench, reader, clock
        self.monitor = monitor
        self.shared_buttons = shared_buttons
        self.wb.shared_model_buttons = shared_buttons
        self.standalone = standalone
        self.local_id = secrets.token_hex(16)
        self.standby_input = None
        self.lock = threading.RLock()
        self.session = None
        self.origin = None
        self.message = '开机手柄就绪；松开按键后单按 A 默认站姿或 X 零速保持' if standalone else '未接管；手柄输入只用于显示'
        self.last_input = None
        self.last_poll = None
        self.last_heartbeat = None
        self.heartbeat_count = 0
        self.last_input_failure = None
        self.stop_event = threading.Event()
        self.threads = []

    def status(self):
        # Immutable scalar projection: never hold the gamepad command lock
        # while publishing or renewing the separate control lease.
        return dict(protocol=2, active=self.session is not None,
                    shared_model_buttons=self.shared_buttons,
                    behavior_buttons=dict(BEHAVIOR_BUTTONS),
                    standalone_ready=self.standalone and self.standby_input is not None,
                    browser_setup_required=not self.standalone,
                    source='x5_local', requires_browser_heartbeat=False,
                    message=self.message, heartbeat_count=self.heartbeat_count,
                    heartbeat_age_ms=None if self.last_heartbeat is None else
                    round((self.clock()-self.last_heartbeat)*1000, 1),
                    radio_liveness_verified=False,
                    last_input_failure=copy.deepcopy(self.last_input_failure))

    def _context(self):
        context = self.wb.runtime.control_context()
        with self.wb.lock:
            owner = copy.deepcopy(self.wb.control_owner)
        return context, context.get('hardware') or {}, owner

    def _owned(self, session):
        context, hardware, owner = self._context()
        if self.shared_buttons:
            valid = (self.session is session
                and context.get('connection_sequence') == session['connection']
                and context.get('phase') == 'ready' and context.get('owns_connection') is True
                and hardware.get('connected') is True and hardware.get('mode') in MODEL_CONTROL_MODES
                and not hardware.get('joint_follow', {}).get('active'))
        else:
            valid = (self.session is session and owner is not None
                     and owner['id'] == session['id'] and owner['principal'] == PRINCIPAL
                     and owner['generation'] == hardware.get('generation')
                     and context.get('connection_sequence') == session['connection']
                     and context.get('phase') == 'ready' and context.get('owns_connection') is True
                     and hardware.get('connected') is True and hardware.get('mode') in MODEL_CONTROL_MODES)
        return valid, hardware

    def _action(self, session, action, **extra):
        valid, hardware = self._owned(session)
        if not valid:
            raise ValueError('手柄控制权或动作状态已变化；未执行')
        client, principal = session['id'], PRINCIPAL
        if self.shared_buttons and action == 'heartbeat':
            _, _, owner = self._context()
            if not owner:
                return False
            client, principal = owner['id'], owner['principal']
        body = dict(action=action, page_generation=self.wb.generation,
                    generation=hardware['generation'], control_id=client, **extra)
        invoke = lambda: self.wb.motion_action(body, principal=principal, live=True)
        try:
            return invoke() if self.monitor is None else self.monitor.call('x5_local', body, invoke)
        except ValueError:
            if self.shared_buttons and action == 'heartbeat':
                # START/STOP may commit after this heartbeat captured its
                # generation. Never undo the newer explicit command by
                # treating its rejected old heartbeat as an input failure.
                if self.session is not session:
                    return False
                current, latest = self._owned(session)
                if current and latest.get('generation') != body['generation']:
                    return False
            raise

    def request(self, body, principal):
        if (type(body) is not dict or set(body) - {
                'action', 'page_generation', 'generation', 'control_id', 'support_confirmed'}
                or body.get('action') not in ('gamepad-enable', 'gamepad-disable')
                or type(body.get('page_generation')) is not int
                or body['page_generation'] != self.wb.generation
                or type(body.get('generation')) is not int
                or type(body.get('control_id')) is not str
                or re.fullmatch('[0-9a-f]{32}', body['control_id']) is None
                or not principal):
            raise ValueError('手柄请求或页面会话无效')
        if not self.lock.acquire(blocking=False):
            raise ValueError('手柄操作尚未完成；请求未排队')
        try:
            with self.wb.io_lock:
                context, hardware, owner = self._context()
                if self.wb.closed or hardware.get('generation') != body['generation']:
                    raise ValueError('运控状态已变化，请刷新后重试')
                if body['action'] == 'gamepad-disable':
                    if not self.origin or any(self.origin[k] != v for k, v in (
                            ('id', body['control_id']), ('principal', principal))):
                        raise ValueError('请由启用手柄的页面交回控制；停止按钮始终可用')
                    self._revoke('已交回网页；不自动启动动作', recover=True)
                else:
                    if self.session is not None:
                        raise ValueError('手柄已接管，不能重复启用')
                    if body.get('support_confirmed') is not True:
                        raise ValueError('请确认防跌准备，再启用手柄')
                    if (owner is not None and (owner['id'] != body['control_id']
                            or owner['principal'] != principal or owner['generation'] != body['generation'])
                            or owner is None and hardware.get('mode') != 'off'):
                        raise ValueError('请先由本页进入固定站姿，再交给手柄')
                    current = self.wb.runtime.snapshot(compact=True).get('hardware') or {}
                    if (context.get('phase') != 'ready' or context.get('owns_connection') is not True
                            or hardware.get('mode') not in ('off', 'hold') or hardware.get('connected') is not True
                            or current.get('generation') != body['generation']
                            or hardware.get('mode') == 'hold' and (
                                current.get('armed') is not True or current.get('hold_ready') is not True)
                            or hardware.get('mode') == 'off' and current.get('armed') is not False
                            or current.get('pending_stop') or current.get('error')):
                        raise ValueError('须已连接且处于扭矩关闭或已到位固定站姿；接管不会启动动作')
                    input_state = self.reader()
                    if (input_state.get('connected') is not True or any(input_state['buttons'])
                            or any(input_state['axes'][i] != 0 for i in (0, 1, 2, 3, 6, 7))):
                        raise ValueError('接收器须在线；请松开全部按键并让摇杆回中')
                    # OFF takeover does not ARM; only a fresh explicit A press
                    # may request the existing guarded HOLD start afterwards.
                    # io_lock serializes all START/command requests with this.
                    with self.wb.lock:
                        if self.wb.control_owner != owner:
                            raise ValueError('控制权已变化，未接管')
                        self.origin = copy.deepcopy(owner) if owner else dict(
                            id=body['control_id'], principal=principal, generation=body['generation'])
                        session = dict(id=secrets.token_hex(16), connection=context['connection_sequence'],
                                       input_session=input_state['session'])
                        self.wb.control_owner = dict(id=session['id'], principal=PRINCIPAL,
                                                     generation=body['generation'])
                        self.session = session
                    self.last_input = input_state
                    self.last_poll = self.clock()
                    self.last_input_failure = None
                    self.message = 'X5 本地手柄已接管；等待新的按键，不自动启动模型'
                result = self.wb.snapshot(live=True)
                result['motion']['gamepad_control'] = self.status()
                return result
        finally:
            self.lock.release()

    def _revoke(self, reason, *, recover):
        session = self.session
        failure = None
        held_generation = None
        if session is not None:
            try:
                valid, h = self._owned(session)
                if recover and valid and h['mode'] == 'walk':
                    # Use the existing guarded HOLD transition, never STOP.
                    # If a gate rejects, revoke the local heartbeat;
                    # the unchanged controller watchdog handles the fault.
                    self._action(session, 'start', mode='hold', support_confirmed=True)
                    # Capture the resulting generation only after confirmed
                    # HOLD; a refused transition must never renew old WALK.
                    _, held, _ = self._context()
                    if held.get('mode') == 'hold':
                        held_generation = held.get('generation')
                elif recover and valid and h['mode'] == 'hold':
                    held_generation = h.get('generation')
                elif recover and valid and h['mode'] in BEHAVIOR_LABELS:
                    self._action(session, 'stop', stop_source='button')
            except Exception as error:
                failure = str(error)
            finally:
                self.session = None
                try:
                    context, held, _ = self._context()
                except Exception as error:
                    context, held = {}, {}
                    failure = failure or str(error)
                with self.wb.lock:
                    owner = self.wb.control_owner
                    if owner and owner['id'] == session['id'] and owner['principal'] == PRINCIPAL:
                        # X5 adaptation, not an upstream model change: after
                        # input loss the verified HOLD must have a lifetime
                        # owner. Previously origin=None left a gamepad owner
                        # with no gamepad session; both heartbeat providers
                        # skipped it and the existing 1s watchdog released.
                        # Transfer only the same live HOLD generation/connection.
                        # STOP/new commands still invalidate this generation;
                        # CommandRuntime independently rechecks it before renewal.
                        if (failure is None and held_generation is not None
                                and context.get('connection_sequence') == session['connection']
                                and context.get('phase') == 'ready'
                                and context.get('owns_connection') is True
                                and held.get('connected') is True and held.get('mode') == 'hold'
                                and held.get('armed') is True and not held.get('pending_stop')
                                and held.get('generation') == owner['generation'] == held_generation):
                            self.wb.control_owner = (dict(self.origin, generation=held_generation)
                                if self.origin else dict(id=session['id'],
                                    principal=HOLD_PRINCIPAL, generation=held_generation))
        self.standby_input = None
        self.message = reason if failure is None else reason + '；恢复站姿未确认：' + failure
        if failure:
            raise ValueError(self.message)

    def heartbeat_once(self):
        session = self.session
        if session is None or self.last_poll is None or not 0 <= self.clock()-self.last_poll <= .2:
            return
        valid, h = self._owned(session)
        if not valid or h['mode'] not in MODEL_CONTROL_MODES[1:]:
            return
        if self._action(session, 'heartbeat', background=False) is False:
            return
        self.last_heartbeat = self.clock()
        self.heartbeat_count += 1

    def poll_once(self):
        with self.lock:
            session = self.session
            if session is None:
                if not self.standalone:
                    return
                next_state = self.reader()
                previous = self.standby_input
                if next_state.get('connected') is not True:
                    self.standby_input = None
                    return
                # Establish a neutral baseline after boot, STOP or reconnect.
                # Initial/held buttons and historical events cannot start motion.
                if previous is None or previous['session'] != next_state['session']:
                    if not any(next_state['buttons']) and not any(next_state['axes'][i] for i in (0,1,2,3,6,7)):
                        self.standby_input = next_state
                    else:
                        self.standby_input = None
                    return
                try:
                    session = self._standalone_claim(previous, next_state)
                except ValueError as error:
                    self.message = '本次按键未执行：' + str(error)
                    self.standby_input = None
                    return
                if session is None:
                    return
            valid, h = self._owned(session)
            if not valid:
                self._revoke('动作或连接已变化，手柄已退出；不会自动重启', recover=False)
                return
            next_state = self.reader()
            previous = self.last_input
            if (next_state.get('connected') is not True
                    or next_state.get('session') != session['input_session']):
                changed = (next_state.get('connected') is True
                           and next_state.get('session') != session['input_session'])
                detail = next_state.get('error') or ('输入会话已变化' if changed else '接收器未就绪')
                self.last_input_failure = dict(reason=detail, age_ms=next_state.get('age_ms'),
                    recorded_monotonic=self.clock(), input_session=next_state.get('session'),
                    expected_session=session['input_session'], sequence=next_state.get('sequence'))
                instruction = ('再单按 A 或 X' if self.standalone else '再点击「交给手柄」')
                self._revoke('手柄输入校验未通过，已退出：' + detail +
                             '。连接恢复后须松开按键，' + instruction, recover=True)
                return
            events = [e for e in next_state['events'] if e['sequence'] > previous['sequence']]
            count = next_state['sequence']-previous['sequence']
            expected = list(range(previous['sequence']+1, next_state['sequence']+1)) if 0 <= count <= 64 else None
            if (next_state['sequence'] < previous['sequence'] or expected is None
                    or [e['sequence'] for e in events] != expected
                    or any(not 0 <= e['age_ms'] <= 300 for e in events)):
                self._revoke('输入事件过期或缺失，手柄已退出；不会回放旧按键', recover=True)
                return
            axes, buttons = list(previous['axes']), list(previous['buttons'])
            for event in events:
                (buttons if event['kind'] == 1 else axes)[event['number']] = event['value']
            if axes != next_state['axes'] or buttons != next_state['buttons']:
                self._revoke('输入状态不连续，手柄已退出', recover=True)
                return
            self.last_input = next_state
            self.last_poll = self.clock()
            pressed = {e['number'] for e in events if e['kind'] == 1 and e['value'] == 1}
            self._buttons(session, h, pressed)

    def _buttons(self, session, h, pressed):
        # Explicit emergency release wins over every other event. Normal
        # cancellation never sends STOP. No command is retried.
        if 10 in pressed:
            try:
                self._action(session, 'stop', stop_source='button')
                self.message = 'SELECT：已确认停止并释放扭矩，请托稳机身'
            finally:
                self.session = None
                self.standby_input = None
            return
        if self.shared_buttons:
            if len(pressed & OPERATION_BUTTONS) > 1:
                self.message = '同时按下多个操作键，本次未执行；请单独按一个按钮'
                return
            try:
                if self._connection_button(pressed, self.last_input):
                    return
            except ValueError as error:
                self.message = '本次按键未执行：' + str(error)
                return
        if h['mode'] in BEHAVIOR_LABELS and pressed & OPERATION_BUTTONS:
            self.message = '本轮模型动作尚未结束；需要中止请按 SELECT'
            return
        behaviors = pressed & BEHAVIOR_BUTTONS.keys()
        if behaviors:
            if len(pressed & OPERATION_BUTTONS) != 1:
                self.message = '同时按下多个操作键，本次未执行；请单独按一个按钮'
                return
            mode = BEHAVIOR_BUTTONS[next(iter(behaviors))]
            if h['mode'] not in ('off', 'hold', 'walk'):
                self.message = '本轮模型动作尚未结束；需要中止请按 SELECT'
                return
            try:
                # A fresh dedicated key is the same explicit confirmation as
                # the page checkbox. Admission/limits stay in start_mode.
                self._action(session, 'start', mode=mode,
                    support_confirmed=True, behavior_confirmed=True)
                self.message = '已请求' + BEHAVIOR_LABELS[mode] + '模型 · 单次；结束后停止卸力，SELECT 可中止'
            except ValueError as error:
                self.message = '本次按键未执行：' + str(error)
            return
        if 11 in pressed and (self.shared_buttons or self.origin is None):
            self._action(session, 'start', mode='hold', support_confirmed=True)
            self.message = 'START：已回到默认站姿；继续由手柄控制'
            return
        if 11 in pressed:
            self._revoke('已回到默认站姿并交回网页；不自动启动行走', recover=True)
            return
        if 1 in pressed:
            if h['mode'] == 'walk':
                self._action(session, 'command', forward=0, side=0, yaw=0)
                session['command'] = (0., 0., 0.)
                self.message = '已取消前进，回到零速；模型继续平衡'
            return
        try:
            actions = pressed & {0, 3, 4}
            if len(actions) > 1:
                self.message = '同时按下多个动作键，本次未执行；请单独按一个按钮'
                return
            if 0 in actions:
                self._action(session, 'start', mode='hold', support_confirmed=True)
                self.message = '已请求缓慢进入默认固定站姿；实际到位以关节反馈为准'
            elif 3 in actions and h['mode'] in ('off', 'hold', 'walk'):
                if h['mode'] == 'walk':
                    self._action(session, 'command', forward=0, side=0, yaw=0)
                else:
                    self._action(session, 'start', mode='walk', support_confirmed=True)
                session['command'] = (0., 0., 0.)
                self.message = '已启动零速保持；使用现有行走模型和保护'
            elif 4 in actions and h['mode'] == 'walk':
                current_h = (self.wb.runtime.snapshot(compact=True).get('hardware') or {})
                if current_h.get('generation') != h.get('generation'):
                    raise ValueError('运控状态已变化，请重新单按按钮')
                current = current_h.get('command')
                moving = (any(current.get(k, 0) != 0 for k in ('forward', 'side', 'yaw'))
                          if isinstance(current, dict) else any(session.get('command', (0., 0., 0.))))
                target = (0., 0., 0.) if moving else (.3, 0., 0.)
                self._action(session, 'command', forward=target[0], side=target[1], yaw=target[2])
                session['command'] = target
                self.message = '已回零速；模型继续平衡' if moving else '已请求持续前进 0.30 m/s；再次按 Y 或按 B 回零速'
            elif actions:
                self.message = '本次按键未执行：先进入默认站姿，再启动零速保持，最后触发前进'
        except ValueError as error:
            self.message = '本次按键未执行：' + str(error)

    def _standalone_claim(self, previous, next_state):
        events = [e for e in next_state['events'] if e['sequence'] > previous['sequence']]
        count = next_state['sequence'] - previous['sequence']
        if (not 0 <= count <= 64 or [e['sequence'] for e in events] !=
                list(range(previous['sequence']+1, next_state['sequence']+1)) or
                any(not 0 <= e['age_ms'] <= 300 for e in events)):
            raise ValueError('输入事件过期或缺失；请松开按键后重试')
        axes, buttons = list(previous['axes']), list(previous['buttons'])
        for e in events:
            (buttons if e['kind'] == 1 else axes)[e['number']] = e['value']
        if axes != next_state['axes'] or buttons != next_state['buttons']:
            raise ValueError('输入状态不连续；请松开按键后重试')
        self.standby_input = next_state
        pressed = {e['number'] for e in events if e['kind'] == 1 and e['value'] == 1}
        actions = pressed & (OPERATION_BUTTONS if self.shared_buttons else {0,3,4,8,9,14})
        if 10 not in pressed and (len(actions) > 1 or not (pressed & (OPERATION_BUTTONS if self.shared_buttons else {0,3,8,9,14}))):
            if len(actions) > 1:
                self.message = '同时按下多个操作键，本次未执行；请单独按一个按钮'
            return None
        # STOP bypasses io_lock just like the workbench emergency path, so a
        # browser START preflight cannot delay the local emergency request.
        if 10 in pressed:
            body = dict(action='stop', page_generation=self.wb.generation,
                        control_id=self.local_id, stop_source='button')
            invoke = lambda: self.wb.motion_action(body, principal=PRINCIPAL, live=True)
            invoke() if self.monitor is None else self.monitor.call('x5_local', body, invoke)
            self.message = 'SELECT：已确认停止并释放扭矩，请托稳机身'
            self.standby_input = None
            return None
        if self.shared_buttons and self._connection_button(pressed, next_state):
            return None
        with self.wb.io_lock:
            context, h, owner = self._context()
            recovered_hold = (owner and owner['id'] == self.local_id
                and owner['principal'] == HOLD_PRINCIPAL and h.get('mode') == 'hold'
                and owner['generation'] == h.get('generation'))
            if not self.shared_buttons and owner and not recovered_hold and (owner['id'] != self.local_id or owner['principal'] != PRINCIPAL):
                raise ValueError('当前由网页或其他会话控制；请先明确停止')
            if context.get('owns_connection') is not True:
                if not pressed & ({0,3} | BEHAVIOR_BUTTONS.keys()):
                    raise ValueError('请先按 A、X 或模型动作键连接并启动对应模式')
                self._review_unopened_startup_fault(next_state)
                body = dict(action='connect', page_generation=self.wb.generation, control_id=self.local_id)
                invoke = lambda: self.wb.motion_action(body, principal=PRINCIPAL, live=True)
                invoke() if self.monitor is None else self.monitor.call('x5_local', body, invoke)
                context, h, owner = self._context()
            if (context.get('phase') != 'ready' or context.get('owns_connection') is not True or
                    h.get('connected') is not True or h.get('mode') not in (MODEL_CONTROL_MODES if self.shared_buttons else ('off','hold')) or
                    h.get('pending_stop') or (self.shared_buttons and h.get('joint_follow', {}).get('active'))
                    or (h.get('error') and not self.shared_buttons) or
                    h.get('mode') == 'off' and (h.get('release_confirmed') is not True
                        or h.get('armed') is True)):
                raise ValueError('连接或扭矩状态未就绪；保留现有保护')
            latest = self.reader()
            if not self._same_press_or_release(next_state, latest):
                raise ValueError('连接期间输入已变化；未启动，请重新单按动作键')
            with self.wb.lock:
                if self.wb.control_owner != owner or self.wb.closed:
                    raise ValueError('控制权已变化；未启动')
                session = dict(id=self.local_id, connection=context['connection_sequence'],
                               input_session=next_state['session'])
                if not self.shared_buttons:
                    self.wb.control_owner = dict(id=self.local_id, principal=PRINCIPAL, generation=h['generation'])
                self.origin = None
                self.session = session
            self.last_input = latest
            self.last_poll = self.clock()
            self._buttons(session, h, pressed)
            return session

    def _connection_button(self, pressed, input_state):
        """Aliases for the page's connect/disconnect, with no model START."""
        if not pressed & {6,7}:
            return False
        action = 'disconnect' if 7 in pressed else 'connect'
        with self.wb.io_lock:
            context, _, _ = self._context()
            if action == 'connect' and context.get('owns_connection') is True:
                self.message = 'L1：模型已加载、运控已连接；未启动动作'
                return True
            if action == 'connect':
                self._review_unopened_startup_fault(input_state)
            body = dict(action=action, page_generation=self.wb.generation,
                        control_id=self.local_id)
            invoke = lambda: self.wb.motion_action(body, principal=PRINCIPAL, live=True)
            try:
                invoke() if self.monitor is None else self.monitor.call('x5_local', body, invoke)
                self.message = ('L1：已加载并连接运控；保持扭矩关闭，未启动动作'
                                if action == 'connect' else
                                'R1：已停止并断开运控；与网页卸载操作一致')
            finally:
                if action == 'disconnect':
                    self.session = None
                    self.standby_input = None
        return True

    @staticmethod
    def _same_press_or_release(before, after):
        if after.get('connected') is not True or after['session'] != before['session']:
            return False
        if after['sequence'] == before['sequence']:
            return after['axes'] == before['axes'] and after['buttons'] == before['buttons']
        # A normal brief button press may be released during connection checks.
        # Accept only that one release; any new press (including SELECT), gap,
        # axis change or input-session change cancels the pending start.
        events = [e for e in after['events'] if e['sequence'] > before['sequence']]
        if after['sequence'] != before['sequence'] + 1 or len(events) != 1:
            return False
        event = events[0]
        if (event['sequence'] != after['sequence'] or event['kind'] != 1 or event['value'] != 0
                or event['number'] not in OPERATION_BUTTONS or before['buttons'][event['number']] != 1):
            return False
        buttons = list(before['buttons']);buttons[event['number']] = 0
        return after['buttons'] == buttons and after['axes'] == before['axes']

    def _review_unopened_startup_fault(self, input_state):
        service = self.wb.calibration_service
        if service is None:
            return
        session = service.session
        with session.lock:
            # Only the existing, narrowly scoped unopened trust-fault review.
            # Mapping changes and faults from a live maintenance session remain
            # blocked. The callback needs all 15 torque-OFF rows plus closure.
            maintenance_review = session._maintenance_can_reconcile_unopened_trust()
            if not maintenance_review:
                hold = session.recovery_recheck
                if (not isinstance(hold, dict) or hold.get('category') != 'trust' or
                        session.maintenance_fault_hold):
                    return
                blocked = session.connection_check_blocked_reason()
                if blocked:
                    raise ValueError(blocked)
            generation = session.generation
        self.message = '正在复查开机连接与舵机反馈；检查期间保持扭矩关闭'
        if maintenance_review:
            self.wb.calibration_action('/api/servo-maintenance',
                dict(action='review', generation=generation), principal=PRINCIPAL)
        else:
            self.wb.calibration_action('/api/check-connection',
                dict(purpose='joints', generation=generation), principal=PRINCIPAL)
        deadline = time.monotonic() + 15
        while True:
            if not maintenance_review:
                with session.lock:
                    checked = copy.deepcopy(session.feedback_recheck or {})
                    checking = copy.deepcopy(session.connection_check)
                if checking.get('phase') == 'passed' and checked.get('phase') == 'passed':
                    service.end_for_motion()
                    state = {}
                    break
                if checking.get('phase') == 'failed' or (checking.get('phase') == 'passed' and checked.get('phase') == 'failed'):
                    raise ValueError(checked.get('error') or checking.get('error') or '连接复查未通过')
                if time.monotonic() >= deadline:
                    raise ValueError('连接复查尚未完成；本次未启动，不会自动重试')
                time.sleep(.05)
                continue
            state = session.maintenance.snapshot()
            worker = getattr(session.maintenance, '_thread', None)
            if (not state.get('owns_connection') and not state.get('busy') and
                    (worker is None or not worker.is_alive())):
                break
            if time.monotonic() >= deadline:
                raise ValueError('连接复查尚未完成；本次未启动，不会自动重试')
            time.sleep(.05)
        with session.lock:
            if session.maintenance_fault_hold or session.recovery_recheck is not None:
                raise ValueError(state.get('error') or '连接复查未通过；故障锁保留')
        latest = self.reader()
        if not self._same_press_or_release(input_state, latest):
            raise ValueError('复查期间输入已变化；未启动，请重新单按动作键')

    def start(self):
        def run(callback, period):
            while not self.stop_event.wait(period):
                try:
                    callback()
                except Exception as error:
                    with self.lock:
                        try:
                            self._revoke('手柄控制中断：' + str(error), recover=True)
                        except Exception:
                            pass
        for callback, period in ((self.poll_once, PERIOD), (self.heartbeat_once, .1)):
            thread = threading.Thread(target=run, args=(callback, period), daemon=True)
            self.threads.append(thread)
            thread.start()

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=.5)
