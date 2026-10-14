"""Pure, passive single-joint calibration math; no device or transport access."""
from __future__ import annotations

import copy
import math

IDS = (10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34)
from model_reference import WALK_DEFAULT_DEG
DEFAULT_DEG = WALK_DEFAULT_DEG
# Endpoints are the user's explicitly selected comfortable safe positions.
# No automatic expansion of existing bounds, nor unmeasured 12-tick trimming.
MARGIN = 0


class RangeValidationError(ValueError):
    """Stable ordinary-joint endpoints failed geometry, not feedback health."""
    def __init__(self, reason, *, sid, a, b, low, high, minimum, maximum, zero, stand, kind):
        self.details = dict(id=sid, range_a=a, range_b=b, raw_min=low, raw_max=high,
                            trimmed_min=minimum, trimmed_max=maximum, zero=zero,
                            stand=stand, kind=kind, reason=reason)
        super().__init__(f'{reason}。采样 A={a}、B={b}；所选范围 {minimum}–{maximum}；'
                         f'原零点 {zero}；默认站姿 {stand}。可重采安全位置 A 或 B，未自动保存。')


def zero_signature(config):
    return tuple((joint['id'], joint['zero']) for joint in config['joints'])


def position(value):
    if type(value) is not int or not 0 <= value <= 4095:
        raise ValueError('关节编码器值无效')
    return value


def infer_sign(start, end):
    delta = position(end) - position(start)
    if not 0 < abs(delta) < 2048:
        raise ValueError('两次读数必须不同且不跨半圈；请按已确认正方向轻摆后重采')
    return 1 if delta > 0 else -1


def safe_limits(a, b, zero, sign, degrees, sid=None):
    if sid is not None and (type(sid) is not int or sid not in IDS):
        raise ValueError('未知关节 ID')
    low, high = sorted((position(a), position(b)))
    position(zero)
    if (type(sign) is not int or sign not in (-1, 1) or
            type(degrees) not in (int, float) or not math.isfinite(degrees)):
        raise ValueError('模型关节方向或默认站姿无效')
    stand = int(round(zero + sign * degrees * 4095. / 360.))
    minimum, maximum = low, high
    if sid == 34 and max((a-zero)*sign, (b-zero)*sign) <= 0:
        raise ValueError('嘴部范围没有包含沿已确认正方向张开的读数，未保存')
    if minimum >= maximum or not minimum <= zero <= maximum:
        reason = ('两侧位置相同，范围为空，本关节未保存' if minimum >= maximum else
                  '所选范围未包含原零点，本关节未保存')
        if sid != 34:
            raise RangeValidationError(reason, sid=sid, a=a, b=b, low=low, high=high,
                                       minimum=minimum, maximum=maximum, zero=zero, stand=stand,
                                       kind='empty_range' if minimum >= maximum else 'zero_outside')
        raise ValueError(reason)
    if not minimum <= stand <= maximum:
        reason = '默认站姿超出所选范围，本关节未保存；请核对方向与两侧安全位置'
        if sid != 34:
            raise RangeValidationError(reason, sid=sid, a=a, b=b, low=low, high=high,
                                       minimum=minimum, maximum=maximum, zero=zero, stand=stand,
                                       kind='stand_outside')
        raise ValueError(reason)
    return minimum, maximum


def completed_count(config):
    count = 0
    for joint, degrees in zip(config['joints'], DEFAULT_DEG):
        minimum, maximum, zero = (joint[key] for key in ('min', 'max', 'zero'))
        if (joint['direction_confirmed'] is True and type(minimum) is int and type(maximum) is int
                and type(zero) is int and minimum < maximum and minimum <= zero <= maximum):
            stand = int(round(zero + joint['sign'] * degrees * 4095. / 360.))
            count += int(minimum <= stand <= maximum)
    return count


class JointWizard:
    def __init__(self):
        self.clear()

    def clear(self, stage='idle', message='选择一个关节，先记录起点，再按正方向图轻摆。', error=None):
        self.value = dict(id=None, stage=stage, busy=False, message=message, error=error,
                          sample_count=0, progress=0., start_position=None, end_position=None,
                          range_a=None, range_b=None, inferred_sign=None, ready_to_save=False,
                          range_retry_allowed=False, last_range_rejection=None)
        self.collection = None
        self.last_sample_us = None
        self.last_position = None
        self.limits = None

    def fail(self, error):
        self.clear('failed', str(error), str(error))

    def cancel(self, message='未保存的关节采样已取消；已保存校准保持不变。'):
        self.clear('cancelled', message)

    def active(self):
        return self.value['id'] is not None and self.value['stage'] not in ('complete', 'failed', 'cancelled')

    def start(self, action, sid, state, now):
        if type(sid) is not int or sid not in IDS:
            raise ValueError('未知关节 ID')
        if self.value['busy']:
            raise ValueError('正在采样，先保持关节静止')
        if action == 'direction-start':
            self.clear()
            self.value['id'] = sid
        elif sid != self.value['id']:
            raise ValueError('请先记录所选关节的起点')
        requirements = {'direction-end': 'start_position', 'range-a': 'inferred_sign', 'range-b': 'range_a'}
        if action != 'direction-start' and (action not in requirements or
                self.value[requirements[action]] is None):
            raise ValueError('请按起点、正方向终点、两侧安全位置的顺序采集')
        raw = position(state['positions'][IDS.index(sid)])
        if self.last_position is not None and abs(raw - self.last_position) >= 2048:
            self.fail('编码器跨零点或大跳变，本关节未保存；请从起点重采')
            raise ValueError(self.value['error'])
        self.last_position, self.last_sample_us = raw, state['sample_us']
        if action == 'direction-end':
            self.value.update(end_position=None, inferred_sign=None, range_a=None, range_b=None)
        elif action == 'range-a':
            self.value.update(range_a=None, range_b=None)
        elif action == 'range-b':
            self.value['range_b'] = None
        self.limits = None
        self.collection = dict(action=action, begin=now, samples=[], first_sample_us=None)
        self.value.update(stage=action, busy=True, error=None, sample_count=0, progress=0.,
                          ready_to_save=False, range_retry_allowed=False,
                          message='等待读取点击后的一份新反馈；确认显示值后才保存。')

    def tick(self, state, now, config):
        if not self.active():
            return
        raw = position(state['positions'][IDS.index(self.value['id'])])
        capture = self.collection
        elapsed = None if capture is None else now - capture['begin']
        if elapsed is not None and not 0 <= elapsed <= 3:
            raise ValueError('关节采样超过 3 秒，本次未保存；请重新开始')
        sample = state['sample_us']
        if type(sample) is not int or not 0 <= sample <= 0xffffffff:
            raise ValueError('舵机采样时钟无效')
        delta = (sample - self.last_sample_us) & 0xffffffff
        if delta >= 0x80000000:
            raise ValueError('舵机采样时钟倒退，本关节未保存')
        if delta > 150000:
            raise ValueError('关节连续反馈中断超过主机 150 毫秒有效期，本关节未保存')
        if abs(raw - self.last_position) >= 2048:
            raise ValueError('编码器跨零点或大跳变，本关节未保存；请从起点重采')
        if delta == 0:
            if raw != self.last_position:
                raise ValueError('同一采样时间出现不同编码器值，本关节未保存')
            return
        self.last_sample_us, self.last_position = sample, raw
        if capture is None:
            return
        capture['samples'].append(raw)
        if capture['first_sample_us'] is None:
            capture['first_sample_us'] = sample
        # The explicit capture click selects one subsequent fresh observation.
        # Show it for user confirmation; no invented sample/noise thresholds.
        self.value.update(sample_count=1, progress=1.)
        value = raw
        action = capture['action']
        if action == 'direction-start':
            self.value.update(start_position=value, stage='await-positive',
                              message='起点已记录。对照正方向图轻摆该关节，放稳后记录正方向终点。')
        elif action == 'direction-end':
            self.value.update(end_position=value,
                              inferred_sign=infer_sign(self.value['start_position'], value),
                              stage='await-range-a', message='方向读数已得到。缓慢摆到第一侧安全位置，勿顶死。')
        elif action == 'range-a':
            self.value.update(range_a=value, stage='await-range-b',
                              message='第一侧已记录。缓慢摆到另一侧安全位置，勿顶死。')
        else:
            joint = config['joints'][IDS.index(self.value['id'])]
            if type(joint['id']) is not int or joint['id'] != self.value['id']:
                raise ValueError('校准配置与所选关节 ID 不一致，本关节未保存')
            try:
                self.limits = safe_limits(self.value['range_a'], value, joint['zero'],
                                          self.value['inferred_sign'], DEFAULT_DEG[IDS.index(joint['id'])], joint['id'])
            except RangeValidationError as error:
                self.limits = None
                self.value.update(range_b=None, ready_to_save=False, stage='await-range-b',
                                  error=str(error), range_retry_allowed=True,
                                  last_range_rejection=copy.deepcopy(error.details),
                                  message='方向与安全位置 A 已保留。可重采 A 或 B；勿硬推关节或越过安全位置。')
            else:
                self.value.update(range_b=value, ready_to_save=True, stage='ready',
                                  range_retry_allowed=False,
                                  message=('嘴部范围包含原零点；确认读数与正方向后保存。'
                                           if joint['id'] == 34 else
                                           '所选两侧范围有效，未自动缩边；确认读数与正方向后保存。'))
        self.collection = None
        self.value.update(busy=False, progress=1.)

    def proposed_config(self, config, sid):
        if sid != self.value['id'] or self.value['busy'] or not self.value['ready_to_save']:
            raise ValueError('当前关节的方向和两侧范围尚未采集完成')
        updated = copy.deepcopy(config)
        joint = updated['joints'][IDS.index(sid)]
        joint.update(sign=self.value['inferred_sign'], direction_confirmed=True,
                     min=self.limits[0], max=self.limits[1])
        if zero_signature(updated) != zero_signature(config) or updated['imu'] != config['imu']:
            raise ValueError('关节采集意外改变了零点或 IMU，未保存')
        return updated

    def proposed_direction_config(self, config, sid):
        """Reuse measured encoder bounds; never infer or expand a physical limit."""
        if (type(sid) is not int or sid not in IDS or sid != self.value['id']
                or self.value['busy'] or self.value['stage'] != 'await-range-a'):
            raise ValueError('请先重新记录起点和正方向位置，再保存方向')
        if sid == 34:
            raise ValueError('嘴部方向需连同轻合、张开范围一起核对')
        sign = infer_sign(self.value['start_position'], self.value['end_position'])
        if sign != self.value['inferred_sign']:
            raise ValueError('方向采样记录不一致，未保存')
        joint = config['joints'][IDS.index(sid)]
        if type(joint['id']) is not int or joint['id'] != sid:
            raise ValueError('校准配置与所选关节 ID 不一致，未保存')
        if joint['direction_confirmed'] is not True:
            raise ValueError('本关节尚无已完成的安全范围，请继续采集 A/B')
        low, high, zero = (position(joint[key]) for key in ('min', 'max', 'zero'))
        if not low < high or not low <= zero <= high:
            raise ValueError('已保存安全范围无效，不能沿用')
        stand = int(round(zero + sign * DEFAULT_DEG[IDS.index(sid)] * 4095. / 360.))
        if not low <= stand <= high:
            raise ValueError('新方向的默认站姿超出已保存范围，请核对零点、方向并重采 A/B；范围未扩大')
        updated = copy.deepcopy(config)
        updated['joints'][IDS.index(sid)].update(sign=sign, direction_confirmed=True)
        return updated

    def saved(self, warning=None):
        self.value.update(stage='complete', ready_to_save=False, busy=False,
                          message=warning or '本关节方向和安全范围已保存；原零点与 IMU 保持不变。',
                          error=None, range_retry_allowed=False)

    def snapshot(self, config):
        result = dict(copy.deepcopy(self.value), completed_count=completed_count(config),
                      direction_reuse_allowed=False, direction_reuse_reason=None)
        if self.value['stage'] == 'await-range-a' and not self.value['busy']:
            try:
                self.proposed_direction_config(config, self.value['id'])
                result['direction_reuse_allowed'] = True
            except (ValueError, KeyError, TypeError) as error:
                result['direction_reuse_reason'] = str(error)
        return result
