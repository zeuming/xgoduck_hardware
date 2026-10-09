#!/usr/bin/env python3
"""Read-only pose sampling and transactional IMU mounting calibration.

The controller mixin is embedded in the RAM calibration child by runtime_source;
no additional module, serial request or firmware change is required on the X5.
Only the 24 right-handed signed axis permutations supported by motion.py are
accepted. Human-confirmed poses identify forward and left; static gravity alone
cannot determine heading.
"""

import inspect
import itertools
import math
import numpy as np


def wizard_rotations():
    """Return the 24 right-handed sensor-to-body signed permutations."""
    result = []
    labels = ('x', 'y', 'z')
    for order in itertools.permutations(range(3)):
        for signs in itertools.product((-1, 1), repeat=3):
            matrix = np.zeros((3, 3), dtype=float)
            for row, (column, sign) in enumerate(zip(order, signs)):
                matrix[row, column] = sign
            if round(float(np.linalg.det(matrix))) == 1:
                axes = [('-' if sign < 0 else '') + labels[column]
                        for column, sign in zip(order, signs)]
                result.append((axes, matrix))
    return result


def wizard_validate_batch(samples):
    """Validate a stationary 100-sample specific-force/gyro batch."""
    values = np.asarray(samples, dtype=float)
    if values.ndim != 2 or values.shape[1] != 6 or len(values) < 100 or not np.isfinite(values).all():
        raise ValueError('有效采样不足，请摆好姿态、保持静止后重新采集')
    acc, gyro = values[:, :3], values[:, 3:]
    acc_mean, gyro_mean = acc.mean(axis=0), gyro.mean(axis=0)
    norms = np.linalg.norm(acc, axis=1)
    norm = float(np.linalg.norm(acc_mean))
    if not 9.0 <= norm <= 10.6 or not np.all((norms >= 8.5) & (norms <= 11.1)):
        raise ValueError('重力读数异常，请检查 IMU 是否固定、连接是否正常')
    if (float(np.max(acc.std(axis=0))) >= .25 or float(norms.std()) >= .2
            or float(np.max(gyro.std(axis=0))) >= .5 or float(np.linalg.norm(gyro_mean)) >= 3):
        raise ValueError('采集时机身发生移动；先摆好姿态，等完全静止再点击')
    return {'acc': acc_mean.tolist(), 'gyro': gyro_mean.tolist()}


def wizard_angle(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    denominator = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denominator <= 0 or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError('IMU 重力读数无效')
    return math.degrees(math.acos(float(np.clip(float(np.dot(a, b)) / denominator, -1., 1.))))


def wizard_find_axes(level_acc, front_acc):
    """Use level and nose-up specific force to select one discrete mounting."""
    level, front = np.asarray(level_acc, dtype=float), np.asarray(front_acc, dtype=float)
    if level.shape != (3,) or front.shape != (3,) or not np.isfinite(level).all() or not np.isfinite(front).all():
        raise ValueError('IMU 重力读数无效')
    ranked = []
    for axes, matrix in wizard_rotations():
        upright, tilted = matrix @ level, matrix @ front
        if wizard_angle(upright, [0, 0, 1]) > 8:
            continue
        tilt = wizard_angle(tilted, [0, 0, 1])
        radians = math.radians(tilt)
        target = [math.sin(radians), 0., math.cos(radians)]
        residual = wizard_angle(tilted, target)
        ranked.append((residual, axes, matrix, tilted))
    if not ranked:
        raise ValueError('机身水平时 IMU 仍明显倾斜；请把模块与机身底面平行固定后重做')
    ranked.sort(key=lambda value: value[0])
    residual, axes, matrix, tilted = ranked[0]
    if len(ranked) < 2 or ranked[1][0] - residual < 12:
        raise ValueError('无法明确辨认鸭嘴方向；请只抬高鸭嘴一侧约 20–40°，不要同时侧倾')
    pitch = math.degrees(math.atan2(float(tilted[0]), float(tilted[2])))
    side = math.degrees(math.atan2(float(tilted[1]), float(tilted[2])))
    if not 20 <= pitch <= 40 or tilted[2] <= 0 or abs(side) > 8 or residual > 8:
        raise ValueError('前倾姿态不合适；请抬高鸭嘴一侧约 20–40°，左右保持平齐')
    return list(axes)


def wizard_verify_pose(pose, acc, axes):
    mapping = next((matrix for candidate, matrix in wizard_rotations() if candidate == list(axes)), None)
    if mapping is None:
        raise ValueError('IMU 安装朝向无效，请从第一步重新识别')
    body = mapping @ np.asarray(acc, dtype=float)
    if body.shape != (3,) or not np.isfinite(body).all():
        raise ValueError('IMU 重力读数无效')
    if pose == 'level':
        if wizard_angle(body, [0, 0, 1]) > 8:
            raise ValueError('机身还没放平；请让躯干底面水平，保持静止后重试')
    elif pose == 'left-up':
        angle = math.degrees(math.atan2(float(body[1]), float(body[2])))
        forward = math.degrees(math.atan2(float(body[0]), float(body[2])))
        if not 20 <= angle <= 40 or body[2] <= 0 or abs(forward) > 8:
            raise ValueError('左右方向验证未通过；请抬高鸭子自己的左侧约 20–40°，鸭嘴与尾部保持同高；若刚才抬错前后，请从第一步重做')
    else:
        raise ValueError('未知的 IMU 验证姿态')
    return body.tolist()


class ImuWizardMixin:
    """Cooperative MotionController mixin; motion is supplied by its child host.

    Call imu_wizard_action via the existing authenticated mutation route. While
    a wizard is active, other calibration routes must call
    _require_no_imu_wizard_locked under the controller lock. status carries the
    public imu_wizard schema; _accept_state_locked collects existing feedback.
    """

    def _imu_wizard_default_locked(self, message='先固定 IMU，再用三个姿态识别朝向，无需填写 XYZ'):
        return {'stage': 'idle', 'expected_action': 'level', 'busy': False,
                'progress': 0., 'message': message, 'error': None,
                'generation': self._generation, 'sample_count': 0, 'required_samples': 100,
                'pose': '机身水平放稳', 'active': False, 'poses': {}, 'axes': None,
                'collection': None}

    def _imu_wizard_get_locked(self):
        if getattr(self, '_imu_wizard', None) is None:
            self._imu_wizard = self._imu_wizard_default_locked()
        return self._imu_wizard

    def _require_no_imu_wizard_locked(self):
        if self._imu_wizard_get_locked()['active']:
            raise motion.MotionError('IMU 向导进行中；请先完成或取消，再修改其他校准')

    def _imu_wizard_cancel_locked(self, message='已取消 IMU 向导，之前保存的校准保持不变'):
        self._imu_wizard = self._imu_wizard_default_locked(message)

    def _imu_wizard_fail_locked(self, message, reset=False):
        wizard = self._imu_wizard_get_locked()
        action = 'level' if reset else (wizard.get('collection') or {}).get('action', wizard['expected_action'])
        if reset:
            wizard['poses'], wizard['axes'] = {}, None
            wizard['generation'] = self._generation
        wizard.update(stage='failed', expected_action=action, busy=False, progress=0.,
                      message=message, error=message, sample_count=0, collection=None)

    def _imu_wizard_health_locked(self, generation):
        if type(generation) is not int or generation != self._generation or self._closed.is_set():
            raise motion.MotionError('连接会话已变化，请从第一步重新识别')
        self._require_off_locked()
        self._require_feedback_locked()
        state = self._state
        if (state['online_mask'] != motion.ALL_MASK or state['fault_mask'] != 0
                or state['torque_mask'] != 0 or state['armed'] is not False):
            raise motion.MotionError('采集已停止：必须全部舵机在线、无故障且扭矩关闭')
        if state['imu_ok'] is not True or self._imu_age_locked() >= motion.FRESH_SECONDS:
            raise motion.MotionError('IMU 反馈中断或过期，请恢复连接后重试')
        for key in ('sample_us', 'imu_us'):
            if type(state[key]) is not int or not 0 <= state[key] <= 0xffffffff:
                raise motion.MotionError('IMU 采样时间无效，请恢复连接后重试')
        return state

    def imu_wizard_action(self, action, expected_generation, *, support_confirmed=False, mounted_confirmed=False):
        if action not in ('level', 'front-up', 'left-up', 'finish', 'cancel'):
            raise motion.MotionError('未知的 IMU 向导步骤')
        with self._lock:
            if action == 'cancel':
                self._imu_wizard_cancel_locked()
            else:
                if support_confirmed is not True or mounted_confirmed is not True:
                    raise motion.MotionError('请先确认机身由支架承重，IMU 已牢固固定在躯干、与底面平行')
                state = self._imu_wizard_health_locked(expected_generation)
                wizard = self._imu_wizard_get_locked()
                if wizard['busy']:
                    raise motion.MotionError('正在采样，请保持当前姿态，等待完成')
                if self._imu_collection is not None:
                    raise motion.MotionError('已有 IMU 校准正在采样，请等待完成')
                if action == 'level':
                    wizard = self._imu_wizard_default_locked()
                    wizard['active'] = True
                    self._imu_wizard = wizard
                elif (wizard['generation'] != expected_generation or wizard['expected_action'] != action
                      or not wizard['active']):
                    raise motion.MotionError('请按顺序完成向导，连接变化后需从第一步重做')
                descriptions = {'level': '机身水平放稳', 'front-up': '抬高鸭嘴一侧约 20–40°，左右保持平齐',
                                'left-up': '抬高鸭子自己的左侧约 20–40°，鸭嘴和尾部保持同高',
                                'finish': '机身重新放平并保持静止'}
                wizard.update(stage=action, busy=True, progress=0., error=None, sample_count=0,
                              message='正在采样约 2 秒，请保持静止', pose=descriptions[action],
                              collection={'action': action, 'begin': self._clock(), 'samples': [],
                                          'last_imu_us': state['imu_us'], 'last_sample_us': state['sample_us']})
        return self.status()

    def _accept_state_locked(self, state):
        super()._accept_state_locked(state)
        self._imu_wizard_tick_locked()

    def _imu_wizard_tick_locked(self):
        wizard = self._imu_wizard_get_locked()
        collection = wizard['collection']
        if collection is None:
            if wizard['active'] and wizard['generation'] != self._generation:
                self._imu_wizard_fail_locked('连接会话已变化，请从第一步重新识别', reset=True)
            return
        try:
            state = self._imu_wizard_health_locked(wizard['generation'])
            elapsed = self._clock() - collection['begin']
            if elapsed < 0 or elapsed > 6:
                raise ValueError('采样超时；需要约 2 秒连续的新鲜数据，请恢复连接后重试')
            imu_delta = (state['imu_us'] - collection['last_imu_us']) & 0xffffffff
            state_delta = (state['sample_us'] - collection['last_sample_us']) & 0xffffffff
            if imu_delta >= 0x80000000 or state_delta >= 0x80000000:
                raise ValueError('采样时间倒退，连接可能已重置；请从第一步重做')
            if imu_delta == 0 or state_delta == 0:
                return
            if imu_delta >= 100000 or state_delta >= 100000:
                raise ValueError('连续采样中断超过 100 毫秒；请恢复稳定反馈后重试')
            collection['last_imu_us'], collection['last_sample_us'] = state['imu_us'], state['sample_us']
            sample = list(state['acc']) + list(state['gyro'])
            if len(sample) != 6 or not np.isfinite(np.asarray(sample, dtype=float)).all():
                raise ValueError('IMU 读数无效，请检查连接后重试')
            collection['samples'].append(sample)
            if len(collection['samples']) > 300:
                raise ValueError('采样节奏异常；请恢复稳定反馈后重试')
            wizard['sample_count'] = len(collection['samples'])
            wizard['progress'] = min(.99, min(wizard['sample_count'] / 100., elapsed / 1.8))
            if wizard['sample_count'] < 100 or elapsed < 1.8:
                return
            batch = wizard_validate_batch(collection['samples'])
            action = collection['action']
            if action == 'level':
                # Vertical can be either polarity of any sensor axis, but it
                # must be nearly parallel to a body axis for this exact mapper.
                dominant = np.zeros(3)
                dominant[int(np.argmax(np.abs(batch['acc'])))] = 1.
                if wizard_angle(np.abs(batch['acc']), dominant) > 8:
                    raise ValueError('IMU 安装明显倾斜；请与机身底面平行固定，机身放平后重试')
                wizard['poses']['level'] = batch
                next_action, message = 'front-up', '第一步完成。抬高鸭嘴一侧约 20–40°，放稳后点击第二步'
            elif action == 'front-up':
                wizard['axes'] = wizard_find_axes(wizard['poses']['level']['acc'], batch['acc'])
                wizard['poses']['front-up'] = batch
                next_action, message = 'left-up', '前后方向已识别。抬高鸭子自己的左侧约 20–40°，放稳后点击第三步'
            elif action == 'left-up':
                wizard_verify_pose('left-up', batch['acc'], wizard['axes'])
                wizard['poses']['left-up'] = batch
                next_action, message = 'finish', '左右方向验证通过。机身回到水平、放稳后点击完成校准'
            else:
                wizard_verify_pose('level', batch['acc'], wizard['axes'])
                if wizard_angle(batch['acc'], wizard['poses']['level']['acc']) > 5:
                    raise ValueError('与第一步水平姿态差异较大；请重新放平。若 IMU 移动过，请从第一步重做')
                previous, dirty = self._config, self._config_dirty
                updated = motion.copy.deepcopy(previous)
                updated['imu'] = {'axes': list(wizard['axes']), 'axes_confirmed': True,
                                  'calibrated': True, 'bias_dps': list(batch['gyro'])}
                self._config, self._config_dirty = updated, True
                try:
                    self._save_locked()
                except BaseException:
                    self._config, self._config_dirty = previous, dirty
                    raise
                self._filter = motion.ImuFilter()
                self._filtered = None
                self._error = None
                wizard.update(stage='complete', expected_action=None, active=False, busy=False,
                              progress=1., message='IMU 朝向与静止漂移已校准并保存，无需填写 XYZ',
                              error=None, collection=None)
                return
            wizard.update(stage=next_action, expected_action=next_action, busy=False, progress=1.,
                          message=message, error=None, collection=None)
        except Exception as error:
            self._imu_wizard_fail_locked(str(error), reset=wizard['generation'] != self._generation)

    def _stop_locked(self, reason=None):
        super()._stop_locked(reason)
        if getattr(self, '_imu_wizard', None) is not None:
            self._imu_wizard_cancel_locked('采样已停止，之前保存的校准保持不变')

    def set_imu_axes(self, axes, axes_confirmed=False):
        with self._lock:
            self._require_no_imu_wizard_locked()
            return super().set_imu_axes(axes, axes_confirmed=axes_confirmed)

    def calibrate_imu(self):
        with self._lock:
            self._require_no_imu_wizard_locked()
            return super().calibrate_imu()

    def status(self):
        with self._lock:
            result = super().status()
            wizard = self._imu_wizard_get_locked()
            keys = ('stage', 'expected_action', 'busy', 'progress', 'message', 'error',
                    'generation', 'sample_count', 'required_samples', 'pose')
            result['imu_wizard'] = {key: wizard[key] for key in keys}
            return result


def runtime_source():
    """Return just the imports, pure helpers and cooperative controller mixin."""
    functions = (wizard_rotations, wizard_validate_batch, wizard_angle,
                 wizard_find_axes, wizard_verify_pose, ImuWizardMixin)
    return ('import itertools, math\nimport numpy as np\n'
            + '\n'.join(inspect.getsource(function) for function in functions))


if __name__ == '__main__':
    raise SystemExit('Use test_imu_wizard.py for local synthetic verification')
