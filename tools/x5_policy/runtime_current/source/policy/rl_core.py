"""Validated ONNX policy contracts and observation/filter math."""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import numpy as np


GYRO_ALPHA = 0.5

VEL_ALPHA = 0.4

FEEDBACK_PERIOD = 0.01  # Firmware sendState: 100 Hz; protocol has no sample timestamp.

GRAVITY_TAU = 0.3

# Static IMU attitude when the robot is standing level, in the policy body frame
# (degrees). Stand still and read grav: pitch = atan2(gx, -gz), roll = atan2(-gy, -gz).
# Those values go here; both accelerometer and gyro are rotated so standing gravity
# becomes [0, 0, -1]. Yaw is unobservable from gravity; leave it 0 unless the board
# is also twisted. Example: pitch inherently 5° → (0.0, 5.0, 0.0).
IMU_BIAS_DEG = (0.0, 0.0, 0.0)  # roll, pitch, yaw

MOTOR_IDS = [
    10, 11, 12, 13, 14,
    20, 21, 22, 23, 24,
    30, 31, 32, 33, 34,
]

# Same pinned model-specific reference used by passive calibration.
from model_reference import WALK_DEFAULT_DEG
DEFAULT_DEG = list(WALK_DEFAULT_DEG)
FIXED_STANCE_OVERRIDES = {}

POLICY_IDS = [
    10, 11, 12, 13, 14,
    30, 31, 32, 33,
    20, 21, 22, 23, 24,
]

POLICY_INDEX = np.array([MOTOR_IDS.index(i) for i in POLICY_IDS])

HOME_DEG = np.array(DEFAULT_DEG, dtype=np.float32)[POLICY_INDEX]

MOUTH_INDEX = MOTOR_IDS.index(34)

MOUTH_MAX_DEG = 30.0

OBS_DIM = 61

ACTION_DIM = 14

ACTION_SCALE = 1.0

ACTION_ALPHA = 0.45

ACTION_ALPHA_HEAD = 0.45

POLICY_JOINT_NAMES = (
    'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
    'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
    'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle',
)
POLICY_OBSERVATION_NAMES = (
    'base_ang_vel', 'projected_gravity', 'joint_pos', 'joint_vel',
    'actions', 'command', 'head_command', 'body_command',
)


def policy_profile_from_metadata(metadata):
    """Validate the policy contract; no missing metadata has a fallback."""
    required = ('joint_names', 'observation_names', 'default_joint_pos', 'action_scale')
    if not isinstance(metadata, dict) or any(type(metadata.get(k)) is not str for k in required):
        raise ValueError('模型元数据缺少关节、观察量、默认姿态或动作比例')
    if tuple(v.strip() for v in metadata['joint_names'].split(',')) != POLICY_JOINT_NAMES:
        raise ValueError('模型元数据关节顺序与控制器不一致')
    if tuple(v.strip() for v in metadata['observation_names'].split(',')) != POLICY_OBSERVATION_NAMES:
        raise ValueError('模型元数据观察量顺序与 61 维布局不一致')
    try:
        home = np.asarray([float(v) for v in metadata['default_joint_pos'].split(',')], dtype=np.float64)
        scale = float(metadata['action_scale'])
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('模型元数据默认姿态或动作比例无效') from error
    if home.shape != (ACTION_DIM,) or not np.isfinite(home).all() or not math.isfinite(scale) or scale <= 0:
        raise ValueError('模型元数据需要 14 个有限默认角度和正的有限动作比例')
    home.setflags(write=False)
    return dict(home_rad=home, scale=scale)


def policy_profile(session):
    """Read ONNX metadata and validate its exact float32 61 -> 14 interface."""
    try:
        inputs, outputs = session.get_inputs(), session.get_outputs()
        metadata = session.get_modelmeta().custom_metadata_map
    except Exception as error:
        raise ValueError('模型接口或元数据无法读取') from error
    if (len(inputs) != 1 or list(inputs[0].shape) != [1, OBS_DIM] or inputs[0].type != 'tensor(float)' or
            len(outputs) != 1 or list(outputs[0].shape) != [1, ACTION_DIM] or outputs[0].type != 'tensor(float)'):
        raise ValueError('模型必须使用 float32 [1,61] 观察量和 [1,14] 动作')
    return policy_profile_from_metadata(metadata)


def stance_from_home(home_rad):
    home = np.asarray(home_rad, dtype=np.float64)
    if home.shape != (ACTION_DIM,) or not np.isfinite(home).all():
        raise ValueError('模型默认姿态需要 14 个有限弧度值')
    stance = np.zeros(len(MOTOR_IDS), dtype=np.float64)
    stance[POLICY_INDEX] = np.rad2deg(home)
    return stance.tolist()

@dataclass
class State:
    acc: List[float]
    gyro: List[float]
    angles: List[float]  # MOTOR_IDS order
    vels: List[float]
    grav: List[float] = field(default_factory=lambda: [0.0, 0.0, -1.0])
    gyro_rad: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

def _rotate_imu_bias(vec: List[float]) -> List[float]:
    """Rotate an IMU vector to cancel IMU_BIAS_DEG. Identity when the bias is zero."""
    roll, pitch, yaw = IMU_BIAS_DEG
    if roll == 0.0 and pitch == 0.0 and yaw == 0.0:
        return vec
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    x, yv, z = vec
    if y:
        cy, sy = math.cos(y), math.sin(y)
        x, yv = x * cy - yv * sy, x * sy + yv * cy
    if r:
        cr, sr = math.cos(r), math.sin(r)
        yv, z = yv * cr - z * sr, yv * sr + z * cr
    if p:
        cp, sp = math.cos(p), math.sin(p)
        x, z = x * cp + z * sp, -x * sp + z * cp
    return [x, yv, z]


class ImuFilter:
    """Body-frame gravity: gyro prediction plus accel-magnitude-weighted correction. No bias estimate."""

    def __init__(self, gyro_alpha: float = GYRO_ALPHA, vel_alpha: float = VEL_ALPHA) -> None:
        self.gyro_alpha = gyro_alpha
        self.vel_alpha = vel_alpha
        self._gravity: Optional[List[float]] = None
        self._gyro: Optional[List[float]] = None
        self._vel: Optional[List[float]] = None

    def update(self, st: State, dt: float = FEEDBACK_PERIOD) -> State:
        acc_n = _rotate_imu_bias([st.acc[2], -st.acc[0], -st.acc[1]])
        gyro_n = _rotate_imu_bias([st.gyro[2], -st.gyro[0], -st.gyro[1]])
        norm = math.sqrt(sum(v * v for v in acc_n))
        acc_unit = [v / norm for v in acc_n] if norm > 1e-6 else None
        # Large gaps cannot be reconstructed from a single angular-rate sample.
        reset = self._gravity is None or dt > 0.1
        dt = max(0.001, min(dt, 0.1))
        if reset:
            self._gravity = acc_unit if acc_unit is not None else [0.0, 0.0, -1.0]
            self._gyro = gyro_n
            self._vel = list(st.vels)
        else:
            gx, gy, gz = self._gravity
            wx, wy, wz = [math.radians(v) for v in gyro_n]
            # A world-fixed vector in rotating body coordinates: dg/dt = -omega x g.
            predicted = [gx + (wz * gy - wy * gz) * dt,
                         gy + (wx * gz - wz * gx) * dt,
                         gz + (wy * gx - wx * gy) * dt]
            pnorm = math.sqrt(sum(v * v for v in predicted))
            predicted = [v / pnorm for v in predicted]
            trust = max(0.0, 1.0 - abs(norm / 9.8 - 1.0) / 0.2)
            gain = (1.0 - math.exp(-dt / GRAVITY_TAU)) * trust
            if acc_unit is not None:
                predicted = [(1 - gain) * p + gain * a for p, a in zip(predicted, acc_unit)]
            pnorm = math.sqrt(sum(v * v for v in predicted))
            self._gravity = [v / pnorm for v in predicted]
            # Preserve the previous smoothing at 100 Hz, also when USB delivers batches.
            ga = self.gyro_alpha ** (dt / FEEDBACK_PERIOD)
            va = self.vel_alpha ** (dt / FEEDBACK_PERIOD)
            self._gyro = [ga * o + (1 - ga) * n for o, n in zip(self._gyro, gyro_n)]
            self._vel = [va * o + (1 - va) * n for o, n in zip(self._vel, st.vels)]
        st.grav = list(self._gravity)
        st.gyro_rad = [v * math.pi / 180.0 for v in self._gyro]
        st.vels = list(self._vel)
        return st

def policy_from_state(st: State, *, home_rad=None) -> Tuple[np.ndarray, np.ndarray]:
    """Absolute joint degrees minus the home pose, then converted to radians."""
    q_deg = np.asarray(st.angles, dtype=np.float32)[POLICY_INDEX]
    v_deg = np.asarray(st.vels, dtype=np.float32)[POLICY_INDEX]
    home = np.deg2rad(HOME_DEG) if home_rad is None else np.asarray(home_rad, dtype=np.float64)
    if home.shape != (ACTION_DIM,) or not np.isfinite(home).all():
        raise ValueError('模型默认姿态需要 14 个有限弧度值')
    return np.deg2rad(q_deg) - home, np.deg2rad(v_deg)

def build_obs(
    st: State,
    prev_action: np.ndarray,
    twist: np.ndarray,
    head: np.ndarray,
    out: np.ndarray | None = None,
    *, home_rad=None,
) -> np.ndarray:
    obs = np.empty(OBS_DIM, dtype=np.float32) if out is None else out
    obs[0:3] = st.gyro_rad
    obs[3:6] = st.grav
    obs[6:20], obs[20:34] = policy_from_state(st, home_rad=home_rad)
    obs[34:48] = prev_action
    obs[48:51] = twist
    obs[51:55] = head
    obs[55:61] = 0.0
    return obs

def action_to_deg(action: np.ndarray, mouth_deg: float = 0.0, *, home_rad=None,
                  action_scale=ACTION_SCALE) -> List[float]:
    """Policy output is radians relative to home. Convert to degrees and add the home pose."""
    home = np.deg2rad(HOME_DEG) if home_rad is None else np.asarray(home_rad, dtype=np.float64)
    if home.shape != (ACTION_DIM,) or not np.isfinite(home).all() or not math.isfinite(action_scale) or action_scale <= 0:
        raise ValueError('模型默认姿态或动作比例无效')
    q_deg = np.rad2deg(home + action_scale * action)
    targets = np.zeros(len(MOTOR_IDS), dtype=np.float32)
    targets[POLICY_INDEX] = q_deg
    targets[MOUTH_INDEX] = float(np.clip(mouth_deg, 0.0, MOUTH_MAX_DEG))
    return targets.tolist()
