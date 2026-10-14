#!/usr/bin/env python3
r"""Bounded, CPU-only ONNX/BAM replay using one explicit calibration snapshot.

No serial, motion-controller, deployment or calibration-writing APIs are imported.
Example (requires numpy, onnxruntime, mujoco, colorama and the BAM source tree):

  python tools/x5_policy/audit_policy_sim.py \
    --snapshot tmp/standing-audit-20261008/live-status.json \
    --speeds 0 .2 --executors policy fixed-head --slew-ticks-per-second 600 \
    --output tmp/standing-audit-20261008/policy-sim.json

All joint angles, zeros and saved mechanical limits come from that snapshot or
the selected ONNX metadata. No historic capture/calibration is substituted.
"""
from __future__ import annotations

import argparse
from collections import deque
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from model_adaptation_contract import load_gain_manifest

from rl_core import (ACTION_ALPHA, ACTION_ALPHA_HEAD, FIXED_STANCE_OVERRIDES,
                     MOTOR_IDS, POLICY_IDS, POLICY_INDEX, POLICY_JOINT_NAMES,
                     ImuFilter, State, action_to_deg, build_obs, policy_profile,
                     stance_from_home)


ROOT = Path(__file__).resolve().parents[2]
REFERENCE = ROOT / 'output/xgoduck_rl_reference'

# Copied from the audited official sources, not fitted to make a rollout pass:
# vendor/bam/bam/mjlab.py::_set_friction_stiffness, and
# src/mjlab_microduck/robot/xgoduck_constants.py::XGODUCK_COLLISION.
OFFICIAL_FRICTION_SOLREF = (-5.0e4, -2.0e2)
OFFICIAL_FRICTION_SOLIMP = (.99, .9999, .001, .5, 2.)


def apply_official_contact_contract(spec):
    """Match named collision settings and disable unnamed collision meshes.

    This reproduces only the referenced spec edits. It does not claim parity
    with the GPU solver, domain randomization or an unidentified checkpoint.
    """
    for geom in spec.geoms:
        if not geom.name:
            geom.contype = geom.conaffinity = 0
        elif geom.name.endswith('_collision'):
            head_or_base = geom.name in ('trunk_collision', 'head_collision')
            foot = geom.name in ('left_foot_collision', 'right_foot_collision')
            geom.contype = 3 if head_or_base else 1
            geom.conaffinity = 2 if head_or_base else 0
            geom.condim = 3 if foot else 1
            if foot:
                geom.priority = 1
                geom.friction[0] = 1.


def smooth_action(previous_smooth, action, leg_alpha, head_alpha):
    """Runtime EMA, separate from raw previous-action policy observations."""
    for alpha in (leg_alpha, head_alpha):
        if not math.isfinite(alpha) or not 0 <= alpha < 1:
            raise ValueError('Action EMA alpha must be finite and within [0, 1)')
    result = np.asarray(previous_smooth, dtype=np.float32).copy()
    if result.shape != (14,) or np.asarray(action).shape != (14,):
        raise ValueError('Action EMA requires 14 policy joints')
    for sl, alpha in ((slice(0, 5), leg_alpha), (slice(5, 9), head_alpha),
                      (slice(9, 14), leg_alpha)):
        result[sl] = alpha * result[sl] + (1 - alpha) * action[sl]
    return result


def scheduled_command(at, speed, warmup, pulse, ramp):
    elapsed = at - warmup
    if elapsed < 0 or (pulse is not None and elapsed >= pulse):
        return 0.
    fraction = 1. if ramp == 0 else min(1., elapsed / ramp)
    if pulse is not None and ramp:
        fraction = min(fraction, (pulse - elapsed) / ramp)
    return speed * max(0., fraction)


def command_distance(end, speed, warmup, pulse, ramp):
    """Integral of the requested trapezoidal velocity, including partial runs."""
    t = max(0., end - warmup)
    if pulse is not None:
        t = min(t, pulse)
    if ramp == 0:
        return speed * t
    area = t*t/(2*ramp) if t < ramp else t-ramp/2
    if pulse is not None:
        area -= max(0., t-(pulse-ramp))**2/(2*ramp)
    return speed * area


def execution_action(raw, profile, cfg, decoder):
    """Frozen deployed roll adapter for A/B screening; never edits saved bounds.

    Previous-action observations must still use the ORIGINAL raw actor output.
    This is compared against the deployed decoder before interpreting results.
    """
    if decoder == 'metadata':
        return raw.copy()
    result = raw.copy()
    by_id = {j['id']: j for j in cfg['joints']}
    for sid, index in ((11, 1), (21, 10)):
        row = by_id[sid]
        lo, hi = sorted((tick-row['zero'])*360/4095*row['sign']
                        for tick in (row['min']+3, row['max']-3))
        if not lo < -16 < 16 < hi:
            raise ValueError('Deployed roll map cannot fit saved bounds')
        angle = math.degrees(profile['home_rad'][index] + profile['scale']*.95*float(raw[index]))
        mapped = (lo+(-16-lo)*math.exp((angle+16)/(-16-lo)) if angle < -16 else
                  hi-(hi-16)*math.exp(-(angle-16)/(hi-16)) if angle > 16 else angle)
        result[index] = (math.radians(mapped)-profile['home_rad'][index])/profile['scale']
    return result


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def assess_result(result):
    """Keep simulation survival, standing and actual stepping distinct.

    These are screening criteria for an offline candidate, never permission to
    enable a physical robot. Counterfactual continuation cannot hide even one
    requested, applied or measured mechanical range violation.
    """
    failures = []
    if result.get('range_ignoring_counterfactual'):
        failures.append('range_ignoring_counterfactual_not_a_release_candidate')
    if result['ended']['reason'] != 'duration':
        failures.append('did_not_complete_duration')
    if result['max_tilt_deg'] >= 15:
        failures.append('tilt_reached_15_deg')
    for stage in ('requested_execution_target', 'mcu_applied_goal', 'actual_joint'):
        if result['outputs'][stage]['violating_samples']:
            failures.append(stage + '_outside_saved_range')
    feet = result['actual_foot_motion']['feet']
    speed = result['speed_m_s']
    completed = {side: sum(bool(lift['completed']) and
        lift['duration_s'] >= .05 - 1e-9 and
        lift['opposite_contact_s'] >= .05 - 1e-9
        for lift in foot['qualifying_lift_intervals'])
        for side, foot in feet.items()}
    if speed == 0:
        task = 'standing'
        if any(foot['contact_fraction'] < .9 for foot in feet.values()):
            failures.append('both_feet_not_regularly_in_contact')
        if math.hypot(*result['displacement_m']) > .03:
            failures.append('standing_drift_exceeded_3_cm')
    else:
        task = 'walking'
        if any(n < 2 for n in completed.values()):
            failures.append('fewer_than_two_supported_completed_lifts_per_foot')
        progress = result.get('command_progress')
        if progress is None or progress['origin_at_s'] is None:
            failures.append('command_window_progress_not_measured')
        elif progress['direction_progress_m'] <= 0:
            failures.append('no_positive_progress_in_command_direction')
    return dict(task=task, offline_screen_passed=not failures,
                failures=failures, completed_supported_lifts=completed,
                physical_standing_verified=False, physical_walking_verified=False,
                deployment_authorized=False,
                progress_rule='Positive displacement after zero warmup, plus two supported landed lifts per foot; velocity tracking error is reported separately, with no invented hardware acceptance tolerance.',
                interpretation='Offline screening only; physical reference, model fidelity and attended acceptance remain required.')


def rounded_ticks(value):
    """Host target rounding is Python round; MCU goal rounding is separate."""
    return np.rint(value).astype(np.int64)


def validate_snapshot(snapshot):
    cfg = snapshot.get('calibration')
    if not isinstance(cfg, dict) or cfg.get('version') != 1:
        raise ValueError('Snapshot must contain calibration version 1')
    joints = cfg.get('joints')
    if not isinstance(joints, list) or [j.get('id') for j in joints] != MOTOR_IDS:
        raise ValueError('Snapshot calibration must have all 15 joints in MOTOR_IDS order')
    for joint in joints:
        if (any(type(joint.get(k)) is not int for k in ('zero', 'min', 'max', 'sign'))
                or joint['sign'] not in (-1, 1)
                or not 0 <= joint['min'] <= joint['zero'] <= joint['max'] <= 4095
                or joint['min'] == joint['max']):
            raise ValueError('Invalid snapshot calibration for ID ' + str(joint['id']))
    motion = snapshot.get('motion') or {}
    # Prefer a present hardware sample; never combine two snapshots or use a
    # last_hardware sample without explicitly identifying it in the report.
    hardware, source = motion.get('hardware'), 'motion.hardware'
    if not hardware:
        hardware, source = motion.get('last_hardware'), 'motion.last_hardware'
    if not isinstance(hardware, dict):
        raise ValueError('Snapshot has no motion hardware sample')
    raw = hardware.get('rawdata')
    if not isinstance(raw, list) or sorted(r.get('id') for r in raw) != sorted(MOTOR_IDS):
        raise ValueError('Snapshot hardware must contain one raw position per joint')
    positions = {row['id']: row['position'] for row in raw}
    if any(type(p) is not int or not 0 <= p <= 4095 for p in positions.values()):
        raise ValueError('Snapshot has invalid raw servo ticks')
    angles = np.asarray([(positions[j['id']] - j['zero']) * 360 / 4095 * j['sign']
                         for j in joints], dtype=np.float64)
    return cfg, hardware, source, angles


def angles_to_ticks(angles, cfg):
    return rounded_ticks([j['zero'] + angle / 360 * 4095 * j['sign']
                          for angle, j in zip(angles, cfg['joints'])])


def ticks_to_angles(ticks, cfg):
    return np.asarray([(tick - j['zero']) * 360 / 4095 * j['sign']
                       for tick, j in zip(ticks, cfg['joints'])])


def refresh_observation_data(mj, model, data, observed):
    """Observe the integrated state without altering BAM's preceding solve.

    mj_step integrates qpos/qvel after computing xmat, cvel and contacts, so its
    derived fields describe the preceding physics time. Forward a separate copy
    for coherent body/foot observations; leave the integration data's constraint
    and actuator forces intact for MujocoController's next friction update.
    """
    mj.mj_copyData(observed, model, data)
    mj.mj_forward(model, observed)


class Trace:
    """Keep each output stage independent; do not hide rejected targets."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.minimum, self.maximum = None, None
        self.first, self.violating_frames, self.count = None, 0, 0
        self.ids = set()

    def add(self, angles, at):
        values = np.asarray(angles, dtype=np.float64)
        if values.shape != (15,) or not np.isfinite(values).all():
            raise ValueError('Nonfinite or malformed simulated joint target')
        self.minimum = values.copy() if self.minimum is None else np.minimum(self.minimum, values)
        self.maximum = values.copy() if self.maximum is None else np.maximum(self.maximum, values)
        ticks = angles_to_ticks(values, self.cfg)
        bad = [dict(id=j['id'], angle_deg=float(angle), tick=int(tick),
                    min_tick=j['min'], max_tick=j['max'],
                    excess_tick=int(max(j['min'] - tick, tick - j['max'])))
               for angle, tick, j in zip(values, ticks, self.cfg['joints'])
               if not j['min'] <= tick <= j['max']]
        self.count += 1
        if bad:
            self.ids.update(j['id'] for j in bad)
            self.violating_frames += 1
            if self.first is None:
                self.first = dict(time_s=float(at), joints=bad)
        return bad

    def report(self):
        return dict(samples=self.count,
                    angle_min_deg=None if self.minimum is None else self.minimum.tolist(),
                    angle_max_deg=None if self.maximum is None else self.maximum.tolist(),
                    first_range_violation=self.first, violating_samples=self.violating_frames,
                    violating_ids=sorted(self.ids))


class FootMotionTrace:
    """Measure simulated collision contacts and whole-foot clearance, not targets.

    A qualifying lift needs >=3 mm clearance for >=50 ms. Contact changes alone
    can be solver chatter or rocking and are not counted as successful steps.
    """

    def __init__(self, mj, model, data, dt):
        self.mj, self.model, self.data, self.dt = mj, model, data, dt
        self.floor = model.geom('policy_audit_floor').id
        self.specs, self.rows = {}, {}
        for side in ('left', 'right'):
            geom, site = model.geom(side + '_foot_collision').id, model.site(side + '_foot').id
            mesh = int(model.geom_dataid[geom])
            start, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
            self.specs[side] = (geom, site, np.asarray(model.mesh_vert[start:start+count], float).copy())
            self.rows[side] = dict(contact_samples=0, samples=0, contact_changes=[],
                lift_intervals=[], active_lift=None, last_contact=None, minimum=None, maximum=None,
                min_clearance_m=None, max_clearance_m=None)

    def measure(self):
        measured = {}
        forces = {side: 0. for side in self.specs}
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            pair = {int(contact.geom1), int(contact.geom2)}
            for side, (geom, _, _) in self.specs.items():
                if pair == {self.floor, geom}:
                    force = np.zeros(6)
                    self.mj.mj_contactForce(self.model, self.data, i, force)
                    forces[side] += max(0., float(force[0]))
        for side, (geom, site, vertices) in self.specs.items():
            rotation = self.data.geom_xmat[geom].reshape(3, 3)
            world_vertices = np.einsum('ij,kj->ki', rotation, vertices) + self.data.geom_xpos[geom]
            if not np.isfinite(world_vertices).all() or not math.isfinite(forces[side]):
                raise ValueError('Nonfinite simulated foot geometry/contact')
            minimum_z = float(world_vertices[:, 2].min())
            measured[side] = dict(site_position_m=self.data.site_xpos[site].tolist(),
                mesh_min_height_m=minimum_z, normal_force_n=forces[side],
                contact=forces[side] >= .001,
                lifted=minimum_z >= .003 and forces[side] < .001)
        return measured

    def add(self, at, measured):
        for side, foot in measured.items():
            row, position = self.rows[side], np.asarray(foot['site_position_m'])
            row['samples'] += 1
            row['contact_samples'] += int(foot['contact'])
            row['minimum'] = position if row['minimum'] is None else np.minimum(row['minimum'], position)
            row['maximum'] = position if row['maximum'] is None else np.maximum(row['maximum'], position)
            height = foot['mesh_min_height_m']
            row['min_clearance_m'] = height if row['min_clearance_m'] is None else min(row['min_clearance_m'], height)
            row['max_clearance_m'] = height if row['max_clearance_m'] is None else max(row['max_clearance_m'], height)
            if row['last_contact'] is not None and foot['contact'] != row['last_contact']:
                row['contact_changes'].append(dict(time_s=float(at), contact=foot['contact']))
            row['last_contact'] = foot['contact']
            if foot['lifted']:
                if row['active_lift'] is None:
                    row['active_lift'] = dict(start_s=float(at), max_clearance_m=height,
                                              clearance_s=0., opposite_contact_s=0.)
                lift = row['active_lift']
                lift['clearance_s'] += self.dt
                lift['max_clearance_m'] = max(lift['max_clearance_m'], height)
                if measured['right' if side == 'left' else 'left']['contact']:
                    lift['opposite_contact_s'] += self.dt
            elif foot['contact'] and row['active_lift'] is not None:
                self._end_lift(row, at, completed=True)

    def _end_lift(self, row, at, *, completed):
        lift, row['active_lift'] = row['active_lift'], None
        lift.update(end_s=float(at), duration_s=float(at) - lift['start_s'], completed=completed)
        row['lift_intervals'].append(lift)

    def report(self, at):
        result = {}
        for side, row in self.rows.items():
            if row['active_lift'] is not None:
                self._end_lift(row, at, completed=False)
            lifts = [lift for lift in row['lift_intervals'] if lift['clearance_s'] >= .05 - 1e-9]
            result[side] = dict(contact_fraction=row['contact_samples']/max(1, row['samples']),
                contact_changes=row['contact_changes'], qualifying_lift_intervals=lifts,
                min_mesh_height_m=row['min_clearance_m'], max_mesh_height_m=row['max_clearance_m'],
                site_position_min_m=row['minimum'].tolist(), site_position_max_m=row['maximum'].tolist())
        return dict(contact_force_threshold_n=.001, lift_clearance_threshold_m=.003,
                    lift_duration_threshold_s=.05, feet=result,
                    interpretation='Simulated actual contacts/poses; changes, clearance and displacement alone do not prove walking.')


def load_dependencies(bam_root):
    if not (bam_root / 'bam/model.py').is_file():
        raise ValueError('--bam-root must contain bam/model.py')
    sys.path.insert(0, str(bam_root))
    try:
        import mujoco
        import onnxruntime
        from bam.model import load_model
        from bam.mujoco import MujocoController
    except ImportError as error:
        raise ValueError('CPU audit dependencies required: numpy, onnxruntime, mujoco, '
                         'colorama and BAM. Missing: ' + str(error)) from error
    return mujoco, onnxruntime, load_model, MujocoController


def make_sim(args, mj, load_model, controller_type):
    spec = mj.MjSpec.from_file(str(args.xml))
    official_contract = getattr(args, 'physics_contract', 'official-bam') == 'official-bam'
    if official_contract:
        apply_official_contact_contract(spec)
    spec.worldbody.add_geom(name='policy_audit_floor', type=mj.mjtGeom.mjGEOM_PLANE,
                            size=[0, 0, .05], contype=1, conaffinity=1,
                            friction=[1, .005, .0001])
    bam = load_model(str(args.bam_params))
    gain_path = getattr(args, 'kp_by_id', None)
    gains = load_gain_manifest(gain_path) if gain_path else None
    bam.actuator.kp = (np.asarray([gains['kp_by_id'][str(sid)] for sid in POLICY_IDS], dtype=float)
                       if gains else args.kp)
    bam.actuator.vin = args.voltage
    if len(spec.actuators) != len(POLICY_JOINT_NAMES):
        raise ValueError('This audit requires exactly the 14 policy actuators in the MJCF')
    torque_limit = args.voltage * bam.kt.value / bam.R.value
    for actuator in spec.actuators:
        actuator.set_to_motor()
        actuator.forcerange = [-torque_limit, torque_limit]
        actuator.forcelimited = True
        actuator.gear = [1., 0, 0, 0, 0, 0]
    for joint in spec.joints:
        if joint.name in POLICY_JOINT_NAMES:
            joint.damping = np.zeros((3, 1))
            joint.frictionloss, joint.armature = 0., bam.armature.value
            if official_contract:
                joint.solref_friction = OFFICIAL_FRICTION_SOLREF
                joint.solimp_friction = OFFICIAL_FRICTION_SOLIMP
    model = spec.compile()
    model.opt.timestep = args.physics_dt_ms / 1000
    cad_mass = float(model.body_mass.sum())
    if args.mass_kg is not None:
        scale = args.mass_kg / cad_mass
        model.body_mass[:] *= scale
        model.body_inertia[:] *= scale
    data = mj.MjData(model)
    controller = controller_type(bam, list(POLICY_JOINT_NAMES), model, data)
    q_indices = model.jnt_qposadr[[model.joint(name).id for name in POLICY_JOINT_NAMES]]
    v_indices = model.jnt_dofadr[[model.joint(name).id for name in POLICY_JOINT_NAMES]]
    trunk = model.body('trunk_base').id
    root_joint = int(model.body_jntadr[trunk])
    if model.jnt_type[root_joint] != mj.mjtJoint.mjJNT_FREE:
        raise ValueError('trunk_base must have a free joint')
    return model, data, bam, controller, q_indices, v_indices, trunk, root_joint, cad_mass


def rollout(args, dependencies, session, profile, cfg, hardware, snapshot_angles, speed, executor):
    mj, _, load_model, controller_type = dependencies
    model, data, bam, controller, qi, vi, trunk, root_joint, cad_mass = make_sim(
        args, mj, load_model, controller_type)
    home = profile['home_rad']
    original_metadata_home = policy_profile(session)['home_rad']
    if args.head_source == 'metadata-home':
        head_deg = np.rad2deg(home[5:9])
    elif args.head_source == 'captured':
        captured = hardware.get('hold_head_deg')
        if not isinstance(captured, dict) or any(str(sid) not in captured for sid in (30, 31, 32, 33)):
            raise ValueError('Selected snapshot has no complete captured hold_head_deg; choose --head-source snapshot')
        head_deg = np.asarray([captured[str(sid)] for sid in (30, 31, 32, 33)], dtype=np.float64)
    else:
        head_deg = snapshot_angles[10:14]
    if not np.isfinite(head_deg).all():
        raise ValueError('Snapshot head pose is nonfinite')
    initial = snapshot_angles.copy() if args.initial_pose == 'snapshot' else np.asarray(stance_from_home(home))
    if args.initial_pose == 'hold':
        for sid, angle in FIXED_STANCE_OVERRIDES.items():
            initial[MOTOR_IDS.index(sid)] = angle
    initial[10:14] = head_deg
    # ARM obtains integer encoder ticks; begin MCU continuous goals at exactly
    # that representable position, rather than a fractional synthetic tick.
    initial = ticks_to_angles(angles_to_ticks(initial, cfg), cfg)
    root_adr = int(model.jnt_qposadr[root_joint])
    roll = math.radians(args.initial_roll_deg)
    pitch = math.radians(args.initial_pitch_deg)
    # Rz(0) Ry(pitch) Rx(roll), wxyz.
    cr, sr, cp, sp = math.cos(roll/2), math.sin(roll/2), math.cos(pitch/2), math.sin(pitch/2)
    data.qpos[root_adr:root_adr+7] = [0, 0, args.root_height_m, cp*cr, cp*sr, sp*cr, -sp*sr]
    data.qpos[qi] = np.deg2rad(initial[POLICY_INDEX])
    data.qvel[:] = 0
    data.ctrl[:] = 0
    mj.mj_forward(model, data)
    controller.reset(data.qpos)
    bam.actuator.q_target_smooth = data.qpos[qi].copy()
    controller.last_ts = float(data.time)
    observed = mj.MjData(model)
    refresh_observation_data(mj, model, data, observed)
    dt = float(model.opt.timestep)
    # Inference runs on the first physics grid point at/after each requested
    # deadline. This permits 35 Hz on a 5 ms grid without claiming exact
    # 28.571 ms transport timing (the intervals will alternate 25/30 ms).
    next_control_at, control_times = 0., []
    feedback_steps = round(args.feedback_delay_ms / 1000 / dt)
    execution_steps = round(args.execution_delay_ms / 1000 / dt)
    slew_steps = round(.01 / dt)
    zeros = np.asarray([j['zero'] for j in cfg['joints']], dtype=np.float64)
    signs = np.asarray([j['sign'] for j in cfg['joints']])
    mouth_angle = float(initial[14])
    filter_mode = getattr(args, 'observation_filter', 'truth')
    observation_filter = ImuFilter() if filter_mode == 'host' else None
    last_filtered_at = None

    def current_state():
        angles, vels = np.zeros(15), np.zeros(15)
        angles[POLICY_INDEX] = np.rad2deg(observed.qpos[qi])
        vels[POLICY_INDEX] = np.rad2deg(observed.qvel[vi])
        angles[14] = mouth_angle  # No jaw actuator in this 14-joint MJCF.
        rotation = observed.xmat[trunk].reshape(3, 3)
        gravity = rotation.T @ np.array([0., 0., -1.])
        velocity = np.zeros(6)
        mj.mj_objectVelocity(model, observed, mj.mjtObj.mjOBJ_BODY, trunk, velocity, 1)
        # The XML IMU site's frame equals trunk_base. MuJoCo accelerometer
        # reports specific force; native motion.py supplies its negative to
        # ImuFilter through the filter's legacy sensor-axis inverse.
        acceleration = observed.sensor('imu_accel').data
        angular = observed.sensor('imu_ang_vel').data
        sensor_acc = [acceleration[1], acceleration[2], -acceleration[0]]
        body_gyro_deg = np.rad2deg(angular)
        sensor_gyro = [-body_gyro_deg[1], -body_gyro_deg[2], body_gyro_deg[0]]
        return State(sensor_acc, sensor_gyro, angles.tolist(), vels.tolist(),
                     gravity.tolist(), velocity[:3].tolist())

    traces = {name: Trace(cfg) for name in ('onnx_unfiltered_target', 'retargeted_unfiltered_target',
              'adapted_pre_ema_target', 'policy_smoothed_target',
              'requested_execution_target', 'mcu_applied_goal', 'actual_joint')}
    state_history = deque([current_state() for _ in range(feedback_steps + 1)], maxlen=feedback_steps + 1)
    initial_gravity = current_state().grav
    queue = deque([angles_to_ticks(initial, cfg) for _ in range(execution_steps)])
    # Continuous goals begin at the actual arm position, as on the MCU.
    continuous = zeros + initial / 360 * 4095 * signs
    desired = angles_to_ticks(initial, cfg)
    applied = initial.copy()
    previous, smooth = np.zeros(14, np.float32), np.zeros(14, np.float32)
    head_command = (np.deg2rad(head_deg) - home[5:9]).astype(np.float32)
    action_min, action_max = None, None
    max_tilt, min_height, head_deviation = 0., args.root_height_m, np.zeros(4)
    frames, motion_frames, ended = [], [], None
    foot_trace = FootMotionTrace(mj, model, observed, dt)
    total_steps = math.ceil(args.duration_s / dt)
    progress_origin = None
    progress_origin_at = None
    for step in range(total_steps):
        if progress_origin is None and data.time + 1e-9 >= args.zero_warmup_s:
            progress_origin = float(data.qpos[root_adr])
            progress_origin_at = float(data.time)
        st = current_state()
        tilt = math.degrees(math.acos(float(np.clip(-st.grav[2], -1, 1))))
        feet = foot_trace.measure()
        foot_trace.add(data.time, feet)
        max_tilt, min_height = max(max_tilt, tilt), min(min_height, float(data.qpos[root_adr+2]))
        head_deviation = np.maximum(head_deviation, abs(np.asarray(st.angles[10:14]) - head_deg))
        if tilt >= 40 or data.qpos[root_adr+2] < .08:
            ended = dict(reason='fall_criterion', time_s=float(data.time), tilt_deg=tilt)
            break
        actual_bad = traces['actual_joint'].add(st.angles, data.time)
        if actual_bad and not args.continue_after_range_violation:
            ended = dict(reason='actual_joint_range', time_s=float(data.time), joints=actual_bad)
            break
        if data.time + 1e-9 >= next_control_at:
            control_times.append(float(data.time))
            next_control_at += 1 / args.hz
            command = np.array([scheduled_command(data.time, speed,
                getattr(args, 'zero_warmup_s', 0.), getattr(args, 'pulse_s', None), args.ramp_seconds), 0., 0.], np.float32)
            policy_state = state_history[0]
            if observation_filter is not None:
                # One new feedback sample per control cycle; mutating a deque
                # sample would apply filtering repeatedly to the same history.
                filter_dt = 1 / args.hz if last_filtered_at is None else data.time - last_filtered_at
                policy_state = observation_filter.update(copy.deepcopy(policy_state), filter_dt)
                last_filtered_at = float(data.time)
            obs = build_obs(policy_state, previous, command, head_command, home_rad=home).reshape(1, 61)
            if not np.isfinite(obs).all():
                raise ValueError('Nonfinite simulated observation')
            action = np.asarray(session.run(None, {session.get_inputs()[0].name: obs})[0])
            if action.shape != (1, 14) or not np.isfinite(action).all():
                raise ValueError('ONNX emitted invalid output')
            action = action[0]
            adapted = execution_action(action, profile, cfg, getattr(args, 'decoder', 'metadata'))
            action_min = action.copy() if action_min is None else np.minimum(action_min, action)
            action_max = action.copy() if action_max is None else np.maximum(action_max, action)
            smooth = smooth_action(smooth, adapted, args.action_alpha, args.head_action_alpha)
            raw_target = action_to_deg(action, home_rad=original_metadata_home, action_scale=profile['scale'])
            retargeted = action_to_deg(action, home_rad=home, action_scale=profile['scale'])
            before_ema = action_to_deg(adapted, home_rad=home, action_scale=profile['scale'])
            target = action_to_deg(smooth, home_rad=home, action_scale=profile['scale'])
            requested = np.asarray(target)
            if executor == 'fixed-head':
                requested[10:14] = head_deg
            traces['onnx_unfiltered_target'].add(raw_target, data.time)
            traces['retargeted_unfiltered_target'].add(retargeted, data.time)
            traces['adapted_pre_ema_target'].add(before_ema, data.time)
            traces['policy_smoothed_target'].add(target, data.time)
            requested_bad = traces['requested_execution_target'].add(requested, data.time)
            if args.include_frames:
                motion_frames.append(dict(time_s=float(data.time),
                    base_position_m=data.qpos[root_adr:root_adr+3].tolist(),
                    base_quaternion_wxyz=data.qpos[root_adr+3:root_adr+7].tolist(),
                    actual_joint_deg=st.angles, tilt_deg=tilt, feet=feet))
                frames.append(dict(time_s=float(data.time), command=command.tolist(),
                                   simulated_base_position_m=data.qpos[root_adr:root_adr+3].tolist(),
                                   simulated_joint_deg=st.angles,
                                   onnx_action=action.tolist(), policy_target_deg=target,
                                   requested_execution_deg=requested.tolist()))
            if requested_bad and not args.continue_after_range_violation:
                ended = dict(reason='requested_execution_range', time_s=float(data.time), joints=requested_bad)
                break
            previous = action.copy()  # Always RAW neural action; never physical overrides.
            desired = angles_to_ticks(requested, cfg)
        queue.append(desired.copy())
        delayed_desired = queue.popleft()
        if args.slew_ticks_per_second:
            if step > 0 and step % slew_steps == 0:
                bound = args.slew_ticks_per_second * min(slew_steps * dt, .05)
                continuous += np.clip(delayed_desired - continuous, -bound, bound)
                # MCU lroundf for nonnegative ticks. No mechanical-limit clamp.
                applied = ticks_to_angles(np.floor(continuous + .5), cfg)
        else:
            applied = ticks_to_angles(delayed_desired, cfg)
        traces['mcu_applied_goal'].add(applied, data.time)
        controller.q_target = np.deg2rad(applied[POLICY_INDEX])
        controller.update()
        mj.mj_step(model, data)
        refresh_observation_data(mj, model, data, observed)
        state_history.append(current_state())
    final = current_state()
    traces['actual_joint'].add(final.angles, data.time)
    final_tilt = math.degrees(math.acos(float(np.clip(-final.grav[2], -1, 1))))
    max_tilt = max(max_tilt, final_tilt)
    head_deviation = np.maximum(head_deviation, abs(np.asarray(final.angles[10:14]) - head_deg))
    if args.include_frames:
        motion_frames.append(dict(time_s=float(data.time),
            base_position_m=data.qpos[root_adr:root_adr+3].tolist(),
            base_quaternion_wxyz=data.qpos[root_adr+3:root_adr+7].tolist(),
            actual_joint_deg=final.angles, tilt_deg=final_tilt, feet=foot_trace.measure()))
    intervals = np.diff(control_times)
    control_schedule = dict(requested_hz=args.hz, samples=len(control_times),
                            mean_interval_ms=float(intervals.mean()*1000) if intervals.size else None,
                            min_interval_ms=float(intervals.min()*1000) if intervals.size else None,
                            max_interval_ms=float(intervals.max()*1000) if intervals.size else None,
                            mean_hz=float(1/intervals.mean()) if intervals.size else None,
                            physics_grid_ms=args.physics_dt_ms)
    result = dict(speed_m_s=speed, executor=executor, simulated_s=float(data.time),
                  range_ignoring_counterfactual=bool(args.continue_after_range_violation),
                  ended=ended or dict(reason='duration', time_s=float(data.time)),
                  cad_mass_kg=cad_mass, simulated_mass_kg=float(model.body_mass.sum()),
                  initial_joint_deg=initial.tolist(), head_command_rad=head_command.tolist(),
                  initial_trunk_gravity=initial_gravity,
                  control_schedule=control_schedule,
                  initial_raw_ticks=angles_to_ticks(initial, cfg).tolist(),
                  displacement_m=data.qpos[root_adr:root_adr+2].tolist(),
                  max_tilt_deg=max_tilt, final_tilt_deg=final_tilt,
                  min_root_height_m=min(min_height, float(data.qpos[root_adr+2])),
                  final_root_height_m=float(data.qpos[root_adr+2]),
                  head_max_deviation_deg=head_deviation.tolist(),
                  actual_foot_motion=foot_trace.report(data.time),
                  onnx_action_min=None if action_min is None else action_min.tolist(),
                  onnx_action_max=None if action_max is None else action_max.tolist(),
                  outputs={name: trace.report() for name, trace in traces.items()})
    planned = command_distance(args.duration_s, speed, args.zero_warmup_s, args.pulse_s, args.ramp_seconds)
    observed = command_distance(float(data.time), speed, args.zero_warmup_s, args.pulse_s, args.ramp_seconds)
    progress = float(data.qpos[root_adr]) - progress_origin if progress_origin is not None else None
    result['command_progress'] = dict(origin_at_s=progress_origin_at,
        origin_x_m=progress_origin, planned_distance_m=planned,
        scheduled_distance_before_stop_m=observed, displacement_after_warmup_m=progress,
        direction_progress_m=math.copysign(1., speed)*progress if progress is not None else None,
        distance_tracking_error_m=progress-observed if progress is not None else None,
        achieved_fraction_of_planned=progress/planned if progress is not None and planned else None,
        scope='Simulated root displacement from warmup end through recovery; excludes initial standing drift. Not an odometry or physical gait acceptance test.')
    if args.include_frames:
        result['frames'] = frames
        result['actual_motion_frames'] = motion_frames
    result['assessment'] = assess_result(result)
    return result


def parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--snapshot', type=Path, required=True)
    p.add_argument('--model', type=Path, default=ROOT / 'output/x5_policy_bundle/models/xgoduck_walk.onnx')
    p.add_argument('--xml', type=Path, default=REFERENCE / 'src/mjlab_microduck/robot/xgoduck/robot_walk.xml')
    p.add_argument('--bam-root', type=Path, default=REFERENCE / 'vendor/bam')
    p.add_argument('--bam-params', type=Path, help='1910_m6.json file or directory containing it; default: XML sibling params/')
    p.add_argument('--head-source', choices=('snapshot', 'captured', 'metadata-home'), default='snapshot')
    p.add_argument('--initial-pose', choices=('hold', 'snapshot'), default='hold')
    p.add_argument('--speeds', type=float, nargs='+', default=[0., .2])
    p.add_argument('--executors', choices=('policy', 'fixed-head'), nargs='+', default=['fixed-head'])
    p.add_argument('--duration-s', type=float, default=6.)
    p.add_argument('--hz', type=float, default=50.)
    p.add_argument('--physics-dt-ms', type=float, default=5.)
    p.add_argument('--physics-contract', choices=('official-bam', 'legacy-cpu'), default='official-bam',
                   help='official-bam matches audited friction/contact spec settings; legacy-cpu exists only for reproducible A/B comparisons')
    p.add_argument('--feedback-delay-ms', type=float, default=40.)
    p.add_argument('--execution-delay-ms', type=float, default=40.)
    p.add_argument('--observation-filter', choices=('truth', 'host'), default='truth',
                   help='truth uses exact simulated gravity/velocities; host applies native ImuFilter to simulated IMU specific force and rates at the control cadence')
    p.add_argument('--slew-ticks-per-second', type=float, default=0., help='0 disables MCU slew; 600 models present firmware goal slew')
    p.add_argument('--action-alpha', type=float, default=ACTION_ALPHA,
                   help='Leg/yaw action EMA alpha: 0 applies actor actions directly; .45 is the audited hardware-runtime value')
    p.add_argument('--head-action-alpha', type=float, default=ACTION_ALPHA_HEAD,
                   help='Head action EMA alpha, policy indices 5:9; head freezing remains a separate executor setting')
    p.add_argument('--ramp-seconds', type=float, default=0.)
    p.add_argument('--zero-warmup-s', type=float, default=0.)
    p.add_argument('--pulse-s', type=float, help='Bounded forward duration, followed by zero command')
    p.add_argument('--decoder', choices=('metadata', 'homeprime-roll95'), default='metadata',
                   help='Compare original metadata with the frozen currently deployed HOME/roll adapter')
    p.add_argument('--mass-kg', type=float, default=.873, help='Uniform CAD mass/inertia scaling; a hypothesis about mass distribution')
    p.add_argument('--voltage', type=float, default=7.7)
    gain_group = p.add_mutually_exclusive_group()
    gain_group.add_argument('--kp', type=float, default=5., help='Uniform KP hypothesis, not a hardware register measurement')
    gain_group.add_argument('--kp-by-id', type=Path,
        help='Source-verified per-ID gain manifest; maps 15 saved IDs to the 14 simulated policy actuators')
    p.add_argument('--root-height-m', type=float, default=.14)
    p.add_argument('--initial-roll-deg', type=float, default=0.)
    p.add_argument('--initial-pitch-deg', type=float, default=0.)
    p.add_argument('--continue-after-range-violation', action='store_true', help='Counterfactual dynamics after a real controller would stop')
    p.add_argument('--include-frames', action='store_true')
    p.add_argument('--output', type=Path, help='New report path; input files are never modified')
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    try:
        for key, value in vars(args).items():
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(key + ' must be finite')
        if (not 0 < args.duration_s <= 30 or not 0 < args.hz <= 200
                or not 0 < args.physics_dt_ms <= 10 or not 0 < args.mass_kg <= 5
                or not 0 < args.voltage <= 12 or not 0 < args.kp <= 100
                or not .08 < args.root_height_m < 1):
            raise ValueError('Invalid finite duration/frequency/mass/actuator settings')
        if any(not math.isfinite(v) or abs(v) > .5 for v in args.speeds) or len(args.speeds)*len(args.executors) > 8:
            raise ValueError('At most 8 cases; each speed must be finite and within ±0.5 m/s')
        if any(not 0 <= v <= 1000 for v in (args.feedback_delay_ms, args.execution_delay_ms)) or not 0 <= args.slew_ticks_per_second <= 10000 or not 0 <= args.ramp_seconds <= 30:
            raise ValueError('Delays/slew/ramp must be nonnegative and bounded')
        if any(not 0 <= v < 1 for v in (args.action_alpha, args.head_action_alpha)):
            raise ValueError('Action EMA alpha must be within [0, 1)')
        if not 0 <= args.zero_warmup_s < args.duration_s or (args.pulse_s is not None and
                (not 0 < args.pulse_s <= 30 or args.ramp_seconds*2 > args.pulse_s
                 or args.zero_warmup_s + args.pulse_s >= args.duration_s)):
            raise ValueError('Pulse needs a valid ramp and a post-pulse zero observation window')
        dt = args.physics_dt_ms / 1000
        if args.hz * dt > 1 + 1e-9:
            raise ValueError('Control frequency cannot exceed the physics grid frequency')
        for name, count in [('10 ms MCU goal period', .01 / dt),
                            ('feedback delay', args.feedback_delay_ms / 1000 / dt),
                            ('execution delay', args.execution_delay_ms / 1000 / dt)]:
            if not math.isclose(count, round(count), abs_tol=1e-7) or name == '10 ms MCU goal period' and count < 1:
                raise ValueError(name + ' must be an integer number of physics steps')
        for name in ('snapshot', 'model', 'xml', 'bam_root'):
            setattr(args, name, getattr(args, name).resolve())
        args.bam_params = (args.bam_params or args.xml.parent / 'params').resolve()
        if args.bam_params.is_dir():
            args.bam_params /= '1910_m6.json'
        if args.output and args.output.resolve() in (args.snapshot, args.model, args.xml, args.bam_params):
            raise ValueError('--output must not overwrite an input file')
        snapshot = json.loads(args.snapshot.read_text())
        cfg, hardware, hardware_source, angles = validate_snapshot(snapshot)
        gain_manifest = load_gain_manifest(args.kp_by_id) if args.kp_by_id else None
        dependencies = load_dependencies(args.bam_root)
        mj, ort, _, _ = dependencies
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = opts.inter_op_num_threads = 1
        session = ort.InferenceSession(str(args.model), sess_options=opts, providers=['CPUExecutionProvider'])
        profile = policy_profile(session)
        if args.decoder == 'homeprime-roll95':
            profile = dict(profile, home_rad=profile['home_rad'].copy())
            for sid, offset in {12:-1.,14:-1.,22:1.,24:1.}.items():
                profile['home_rad'][POLICY_IDS.index(sid)] += math.radians(offset)
        report = dict(kind='bounded_cpu_onnx_bam_audit_not_hardware_validation',
                      uart_opens=0, motor_commands=0, calibration_writes=0,
                      snapshot=str(args.snapshot), hardware_sample_source=hardware_source,
                      snapshot_hardware_mode=hardware.get('mode'),
                      snapshot_hardware_imu=hardware.get('imu'),
                      initial_trunk_assumption=dict(height_m=args.root_height_m,
                          roll_deg=args.initial_roll_deg, pitch_deg=args.initial_pitch_deg,
                          yaw_deg=0., linear_velocity_m_s=[0.,0.,0.], angular_velocity_rad_s=[0.,0.,0.]),
                      source_sha256={name:digest(path) for name,path in (
                          ('snapshot',args.snapshot),('model',args.model),('xml',args.xml),
                          ('bam_params',args.bam_params),('audit_script',Path(__file__)),
                          ('rl_core',Path(__file__).with_name('rl_core.py')),
                          ('bam_model',args.bam_root / 'bam/model.py'),
                          ('bam_mujoco',args.bam_root / 'bam/mujoco.py'),
                          ('bam_mjlab_spec_contract',REFERENCE / 'vendor/bam/bam/mjlab.py'),
                          ('training_robot_spec_contract',REFERENCE / 'src/mjlab_microduck/robot/xgoduck_constants.py'),
                          ('bam_actuator',args.bam_root / 'bam/actuator.py'),
                          ('bam_feetech_actuator',args.bam_root / 'bam/feetech/actuator.py'))},
                      calibration_sha256=hashlib.sha256(json.dumps(cfg, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                      calibration=cfg, motor_order=MOTOR_IDS, policy_order=POLICY_IDS,
                      snapshot_joint_deg_recomputed_from_raw_ticks=angles.tolist(),
                      versions=dict(mujoco=mj.__version__, onnxruntime=ort.__version__),
                      actuator_gain_contract=(dict(gain_manifest,
                          manifest_sha256=digest(args.kp_by_id), simulated_policy_order=POLICY_IDS,
                          simulated_kp=[gain_manifest['kp_by_id'][str(sid)] for sid in POLICY_IDS],
                          firmware_kd_independently_modeled=False, mouth_actuator_simulated=False)
                          if gain_manifest else dict(kind='uniform_kp_hypothesis', kp=args.kp,
                              current_servo_registers_measured=False,
                              firmware_kd_independently_modeled=False,
                              interpretation='Scalar historical/experimental assumption, not the installed gain configuration')),
                      source_contract=dict(
                          physics=args.physics_contract,
                          official_friction_solref=list(OFFICIAL_FRICTION_SOLREF) if args.physics_contract == 'official-bam' else None,
                          official_friction_solimp=list(OFFICIAL_FRICTION_SOLIMP) if args.physics_contract == 'official-bam' else None,
                          hardware_runtime_ema=dict(leg=ACTION_ALPHA, head=ACTION_ALPHA_HEAD),
                          selected_ema=dict(leg=args.action_alpha, head=args.head_action_alpha),
                          full_training_environment_reproduced=False,
                          deployed=False),
                      settings={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items() if k != 'output'},
                      limitations=[
                          'CPU MuJoCo/vendor NumPy BAM is not the exact GPU checkpoint training environment.',
                          'official-bam matches the audited source friction/contact edits only; checkpoint provenance, domain randomization and solver parity remain unverified.',
                          'The selected calibration is preserved; this audit does not independently establish physical zero/sign or IMU-to-CAD correspondence.',
                          'HOLD initialization uses metadata legs plus current fixed-stance overrides; snapshot initialization uses raw ticks. Neither infers support, ground contact or trunk height from hardware.',
                          'Initial trunk attitude is an explicit CLI assumption; snapshot IMU tilt is not silently cancelled or treated as CAD truth.',
                          'Control deadlines are quantized upward to the physics grid; reported intervals describe this discretization, not measured host or UART jitter.',
                          'Uniform total mass scaling assumes CAD mass distribution; X5 battery/harness mass locations and collisions are not measured.',
                          'BAM KP may use a scalar hypothesis or source-derived per-ID vector; neither is a current register readback or complete identification of firmware KD and loaded actuator response.',
                          ('Feedback uses delayed MuJoCo IMU specific force/rates through the native ImuFilter at the control cadence.'
                           if args.observation_filter == 'host' else
                           'Feedback contains ideal delayed gravity/angular/joint states; native IMU/velocity filtering is omitted.'),
                          'IMU mounting errors/noise, asynchronous servo/IMU acquisition, encoder quantization, transport jitter and packet loss are omitted.',
                          'Nonzero slew models continuous MCU goals at 10 ms with lroundf and 50 ms elapsed cap, without mechanical range clamping; the deployed value is 600 ticks/s.',
                          'Runtime head-deviation, confirmation, heartbeat and watchdog gates are not authorization by this offline experiment. Range-stop behavior is reported explicitly.',
                          'The 14-joint MJCF has no mouth actuator. Actual mouth angle is held at its initial value for reporting; policy mouth requests remain zero.',
                          'Survival or displacement alone does not establish successful walking or safe hardware operation.'],
                      results=[])
        for executor in args.executors:
            for speed in args.speeds:
                result = rollout(args, dependencies, session, profile, cfg, hardware, angles, speed, executor)
                report['results'].append(result)
                print(f"{executor}, {speed:g} m/s: {result['ended']['reason']} at "
                      f"{result['simulated_s']:.3f} s, max tilt {result['max_tilt_deg']:.2f} deg", file=sys.stderr)
        content = json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(content)
            print(str(args.output.resolve()))
        else:
            print(content, end='')
        return 0
    except (ValueError, OSError, KeyError, TypeError) as error:
        p.error(str(error))


if __name__ == '__main__':
    sys.exit(main())
