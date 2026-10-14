"""Offline-only bridge to a reviewed deployment's actual policy/encoder code.

No UART is constructed. Saved calibration is an explicit coordinate assumption,
not proof that encoder zero agrees with the physical CAD zero. This bridge tests
the output path, not the hardware sensor filter, scheduling, or power supply.
"""
import copy
import hashlib
import importlib
import json
import math
import sys
from pathlib import Path
import numpy as np


class InstalledActionPath:
    def __init__(self, candidate, calibration_path, head_profile, session):
        self.root = Path(candidate).resolve()
        policy = self.root / 'policy'
        sys.path.insert(0, str(policy))
        import motion
        import rl_core
        for module in (motion, rl_core, importlib.import_module('production_motion')):
            if Path(module.__file__).resolve().parent != policy:
                raise RuntimeError('Another policy module was imported before the declared deployment')
        self.motion, self.core = motion, rl_core
        self.now = 0.
        self.controller = motion.MotionController({'walk': session}, calibration_path,
                                                   walk_head_command_rad=head_profile,
                                                   clock=lambda: self.now)
        assert self.controller._validation_only and self.controller._transport is None
        self.calibration = json.loads(Path(calibration_path).read_text())
        assert self.controller._config == self.calibration
        self.joints = {j['id']: j for j in self.calibration['joints']}
        self.previous = np.zeros(14, np.float32)
        self.smooth = self.previous.copy()
        self.was_holding = False
        self.last = None
        self.provenance = dict(
            scope='actual deployed decoder, HOLD handoff, execution projection and encoder rounding; ideal simulated sensor input',
            candidate=str(self.root), hardware_access=False,
            calibration_sha256=hashlib.sha256(Path(calibration_path).read_bytes()).hexdigest(),
            source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in sorted(policy.glob('*.py'))},
            physical_zero_alignment_assumed=True, physical_balance_verified=False)

    def overlay(self, candidate):
        """Declare a full current calibration overlay, separate from range-only API."""
        from model_adaptation_contract import saved_joint_contract
        from offline_calibration_overlay import digest
        out = copy.deepcopy(candidate)
        before = {j['id']: j for j in out['calibration']['joints']}
        rows = saved_joint_contract(self.calibration)
        self.provenance['calibration_changes'] = [dict(id=j['id'], changes={
            k: [before[j['id']][k], j[k]] for k in ('zero_tick','sign','min_tick','max_tick')
            if before[j['id']][k] != j[k]}) for j in rows if any(
            before[j['id']][k] != j[k] for k in ('zero_tick','sign','min_tick','max_tick'))]
        out['calibration'] = dict(out['calibration'], sha256=digest(self.calibration), joints=rows)
        by_id = {j['id']: j for j in rows}
        out['lower_rad'] = [math.radians(by_id[s]['min_deg']) for s in out['policy_joint_ids']]
        out['upper_rad'] = [math.radians(by_id[s]['max_deg']) for s in out['policy_joint_ids']]
        return out

    def observation_from_sensors(self, model_angles, model_velocities, acc_body,
                                 gyro_body_rad_s, sample_time, command):
        """Replay the deployed feedback conversion/filter on simulated sensors.

        The CAD imu site, exact timestamps and ideal axis calibration are
        assumptions. This does not identify real mounting error or supply noise.
        """
        c = self.controller
        self.now = float(sample_time)
        _, matrix = self.motion._axis_matrix(self.calibration['imu']['axes'])
        pos = [j['zero'] for j in c._config['joints']]
        vel = [0] * 15
        for i, sid in enumerate(self.core.POLICY_IDS):
            j = self.joints[sid]; index = self.core.MOTOR_IDS.index(sid)
            pos[index] = int(round(j['zero']+math.degrees(float(model_angles[i]))/360*4095*j['sign']))
            vel[index] = int(round(math.degrees(float(model_velocities[i])) /
                (self.motion.SPEED_COUNTS_PER_UNIT*360/4096*j['sign'])))
        sample = int(round(sample_time*1e6))
        raw = dict(sample_us=sample, imu_us=sample, armed=True, configured=True,
                   cookie=1, online_mask=self.motion.ALL_MASK, fault_mask=0,
                   torque_mask=0, torque_known_mask=0, imu_ok=True, last_error=0,
                   positions=pos, velocities=vel, source_runtime=True,
                   acc=(matrix.T@np.asarray(acc_body)).tolist(),
                   gyro=(matrix.T@np.rad2deg(gyro_body_rad_s)+self.calibration['imu']['bias_dps']).tolist())
        c._accept_state_locked(raw)
        assert c._filtered is not None
        return c._prepare_policy_observation('walk',c._filtered,self.previous,
            np.asarray(command,np.float32),0.,head_command=c._walk_head_command_locked())

    def step(self, raw, holding):
        c = self.controller
        if holding:
            requested = c._fixed_stance_locked()
            c._mode = 'hold'
        else:
            if self.was_holding:
                self.smooth = c._policy_start_smooth_locked('walk')
            c._mode = 'walk'
            requested, self.previous, self.smooth, _ = c._decode_policy_action(
                'walk', raw, self.previous, self.smooth, 0., 0.)
        projected = c._execution_target_locked(c._mode, requested)
        ticks = c._raw_targets_locked(projected, mode=c._mode)
        c._last_executed_target = list(projected)
        ids, motor_ids = self.core.POLICY_IDS, self.core.MOTOR_IDS
        # Plant target is the exact integer encoder command translated using
        # that same saved coordinate map, including its sign and quantization.
        physical = [math.radians((ticks[motor_ids.index(s)]-self.joints[s]['zero'])
                        *360/4095*self.joints[s]['sign']) for s in ids]
        self.last = dict(requested_policy_rad=[math.radians(requested[motor_ids.index(s)]) for s in ids],
                         target_policy_rad=physical, ticks=ticks,
                         projection=copy.deepcopy(c._target_projection),
                         previous_raw=self.previous.tolist(), ema=self.smooth.tolist())
        self.was_holding = holding
        return np.asarray(physical, np.float32)
