"""864 g four-task adapter bound to the reviewed production action contract.

Offline simulation only. Original task rewards and terminations are retained.
The body scenarios are explicitly uncertain priors, not measured COM values.
This supersedes the old fixed-head/range-failure/600-tick training adapter.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, fields
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch
from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
from mjlab.managers import EventTermCfg

POLICY_IDS = (10, 11, 12, 13, 14, 30, 31, 32, 33, 20, 21, 22, 23, 24)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_contract(path):
    path = Path(path).resolve()
    contract = json.loads(path.read_text())
    for item in contract['files'].values():
        item['resolved_path'] = str((path.parent / item['path']).resolve())
        if sha(item['resolved_path']) != item['sha256']:
            raise ValueError('Bound training input changed: ' + item['path'])
    if contract['body']['total_mass_kg'] != .864:
        raise ValueError('Use the authoritative 864 g measurement')
    return contract


class RuntimeDecoder:
    """Batched equivalent of float32 EMA -> float32 degrees -> integer target.

    Projection is conditional min+1/max-1 ONLY outside the saved tick interval.
    No fixed-head override, slew, actual-position stop or feedback fabrication.
    Keep double precision for Python/numpy's HOME/degrees/encoder arithmetic.
    """
    def __init__(self, model, joints, count, device, head_seed=None):
        tensor = lambda v: torch.as_tensor(v, dtype=torch.float64, device=device)
        rows = {j['id']: j for j in joints}
        self.home = tensor(model['home_rad'])
        self.scale = model['action_scale']
        self.zero = tensor([rows[i]['zero'] for i in POLICY_IDS])
        self.sign = tensor([rows[i]['sign'] for i in POLICY_IDS])
        self.low = tensor([rows[i]['min'] for i in POLICY_IDS])
        self.high = tensor([rows[i]['max'] for i in POLICY_IDS])
        self.alpha = .15 if model['mode'] == 'roulade' else .45
        self.seed = torch.zeros(14, device=device, dtype=torch.float32)
        if head_seed is not None:
            self.seed[5:9] = torch.tensor(head_seed, device=device, dtype=torch.float32)
        self.ema = self.seed.repeat(count, 1)
        self.ticks = torch.zeros(count, 14, device=device, dtype=torch.int64)
        self.projected = torch.zeros_like(self.ticks, dtype=torch.bool)

    def reset(self, ids=None):
        self.ema[slice(None) if ids is None else ids] = self.seed

    def step(self, raw):
        if raw.shape != self.ema.shape:
            raise ValueError('Expected batched 14D policy output')
        self.ema.mul_(self.alpha)
        self.ema.add_(raw.to(torch.float32) * (1-self.alpha))
        # action_to_deg casts the complete target vector to float32.
        degree = torch.rad2deg(self.home + self.scale*self.ema).to(torch.float32)
        ticks = torch.round(self.zero + degree.to(torch.float64)/360*4095*self.sign)
        bounded = torch.where(ticks < self.low, self.low+1,
                              torch.where(ticks > self.high, self.high-1, ticks))
        self.projected = ticks != bounded
        self.ticks = bounded.to(torch.int64)
        return ((bounded-self.zero)*360/4095*self.sign*(math.pi/180)).to(torch.float32)

    def position_observation(self, positions):
        ticks = torch.round(self.zero + torch.rad2deg(positions.to(torch.float64))/360*4095*self.sign)
        degrees = ((ticks-self.zero)*360/4095*self.sign).to(torch.float32)
        return (torch.deg2rad(degrees).to(torch.float64)-self.home).to(torch.float32)

    def velocity_observation(self, velocities):
        raw = torch.round(torch.rad2deg(velocities.to(torch.float64))/(50*360/4096*self.sign))
        degrees = (raw*50*360/4096*self.sign).to(torch.float32)
        return torch.deg2rad(degrees)


@dataclass(kw_only=True)
class RuntimeActionCfg(JointPositionActionCfg):
    model_contract: dict
    calibration: list
    head_seed: list | None = None
    reset_freeze_seconds: float = 0.

    def build(self, env):
        return RuntimeAction(self, env)


class RuntimeAction(JointPositionAction):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        if tuple(self._target_names) != tuple(cfg.model_contract['joint_names']):
            raise ValueError('Actuator order differs from pinned ONNX joint order')
        self.decoder = RuntimeDecoder(cfg.model_contract, cfg.calibration,
                                      self.num_envs, self.device, cfg.head_seed)
        self._processed_actions = self.decoder.step(torch.zeros_like(self._raw_actions))
        self.decoder.reset()

    def process_actions(self, actions):
        raw = actions
        if self.cfg.reset_freeze_seconds:
            # Official getup task drops then freezes HOME for 2 s. This is
            # training reset preparation, not a new runtime getup delay.
            frozen = self._env.episode_length_buf*self._env.step_dt < self.cfg.reset_freeze_seconds
            raw = torch.where(frozen[:, None], torch.zeros_like(actions), actions)
            self._env.action_manager._action[frozen] = 0.
        self._raw_actions[:] = raw
        self._processed_actions = self.decoder.step(raw)

    def apply_actions(self):
        # Encoder calibration uncertainty belongs to observations. Unlike the
        # old source action, do not subtract a fictitious per-env encoder bias
        # from the physical target after this calibrated encoder projection.
        self._entity.set_joint_position_target(self._processed_actions, joint_ids=self._target_ids)

    def reset(self, env_ids=None):
        super().reset(env_ids)
        self.decoder.reset(env_ids)


def quantized_position(env, asset_cfg, biased=False):
    term = env.action_manager.get_term('joint_pos')
    asset = env.scene[asset_cfg.name]
    positions = asset.data.joint_pos[:, term._target_ids]
    obs = term.decoder.position_observation(positions)
    # Retain source actor encoder-noise DR as a declared observation uncertainty;
    # it does not alter goals or pretend the real calibration is identified.
    if biased:
        obs = obs + asset.data.encoder_bias[:, term._target_ids]
    return obs


def quantized_velocity(env, asset_cfg):
    term = env.action_manager.get_term('joint_pos')
    return term.decoder.velocity_observation(env.scene[asset_cfg.name].data.joint_vel[:, term._target_ids])


def gains(env, env_ids):
    # Current firmware uses source-runtime KP6 for all 14 policy servos. ID34
    # mouth KP10 is not an actor output and is absent from the source action.
    del env_ids
    for actuator in env.scene['robot'].actuators:
        if len(actuator.target_names)!=14:
            raise ValueError('Expected the original shared 14-servo battery group')
        actuator._base_kp = torch.full((1,14),6.,device=env.device,dtype=torch.float32)


def reset_head_entry(env, env_ids, head):
    """HOLD-to-WALK starts with the retained compensated head, not a new HOME."""
    asset=env.scene['robot']
    ids=torch.arange(env.num_envs,device=env.device) if env_ids is None else env_ids
    term=env.action_manager.get_term('joint_pos')
    joints=term._target_ids[5:9]
    position=asset.data.joint_pos[ids][:,joints].clone()
    position+=torch.tensor(head,device=env.device)
    velocity=asset.data.joint_vel[ids][:,joints].clone()
    asset.write_joint_state_to_sim(position,velocity,joint_ids=joints,env_ids=ids)


def configure(contract, mode, body_name, *, play=False):
    from mjlab_microduck.tasks import (_xgoduck_velocity_cfg, XgoduckRlCfg,
        _xgoduck_standup_cfg, XgoduckStandUpRlCfg, _xgoduck_ground_pick_cfg,
        XgoduckGroundPickRlCfg)
    from mjlab_microduck.tasks.xgoduck_tricks_env_cfg import make_xgoduck_roulade_env_cfg, XgoduckRouladeRlCfg
    from offline_mass_distribution import scenario_spec
    choices = {'walk': (_xgoduck_velocity_cfg, XgoduckRlCfg),
               'getup': (_xgoduck_standup_cfg, XgoduckStandUpRlCfg),
               'pick': (_xgoduck_ground_pick_cfg, XgoduckGroundPickRlCfg),
               'roulade': (make_xgoduck_roulade_env_cfg, XgoduckRouladeRlCfg)}
    factory, rl = choices[mode]
    cfg, rl = copy.deepcopy(factory(play=play)), copy.deepcopy(rl)
    model = contract['models'][mode]
    calibration = json.loads(Path(contract['files']['calibration']['resolved_path']).read_text())['joints']
    body = contract['files']['body_'+body_name]['resolved_path']
    body_scenario=json.loads(Path(body).read_text())
    robot = cfg.scene.entities['robot']
    robot.spec_fn, inertia_proof = scenario_spec(robot.spec_fn, body)
    if body_scenario.get('collision_envelope'):
        # Photos/rulers constrain an approximate outer envelope, not an exact
        # mesh. Keep this explicitly separate from the source-geometry control;
        # getup/roll must encounter the rear package rather than pass through it.
        inertial_builder=robot.spec_fn
        envelope=body_scenario['collision_envelope']
        def with_backpack():
            import mujoco
            spec=inertial_builder()
            hip=np.mean([spec.body(n).pos for n in ('yaw2roll','bearing_roll')],axis=0)
            position=hip.copy();position[0]-=body_scenario['rear_of_hip_m']
            position[2]+=envelope['center_z_above_hip_m']
            spec.body('trunk_base').add_geom(name='backpack_collision',
                type=mujoco.mjtGeom.mjGEOM_BOX,size=np.array(envelope['full_size_m'])/2,
                pos=position,mass=0.,contype=1,conaffinity=0,rgba=[.2,.2,.2,1])
            # The inertial verifier compiled this spec before adding a geom.
            # Refresh MuJoCo's name lookup before CollisionCfg resolves it.
            spec.compile()
            if spec.geom('backpack_collision') is None:
                raise ValueError('Backpack geometry missing from spec name lookup')
            return spec
        robot.spec_fn=with_backpack
    robot.init_state.joint_pos = dict(zip(model['joint_names'], model['home_rad']))
    # Preserve task-specific fall/drop/reset logic. Only reset_base's z is
    # an OFFSET from init_state, whereas the source XgoDuck wrapper writes an
    # absolute height there. Correct this same established reference mismatch.
    if 'reset_base' in cfg.events:
        lo, hi = cfg.events['reset_base'].params['pose_range'].get('z', (0., 0.))
        cfg.events['reset_base'].params['pose_range']['z'] = (lo-robot.init_state.pos[2], hi-robot.init_state.pos[2])
    head = contract['head_command_rad'] if mode == 'walk' else [0.]*4
    if mode == 'walk':
        if 'head_pose' in cfg.commands:
            cfg.commands['head_pose'].ranges = tuple((v, v) for v in head)
        # Source curriculum expands head/body commands; deployed head is fixed
        # and build_obs always supplies six zero body commands. Do not train
        # the actor on commands that this production host never submits.
        if 'body_pose' in cfg.commands:
            cfg.commands['body_pose'].ranges = ((0., 0.),)*6
        # Only command widening conflicts with the deployed constant inputs.
        # Keep upstream reward-weight and physical-randomization curricula:
        # filtering by the words "head"/"body" also removed those accidentally
        # in the archived v4 experiment. This correction is for future bundles;
        # it does not alter an already uploaded experiment or its evidence.
        for key in ('head_pose_range', 'body_pose_range'):
            cfg.curriculum.pop(key, None)
    if mode == 'getup':
        for name in ('head_pose', 'body_pose'):
            if name in cfg.commands:
                cfg.commands[name].ranges = ((0., 0.),)*(4 if name == 'head_pose' else 6)
        # Commands match runtime zeros; source reward and COM curricula remain.
        for key in ('head_pose_range', 'body_pose_range'):
            cfg.curriculum.pop(key, None)
    if mode == 'pick':
        # Runtime button always starts cos(phase)=1, sin(phase)=0 from standing.
        # Source exposes this exact option; randomized phase with HOME reset is
        # a different initial-condition task and must not masquerade as runtime.
        cfg.commands['twist'].randomize_phase=False
    old = cfg.actions['joint_pos']
    kwargs = {f.name: copy.deepcopy(getattr(old, f.name)) for f in fields(JointPositionActionCfg)}
    kwargs.update(scale=model['action_scale'], clip=None, model_contract=model,
                  calibration=calibration, head_seed=head if mode == 'walk' else None,
                  reset_freeze_seconds=float(getattr(old, 'freeze_s', 0.)))
    cfg.actions['joint_pos'] = RuntimeActionCfg(**kwargs)
    cfg.events['installed_gains'] = EventTermCfg(func=gains, mode='startup')
    if mode == 'walk':
        cfg.events['hold_head_entry'] = EventTermCfg(func=reset_head_entry,mode='reset',params={'head':head})
    for group in ('actor', 'critic'):
        cfg.observations[group].terms['joint_pos'].func = quantized_position
        cfg.observations[group].terms['joint_vel'].func = quantized_velocity
    rl.upload_model = False
    rl.algorithm.symmetry_cfg = None  # Saved left/right limits need not mirror.
    rl.logger = 'tensorboard'
    declaration = dict(mode=mode, body_scenario=body_name, inertia=inertia_proof,
        original_rewards_unchanged=True, original_terminations_unchanged=True,
        task_reset_freeze_seconds=kwargs['reset_freeze_seconds'],
        deployed_action_path='raw14 history; original EMA; model-specific HOME; conditional tick projection; quantized goal',
        sensor_model='CAD IMU plus original source noise/delay; quantized saved encoder coordinates; real mounting/filter/timing uncertainty NOT identified',
        hardware_verified=False, mechanical_zero_alignment_assumed=True,
        body_identified=False, mouth_model='original passive jaw; runtime ID34 30-degree mouth actuation is not modeled')
    declaration['backpack_collision_envelope']=body_scenario.get('collision_envelope')
    return cfg, rl, declaration
