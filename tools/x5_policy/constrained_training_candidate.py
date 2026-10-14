#!/usr/bin/env python3
"""Offline, untrained fixed-head XgoDuck training adapter for mjlab 1.3.0.

No command in this file trains a policy or talks to hardware. The optional mjlab
factory creates a config; installing its dependency stack is a separate step.
Saved operating ranges remain rejection criteria, never synthetic hard stops.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import hashlib
import importlib.metadata
import importlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from model_adaptation_contract import ROOT, saved_joint_contract, sha256
from rl_core import POLICY_IDS, POLICY_JOINT_NAMES, POLICY_OBSERVATION_NAMES, policy_profile

HEAD_INDICES = (5, 6, 7, 8)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def grounded_reset_placement(candidate):
    """Place the metadata HOME on the CAD soles with upstream's stated 1mm gap.

    mjlab reset_root_state_uniform adds pose_range to default_root_state. The
    upstream XgoDuck task supplies an absolute ~0.139m there, doubling spawn Z.
    Keep physical limits unchanged and derive nominal Z from existing geometry.
    """
    import mujoco
    xml = Path(candidate['source_files']['training_robot']['path']).parent/'xgoduck/robot_walk.xml'
    model = mujoco.MjModel.from_xml_path(str(xml)); data = mujoco.MjData(model)
    free = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE)
    if len(free) != 1: raise ValueError('Expected one free root joint')
    root = int(model.jnt_qposadr[int(free[0])])
    data.qpos[root:root+7] = [0,0,0,1,0,0,0]
    initial = np.asarray(candidate['home_rad']).copy()
    initial[list(HEAD_INDICES)] = candidate['held_head_rad']
    for name,value in zip(candidate['joint_names'],initial):
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = value
    mujoco.mj_forward(model,data)
    heights = {}; mesh_hashes = {}
    for side in ('left','right'):
        gid = model.geom(side+'_foot_collision').id; mid = int(model.geom_dataid[gid])
        start,count = int(model.mesh_vertadr[mid]),int(model.mesh_vertnum[mid])
        vertices = np.asarray(model.mesh_vert[start:start+count],dtype=np.float64)
        world = vertices@data.geom_xmat[gid].reshape(3,3).T+data.geom_xpos[gid]
        heights[side] = float(world[:,2].min())
        mesh_hashes[side] = hashlib.sha256(vertices.tobytes()).hexdigest()
    clearance = .001  # xgoduck_constants.py documents 1mm sole clearance.
    return dict(root_z_m=clearance-min(heights.values()),sole_clearance_m=clearance,
        relative_reset_z_range_m=[0.,0.],root_zero_sole_min_z_m=heights,
        xml_path=str(xml),xml_sha256=sha256(xml),foot_mesh_vertex_sha256=mesh_hashes,
        orientation='nominal identity; source orientation randomization remains a declared assumption')


def build_candidate(contract_path, snapshot_path):
    contract = json.loads(Path(contract_path).read_text())
    snapshot = json.loads(Path(snapshot_path).read_text())
    calibration = snapshot['calibration']
    if digest(calibration) != contract['calibration']['sha256']:
        raise ValueError('Snapshot calibration differs from the reviewed contract')
    if saved_joint_contract(calibration) != contract['calibration']['joints']:
        raise ValueError('Saved ranges differ from the reviewed contract')
    hardware = snapshot['motion'].get('hardware') or snapshot['motion'].get('last_hardware')
    head = hardware['hold_head_deg']
    home = contract['model']['home_rad']
    rows = {j['id']: j for j in contract['calibration']['joints']}
    held = [math.radians(head[str(POLICY_IDS[i])]) for i in HEAD_INDICES]
    package = Path(contract['source_files']['training_task']['path']).parents[1]
    extra_sources = [package/'tasks/__init__.py',package/'actuator/friction_dr_bam.py',package.parents[1]/'pyproject.toml']
    result = dict(schema_version=1, kind='untrained_constrained_training_candidate',
        trained=False, physical_release_ready=False, hardware_eligible=False, hardware_access=False,
        production_loader_integrated=False, arbitrary_runtime_head_capture_compatible=False,
        adapter_source=dict(path=str(Path(__file__).resolve()),sha256=sha256(__file__)),
        source_contract=dict(path=str(Path(contract_path).resolve()), sha256=sha256(contract_path)),
        source_snapshot=dict(path=str(Path(snapshot_path).resolve()), sha256=sha256(snapshot_path)),
        source_model=contract['model'], calibration=contract['calibration'],
        source_files=contract['source_files'],
        integration_sources=[dict(path=str(p.resolve()),sha256=sha256(p)) for p in extra_sources],
        source_expected_gains=contract['runtime_differences']['source_expected_gains'],
        policy_joint_ids=list(POLICY_IDS), joint_names=list(POLICY_JOINT_NAMES),
        observation_names=list(POLICY_OBSERVATION_NAMES),
        home_rad=home, action_scale=contract['model']['action_scale'],
        lower_rad=[math.radians(rows[i]['min_deg']) for i in POLICY_IDS],
        upper_rad=[math.radians(rows[i]['max_deg']) for i in POLICY_IDS],
        held_head_rad=held, held_head_source='captured torque-off pose; physical clearance unverified',
        head_command_rad=[held[k]-home[i] for k,i in enumerate(HEAD_INDICES)],
        action=dict(alpha=contract['runtime_differences']['action_ema']['leg'],
            control_dt_s=.02, goal_slew_ticks_s=600., ticks_per_turn=4095,
            formula='HOME + scale * raw action; replace head; reject invalid request; EMA; explicit goal slew',
            previous_action='raw unmodified 14-dimensional actor output',
            legacy_homeprime_roll95_applied=False,
            invalid_request='hold previous accepted target and terminate episode; never qualify clipped output'),
        training_scope=dict(task='Mjlab-Velocity-Flat-XgoDuck', fixed_head=True,
            body_command=[0.]*6, reset='exact metadata HOME legs and captured head; no random joint offset',
            saved_limits='soft training constraints and requested/applied/actual termination checks; CAD hard limits unchanged',
            raw_head_outputs='ignored by plant, still retained in actor action history',
            encoder_bias=False, symmetry_augmentation=False,
            upstream_domain_randomization='retained as source assumptions except encoder bias and head/body command curricula',
            supply_model='one upstream BAM group, shared vin sample and sum of all 14 joint loads; per-joint KP vector',
            known_mass_kg=contract['physical_robot']['total_mass_kg'], cad_mass_distribution='retained unvalidated; not silently uniformly scaled'),
        required_versions=dict(python='>=3.12,<3.13', mjlab='1.3.0', torch='2.9.1', warp_lang='1.12.0'),
        unresolved=['actual loaded motor response and KD-register semantics', 'mass distribution/COM/inertia for 871g',
            'safe captured head clearance', 'firmware substep slew/latency equivalence',
            'mouth ID34 has no independent actuator in this 14-output training plant',
            'all 15 physical joint frames and collision margins', 'new training convergence and supported-walk acceptance'],
        note='Implemented adapter candidate, not trained weights or proof of physical feasibility.')
    result['reset_placement'] = grounded_reset_placement(result)
    validate_candidate(result)
    return result


def validate_candidate(candidate, *, verify_sources=True):
    c = candidate
    if c.get('kind') != 'untrained_constrained_training_candidate' or c.get('schema_version') != 1:
        raise ValueError('Unsupported candidate format')
    if c.get('trained') is not False or c.get('physical_release_ready') is not False:
        raise ValueError('This candidate cannot assert training or physical acceptance')
    if c['policy_joint_ids'] != list(POLICY_IDS) or c['joint_names'] != list(POLICY_JOINT_NAMES):
        raise ValueError('Joint order changed')
    if c['observation_names'] != list(POLICY_OBSERVATION_NAMES):
        raise ValueError('Observation layout changed')
    vectors = [np.asarray(c[k], dtype=float) for k in ('home_rad','lower_rad','upper_rad')]
    if any(v.shape != (14,) or not np.isfinite(v).all() for v in vectors):
        raise ValueError('Need finite 14-element HOME and saved limits')
    home, low, high = vectors
    held = np.asarray(c['held_head_rad'], dtype=float)
    if held.shape != (4,) or not np.isfinite(held).all() or (low >= high).any():
        raise ValueError('Invalid held pose or saved bounds')
    initial = home.copy(); initial[list(HEAD_INDICES)] = held
    if ((initial < low) | (initial > high)).any():
        raise ValueError('Initial plant pose is outside a saved bound')
    action = c['action']
    if (any(type(action.get(k)) not in (int,float) or not math.isfinite(action[k])
            for k in ('alpha','control_dt_s','goal_slew_ticks_s'))
            or action.get('ticks_per_turn') != 4095
            or not (0 <= action['alpha'] < 1 and action['control_dt_s'] > 0 and action['goal_slew_ticks_s'] >= 0)):
        raise ValueError('Invalid EMA, period or slew')
    if not math.isfinite(c['action_scale']) or c['action_scale'] <= 0:
        raise ValueError('Invalid action scale')
    if not np.allclose(c['head_command_rad'], held-home[list(HEAD_INDICES)], atol=0, rtol=0):
        raise ValueError('Head command must be the held-pose delta from metadata HOME')
    if verify_sources:
        for source in [c['adapter_source'], c['source_contract'], c['source_snapshot'], c['source_model'], *c['source_files'].values(), *c['integration_sources']]:
            if sha256(source['path']) != source['sha256']:
                raise ValueError(f"Pinned source changed: {source['path']}")
        # Referencing the original file is insufficient: an altered adapter copy
        # could otherwise validate the old source while executing different code.
        # A byte-identical copy is permitted; a different implementation is not.
        if sha256(Path(__file__).resolve()) != c['adapter_source']['sha256']:
            raise ValueError('Executing adapter source differs from the pinned implementation')
        contract = json.loads(Path(c['source_contract']['path']).read_text())
        if (c['calibration'] != contract['calibration'] or c['home_rad'] != contract['model']['home_rad']
                or c['action_scale'] != contract['model']['action_scale']
                or c['source_expected_gains'] != contract['runtime_differences']['source_expected_gains']):
            raise ValueError('Saved calibration or metadata HOME was altered')
        snapshot = json.loads(Path(c['source_snapshot']['path']).read_text())
        hardware = snapshot['motion'].get('hardware') or snapshot['motion'].get('last_hardware')
        expected_head = [math.radians(hardware['hold_head_deg'][str(POLICY_IDS[i])]) for i in HEAD_INDICES]
        if c['held_head_rad'] != expected_head:
            raise ValueError('Held setpoint differs from the pinned capture')
        rows = {j['id']:j for j in c['calibration']['joints']}
        if c.get('reset_placement') != grounded_reset_placement(c):
            raise ValueError('Grounded reset placement differs from pinned CAD/HOME geometry')
        for key, bound in [('lower_rad','min_deg'),('upper_rad','max_deg')]:
            if c[key] != [math.radians(rows[i][bound]) for i in POLICY_IDS]:
                raise ValueError('A saved range was altered')
    return c


def save_candidate(candidate, path):
    validate_candidate(candidate)
    payload = dict(candidate=candidate, candidate_sha256=digest(candidate))
    Path(path).write_text(json.dumps(payload, indent=2, allow_nan=False)+'\n')


def load_candidate(path):
    payload = json.loads(Path(path).read_text())
    if digest(payload['candidate']) != payload['candidate_sha256']:
        raise ValueError('Candidate checksum mismatch')
    return validate_candidate(payload['candidate'])


def transform_actions(raw, home, scale, head_mask, held_full, lower, upper,
                      previous_ema, previous_goal, alpha, max_delta, ops):
    """Shared numpy/torch algebra. A rejected row holds; the episode must fail.

    Goal slew is intentional plant behavior. Range clipping is never used.
    The raw actor array is not modified, including its four masked head entries.
    """
    decoded = home + scale*raw
    requested = ops.where(head_mask, held_full, decoded)
    rejected = (~ops.isfinite(raw)).any(axis=-1) | (~ops.isfinite(requested)).any(axis=-1)
    rejected = rejected | ((requested < lower) | (requested > upper)).any(axis=-1)
    finite_request = ops.where(rejected[..., None], previous_ema, requested)
    smoothed = alpha*previous_ema + (1-alpha)*finite_request
    # Head servos hold a setpoint in every physics step, not a post-policy display override.
    smoothed = ops.where(head_mask, held_full, smoothed)
    if max_delta > 0:
        step = ops.minimum(ops.maximum(smoothed-previous_goal, -max_delta), max_delta)
        goal = previous_goal + step
    else:
        goal = smoothed
    rejected = rejected | (~ops.isfinite(goal)).any(axis=-1) | ((goal < lower) | (goal > upper)).any(axis=-1)
    goal = ops.where(rejected[..., None], previous_goal, goal)
    smoothed = ops.where(rejected[..., None], previous_ema, smoothed)
    return requested, smoothed, goal, rejected


class CpuActionPlant:
    """Small deterministic reference for the exact action transform, no physics."""
    def __init__(self, candidate, num_envs=1):
        self.candidate = validate_candidate(candidate)
        c = candidate
        self.home = np.asarray(c['home_rad'])
        self.low, self.high = np.asarray(c['lower_rad']), np.asarray(c['upper_rad'])
        self.mask = np.array([i in HEAD_INDICES for i in range(14)])
        self.held = self.home.copy(); self.held[list(HEAD_INDICES)] = c['held_head_rad']
        self.ema = np.broadcast_to(self.held, (num_envs,14)).copy()
        self.goal = self.ema.copy()
        self.failed = np.zeros(num_envs, dtype=bool)

    def process(self, raw):
        raw = np.asarray(raw, dtype=float)
        if raw.shape != self.goal.shape:
            raise ValueError('Wrong raw action shape')
        a = self.candidate['action']
        max_delta = np.asarray(a['goal_slew_ticks_s']*2*math.pi/a['ticks_per_turn']*a['control_dt_s'])
        requested, ema, goal, rejected = transform_actions(raw, self.home, self.candidate['action_scale'],
            self.mask, self.held, self.low, self.high, self.ema, self.goal, a['alpha'], max_delta, np)
        self.failed |= rejected
        self.ema = np.where(self.failed[:,None], self.ema, ema)
        self.goal = np.where(self.failed[:,None], self.goal, goal)
        return dict(raw=raw.copy(), requested=requested, applied=self.goal.copy(), episode_failed=self.failed.copy())


def check_model_compatibility(candidate, model_path, *, require_training_provenance=True):
    """Load an ONNX export; layout agreement alone cannot admit a trained policy."""
    import onnxruntime as ort
    options = ort.SessionOptions(); options.intra_op_num_threads = 1; options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model_path), options, providers=['CPUExecutionProvider'])
    profile = policy_profile(session)
    if (not np.allclose(profile['home_rad'], candidate['home_rad'], atol=1e-7, rtol=0)
            or profile['scale'] != candidate['action_scale']):
        raise ValueError('Export HOME/action scale differs from training candidate')
    metadata = session.get_modelmeta().custom_metadata_map
    owned_export = metadata.get('constrained_training_candidate_sha256') == digest(candidate)
    if require_training_provenance and not owned_export:
        raise ValueError('No matching constrained-training provenance; legacy model cannot be called retrained')
    return dict(model_sha256=sha256(model_path), io_metadata_compatible=True,
                matching_training_candidate_metadata=owned_export,
                training_run_verified=False, walking_verified=False)


def training_export_metadata(candidate):
    """Attach these exact fields when exporting newly trained weights.

    This is an offline helper, not a production loader change. A matching tag
    binds the contract; a separate checkpoint/run manifest must prove training.
    """
    validate_candidate(candidate)
    return dict(constrained_training_candidate_sha256=digest(candidate),
                constrained_training_held_head_rad=json.dumps(candidate['held_head_rad']),
                constrained_training_hardware_eligible='false')


def check_runtime_head_setpoint(candidate, held_head_rad):
    """Reject arbitrary capture-head reuse; this helper is not wired to the host."""
    held = np.asarray(held_head_rad,dtype=float)
    if held.shape != (4,) or not np.isfinite(held).all() or not np.allclose(
            held,candidate['held_head_rad'],atol=1e-7,rtol=0):
        raise ValueError('Runtime head setpoint differs from the training plant')
    return True


def cpu_structure_check(candidate):
    """Compile official CAD; verify named joint mappings without changing stops."""
    import mujoco
    source = Path(candidate['source_files']['training_robot']['path'])
    xml = source.parent/'xgoduck/robot_walk.xml'
    model = mujoco.MjModel.from_xml_path(str(xml))
    joints = []
    for i,name in enumerate(candidate['joint_names']):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0: raise ValueError(f'Missing training joint: {name}')
        actuator_ids = [a for a in range(model.nu) if model.actuator_trnid[a,0] == jid]
        if len(actuator_ids) != 1: raise ValueError(f'Need one actuator for {name}, found {actuator_ids}')
        joints.append(dict(id=POLICY_IDS[i], name=name, cad_joint_index=jid,
            cad_actuator_index=actuator_ids[0], target_index_semantics='entity-local joint index, never actuator ID',
            cad_hard_limit_rad=model.jnt_range[jid].tolist(), saved_operating_limit_rad=[candidate['lower_rad'][i],candidate['upper_rad'][i]]))
    plant = CpuActionPlant(candidate)
    out = plant.process(np.zeros((1,14)))
    if out['episode_failed'].any(): raise ValueError('Zero action cannot enter candidate plant')
    return dict(cad_compiled=True, xml_sha256=sha256(xml), original_hard_limits_unchanged=True,
        zero_command_transform_passed=True, joints=joints, full_mjlab_environment_loaded=False,
        trained=False, supported_walking_tested=False)


def dependency_status():
    versions = {}
    for package in ('mjlab','torch','warp-lang','mujoco','numpy'):
        try: versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError: versions[package] = None
    return dict(python=sys.version.split()[0], installed=versions,
                supported_python=sys.version_info[:2] == (3,12), full_environment_tested=False)


# The concrete integration class is defined only with the genuine upstream APIs.
# No fake dependency classes stand in for a working mjlab environment.
try:
    import torch
    from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
    from mjlab.managers import EventTermCfg, TerminationTermCfg
    _MJLAB_AVAILABLE = True
except ModuleNotFoundError:
    _MJLAB_AVAILABLE = False

if _MJLAB_AVAILABLE:
    @dataclass(kw_only=True)
    class ConstrainedJointPositionActionCfg(JointPositionActionCfg):
        candidate: dict
        def build(self, env):
            return ConstrainedJointPositionAction(self, env)

    class ConstrainedJointPositionAction(JointPositionAction):
        def __init__(self, cfg, env):
            super().__init__(cfg, env)
            c = validate_candidate(cfg.candidate)
            if list(self.target_names) != c['joint_names'] or self.action_dim != 14:
                raise ValueError('mjlab actuator order differs from metadata contract')
            if not math.isclose(env.step_dt,c['action']['control_dt_s'], abs_tol=1e-10):
                raise ValueError('Training cadence differs from declared plant')
            tensor = lambda value: torch.tensor(value, device=self.device, dtype=torch.float32)
            self._home = tensor(c['home_rad']); self._low = tensor(c['lower_rad']); self._high = tensor(c['upper_rad'])
            self._held = self._home.clone(); self._held[list(HEAD_INDICES)] = tensor(c['held_head_rad'])
            self._mask = torch.tensor([i in HEAD_INDICES for i in range(14)], device=self.device)
            self._ema = self._held.repeat(self.num_envs,1); self._goal = self._ema.clone()
            self.range_failed = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
            self._max_delta = tensor(c['action']['goal_slew_ticks_s']*2*math.pi/c['action']['ticks_per_turn']*env.step_dt)
            if not torch.allclose(self._offset, self._home.expand_as(self._offset), atol=1e-6):
                raise ValueError('Default joint positions no longer equal metadata HOME')

        def process_actions(self, actions):
            self._raw_actions[:] = actions
            c = self.cfg.candidate
            requested, ema, goal, failed = transform_actions(actions,self._home,c['action_scale'],
                self._mask,self._held,self._low,self._high,self._ema,self._goal,c['action']['alpha'],self._max_delta,torch)
            self.range_failed |= failed
            self.requested_targets = requested
            self._ema = torch.where(self.range_failed[:,None],self._ema,ema)
            self._goal = torch.where(self.range_failed[:,None],self._goal,goal)
            self._processed_actions = self._goal

        def apply_actions(self):
            # Catch actual excursions at every physics substep, not just policy ticks.
            actual = self._entity.data.joint_pos[:,self._target_ids]
            self.range_failed |= (~torch.isfinite(actual)).any(dim=-1) | ((actual < self._low) | (actual > self._high)).any(dim=-1)
            # Bias is explicitly disabled in this candidate; reject a silently re-enabled bias.
            if torch.any(self._entity.data.encoder_bias[:,self._target_ids] != 0):
                raise ValueError('Encoder bias changed the constrained execution contract')
            super().apply_actions()

        def reset(self, env_ids=None):
            super().reset(env_ids)
            ids = slice(None) if env_ids is None else env_ids
            self._ema[ids] = self._held; self._goal[ids] = self._held
            self._processed_actions[ids] = self._held; self.range_failed[ids] = False


def configure_single_battery_gain_vector(actuators, candidate, as_tensor=np.asarray):
    """Pinned BAM supports (1,J) base KP times (N,1) DR scales.

    Preserve one actuator object's vin_tensor, vin_drop_gain and total torque
    accumulation. Separate groups would change the source's shared battery model.
    This small structural operation is testable without pretending to run Warp.
    """
    if len(actuators) != 1:
        raise ValueError('Per-joint KP must retain one shared-battery BAM group')
    actuator = actuators[0]
    names = list(actuator.target_names)
    if len(names) != 14 or set(names) != set(candidate['joint_names']):
        raise ValueError('Shared-battery group must contain all 14 policy joints')
    if not hasattr(actuator,'_base_kp'):
        raise ValueError('Pinned BAM base-KP API changed')
    lookup = dict(zip(candidate['joint_names'],candidate['policy_joint_ids']))
    vector = [[candidate['source_expected_gains']['kp_by_id'][str(lookup[n])] for n in names]]
    actuator._base_kp = as_tensor(vector)


def _set_source_gain_vector(env, env_ids, candidate):
    # All actuators have initialized before startup events; set the shared group's
    # base vector once. Upstream gain DR/reset touches kp_scale, not _base_kp.
    configure_single_battery_gain_vector(env.scene['robot'].actuators,candidate,
        lambda value: torch.tensor(value,device=env.device,dtype=torch.float32))


def _range_failure(env):
    action = env.action_manager.get_term('joint_pos')
    actual = action._entity.data.joint_pos[:,action.target_ids]
    return action.range_failed | (~torch.isfinite(actual)).any(dim=-1) | ((actual < action._low) | (actual > action._high)).any(dim=-1)


def _reset_candidate_joints(env, env_ids, candidate):
    asset = env.scene['robot']
    env_ids = torch.arange(env.num_envs,device=env.device) if env_ids is None else env_ids
    indices = torch.tensor([list(asset.joint_names).index(n) for n in candidate['joint_names']], device=env.device)
    initial = torch.tensor(candidate['home_rad'], device=env.device)
    initial[list(HEAD_INDICES)] = torch.tensor(candidate['held_head_rad'],device=env.device)
    # Leave default_joint_pos intact: observations/export must retain metadata HOME.
    lower = torch.tensor(candidate['lower_rad'],device=env.device)
    upper = torch.tensor(candidate['upper_rad'],device=env.device)
    asset.data.soft_joint_pos_limits[:,indices,0] = lower
    asset.data.soft_joint_pos_limits[:,indices,1] = upper
    position = initial.repeat(len(env_ids),1)
    asset.write_joint_state_to_sim(position,torch.zeros_like(position),joint_ids=indices,env_ids=env_ids)


def make_mjlab_configs(candidate, *, play=False):
    """Build the genuine upstream config, without starting an environment/train job.

    Run only in the pinned upstream dependency environment. Its full environment
    loading and GPU behavior are deliberately separate acceptance gates.
    """
    validate_candidate(candidate)
    if not _MJLAB_AVAILABLE or importlib.metadata.version('mjlab') != '1.3.0':
        raise RuntimeError('Requires genuine mjlab==1.3.0 with torch/warp and the local xgoduck_rl package')
    if sys.version_info[:2] != (3,12):
        raise RuntimeError('Upstream training requires Python >=3.12,<3.13')
    if '--hf-jobs' in sys.argv:
        raise RuntimeError('This adapter does not submit remote jobs')
    from mjlab_microduck.tasks import _xgoduck_velocity_cfg, XgoduckRlCfg
    expected_modules = {
        'mjlab_microduck.tasks':candidate['integration_sources'][0]['path'],
        'mjlab_microduck.actuator.friction_dr_bam':candidate['integration_sources'][1]['path'],
        'bam.mjlab':candidate['source_files']['training_bam']['path'],
    }
    for name,path in expected_modules.items():
        if Path(importlib.import_module(name).__file__).resolve() != Path(path).resolve():
            raise RuntimeError(f'Loaded package {name} differs from the pinned local source')
    cfg = copy.deepcopy(_xgoduck_velocity_cfg(play=play))
    robot = cfg.scene.entities['robot']
    robot.init_state.joint_pos = {name:value for name,value in zip(candidate['joint_names'],candidate['home_rad'])}
    robot.init_state.pos = (*robot.init_state.pos[:2],candidate['reset_placement']['root_z_m'])
    cfg.events['reset_base'].params['pose_range']['z'] = (0.,0.)
    old = cfg.actions['joint_pos']
    cfg.actions['joint_pos'] = ConstrainedJointPositionActionCfg(entity_name=old.entity_name,
        actuator_names=old.actuator_names, scale=candidate['action_scale'], offset=0., preserve_order=old.preserve_order,
        use_default_offset=True, clip=None, candidate=copy.deepcopy(candidate))
    # One BAM group preserves its shared battery sample and sum-of-all-joint load.
    # Splitting groups by KP would silently create three independent batteries.
    if len(robot.articulation.actuators) != 1:
        raise ValueError('Expected one shared-battery BAM actuator group')
    robot.articulation.actuators[0].kp_fw = float(candidate['source_expected_gains']['kp_by_id']['10'])
    cfg.events['source_expected_kp_vector'] = EventTermCfg(
        func=_set_source_gain_vector, mode='startup', params={'candidate':candidate})
    cfg.events.pop('encoder_bias',None)
    for group in ('actor','critic'):
        cfg.observations[group].terms['joint_pos'].params['biased'] = False
    cfg.commands['head_pose'].ranges = tuple((v,v) for v in candidate['head_command_rad'])
    cfg.commands['body_pose'].ranges = ((0.,0.),)*6
    for name in ('head_pose_range','body_pose_range','head_pose_bias_weight'):
        cfg.curriculum.pop(name,None)
    for name in ('head_pose_tracking','head_pose_bias','body_pose_tracking'):
        cfg.rewards.pop(name,None)
    # Replace upstream random reset of all joints. Fail closed if its API changes.
    reset_names = [name for name,event in cfg.events.items() if getattr(event.func,'__name__','') == 'reset_joints_by_offset']
    if len(reset_names) != 1:
        raise ValueError(f'Expected one upstream reset_joints_by_offset event, found {reset_names}')
    cfg.events[reset_names[0]] = EventTermCfg(func=_reset_candidate_joints, mode='reset', params={'candidate':candidate})
    cfg.terminations['saved_range_violation'] = TerminationTermCfg(func=_range_failure)
    timestep = cfg.sim.mujoco.timestep
    cfg.decimation = round(candidate['action']['control_dt_s']/timestep)
    if not math.isclose(cfg.decimation*timestep,candidate['action']['control_dt_s'],abs_tol=1e-10):
        raise ValueError('Physics timestep cannot represent the declared control period')
    rl = copy.deepcopy(XgoduckRlCfg)
    rl.algorithm.symmetry_cfg = None  # Saved left/right bounds and captured head are asymmetric.
    rl.upload_model = False
    return cfg, rl


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path)
    parser.add_argument('--snapshot', type=Path)
    parser.add_argument('--candidate', type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--check-mjlab-config',action='store_true')
    args = parser.parse_args(argv)
    candidate = load_candidate(args.candidate) if args.candidate else build_candidate(args.contract,args.snapshot)
    args.output.mkdir(parents=True,exist_ok=True)
    save_candidate(candidate,args.output/'candidate.json')
    report = dict(candidate_sha256=digest(candidate),dependency_status=dependency_status(),cpu=cpu_structure_check(candidate))
    report['legacy_model_check'] = check_model_compatibility(candidate,candidate['source_model']['path'],require_training_provenance=False)
    try:
        check_model_compatibility(candidate,candidate['source_model']['path'])
    except ValueError as error:
        report['legacy_model_training_admission'] = dict(admitted=False,reason=str(error))
    report['production_loader_integrated'] = False
    if args.check_mjlab_config:
        try:
            make_mjlab_configs(candidate)
            report['mjlab_config'] = dict(built=True, environment_loaded=False, trained=False)
        except (RuntimeError,ModuleNotFoundError) as error:
            report['mjlab_config'] = dict(built=False, error=str(error),environment_loaded=False,trained=False)
    (args.output/'checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
