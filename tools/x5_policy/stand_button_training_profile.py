"""Explicit episode-local command task; never changes plant/reward/guards."""
from __future__ import annotations
from dataclasses import dataclass, fields, asdict
from pathlib import Path
import copy
import hashlib
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[2]
PULSE_SOURCE = ROOT/'output/duck_sim/adaptation-20261011/candidate/policy/bounded_forward.py'
PULSE_SHA = '8e42dc10b259de0bc2a800c5209e0a3d3889f10b5faeb980206ec16e0a2ce22d'
PROFILE = dict(name='stand-button-v1',warmup_s=5.,duration_s=2.,peak_m_s=.2,
               ramp_s=.5,recovery_s=3.,after_schedule='hold_zero_until_original_episode_timeout',
               clock='per_environment_episode_length_buf',random_resampling=False,
               reset='selected_environments_restart_at_zero')

def source_pulse():
    if hashlib.sha256(PULSE_SOURCE.read_bytes()).hexdigest()!=PULSE_SHA:
        raise ValueError('Frozen button source hash differs')
    spec=importlib.util.spec_from_file_location('frozen_training_button',PULSE_SOURCE)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    for key,name in [('warmup_s','WARMUP_S'),('duration_s','DURATION_S'),('peak_m_s','PEAK_M_S'),
                     ('ramp_s','RAMP_S'),('recovery_s','RECOVERY_S')]:
        if getattr(module,name)!=PROFILE[key]:raise ValueError('Frozen button constants differ')
    pulse=module.ForwardPulse();pulse.begin(PROFILE['warmup_s'],0)
    return pulse

def command_at_steps(steps,dt):
    import torch
    elapsed=steps.to(torch.float64)*dt-PROFILE['warmup_s']
    fraction=torch.minimum(elapsed/PROFILE['ramp_s'],(PROFILE['duration_s']-elapsed)/PROFILE['ramp_s'])
    return (PROFILE['peak_m_s']*fraction.clamp(0.,1.)).to(torch.float32)

def apply_task_profile(cfg,name='upstream-velocity'):
    """Default is a no-op. The explicit task has exactly one twist producer."""
    if name=='upstream-velocity':return dict(name=name,modified=False)
    if name!='stand-button':raise ValueError('Unknown task profile')
    from mjlab.tasks.velocity.mdp.velocity_command import UniformVelocityCommand,UniformVelocityCommandCfg
    import torch
    source_pulse()
    class StandButtonCommand(UniformVelocityCommand):
        def _resample(self,env_ids):
            # Reset bookkeeping only: no random timer or command sampling.
            self.time_left[env_ids]=float('inf')
            self._resample_command(env_ids)
            self.command_counter[env_ids]+=1
        def _update_metrics(self):
            # These diagnostic integrals are normalized by the original episode
            # horizon, not the disabled 1e9-second random-resampling timer.
            denominator=self._env.cfg.episode_length_s/self._env.step_dt
            self.metrics['error_vel_xy']+=torch.linalg.vector_norm(
                self.vel_command_b[:,:2]-self.robot.data.root_link_lin_vel_b[:,:2],dim=1)/denominator
            self.metrics['error_vel_yaw']+=(self.vel_command_b[:,2]-self.robot.data.root_link_ang_vel_b[:,2]).abs()/denominator
        def _resample_command(self,env_ids):
            # CommandManager.reset runs before episode_length_buf is zeroed.
            # Set selected commands directly; never consult stale episode age.
            self.vel_command_b[env_ids]=0.;self.vel_command_w[env_ids]=0.
            self.is_standing_env[env_ids]=True
            self.is_heading_env[env_ids]=False;self.is_world_env[env_ids]=False
            self.is_forward_env[env_ids]=False
        def _update_command(self):
            vx=command_at_steps(self._env.episode_length_buf,self._env.step_dt)
            self.vel_command_b.zero_();self.vel_command_b[:,0]=vx
            self.vel_command_w.copy_(self.vel_command_b)
            self.is_standing_env.copy_(vx==0)
        def compute(self,dt):
            # Physics/reward consume previous observation's command. This runs
            # afterwards and publishes episode t+dt for the next observation.
            if dt>0:self._update_metrics()
            self._update_command()
            # No base compute: no random timer or GUI command producer.
    @dataclass(kw_only=True)
    class StandButtonCommandCfg(UniformVelocityCommandCfg):
        def build(self,env):return StandButtonCommand(self,env)
    original=cfg.commands['twist']
    old_config=asdict(original)
    arguments={f.name:copy.deepcopy(getattr(original,f.name)) for f in fields(UniformVelocityCommandCfg)}
    arguments.update(heading_command=False,rel_heading_envs=0.,rel_world_envs=0.,
                     rel_forward_envs=0.,rel_standing_envs=0.,init_velocity_prob=0.,
                     resampling_time_range=(1e9,1e9))
    arguments['ranges']=UniformVelocityCommandCfg.Ranges(lin_vel_x=(0.,.2),lin_vel_y=(0.,0.),ang_vel_z=(0.,0.))
    cfg.commands['twist']=StandButtonCommandCfg(**arguments)
    removed={}
    for key,value in list(cfg.curriculum.items()):
        if key=='standing_envs' or value.params.get('command_name')=='twist':
            removed[key]=dict(function=value.func.__module__+'.'+value.func.__name__,params=value.params)
            del cfg.curriculum[key]
    # Do not permit a second step event writer such as the diagnostic override.
    for key,value in cfg.events.items():
        if 'twist' in key or value.func.__name__=='scheduled_diagnostic_twist':
            raise ValueError('Another twist writer is present: '+key)
    report=dict(**PROFILE,modified=True,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        pulse_source=dict(path=str(PULSE_SOURCE),sha256=PULSE_SHA),
        original_command_type=type(original).__module__+'.'+type(original).__name__,
        original_command_config=old_config,removed_twist_curricula=removed,
        effective_command_config=asdict(cfg.commands['twist']),
        diagnostic_error_normalization='error integral divided by unchanged configured episode duration; not by disabled resampling timer',
        unchanged=['plant','reward_functions_and_weights','gains','saved_joint_ranges','termination_thresholds','episode_timeout'])
    report['spec_sha256']=hashlib.sha256(json.dumps(report,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return report
