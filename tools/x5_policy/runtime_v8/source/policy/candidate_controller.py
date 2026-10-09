"""Isolated research MotionController; no native service/serial default entry.

Not hardware qualified. Constant-command robustness was rejected by the
unchanged2deg head gate. This module is exercised only with fake transports.
It neither modifies the original actor/metadata nor exports a policy.
"""
from __future__ import annotations
import copy
import hashlib
import importlib.util
import math
from pathlib import Path
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent
POLICY=HERE
DECODER=HERE/'candidate_math.py'
SOURCE_PINS={POLICY/'production_motion.py':'b0f563aa98ace779b67f442f5cc70b3ffb331b8a90520c3fa6809b8f63cbbf1b',
    POLICY/'rl_core.py':'880b316654373aea8708c475c8fad31609e23d5791dbf3a04e94a9c9b28dfcc0',
    DECODER:'b1f4209b5daad8bbc61ce98f4c7a0bbe11db99913f6cf23ac54ccd7162a48058'}


def verify_source_pins():
    for path,expected in SOURCE_PINS.items():
        if hashlib.sha256(path.read_bytes()).hexdigest()!=expected:
            raise RuntimeError('Research inheritance source changed: '+str(path))


verify_source_pins()
sys.path.insert(0,str(POLICY))
from production_motion import MotionController,MotionError,MODEL_MODES,FRESH_SECONDS
from rl_core import ACTION_ALPHA,ACTION_ALPHA_HEAD,MOTOR_IDS,action_to_deg,build_obs,stance_from_home
import production_motion
import rl_core as production_core
if Path(production_motion.__file__).resolve()!=(POLICY/'production_motion.py').resolve() or Path(production_core.__file__).resolve()!=(POLICY/'rl_core.py').resolve():
    raise RuntimeError('Research inheritance imported an unexpected production module')
spec=importlib.util.spec_from_file_location('research_frozen_bounded_roll_math',DECODER)
math_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(math_module)
ExponentialRollMap=math_module.ExponentialRollMap
ORIGINAL_HOME=np.asarray([0.,-.087,-.349,-.005,.349,.349,.349,0.,0.,0.,.087,.349,.005,-.349])
HOME_OFFSETS_DEG={12:-1.0,14:-1.0,22:1.0,24:1.0}
POLICY_IDS=[10,11,12,13,14,30,31,32,33,20,21,22,23,24]


class CandidateTarget(list):
    """Per-inference diagnostics travel with the target, not session state."""
    def __init__(self,values,diagnostics):
        super().__init__(values)
        self.diagnostics=diagnostics


class ResearchMotionController(MotionController):
    def __init__(self,sessions,config_path=None,port='research-fake-only',*,
                 transport_factory=None,clock=time.monotonic,
                 research_walk_enabled=False,research_nonzero_command=False,
                 hold_hip_pitch_bias_deg=0):
        verify_source_pins()
        if transport_factory is None:
            raise MotionError('Research controller requires an explicitly injected isolated transport; no serial default exists')
        if type(research_walk_enabled) is not bool or type(research_nonzero_command) is not bool:
            raise MotionError('Research flags must be explicit booleans')
        if research_nonzero_command and not research_walk_enabled:
            raise MotionError('Nonzero research commands require explicit research walk enablement')
        if type(hold_hip_pitch_bias_deg) not in (int,float) or hold_hip_pitch_bias_deg!=0:
            raise MotionError('Consistent standing HOME rejects every nonzero/invalid extra HOLD hip bias')
        self._research_walk_enabled=research_walk_enabled
        self._research_nonzero_command=research_nonzero_command
        self._candidate_last_inference=None
        self._walk_warmup_generation=None
        self._zero_target_first=self._zero_target_last=None
        self._zero_target_count=0
        self._walk_warmup_complete=False
        super().__init__(sessions,config_path,port,transport_factory=transport_factory,clock=clock,
            require_head_clearance=True,allow_fixed_head_walk=research_walk_enabled,hold_hip_pitch_bias_deg=0)
        if 'walk' not in self._policy_profiles:
            raise MotionError('Research HOMEprime requires the original walk actor')
        original=self._policy_profiles['walk']
        if not np.array_equal(original['home_rad'],ORIGINAL_HOME) or original['scale']!=1.:
            raise MotionError('Original actor metadata does not match the pinned HOME20/action scale')
        self._original_metadata_profiles={name:dict(home_rad=value['home_rad'].tolist(),scale=value['scale'])
            for name,value in self._policy_profiles.items()}
        home=original['home_rad'].copy()
        for sid,offset in HOME_OFFSETS_DEG.items():home[POLICY_IDS.index(sid)]+=math.radians(offset)
        home.setflags(write=False)
        self._policy_profiles['walk']=dict(home_rad=home,scale=original['scale'])
        self._stand_deg=stance_from_home(home)

    @property
    def research_walk_enabled(self):return self._research_walk_enabled

    @property
    def research_nonzero_command(self):return self._research_nonzero_command

    def _reset_candidate_history_locked(self):
        self._previous_action.fill(0)
        self._smooth_action.fill(0)
        self._reset_walk_warmup_locked()

    def _reset_walk_warmup_locked(self,generation=None):
        self._walk_warmup_generation=generation
        self._zero_target_first=self._zero_target_last=None
        self._zero_target_count=0
        self._walk_warmup_complete=False

    def _walk_warmup_ready_locked(self):
        return (self._walk_warmup_complete and self._mode=='walk' and self._connected
            and self._walk_warmup_generation==self._generation)

    def _record_successful_zero_target_locked(self,when):
        if (self._zero_target_last is None or when<self._zero_target_last
                or when-self._zero_target_last>FRESH_SECONDS):
            self._zero_target_first=when
            self._zero_target_count=0
        self._zero_target_last=when
        self._zero_target_count+=1
        self._walk_warmup_complete=(when-self._zero_target_first>=5.)

    def _stop_locked(self,reason=None):
        super()._stop_locked(reason)
        self._reset_candidate_history_locked()

    def _connect_once(self):
        with self._lock:self._reset_candidate_history_locked()
        return super()._connect_once()

    def _disconnect(self,exc):
        result=super()._disconnect(exc)
        with self._lock:
            if not self._connected:self._reset_candidate_history_locked()
        return result

    def _fixed_stance_locked(self):
        target=copy.deepcopy(self._stand_deg)
        if target is not None and self._hold_head_deg is not None:
            for sid,angle in self._hold_head_deg.items():target[MOTOR_IDS.index(sid)]=angle
        return target

    def _require_research_model(self,mode):
        if mode in MODEL_MODES and (mode!='walk' or not self._research_walk_enabled):
            raise MotionError('Research model remains locked; only explicit candidate walk is supported')

    def _motion_gate_locked(self,mode=None):
        self._require_research_model(self._mode if mode is None else mode)
        return super()._motion_gate_locked(mode)

    def start_mode(self,mode,support_confirmed=False,*,expected_generation=None,
                   preview_model='walk',behavior_confirmed=False):
        self._require_research_model(preview_model if mode=='shadow' else mode)
        # Parent start resets rawprevious and adaptedEMA only after generation,
        # freshness, preflight and mode-transition checks succeed. Preflight
        # uses temporary zeros and cannot consume the committed raw history.
        result=super().start_mode(mode,support_confirmed,expected_generation=expected_generation,
            preview_model=preview_model,behavior_confirmed=behavior_confirmed)
        with self._lock:
            # STOP/newmode invalidates a returned start and cannot revive an
            # older generation's warmup evidence for the new episode.
            if self._generation==result['generation'] and self._mode==mode:
                self._reset_walk_warmup_locked(self._generation if mode=='walk' else None)
        return self.status()

    def command(self,forward,side,yaw):
        values=[forward,side,yaw]
        if any(isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) for value in values):
            raise MotionError('速度指令需要有限数字')
        if not any(value!=0 for value in values):return super().command(forward,side,yaw)
        if not self._research_walk_enabled or not self._research_nonzero_command:
            raise MotionError('Nonzero research command requires the explicit constructor-only research flag')
        with self._lock:
            if self._mode not in ('walk','shadow'):
                raise MotionError('先进入行走或真实反馈测试模式')
            if not self._walk_warmup_ready_locked():
                raise MotionError('Research nonzero command requires5seconds of continuous successful zero walk TARGETs')
            # Same finite command envelope as parent; no automatic start or
            # lease renewal. The next control boundary still runs every gate.
            self._command[:]=np.clip(values,[-.2,-.1,-.5],[.2,.1,.5])
        return self.status()

    def _control_once(self):
        _phase_pre_wait=time.perf_counter()
        with self._lock:
            _phase_pre_acquired=time.perf_counter()
            generation,mode=self._generation,self._mode
            zero_before=not np.any(self._command)
            # The deque is bounded150. Compare timestamps, not its length:
            # parent appends only after a successful TARGET, including later
            # cycles after the deque has filled.
            before=tuple(self._target_times)
        self._last_candidate_precontrol_phases=(
            ('candidate_lock_wait',_phase_pre_wait),
            ('candidate_lock_acquired',_phase_pre_acquired),
            ('candidate_lock_released',time.perf_counter()))
        result=super()._control_once()
        with self._lock:
            if (mode=='walk' and self._mode=='walk' and generation==self._generation
                    and self._walk_warmup_generation==generation and self._connected
                    and zero_before and not np.any(self._command)
                    and tuple(self._target_times)!=before):
                self._record_successful_zero_target_locked(self._target_times[-1])
        return result

    def _infer_policy_target(self,selected,st,previous,command,smooth,elapsed,*,head_command=None):
        self._require_research_model(selected)
        if selected!='walk':raise MotionError('Research inference only supports candidate walk')
        session=self.sessions.get(selected)
        profile=self._policy_profiles[selected]
        head=np.zeros(4,np.float32) if head_command is None else np.asarray(head_command,np.float32)
        if head.shape!=(4,) or not np.isfinite(head).all():raise MotionError('头部指令需要四个有限弧度值')
        obs=build_obs(st,previous,command,head,home_rad=profile['home_rad']).reshape(1,61)
        began=time.perf_counter()
        raw=np.asarray(session.run(None,{session.get_inputs()[0].name:obs})[0])
        inference_ms=(time.perf_counter()-began)*1000
        if raw.shape!=(1,14) or not np.isfinite(raw).all():raise MotionError('研究策略输出无效，已停止')
        raw=raw.reshape(14).astype(np.float32,copy=True)
        scaled=raw.copy();scaled[[1,10]]*=np.float32(.95)
        adapted=raw.copy()
        with self._lock:calibration=copy.deepcopy(self._config['joints'])
        by_id={row['id']:row for row in calibration}
        roll_trace={}
        for sid,index in ((11,1),(21,10)):
            row=by_id[sid]
            lo,hi=sorted((tick-row['zero'])*360/4095*row['sign'] for tick in (row['min']+3,row['max']-3))
            try:function=ExponentialRollMap(lo,hi,16.)
            except (TypeError,ValueError) as error:raise MotionError('Research roll decoder cannot fit unchanged saved bounds') from error
            # Match the frozen decoder exactly: multiply in Pythonfloat before
            # angle mapping, then round the returned adaptedraw tofloat32.
            target=math.degrees(profile['home_rad'][index]+profile['scale']*.95*float(raw[index]))
            mapped=function(target)
            adapted[index]=(math.radians(mapped)-profile['home_rad'][index])/profile['scale']
            roll_trace[str(sid)]=dict(unconstrained_s95_target_deg=target,adapted_preEMA_target_deg=mapped,
                saved_raw_bounds=[row['min'],row['max']],inner_angle_bounds_deg=[lo,hi])
        next_smooth=smooth.copy()
        for sl,alpha in ((slice(0,5),ACTION_ALPHA),(slice(5,9),ACTION_ALPHA_HEAD),(slice(9,14),ACTION_ALPHA)):
            next_smooth[sl]=alpha*next_smooth[sl]+(1-alpha)*adapted[sl]
        target=action_to_deg(next_smooth,home_rad=profile['home_rad'],action_scale=profile['scale'])
        diagnostics=dict(original_neural_raw_action=raw.tolist(),original_neural_previous_action=np.asarray(previous).tolist(),
            original_ONNX_metadata_target_deg=action_to_deg(raw,home_rad=ORIGINAL_HOME,action_scale=profile['scale']),
            unconstrained_retargeted_target_deg=action_to_deg(raw,home_rad=profile['home_rad'],action_scale=profile['scale']),
            unconstrained_s95_retargeted_target_deg=action_to_deg(scaled,home_rad=profile['home_rad'],action_scale=profile['scale']),
            adapted_raw_action=adapted.tolist(),adapted_preEMA_target_deg=action_to_deg(adapted,home_rad=profile['home_rad'],action_scale=profile['scale']),
            adapted_previous_EMA_action=np.asarray(smooth).tolist(),adapted_EMA_action=next_smooth.tolist(),
            adapted_smoothed_target_deg=list(target),roll_decoder=roll_trace,
            nonlinearity_before_EMA=True,raw_history_is_original_actor=True)
        # Return ORIGINAL raw to the parent commit. Adapted actions only live
        # in next_smooth. There is no session cache or state changed by this
        # inference; obsolete generations discard the whole target/diagnostic.
        return CandidateTarget(target,diagnostics),raw,next_smooth,inference_ms

    def _record_policy_target_locked(self,target):
        super()._record_policy_target_locked(target)
        if isinstance(target,CandidateTarget):
            diagnostic=copy.deepcopy(target.diagnostics)
            diagnostic.update(recorded_generation=self._generation,
                execution_evidence='Computation only; actual TARGET is separately recorded in last_executed_target_deg.')
            self._candidate_last_inference=diagnostic

    def status(self):
        result=super().status()
        with self._lock:
            # Parent protocol1 still describes validated ORIGINAL metadata.
            # The research instance's observation/decode HOME is separately
            # named and never presented as modified ONNX metadata.
            original=copy.deepcopy(getattr(self,'_original_metadata_profiles',{}))
            if original:result['policy_profiles']=copy.deepcopy(original)
            result['candidate_profile']=dict(protocol=1,research_only=True,runtime_scope='candidate_instance',hardware_qualified=False,
                original_ONNX_metadata_profiles=original,
                research_observation_decode_HOMEprime={name:dict(home_rad=value['home_rad'].tolist(),scale=value['scale'])
                    for name,value in self._policy_profiles.items() if name=='walk'},
                HOME_offsets_deg=HOME_OFFSETS_DEG.copy(),s95_roll_scale=.95,C1_exponential_threshold_deg=16.,
                each_roll_saved_bound_interior_ticks=3,rawprevious_storage='Parent _previous_action holds original actorraw; no session cache.',
                reset_boundaries=['successful mode start','STOP/fault','disconnect','reconnect'],
                research_walk_enabled=self._research_walk_enabled,research_nonzero_command=self._research_nonzero_command,
                constant_walk_robustness_rejected=True,needs_profile_HOLD=True,current_ID11_unresolved=False,
                last_inference=copy.deepcopy(self._candidate_last_inference))
            span=0. if self._zero_target_first is None else self._zero_target_last-self._zero_target_first
            result['candidate_profile']['zero_walk_warmup']=dict(required_seconds=5.,
                generation=self._walk_warmup_generation,successful_zero_TARGET_count=self._zero_target_count,
                first_successful_zero_TARGET_monotonic=self._zero_target_first,last_successful_zero_TARGET_monotonic=self._zero_target_last,
                continuous_zero_TARGET_span_seconds=span,maximum_successful_TARGET_gap_seconds=FRESH_SECONDS,
                ready=self._walk_warmup_ready_locked(),software_control_evidence_only=True,physical_standing_verified=False)
            result['fixed_head_walk_zero_velocity_only']=not (self._research_nonzero_command and self._walk_warmup_ready_locked())
        return result
