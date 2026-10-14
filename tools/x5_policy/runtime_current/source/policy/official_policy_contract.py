"""Original walk observation/decoder contract, independent of execution profile.

No I/O entrypoint. Inherit the existing head, range, freshness, session and stop
checks. Saved-range conflicts are reported by those checks, never compressed.
"""
import copy
import numpy as np
from candidate_controller import CandidateTarget, ORIGINAL_HOME
from production_motion import MotionError
from rl_core import ACTION_ALPHA, ACTION_ALPHA_HEAD, action_to_deg, stance_from_home


class OfficialPolicyContractMixin:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # ResearchMotionController retains its old experiments for reproducing
        # archived evidence. This explicit integration variant uses the verified
        # metadata again for BOTH observation centering and action decoding.
        original = self._original_metadata_profiles['walk']
        home = np.asarray(original['home_rad'], dtype=np.float64).copy()
        if not np.array_equal(home, ORIGINAL_HOME) or original['scale'] != 1.:
            raise MotionError('Official walk metadata differs from the verified source')
        home.setflags(write=False)
        self._policy_profiles['walk'] = dict(home_rad=home, scale=original['scale'])
        self._stand_deg = stance_from_home(home)

    def _decode_policy_action(self, selected, raw, previous, smooth, elapsed, inference_ms):
        if selected != 'walk':
            raise MotionError('Official contract only supports the verified walk model')
        raw = np.asarray(raw)
        if raw.shape != (1, 14) or not np.isfinite(raw).all():
            raise MotionError('行走模型输出必须是有限的 [1,14] 数组')
        raw = raw.reshape(14).astype(np.float32, copy=True)
        next_smooth = np.asarray(smooth, dtype=np.float32).copy()
        if next_smooth.shape != (14,) or not np.isfinite(next_smooth).all():
            raise MotionError('行走滤波状态无效')
        # Keep the source's exact slicing/order, including ID20 in [5:10].
        for sl, alpha in ((slice(0, 5), ACTION_ALPHA),
                          (slice(5, 10), ACTION_ALPHA_HEAD),
                          (slice(10, 14), ACTION_ALPHA)):
            next_smooth[sl] *= alpha
            next_smooth[sl] += raw[sl] * (1 - alpha)
        profile = self._policy_profiles[selected]
        target = action_to_deg(next_smooth, home_rad=profile['home_rad'], action_scale=profile['scale'])
        raw_target = action_to_deg(raw, home_rad=profile['home_rad'], action_scale=profile['scale'])
        diagnostics = dict(original_neural_raw_action=raw.tolist(),
            original_neural_previous_action=np.asarray(previous).tolist(),
            original_ONNX_metadata_target_deg=raw_target,
            adapted_raw_action=raw.tolist(), adapted_preEMA_target_deg=raw_target,
            adapted_previous_EMA_action=np.asarray(smooth).tolist(),
            adapted_EMA_action=next_smooth.tolist(), adapted_smoothed_target_deg=list(target),
            roll_decoder={}, nonlinearity_before_EMA=False, raw_history_is_original_actor=True,
            contract='original-walk-home-scale-ema-v1')
        return CandidateTarget(target, diagnostics), raw, next_smooth, inference_ms

    def status(self):
        result = super().status()
        # Do not let inherited historical experiment labels misdescribe the
        # active path. This variant remains unqualified for hardware walking.
        profile = result.get('candidate_profile', {})
        profile.update(HOME_offsets_deg={}, s95_roll_scale=1.,
            C1_exponential_threshold_deg=None, each_roll_saved_bound_interior_ticks=None,
            research_observation_decode_HOMEprime=copy.deepcopy(result.get('policy_profiles', {})),
            execution_math_contract='original-walk-home-scale-ema-v1',
            additional_joint_retargeting=False, hardware_qualified=False)
        result['candidate_profile'] = profile
        return result
