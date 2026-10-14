"""Prepare a pinned, hardware-free candidate diagnostic-cost optimization.

Only candidate_controller.py changes. All movement gates, live feedback checks,
range checks, phase timers and the actual target conversion remain untouched.
Run before merging any separately reviewed production_motion protocol patch.
"""
import argparse
import hashlib
import json
from pathlib import Path


BASE = {
    'candidate_controller.py': '5019439aababd73923619fd87087c89fdd2f86c65ad43ab84721e5cb3a5ae424',
    'production_motion.py': 'a1d9f8082d5c805aa1dea4737048667320759cbe96545c8922bfe1bb76ab5a98',
    'rl_core.py': '880b316654373aea8708c475c8fad31609e23d5791dbf3a04e94a9c9b28dfcc0',
    'candidate_math.py': 'b1f4209b5daad8bbc61ce98f4c7a0bbe11db99913f6cf23ac54ccd7162a48058',
    'status_snapshot.py': '1969adfcee73a9dd8897d9cb5dc7b6a9a7c25829125f4bd1ca767e86102643ca',
}


HELPERS = '''

def _clone_candidate_tree(value):
    """Preserve detached-copy semantics; avoid recursion on flat scalar lists."""
    if type(value) is list:
        if all(item is None or type(item) in (bool,int,float,str,bytes) for item in value):
            return value.copy()
        return [_clone_candidate_tree(item) for item in value]
    if type(value) is dict:
        return {key:_clone_candidate_tree(item) for key,item in value.items()}
    return clone_display_value(value)


def _diagnostic_angles(actions,home,scale):
    """Batch three diagnostic conversions, preserving the original float32 math.

    Actual commanded targets still use the original action_to_deg function.
    Validation matches its home/scale checks; this helper receives only three
    already-validated float32 action vectors and the same profile snapshot.
    """
    home=np.asarray(home,dtype=np.float64)
    if home.shape!=(14,) or not np.isfinite(home).all() or not math.isfinite(scale) or scale<=0:
        raise ValueError('模型默认姿态或动作比例无效')
    # Multiply the float32 action matrix before adding float64 HOME, just as
    # action_to_deg does for each vector. Do not fuse, promote or reassociate it.
    q_deg=np.rad2deg(home+scale*np.stack(actions))
    values=np.zeros((len(actions),len(MOTOR_IDS)),dtype=np.float32)
    values[:,production_core.POLICY_INDEX]=q_deg
    return values.tolist()
'''


def sha(value):
    return hashlib.sha256(value).hexdigest()


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Inspected anchor changed: '+old[:70])
    return text.replace(old,new,1)


def prepare(source,destination):
    source,destination=Path(source),Path(destination)
    for name,expected in BASE.items():
        if sha((source/'policy'/name).read_bytes()) != expected:
            raise ValueError('Baseline differs; inspect before preparing: '+name)
    before=(source/'policy/candidate_controller.py').read_bytes()
    value=once(before.decode(),'\n\nclass CandidateTarget(list):',HELPERS+'\n\nclass CandidateTarget(list):')
    value=once(value,'target=clone_display_value(self._stand_deg)',
                     'target=_clone_candidate_tree(self._stand_deg)')
    value=once(value,"        with self._lock:calibration=clone_display_value(self._config['joints'])\n        by_id={row['id']:row for row in calibration}",
        """        with self._lock:
            source_by_id={row['id']:row for row in self._config['joints']}
            # Only these two joints enter the roll decoder. Detach their rows
            # under the same lock; keep missing-field checks in original order.
            # No calibration values are cached across calls.
            by_id={sid:_clone_candidate_tree(row) for sid,row in source_by_id.items()
                if sid in (11,21)}""")
    value=once(value,"        diagnostics=dict(original_neural_raw_action=raw.tolist(),",
        "        diagnostic_angles=_diagnostic_angles((raw,scaled,adapted),profile['home_rad'],profile['scale'])\n        diagnostics=dict(original_neural_raw_action=raw.tolist(),")
    for index,action in enumerate(('raw','scaled','adapted')):
        labels=('unconstrained_retargeted_target_deg','unconstrained_s95_retargeted_target_deg','adapted_preEMA_target_deg')
        value=once(value,f"{labels[index]}=action_to_deg({action},home_rad=profile['home_rad'],action_scale=profile['scale'])",
            f'{labels[index]}=diagnostic_angles[{index}]')
    value=once(value,'diagnostic=clone_display_value(target.diagnostics)',
                     'diagnostic=_clone_candidate_tree(target.diagnostics)')
    compile(value,'candidate_controller.py','exec')
    destination.mkdir(parents=True,exist_ok=False)
    (destination/'policy').mkdir()
    data=value.encode()
    (destination/'policy/candidate_controller.py').write_bytes(data)
    manifest=dict(kind='candidate_diagnostic_cost_repair',installed=False,device_access=False,
        changes=['batch diagnostic-only angles','copy consumed roll calibration rows only',
                 'detached scalar-list copy fast path'],
        model_changed=False,calibration_changed=False,protection_changed=False,
        target_math_changed=False,phase_timers_changed=False,
        before=BASE,after={'candidate_controller.py':sha(data)},
        merge='Apply this candidate first; separately reviewed production changes must update its SOURCE_PINS.')
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    parser.add_argument('destination',type=Path)
    args=parser.parse_args()
    print(json.dumps(prepare(args.source,args.destination),indent=2))
