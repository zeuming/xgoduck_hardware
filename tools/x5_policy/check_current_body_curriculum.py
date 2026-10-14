"""Audit source reward/physics curricula; no simulator, PPO, or device I/O."""
import argparse,dataclasses,importlib.util,json,sys
from pathlib import Path


def canonical(value):
    def default(x):
        if callable(x):return x.__module__+'.'+x.__qualname__
        return repr(x)
    return json.dumps(dataclasses.asdict(value),sort_keys=True,default=default)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--archived-adapter',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    from current_body_training import configure,load_contract,sha
    from mjlab_microduck.tasks import _xgoduck_velocity_cfg,_xgoduck_standup_cfg,_xgoduck_ground_pick_cfg
    from mjlab_microduck.tasks.xgoduck_tricks_env_cfg import make_xgoduck_roulade_env_cfg
    spec=importlib.util.spec_from_file_location('archived_body_adapter',a.archived_adapter)
    archived=importlib.util.module_from_spec(spec);sys.modules[spec.name]=archived;spec.loader.exec_module(archived)
    c=load_contract(a.contract);rows=[]
    for mode,factory in [('walk',_xgoduck_velocity_cfg),('getup',_xgoduck_standup_cfg),
                         ('pick',_xgoduck_ground_pick_cfg),('roulade',make_xgoduck_roulade_env_cfg)]:
        source=factory(play=False);new,_,_=configure(c,mode,'reference');old,_,_=archived.configure(c,mode,'reference')
        allowed={'head_pose_range','body_pose_range'} if mode in ('walk','getup') else set()
        assert set(new.curriculum)==set(source.curriculum)-allowed
        assert set(new.rewards)==set(source.rewards)
        assert set(new.terminations)==set(source.terminations)
        for name,term in new.curriculum.items():assert canonical(term)==canonical(source.curriculum[name]),name
        for name,term in new.rewards.items():assert canonical(term)==canonical(source.rewards[name]),name
        for name,term in new.terminations.items():assert canonical(term)==canonical(source.terminations[name]),name
        extra_removed=sorted((set(source.curriculum)-set(old.curriculum))-allowed)
        rows.append(dict(mode=mode,rewards_exact=True,terminations_exact=True,
            all_source_noncommand_curricula_exact=True,removed_only_runtime_incompatible_command_widening=sorted(allowed),
            archived_extra_removed={name:dict(function=source.curriculum[name].func.__name__,
                params=source.curriculum[name].params) for name in extra_removed}))
    report=dict(passed=True,scope=__doc__,archived_adapter_sha256=sha(a.archived_adapter),
        corrected_adapter_sha256=sha(Path(__file__).with_name('current_body_training.py')),
        modes=rows,active_remote_bundle_changed=False,hardware_access=False,
        note='Archived v4 comparisons must still use archived v4 evaluator and adapter. New curricula are a different experiment, not checkpoint continuation.')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':main()
