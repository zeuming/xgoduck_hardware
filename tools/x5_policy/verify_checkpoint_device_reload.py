"""Offline real RSL-RL/Adam reload and next-update equality check."""
import argparse,json,tempfile
from pathlib import Path

def verify(device):
    import torch
    from rsl_rl.algorithms import PPO
    from rsl_rl.runners import OnPolicyRunner
    from train_constrained_policy import assert_state_identical
    if device!='cpu' and not torch.cuda.is_available():raise RuntimeError('Requested CUDA is unavailable')
    torch.manual_seed(7)
    def make():
        runner=OnPolicyRunner.__new__(OnPolicyRunner)
        alg=PPO.__new__(PPO);alg.rnd=None
        alg.actor=torch.nn.Linear(4,3).to(device)
        alg.critic=torch.nn.Linear(4,1).to(device)
        alg.optimizer=torch.optim.Adam([*alg.actor.parameters(),*alg.critic.parameters()],lr=1e-3)
        runner.alg=alg
        return runner
    def update(r):
        r.alg.optimizer.zero_grad(set_to_none=True)
        for i,p in enumerate([*r.alg.actor.parameters(),*r.alg.critic.parameters()]):
            p.grad=torch.full_like(p,(i+1)*.01)
        r.alg.optimizer.step()
    original=make();update(original);update(original)
    with tempfile.TemporaryDirectory(prefix='xgoduck-device-reload-') as tmp:
        path=Path(tmp)/'checkpoint.pt'
        state=original.alg.save();state.update(iter=1,infos={'test':'device_reload'})
        torch.save(state,path)
        old_failure_reproduced=False
        if device!='cpu':
            old=make();old.load(str(path),strict=True,map_location=device)
            try:assert_state_identical(original.alg.optimizer.state_dict(),old.alg.optimizer.state_dict())
            except AssertionError as e:
                if 'device' not in str(e):raise
                old_failure_reproduced=True
            if not old_failure_reproduced:raise AssertionError('Expected original CUDA counter relocation did not reproduce')
        fixed=make();fixed.load(str(path),strict=True,map_location='cpu')
        assert_state_identical(original.alg.save(),fixed.alg.save())
        update(original);update(fixed)
        assert_state_identical(original.alg.save(),fixed.alg.save())
        steps=[str(s['step'].device) for s in fixed.alg.optimizer.state.values()]
        moments=[str(s['exp_avg'].device) for s in fixed.alg.optimizer.state.values()]
    return dict(device=device,old_failure_reproduced=old_failure_reproduced,
        full_reload_identical=True,next_update_identical=True,step_devices=steps,moment_devices=moments)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards(accelerator=a.device!='cpu')
    result=verify(a.device)
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
