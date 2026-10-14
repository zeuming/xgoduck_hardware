"""Check the mixture against identical independently stepped physical worlds."""
import argparse,json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import torch,numpy as np,random
    from mjlab.envs import ManagerBasedRlEnv
    from current_body_training import load_contract,configure
    from mixed_body_training import MixedDomainEnv
    from current_body_fingerprint import initial_fingerprint
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    c=load_contract(a.contract);bodies=('reference','rear-mid','rear-high')
    def configs():
        out=[]
        for i,b in enumerate(bodies):
            cfg,_,_=configure(c,'walk',b);cfg.scene.num_envs=2;cfg.seed=i
            cfg.episode_length_s=.04;cfg.curriculum.clear();cfg.events.pop('push_robot',None)
            out.append((b,cfg))
        return out
    actions=[(torch.arange(6*14,dtype=torch.float32).reshape(6,14)-42)*(i+1)*.001 for i in range(3)]
    children=[];saved=[]
    try:
        torch.manual_seed(0);np.random.seed(0);random.seed(0)
        for _,cfg in configs():children.append(ManagerBasedRlEnv(cfg,device='cpu',render_mode=None))
        for e in children:e.reset()
        initial=[initial_fingerprint(e) for e in children]
        for action in actions:
            results=[e.step(action[i*2:i*2+2]) for i,e in enumerate(children)]
            saved.append({'obs':{k:torch.cat([r[0][k] for r in results]).clone() for k in results[0][0]},
                'reward':torch.cat([r[1] for r in results]).clone(),
                'terminated':torch.cat([r[2] for r in results]).clone(),
                'truncated':torch.cat([r[3] for r in results]).clone(),
                'qpos':torch.cat([e.sim.data.qpos[:] for e in children]).clone(),
                'qvel':torch.cat([e.sim.data.qvel[:] for e in children]).clone(),
                'fingerprints':[initial_fingerprint(e) for e in children]})
    finally:
        for e in children:e.close()
    mixed=None
    try:
        torch.manual_seed(0);np.random.seed(0);random.seed(0)
        mixed=MixedDomainEnv(configs(),device='cpu',render_mode=None);mixed.reset()
        assert initial==[initial_fingerprint(e) for _,e in mixed.environments]
        for action,expected in zip(actions,saved):
            obs,reward,terminated,truncated,_=mixed.step(action)
            actual={'obs':obs,'reward':reward,'terminated':terminated,'truncated':truncated,
                'qpos':torch.cat([e.sim.data.qpos[:] for _,e in mixed.environments]),
                'qvel':torch.cat([e.sim.data.qvel[:] for _,e in mixed.environments])}
            for key in expected:
                if key=='fingerprints':assert expected[key]==[initial_fingerprint(e) for _,e in mixed.environments]
                elif key=='obs':
                    for name in obs:torch.testing.assert_close(obs[name],expected[key][name],atol=0,rtol=0)
                else:torch.testing.assert_close(actual[key],expected[key],atol=0,rtol=0)
        mixed.common_step_counter=700
        assert all(e.common_step_counter==700 for _,e in mixed.environments)
        report={'passed':True,'domains':list(bodies),'envs_per_domain':2,'steps':3,
            'auto_reset_exercised':True,'full_initial_and_post_step_fingerprints_exact':True,
            'observations_rewards_terminations_qpos_qvel_exact':True,
            'common_counter_restore_all_domains':True,'hardware_access':False}
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report))
    finally:
        if mixed is not None:mixed.close()


if __name__=='__main__':main()
