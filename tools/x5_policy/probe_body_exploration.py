"""Frozen official-actor exploration contrast; physical state is never injected."""
import argparse,copy,json,math,random,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--mode',choices=('getup','pick','roulade'),required=True)
    p.add_argument('--body',default='rear-high')
    p.add_argument('--num-envs',type=int,default=32)
    p.add_argument('--sigmas',default='0,0.02,0.1')
    p.add_argument('--seed',type=int,default=0)
    p.add_argument('--device',choices=('cpu','cuda:0'),default='cpu')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--wall-seconds',type=float,default=600)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    from train_constrained_policy import install_offline_guards,tensor_state_sha
    install_offline_guards(accelerator=a.device=='cuda:0')
    import torch,numpy as np,mujoco
    from tensordict import TensorDict
    from rsl_rl.models import MLPModel
    from mjlab.envs import ManagerBasedRlEnv
    from current_body_training import load_contract,configure,sha
    from current_body_fingerprint import initial_fingerprint
    from onnx_actor_warmstart import warmstart_actor_from_onnx
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    c=load_contract(a.contract);results=[];reference=None;start=time.monotonic()
    for sigma in [float(v) for v in a.sigmas.split(',')]:
        if not 0<=sigma<=1:raise ValueError('Explicit finite source-scale exploration only')
        torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
        cfg,_,d=configure(c,a.mode,a.body);cfg.scene.num_envs=a.num_envs;cfg.seed=a.seed;cfg.auto_reset=False
        cfg.events.pop('push_robot',None);cfg.curriculum.clear()
        if a.mode=='roulade':cfg.events['set_roulade_state'].params.update(standing_prob=1.,midroll_prob=0.)
        duration=8. if a.mode=='getup' else 4. if a.mode=='pick' else 6.
        cfg.episode_length_s=duration+.02
        env=None;case={'sigma_rad':sigma,'declaration':d}
        try:
            env=ManagerBasedRlEnv(cfg,device=a.device,render_mode=None);obs,_=env.reset(seed=a.seed)
            actor=MLPModel(TensorDict(obs,batch_size=[a.num_envs]),{'actor':['actor']},'actor',14,
                hidden_dims=[512,256,128],activation='elu',obs_normalization=True,
                distribution_cfg={'class_name':'rsl_rl.modules.distribution:GaussianDistribution','init_std':.02}).to(a.device)
            warmstart_actor_from_onnx(actor,c['files'][a.mode]['resolved_path'],expected_sha256=c['files'][a.mode]['sha256'])
            actor.eval();before=tensor_state_sha(actor.state_dict())
            fp=initial_fingerprint(env)
            if reference is None:reference=fp
            elif reference!=fp:raise ValueError('Exploration contrast initial physics/RNG differ')
            case['initial_fingerprint']=fp;case['paired_initial_state_exact']=True
            generator=torch.Generator(device=a.device);generator.manual_seed(8001)
            model=env.sim.mj_model;feet=[]
            for side in ('left','right'):
                ids=[i for i in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').split('/')[-1]==side+'_foot_collision']
                if len(ids)!=1:raise ValueError('Expected foot collision mesh')
                gid=ids[0];mid=int(model.geom_dataid[gid]);v=int(model.mesh_vertadr[mid]);n=int(model.mesh_vertnum[mid])
                feet.append((gid,torch.tensor(np.asarray(model.mesh_vert[v:v+n]).copy(),device=a.device)))
            mouth=[i for i in range(model.nsite) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_SITE,i) or '').split('/')[-1]=='mouth_tip'][0]
            tail_valid=[];tail_positions=[];mouth_min=torch.full((a.num_envs,),float('inf'),device=a.device)
            terminations=torch.zeros(a.num_envs,dtype=torch.bool,device=a.device);projection=0;reward_sum=0.
            pitch=[]
            for step in range(round(duration/env.step_dt)):
                if time.monotonic()-start>a.wall_seconds:raise TimeoutError('Frozen exploration wall budget')
                with torch.inference_mode():
                    action=actor(TensorDict(obs,batch_size=[a.num_envs]),stochastic_output=False)
                    # Draw the same normalized noise even at sigma0; a separate
                    # generator cannot perturb the physical delay/reset RNG.
                    action=action+sigma*torch.randn(action.shape,generator=generator,device=a.device)
                    obs,reward,terminated,truncated,_=env.step(action)
                terminations|=terminated;reward_sum+=float(reward.mean())
                projection+=int(env.action_manager.get_term('joint_pos').decoder.projected.sum())
                g=env.scene['robot'].data.projected_gravity_b
                tilt=torch.acos((-g[:,2]/g.norm(dim=-1)).clamp(-1,1))*180/math.pi
                sensor=env.scene['feet_ground_contact']._extract_sensor_data()
                load=sensor.force[:,:,2].abs();heights=[]
                for gid,vertices in feet:
                    rot=env.sim.data.geom_xmat[:,gid].reshape(-1,3,3)
                    world=torch.matmul(vertices[None],rot.transpose(1,2))+env.sim.data.geom_xpos[:,gid,None,:]
                    heights.append(world[:,:,2].min(-1).values)
                height=torch.stack(heights,dim=-1)
                valid=(tilt<15)&(load>=.1).all(-1)&(height<.0015).all(-1)&~terminations
                at=(step+1)*env.step_dt
                if at>duration-1.000001:
                    tail_valid.append(valid);tail_positions.append(env.scene['robot'].data.root_link_pos_w[:,:2].clone())
                if .8<=at<=1.8:mouth_min=torch.minimum(mouth_min,env.sim.data.site_xpos[:,mouth,2])
                if a.mode=='roulade':pitch.append(torch.atan2(g[:,0],-g[:,2]).cpu().numpy())
            supported=torch.stack(tail_valid).sum(0)
            drift=(tail_positions[-1]-tail_positions[0]).norm(dim=-1)
            recovered=(supported>=.9*len(tail_valid))&(drift<=.005)
            if a.mode=='pick':passed=recovered&(mouth_min<=.04)
            elif a.mode=='roulade':
                turns=np.unwrap(np.stack(pitch),axis=0);span=np.abs(turns-turns[0]).max(0)
                passed=recovered&torch.tensor(span>=1.5*math.pi,device=a.device)
            else:passed=recovered
            if tensor_state_sha(actor.state_dict())!=before:raise ValueError('Frozen actor state changed')
            case.update(passed_envs=int(passed.sum()),num_envs=a.num_envs,recovered_envs=int(recovered.sum()),
                pass_mask=passed.tolist(),recover_mask=recovered.tolist(),last_second_supported_samples=supported.tolist(),
                final_drift_m=drift.tolist(),mouth_descent_min_m=mouth_min.tolist() if a.mode=='pick' else None,
                projected_joint_goals=projection,total_mean_reward=reward_sum,actor_unchanged=True,training_started=False)
            results.append(case)
            (a.output/'summary.json').write_text(json.dumps({'cases':results,'hardware_access':False,'body_identified':False},indent=2)+'\n')
            print(json.dumps({k:case.get(k) for k in ('sigma_rad','passed_envs','num_envs','recovered_envs','actor_unchanged')}),flush=True)
        finally:
            if env is not None:env.close()


if __name__=='__main__':main()
