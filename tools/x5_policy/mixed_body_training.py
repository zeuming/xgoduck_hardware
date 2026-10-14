"""Balanced independent physical domains behind the same 61D/14D policy.

Each domain owns its genuine simulator, actuator/delay histories, contacts and
resets. Concatenation changes neither task rewards nor individual transitions.
No domain identifier or new policy architecture is introduced.
"""
from __future__ import annotations
import copy
from types import SimpleNamespace
import torch


class MixedDomainEnv:
    def __init__(self, configs, device, render_mode=None):
        from mjlab.envs import ManagerBasedRlEnv
        self.environments=[]
        try:
            for name,cfg in configs:
                self.environments.append((name,ManagerBasedRlEnv(cfg,device=device,render_mode=render_mode)))
        except Exception:
            self.close();raise
        first=self.environments[0][1]
        self.device=first.device;self.render_mode=render_mode
        self.num_envs=sum(e.num_envs for _,e in self.environments)
        self.max_episode_length=first.max_episode_length
        self.cfg=copy.deepcopy(first.cfg);self.cfg.scene.num_envs=self.num_envs
        self.observation_space=first.observation_space;self.action_space=first.action_space
        for _,e in self.environments:
            if e.max_episode_length!=self.max_episode_length or e.step_dt!=first.step_dt:
                raise ValueError('Mixture domains must share task horizon and control period')
            if e.action_manager.total_action_dim!=14:raise ValueError('Expected14actor actions in each domain')
        self.observation_manager=SimpleNamespace(compute=self.compute_observations)
        terms=first.termination_manager.active_terms
        if any(e.termination_manager.active_terms!=terms for _,e in self.environments):
            raise ValueError('Mixture domains have different task termination definitions')
        self.termination_manager=SimpleNamespace(active_terms=terms,get_term=self.termination)
        self.action_manager=SimpleNamespace(total_action_dim=14,get_term=self.action_term)

    @property
    def unwrapped(self):return self

    @property
    def common_step_counter(self):
        counts=[e.common_step_counter for _,e in self.environments]
        if len(set(counts))!=1:raise ValueError('Independent domains lost synchronized control steps')
        return counts[0]

    @common_step_counter.setter
    def common_step_counter(self,value):
        for _,e in self.environments:e.common_step_counter=value

    @property
    def episode_length_buf(self):
        return torch.cat([e.episode_length_buf for _,e in self.environments])

    @episode_length_buf.setter
    def episode_length_buf(self,value):
        at=0
        for _,e in self.environments:
            e.episode_length_buf[:]=value[at:at+e.num_envs];at+=e.num_envs

    def compute_observations(self):
        parts=[e.observation_manager.compute() for _,e in self.environments]
        keys=list(parts[0])
        if any(list(p)!=keys for p in parts):raise ValueError('Observation groups differ across domains')
        return {k:torch.cat([p[k] for p in parts]) for k in keys}

    def termination(self,name):
        return torch.cat([e.termination_manager.get_term(name) for _,e in self.environments])

    def action_term(self,name):
        if name!='joint_pos':raise ValueError('Only original joint-position action is supported')
        return SimpleNamespace(decoder=SimpleNamespace(projected=torch.cat([
            e.action_manager.get_term(name).decoder.projected for _,e in self.environments])))

    @staticmethod
    def merge_observations(parts):
        keys=list(parts[0])
        if any(list(p)!=keys for p in parts):raise ValueError('Observation group order differs')
        return {k:torch.cat([p[k] for p in parts]) for k in keys}

    def reset(self,seed=None):
        results=[e.reset(seed=None if seed is None else seed+i) for i,(_,e) in enumerate(self.environments)]
        return self.merge_observations([r[0] for r in results]),{}

    def step(self,actions):
        if actions.shape!=(self.num_envs,14):raise ValueError('Combined batch action shape differs')
        parts=[];at=0;log={}
        for name,e in self.environments:
            result=e.step(actions[at:at+e.num_envs]);at+=e.num_envs;parts.append(result)
            log.update({name+'/'+k:v for k,v in result[4].get('log',{}).items()})
        return (self.merge_observations([r[0] for r in parts]),
                torch.cat([r[1] for r in parts]),torch.cat([r[2] for r in parts]),
                torch.cat([r[3] for r in parts]),{'log':log})

    def close(self):
        for _,e in self.environments:e.close()
