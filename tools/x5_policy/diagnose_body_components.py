"""Paired offline sensitivity to rear inertia and an uncertain rigid envelope.

The native simulator, official actor, BAM, goals and saved limits are retained.
Every branch has the same geometry layout. Only trunk inertia and the envelope
contact mask are varied. This is not a measured body or hardware validation.
"""
import argparse,copy,dataclasses,hashlib,json,math,random,sys,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--frozen-tools',type=Path,required=True)
    p.add_argument('--mode',choices=('getup','pick'),required=True)
    p.add_argument('--seed',type=int,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--wall-seconds',type=float,default=240.)
    p.add_argument('--audit-only',action='store_true',help='Verify initialized field differences without dynamic replay.')
    p.add_argument('--reuse-walk-head-command',action='store_true',
        help='Getup-only counterfactual: reuse the existing production4Dhead command, not new angles.')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(a.frozen_tools.resolve()))
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np,torch,mujoco,mujoco_warp,warp as wp,onnxruntime as ort
    from mjlab.envs import ManagerBasedRlEnv
    from current_body_training import configure,load_contract,sha
    from current_body_measurements import actual_plant_measurements
    from current_body_fingerprint import initial_fingerprint
    from offline_mass_distribution import set_inertial,inertia_matrix
    from simulation_evidence import effective_body_properties
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    c=load_contract(a.contract)
    if a.reuse_walk_head_command and a.mode!='getup':raise ValueError('Head-input counterfactual is getup-only')
    opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
    session=ort.InferenceSession(c['files'][a.mode]['resolved_path'],opts,providers=['CPUExecutionProvider'])
    start=time.monotonic();results=[];initials={};direct_fields={}
    summary=dict(scope=__doc__,mode=a.mode,seed=a.seed,body_identified=False,
        hardware_access=False,hardware_eligible=False,source_adapter_sha256=sha(a.frozen_tools/'current_body_training.py'),
        diagnostic_sha256=sha(__file__),model_sha256=sha(c['files'][a.mode]['resolved_path']),cases=results,
        reused_production_head_command=c['head_command_rad'] if a.reuse_walk_head_command else None)
    def save():
        (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    def backpack_contact(env,gid):
        n=int(env.sim.wp_data.nacon.numpy()[0])
        worlds=env.sim.wp_data.contact.worldid.numpy()[:n]
        pairs=env.sim.wp_data.contact.geom.numpy()[:n]
        selected=np.flatnonzero((worlds==0)&np.any(pairs==gid,axis=1)).astype(np.int32)
        if not len(selected):return dict(count=0,sum_force_norm_n=0.)
        device=env.sim.wp_data.qpos.device
        with wp.ScopedDevice(device):
            ids=wp.array(selected,dtype=wp.int32,device=device)
            forces=wp.zeros(len(selected),dtype=wp.spatial_vector,device=device)
            mujoco_warp.contact_force(env.sim.wp_model,env.sim.wp_data,ids,True,forces)
        return dict(count=len(selected),sum_force_norm_n=float(np.linalg.norm(forces.numpy()[:,:3],axis=1).sum()))
    for inertia,enabled in (('high',True),('high',False),('reference',True),('reference',False)):
        name=inertia+('-envelope' if enabled else '-disabled-envelope')
        out=a.output/name;out.mkdir();env=None;rows=[]
        record=dict(name=name,inertia=inertia,envelope_enabled=enabled,completed=False)
        results.append(record);save()
        try:
            cfg,_,_=configure(c,a.mode,'rear-high')
            reference,_,_=configure(c,a.mode,'reference')
            robot=cfg.scene.entities['robot'];builder=robot.spec_fn;reference_builder=reference.scene.entities['robot'].spec_fn
            def spec_fn():
                spec=builder();before=spec.compile()
                if inertia=='reference':
                    ref=reference_builder().compile();bid=ref.body('trunk_base').id
                    np.testing.assert_array_equal(before.body_mass,ref.body_mass)
                    set_inertial(spec.body('trunk_base'),float(ref.body_mass[bid]),ref.body_ipos[bid],inertia_matrix(ref,bid))
                after=spec.compile()
                for key in ('body_mass','body_pos','body_quat','jnt_pos','jnt_axis','jnt_range','qpos0',
                            'geom_pos','geom_quat','geom_size','geom_contype','geom_conaffinity',
                            'dof_armature','dof_damping'):
                    np.testing.assert_array_equal(getattr(before,key),getattr(after,key))
                assert before.ngeom==after.ngeom
                return spec
            robot.spec_fn=spec_fn
            # Official CollisionCfg reassigns .*_collision masks after spec_fn.
            # Override that exact geom too; a spec-only toggle would be ignored.
            robot.collisions=tuple(dataclasses.replace(x,
                contype={'^backpack_collision$':1 if enabled else 0,**x.contype},
                conaffinity={'^backpack_collision$':0,**x.conaffinity}) for x in robot.collisions)
            cfg.scene.num_envs=1;cfg.seed=a.seed;cfg.auto_reset=False;cfg.curriculum.clear()
            if a.reuse_walk_head_command:
                cfg.commands['head_pose'].ranges=tuple((v,v) for v in c['head_command_rad'])
            cfg.events.pop('push_robot',None);seconds=8. if a.mode=='getup' else 4.
            cfg.episode_length_s=seconds+.02
            torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed)
            env=ManagerBasedRlEnv(cfg,device='cpu',render_mode=None);obs,_=env.reset(seed=a.seed)
            model=env.sim.mj_model
            gids=[i for i in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').split('/')[-1]=='backpack_collision']
            assert len(gids)==1;gid=gids[0]
            mouths=[i for i in range(model.nsite) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_SITE,i) or '').split('/')[-1]=='mouth_tip']
            assert len(mouths)==1;mouth=mouths[0]
            wp_type=np.asarray(env.sim.wp_model.geom_contype.numpy()).reshape(-1)
            wp_affinity=np.asarray(env.sim.wp_model.geom_conaffinity.numpy()).reshape(-1)
            assert int(model.geom_contype[gid])==int(wp_type[gid])==(1 if enabled else 0)
            assert int(model.geom_conaffinity[gid])==int(wp_affinity[gid])==0
            fp=initial_fingerprint(env);initials[name]=fp
            bodies=[i for i in range(model.nbody) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,i) or '').split('/')[-1]=='trunk_base']
            assert len(bodies)==1;trunk=bodies[0]
            direct_fields[name]={key:getattr(env.sim.wp_model,key).numpy().copy() for key in
                ('body_ipos','body_iquat','body_inertia','geom_contype','geom_conaffinity')}
            record.update(initial_fingerprint=fp,initial_observation=obs['actor'][0].tolist(),
                actual_compiled_contact_mask=[int(wp_type[gid]),int(wp_affinity[gid])],geometry_count=model.ngeom,
                trunk_body_index=trunk,backpack_geom_index=gid,
                initial_qpos=env.sim.data.qpos[0].tolist(),initial_qvel=env.sim.data.qvel[0].tolist())
            _,_,record['effective_body']=effective_body_properties(env)
            if name=='high-envelope':
                original=json.loads((a.baseline/'gate.json').read_text())
                assert fp==original['initial_fingerprint'],'combined control differs from archived original baseline'
                record['archived_initial_fingerprint_exact']=True
                if a.reuse_walk_head_command:
                    old_obs=np.asarray(json.loads((a.baseline/'trace.jsonl').read_text().splitlines()[0])['actor_observation'],dtype=np.float32)
                    new_obs=np.asarray(record['initial_observation'],dtype=np.float32)
                    np.testing.assert_array_equal(new_obs[np.r_[0:51,55:61]],old_obs[np.r_[0:51,55:61]])
                    np.testing.assert_array_equal(new_obs[51:55]-old_obs[51:55],np.asarray(c['head_command_rad'],dtype=np.float32))
                    record['only_existing4Dhead_command_changed']=True
            if a.audit_only:
                record.update(completed=True,initial_only=True)
                continue
            term=env.action_manager.get_term('joint_pos');steps=round(seconds/env.step_dt)
            with (out/'trace.jsonl').open('x') as log:
                for step in range(steps):
                    if time.monotonic()-start>a.wall_seconds:raise TimeoutError('component diagnostic wall budget')
                    raw=obs['actor'].cpu().numpy().copy()
                    action=session.run(None,{session.get_inputs()[0].name:raw})[0]
                    obs,_,terminated,truncated,_=env.step(torch.tensor(action))
                    row=actual_plant_measurements(env);contact=backpack_contact(env,gid)
                    if not enabled:assert contact['count']==0,'disabled envelope still produced contacts'
                    row.update(time_s=(step+1)*env.step_dt,actor_observation=raw[0].tolist(),raw_action=action[0].tolist(),
                        processed_joint_rad=term._processed_actions[0].tolist(),transmitted_ticks=term.decoder.ticks[0].tolist(),
                        projected_joint_goals=int(term.decoder.projected.sum()),backpack_contact=contact,
                        mouth_tip_height_m=float(env.sim.data.site_xpos[0,mouth,2]),
                        failed_terms=[k for k in env.termination_manager.active_terms if k!='time_out' and bool(env.termination_manager.get_term(k)[0])],task_complete=step==steps-1)
                    rows.append(row);log.write(json.dumps(row,allow_nan=False)+'\n')
                    if bool(terminated[0]) or bool(truncated[0]):break
            tail=rows[-round(1/env.step_dt):];complete=bool(rows) and rows[-1]['task_complete']
            stable=len(tail)==round(1/env.step_dt) and all(x['tilt_deg']<15 and not x['failed_terms'] for x in tail)
            support=sum(all(x['feet'][s]['vertical_load_n']>=.1 and x['feet'][s]['mesh_min_height_m']<.0015 for s in ('left','right')) for x in tail)
            drift=math.dist(tail[-1]['root_position'][:2],tail[0]['root_position'][:2]) if tail else float('inf')
            accepted=complete and stable and support>=.9*len(tail) and drift<=.005
            if a.mode=='pick':accepted=accepted and min(x['mouth_tip_height_m'] for x in rows if .8<=x['time_s']<=1.8)<=.04
            record.update(completed=True,accepted=accepted,final_supported_frames=support,final_drift_m=drift,
                final_tilt_deg=rows[-1]['tilt_deg'],backpack_contact_frames=sum(x['backpack_contact']['count']>0 for x in rows),
                max_backpack_force_n=max(x['backpack_contact']['sum_force_norm_n'] for x in rows))
            if name=='high-envelope' and not a.reuse_walk_head_command:
                old=[json.loads(x) for x in (a.baseline/'trace.jsonl').read_text().splitlines()]
                assert len(old)==len(rows)
                fields=('root_position','root_quaternion_wxyz','actor_observation','raw_action','transmitted_ticks','processed_joint_rad','feet')
                for i,(x,y) in enumerate(zip(old,rows)):
                    for key in fields:assert x[key]==y[key],(i,key)
                record['archived_trajectory_exact']=True
        except Exception as error:
            record['error']=repr(error);save();raise
        finally:
            if env is not None:env.close()
            save()
    base=results[0]
    for record in results:
        assert record['geometry_count']==base['geometry_count']
        assert record['initial_qpos']==base['initial_qpos'] and record['initial_qvel']==base['initial_qvel']
        assert record['initial_observation']==base['initial_observation']
        changed=[key for key,value in initials[base['name']].items() if initials[record['name']].get(key)!=value]
        record['changed_initialized_fields']=changed
        # Always enforce identical RNG, actuators, battery/delay and BAM state.
        assert not any(key=='torch_rng' or key.startswith(('motor.','delay.','bam.','bam_actuator.')) for key in changed),changed
        allowed=set()
        if record['inertia']=='reference':
            allowed.update('warp_model.'+key for key in ('body_ipos','body_iquat','body_inertia',
                'body_invweight0','dof_invweight0','light_poscom0','actuator_acc0'))
        if not record['envelope_enabled']:
            allowed.update('warp_model.'+key for key in ('geom_contype','nxn_geom_pair_filtered',
                'nxn_pairid','nxn_pairid_filtered'))
        assert set(changed)<=allowed,('undeclared initialized field changes',set(changed)-allowed)
        for key in ('body_ipos','body_iquat','body_inertia'):
            np.testing.assert_array_equal(np.delete(direct_fields[record['name']][key],record['trunk_body_index'],axis=-2),
                np.delete(direct_fields[base['name']][key],base['trunk_body_index'],axis=-2))
        for key in ('geom_contype','geom_conaffinity'):
            np.testing.assert_array_equal(np.delete(direct_fields[record['name']][key].reshape(-1),record['backpack_geom_index']),
                np.delete(direct_fields[base['name']][key].reshape(-1),base['backpack_geom_index']))
        record.update(only_declared_inertia_contact_fields_or_derived_caches_changed=True,
            nontrunk_inertia_exact=True,other_geom_masks_exact=True)
    summary.update(complete=True,paired_qpos_qvel_observation_rng_motor_exact=True,
        field_differences_require_review=False,only_declared_initialized_field_changes=True,
        interpretation='Counterfactual body-prior sensitivity only, not a real body identification or hardware acceptance.')
    save();print(json.dumps({k:summary[k] for k in ('mode','seed','complete','paired_qpos_qvel_observation_rng_motor_exact')}))


if __name__=='__main__':main()
