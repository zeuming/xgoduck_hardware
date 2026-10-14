"""Read-only effective-plant measurements and explicit interval-push ablation.

Never infer a randomized Warp plant's mass/COM from nominal CPU model fields.
The optional zero-push diagnostic still calls the original sampling function
with zero ranges, retaining event timers and random draw shapes. It is a new
diagnostic scenario, not a change to the original training/acceptance task.
"""
from functools import wraps
import hashlib
import inspect
from pathlib import Path


def effective_body_properties(env):
    import numpy as np
    model=env.sim.mj_model
    mass=env.sim.model.body_mass[0].detach().cpu().numpy().astype(float).copy()
    ipos=env.sim.model.body_ipos[0].detach().cpu().numpy().astype(float).copy()
    if mass.shape!=(model.nbody,) or ipos.shape!=(model.nbody,3):
        raise ValueError('Unexpected effective body property layout')
    if not np.isfinite(mass).all() or not np.isfinite(ipos).all() or np.any(mass<0) or mass.sum()<=0:
        raise ValueError('Invalid effective mass/COM')
    info=dict(total_mass_kg=float(mass.sum()),nominal_total_mass_kg=float(model.body_mass.sum()),
        body_mass_kg=mass.tolist(),body_ipos_m=ipos.tolist(),
        mass_sha256=hashlib.sha256(mass.tobytes()).hexdigest(),ipos_sha256=hashlib.sha256(ipos.tobytes()).hexdigest(),
        maximum_mass_difference_kg=float(np.abs(mass-model.body_mass).max()),
        maximum_ipos_difference_m=float(np.abs(ipos-model.body_ipos).max()),
        source='actual environment-zero Warp model fields after startup/reset randomization')
    return mass,ipos,info


def world_com(data,mass,ipos):
    import numpy as np
    world=np.asarray(data.xpos)+np.einsum('bij,bj->bi',np.asarray(data.xmat).reshape(-1,3,3),ipos)
    return (mass[:,None]*world).sum(axis=0)/mass.sum()


def verify_com_at_current_state(env,data,mass,ipos):
    import numpy as np
    independent=world_com(data,mass,ipos)
    effective=(mass[:,None]*env.sim.data.xipos[0].detach().cpu().numpy()).sum(axis=0)/mass.sum()
    np.testing.assert_allclose(independent,effective,atol=3e-7,rtol=0)
    return float(np.max(np.abs(independent-effective)))


def audit_interval_push(cfg,ledger,scale=1.):
    if scale not in (0.,1.):raise ValueError('Only original and zero-push paired diagnostics')
    term=cfg.events.get('push_robot')
    if term is None or term.mode!='interval':raise ValueError('Expected upstream interval push')
    original=term.func;original_ranges=dict(term.params['velocity_range'])
    if getattr(original,'__name__','')!='push_by_setting_velocity':raise ValueError('Unexpected push function')
    source=Path(inspect.getfile(original))
    @wraps(original)
    def recorded(env,env_ids,velocity_range,asset_cfg):
        entity=env.scene[asset_cfg.name]
        before=entity.data.root_link_vel_w[env_ids].clone()
        qvel_before=env.sim.data.qvel[env_ids].clone()
        original(env,env_ids,velocity_range,asset_cfg)
        after=entity.data.root_link_vel_w[env_ids].clone()
        qvel_after=env.sim.data.qvel[env_ids].clone()
        ledger.append(dict(episode_steps=env.episode_length_buf[env_ids].tolist(),
            time_s=(env.episode_length_buf[env_ids]*env.step_dt).tolist(),
            env_ids=env_ids.tolist(),cached_link_before=before.tolist(),cached_link_after=after.tolist(),
            qvel_before=qvel_before.tolist(),qvel_after=qvel_after.tolist(),qvel_delta=(qvel_after-qvel_before).tolist(),
            semantics='qvel is authoritative immediately after write; link velocity cache can still describe pre-write state'))
    term.func=recorded
    term.params['velocity_range']={k:(lo*scale,hi*scale) for k,(lo,hi) in original_ranges.items()}
    return dict(scale=scale,original_ranges=original_ranges,applied_ranges=term.params['velocity_range'],
        interval_range_s=term.interval_range_s,source_path=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        original_function_called=True,event_timers_retained=True,random_draw_shape_retained=True)
