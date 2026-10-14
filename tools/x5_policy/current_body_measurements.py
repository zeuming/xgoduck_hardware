"""Independent raw-contact and whole-sole measurements, original verifier copied unchanged.
No targets, state injection, rewards or simulator stepping.
"""
def actual_plant_measurements(env):
    import numpy as np
    import mujoco
    import mujoco_warp
    import warp as wp
    robot = env.scene['robot']
    gravity = robot.data.projected_gravity_b[0].detach().cpu().numpy()
    tilt = float(np.rad2deg(np.arccos(np.clip(-gravity[2]/np.linalg.norm(gravity),-1,1))))
    contact_sensor = env.scene['feet_ground_contact']
    cached_sensor = contact_sensor.data
    # env.step computes rewards at the last substep, caching Sensor.data, then
    # sim.forward refreshes Warp sensordata for the post-integration qpos.
    # Read original sensor slots without cache mutation for same-time comparison.
    sensor = contact_sensor._extract_sensor_data()
    model = env.sim.mj_model
    nacon = int(env.sim.wp_data.nacon.numpy()[0])
    world_ids = env.sim.wp_data.contact.worldid.numpy()[:nacon]
    indices = np.flatnonzero(world_ids == 0).astype(np.int32)
    pairs = env.sim.wp_data.contact.geom.numpy()[indices]
    if len(indices):
        # A GPU-equipped host defaults Warp launches to CUDA even for this CPU
        # evaluator. Bind both buffers and the launch to the actual plant; do
        # not let machine availability choose a different measurement device.
        device = env.sim.wp_data.qpos.device
        with wp.ScopedDevice(device):
            ids = wp.array(indices,dtype=wp.int32,device=device)
            force_array = wp.zeros(len(indices),dtype=wp.spatial_vector,device=device)
            mujoco_warp.contact_force(env.sim.wp_model,env.sim.wp_data,ids,True,force_array)
        forces = force_array.numpy()
    else:
        forces = np.empty((0,6))
    feet = {}
    for side_index,side in enumerate(('left','right')):
        ids = [g for g in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,g) or '').split('/')[-1] == side+'_foot_collision']
        if len(ids) != 1: raise ValueError('Ambiguous foot collision geometry')
        gid = ids[0]; mid = int(model.geom_dataid[gid])
        start,count = int(model.mesh_vertadr[mid]),int(model.mesh_vertnum[mid])
        vertices = np.asarray(model.mesh_vert[start:start+count],dtype=np.float64)
        position = env.sim.data.geom_xpos[0,gid].detach().cpu().numpy()
        rotation = env.sim.data.geom_xmat[0,gid].detach().cpu().numpy().reshape(3,3)
        height = float((vertices@rotation.T+position)[:,2].min())
        count_contacts = 0; other_geoms = []; primary_to_secondary_force = np.zeros(3)
        for pair,force in zip(pairs,forces):
            if gid in pair:
                count_contacts += 1
                primary_to_secondary_force += force[:3] if pair[0] == gid else -force[:3]
                other = pair[1] if pair[0] == gid else pair[0]
                other_geoms.append(mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,int(other)))
        # ContactSensor uses primary foot -> secondary terrain, hence downward Z.
        # Check the signed vector before taking its magnitude as support load.
        sensor_count = int(sensor.found[0,side_index])
        sensor_force = sensor.force[0,side_index].detach().cpu().numpy()
        if sensor_count != count_contacts:
            raise ValueError(f'{side} sensor contact count {sensor_count} != raw {count_contacts}')
        np.testing.assert_allclose(sensor_force,primary_to_secondary_force,atol=1e-4,rtol=1e-4)
        vertical_load = abs(float(primary_to_secondary_force[2]))
        feet[side] = dict(mesh_min_height_m=height,contact_count=count_contacts,
            vertical_load_n=vertical_load,contact_other_geoms=other_geoms,
            upstream_sensor_contact_count=sensor_count,signed_sensor_force_n=sensor_force.tolist(),
            reward_cached_contact_count=int(cached_sensor.found[0,side_index]),
            reward_cached_signed_force_n=cached_sensor.force[0,side_index].tolist(),
            raw_contact_signed_net_force_n=primary_to_secondary_force.tolist(),
            contact_sensor_raw_consistent=True,contact_source='post-forward original Warp contact.geom/contact_force and same-time original ContactSensor slots',
            reward_cache_time_semantics='last physics substep, up to 5ms earlier than post-forward diagnostic',
            lifted=height>=.003 and vertical_load<.001)
    quaternion = robot.data.root_link_quat_w[0].detach().cpu().numpy()
    w,x,y,z = [float(v) for v in quaternion]
    heading = float(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    return dict(root_position=robot.data.root_link_pos_w[0].tolist(),
        root_quaternion_wxyz=quaternion.tolist(),heading_rad=heading,tilt_deg=tilt,feet=feet,
        warp_contact_count=len(indices),contact_sensor_consistency_passed=True)
