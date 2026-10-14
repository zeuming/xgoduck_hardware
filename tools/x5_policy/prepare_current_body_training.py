"""Prepare explicit, numerical-only training inputs without device access."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--head-profile',type=Path,required=True)
    p.add_argument('--production',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--backpack-collision',action='store_true',help='Explicit approximate photo-envelope collision hypothesis')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    import onnx
    from current_body_training import sha
    files={};models={}
    def copy(source,name):
        shutil.copyfile(source,a.output/name)
        return dict(path=name,sha256=sha(a.output/name))
    for mode in ('walk','getup','pick','roulade'):
        path=a.models/f'xgoduck_{mode}.onnx'
        graph=onnx.load(str(path));meta={m.key:m.value for m in graph.metadata_props}
        files[mode]=copy(path,path.name)
        models[mode]=dict(mode=mode,home_rad=[float(v) for v in meta['default_joint_pos'].split(',')],
            action_scale=float(meta['action_scale']),joint_names=meta['joint_names'].split(','),
            observation_names=meta['observation_names'].split(','))
        assert len(models[mode]['home_rad'])==14
    original=json.loads(a.calibration.read_text())
    # Only explicitly needed numerical motor-coordinate fields. No users,
    # addresses, paths, IMU identity, snapshots, network state or credentials.
    numerical={'joints':[{k:j[k] for k in ('id','zero','sign','min','max')} for j in original['joints']]}
    path=a.output/'calibration-numerical.json';path.write_text(json.dumps(numerical,indent=2)+'\n')
    files['calibration']=dict(path=path.name,sha256=sha(path))
    head=json.loads(a.head_profile.read_text())['walk_head_command_rad']
    scenarios={
        'reference':dict(relocated_mass_kg=0.,rear_of_hip_m=0.,height_above_original_trunk_com_m=0.),
        'rear-mid':dict(relocated_mass_kg=.16,rear_of_hip_m=.06,height_above_original_trunk_com_m=.03),
        'rear-high':dict(relocated_mass_kg=.22,rear_of_hip_m=.09,height_above_original_trunk_com_m=.06),
    }
    for key,scenario in scenarios.items():
        if a.backpack_collision and key!='reference':
            scenario['collision_envelope']=dict(full_size_m=[.05,.065,.10],
                center_z_above_hip_m=.04,identified_geometry=False,
                basis='Approximate5cm rear protrusion and9-11cm external height from user ruler photos; width6.5cm prior includes board/wires; not a calibrated scan')
        scenario.update(total_mass_kg=.864,identified_body=False,
            basis='Total mass measured. Rear position3-9cm user-reported; relocated mass and height are unmeasured robustness priors, not total864-800g or identified board mass.')
        path=a.output/f'body-{key}.json';path.write_text(json.dumps(scenario,indent=2)+'\n')
        files['body_'+key]=dict(path=path.name,sha256=sha(path))
    contract=dict(schema_version=1,files=files,models=models,head_command_rad=head,
        body=dict(total_mass_kg=.864,measured_total_mass=True,mass_distribution_identified=False,
            installation_preserved=True,scenarios=list(scenarios)),
        production_reference=dict(files={n:sha(a.production/'policy'/n) for n in ('motion.py','rl_core.py','production_motion.py')},
            calibration_original_sha256=sha(a.calibration)),
        source_runtime_commit='8cdbbd84710d856581982c9eaf0d5e2970666232',
        source_training_commit='326d77a1122870bdefa2c36403937502c958e69c',
        exact_onnx_training_revision_known=False,hardware_eligible=False,
        limitations=['Unmeasured real COM/inertia and rear collision envelope',
                    'CAD/actual joint zero alignment remains assumed',
                    'Original CAD IMU noise/delay retained; actual installed filter, asynchronous feedback and supply resets not identified',
                    'Original passive jaw does not reproduce separate runtime mouth servo'])
    (a.output/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
    print(json.dumps({'contract':str(a.output/'contract.json'),'files':len(files),
                      'body_identified':False,'training_started':False}))


if __name__=='__main__':main()
