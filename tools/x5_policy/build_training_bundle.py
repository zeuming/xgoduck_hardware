"""Create a local allowlisted training bundle; never connect or upload."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

ROOT=Path(__file__).resolve().parents[2]
TOOLS=('train_constrained_policy.py','constrained_training_candidate.py','model_adaptation_contract.py',
       'rl_core.py','onnx_actor_warmstart.py','bounded_action_distribution.py','stand_button_training_profile.py',
       'slew_matched_exploration.py','validate_training_regression.py','ppo_update_ledger.py',
       'kl_checked_optimizer.py','checkpoint_continuation.py','phase_training_ledger.py',
       'forward_learning_experiment.py',
       'relocate_training_bundle.py')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate',type=Path,required=True)
    parser.add_argument('--baseline',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    def copy(source,name=None):
        source=source.resolve()
        if source.is_symlink() or not source.is_file():raise ValueError('Expected regular input')
        name=name or str(source.relative_to(ROOT))
        destination=output/name;destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source,destination)
    def write(name,value):
        path=output/name;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
    for name in TOOLS:copy(ROOT/'tools/x5_policy'/name)
    c=json.loads(args.candidate.read_text())['candidate']
    # Verify every pinned input before packaging. Sensitive snapshots are read
    # only to extract the numerical fields explicitly required by the adapter.
    sources=[c['adapter_source'],c['source_contract'],c['source_snapshot'],c['source_model'],
             *c['source_files'].values(),*c['integration_sources']]
    for source in sources:
        if sha(source['path'])!=source['sha256']:raise ValueError('Pinned input drift: '+source['path'])
    copy(args.candidate,'inputs/original-candidate.json')
    copy(args.baseline,'inputs/untrained_bounded_actor.onnx')
    copy(Path(c['source_model']['path']))
    for key in ('runtime_math','runtime_policy'):copy(Path(c['source_files'][key]['path']))
    for name in ('pyproject.toml','README.md','LICENSE'):
        copy(ROOT/'output/xgoduck_rl_reference'/name)
    for subtree in ('src/mjlab_microduck','vendor/bam/bam'):
        folder=ROOT/'output/xgoduck_rl_reference'/subtree
        for path in sorted(folder.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and not any(part.startswith('.') for part in path.relative_to(folder).parts):
                if path.suffix.lower() in ('.py','.xml','.json','.stl','.obj','.msh','.png','.jpg','.jpeg','.mtl','.urdf','.csv'):
                    copy(path)
    copy(ROOT/'output/duck_sim/adaptation-20261011/candidate/policy/bounded_forward.py')
    copy(ROOT/'output/duck_sim/repair-20261011/model/training-adapter/smoke_training_adapter.py')
    contract=json.loads(Path(c['source_contract']['path']).read_text())
    snapshot=json.loads(Path(c['source_snapshot']['path']).read_text())
    hardware=snapshot['motion'].get('hardware') or snapshot['motion']['last_hardware']
    write('inputs/minimal-contract.json',dict(calibration=contract['calibration'],model=contract['model'],
        runtime_differences=dict(source_expected_gains=contract['runtime_differences']['source_expected_gains'])))
    write('inputs/minimal-snapshot.json',dict(calibration=snapshot['calibration'],
        motion=dict(hardware=dict(hold_head_deg=hardware['hold_head_deg']))))
    manifest=dict(schema_version=1,original_root=str(ROOT),created_locally=True,uploaded=False,
                  source_candidate_sha256=sha(args.candidate),
                  snapshot_fields=['calibration','motion.hardware.hold_head_deg'],
                  files={str(p.relative_to(output)):sha(p) for p in sorted(output.rglob('*')) if p.is_file()})
    write('bundle-manifest.json',manifest)
    archive=output.with_suffix('.tar.gz')
    if archive.exists():raise FileExistsError(archive)
    with tarfile.open(archive,'w:gz') as handle:
        for path in sorted(output.rglob('*')):
            if path.is_file():handle.add(path,arcname=str(path.relative_to(output)),recursive=False)
    print(json.dumps(dict(directory=str(output),files=len(manifest['files']),archive=str(archive),
                         bytes=archive.stat().st_size,archive_sha256=sha(archive),uploaded=False)))


if __name__=='__main__':main()
