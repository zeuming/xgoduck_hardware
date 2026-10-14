"""Build a credential-free allowlisted four-task training/evaluation archive."""
import argparse,hashlib,json,shutil,tarfile
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();root=Path(__file__).resolve().parents[2]
    a.output=a.output.resolve();a.inputs=a.inputs.resolve()
    a.output.mkdir(parents=True,exist_ok=False)
    tools=('current_body_training.py','train_current_body_policy.py','onnx_actor_warmstart.py',
        'kl_checked_optimizer.py','ppo_update_ledger.py','offline_mass_distribution.py',
        'train_constrained_policy.py','current_body_measurements.py','evaluate_current_body_policy.py',
        'evaluate_slow_gait.py','simulation_evidence.py','mixed_body_training.py','current_body_fingerprint.py',
        'source_update_observer.py','run_source_native_training.py')
    def cp(path,name):
        if not path.is_file() or path.is_symlink():raise ValueError('Expected regular allowlisted file')
        dest=a.output/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,dest)
    for name in tools:cp(root/'tools/x5_policy'/name,Path('tools')/name)
    for path in a.inputs.iterdir():
        if path.is_file():cp(path,Path('inputs')/path.name)
    source=root/'output/xgoduck_rl_reference'
    for folder,prefix in ((source/'src/mjlab_microduck',Path('source/mjlab_microduck')),
                          (source/'vendor/bam/bam',Path('vendor/bam/bam'))):
        for path in sorted(folder.rglob('*')):
            if (path.is_file() and '__pycache__' not in path.parts
                and not any(x.startswith('.') for x in path.relative_to(folder).parts)
                and path.suffix.lower() in ('.py','.xml','.json','.stl','.obj','.msh','.mtl','.urdf','.csv','.png')):
                cp(path,prefix/path.relative_to(folder))
    for name in ('pyproject.toml','README.md','LICENSE'):cp(source/name,Path('source')/name)
    manifest={}
    for path in sorted(a.output.rglob('*')):
        if path.is_file():
            blob=path.read_bytes()
            if path.suffix in ('.py','.json','.toml','.md'):
                for marker in (b'-----BEGIN OPENSSH PRIVATE KEY-----',b'-----BEGIN RSA PRIVATE KEY-----',b'Authorization: Bearer ',b'mcp_code_'):
                    if marker in blob:raise ValueError('Credential marker in allowlisted input')
            manifest[str(path.relative_to(a.output))]=hashlib.sha256(blob).hexdigest()
    (a.output/'bundle-manifest.json').write_text(json.dumps(dict(files=manifest,uploaded=False,
        scope=__doc__,credentials_included=False,photos_included=False),indent=2)+'\n')
    archive=a.output.with_suffix('.tar.gz')
    if archive.exists():raise FileExistsError(archive)
    with tarfile.open(archive,'w:gz') as handle:
        for path in sorted(a.output.rglob('*')):
            if path.is_file():handle.add(path,arcname=str(path.relative_to(a.output)),recursive=False)
    proof={'archive':str(archive.relative_to(root)),'files':len(manifest),'bytes':archive.stat().st_size,
           'sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'uploaded':False}
    a.output.with_suffix('.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof))


if __name__=='__main__':main()
