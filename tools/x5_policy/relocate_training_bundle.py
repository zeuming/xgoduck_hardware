"""Rebind only offline provenance paths after an allowlisted bundle is moved."""
from __future__ import annotations
import copy
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rebase(value,old,new):
    if isinstance(value,str) and value.startswith(old+'/'):return new+value[len(old):]
    if isinstance(value,list):return [rebase(v,old,new) for v in value]
    if isinstance(value,dict):return {k:rebase(v,old,new) for k,v in value.items()}
    return value


def reconcile_platform_geometry(pinned,computed):
    """Permit only audited floating-point roundoff, never new mesh/pose geometry.

    A relocated candidate receives the local calculation and a new identity. The
    adapter continues to require exact equality on that platform. This is not a
    checkpoint migration or a relaxation of any hardware/calibration limit.
    """
    expected=copy.deepcopy(pinned)
    if set(pinned)!=set(computed):raise ValueError('Reset placement fields changed')
    deltas={}
    pairs=[('root_z_m',pinned['root_z_m'],computed['root_z_m'])]
    if set(pinned['root_zero_sole_min_z_m'])!={'left','right'} or set(computed['root_zero_sole_min_z_m'])!={'left','right'}:
        raise ValueError('Sole geometry fields changed')
    pairs += [('sole_'+side,pinned['root_zero_sole_min_z_m'][side],computed['root_zero_sole_min_z_m'][side]) for side in ('left','right')]
    for name,before,after in pairs:
        if type(before) not in (float,int) or type(after) not in (float,int) or not math.isfinite(before) or not math.isfinite(after):
            raise ValueError('Non-finite reset geometry')
        budget=32*max(math.ulp(before),math.ulp(after))
        if abs(after-before)>min(budget,1e-14):raise ValueError('Reset geometry exceeds floating-point roundoff: '+name)
        deltas[name]=after-before
    expected['root_z_m']=computed['root_z_m']
    expected['root_zero_sole_min_z_m']=copy.deepcopy(computed['root_zero_sole_min_z_m'])
    if expected!=computed:raise ValueError('Mesh, XML, clearance or reset policy changed')
    return expected,deltas


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bundle-root',type=Path,default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--recompute-platform-geometry',action='store_true')
    args=parser.parse_args()
    root=args.bundle_root.resolve()
    manifest=json.loads((root/'bundle-manifest.json').read_text())
    for name,expected in manifest['files'].items():
        if sha(root/name)!=expected:raise ValueError('Bundle checksum mismatch: '+name)
    sys.path.insert(0,str(root/'output/xgoduck_rl_reference/src'))
    sys.path.insert(0,str(root/'output/xgoduck_rl_reference/vendor/bam'))
    sys.path.insert(0,str(root/'tools/x5_policy'))
    from constrained_training_candidate import digest,save_candidate,check_model_compatibility,grounded_reset_placement
    original=json.loads((root/'inputs/original-candidate.json').read_text())
    if digest(original['candidate'])!=original['candidate_sha256']:raise ValueError('Original candidate checksum mismatch')
    candidate=rebase(original['candidate'],manifest['original_root'],str(root))
    result=args.output_dir.resolve() if args.output_dir else root/'relocated'
    result.mkdir(exist_ok=False)
    contract=json.loads((root/'inputs/minimal-contract.json').read_text())
    contract=rebase(contract,manifest['original_root'],str(root))
    contract_path=result/'contract.json'
    contract_path.write_text(json.dumps(contract,indent=2)+'\n')
    snapshot=root/'inputs/minimal-snapshot.json'
    candidate['source_contract']=dict(path=str(contract_path),sha256=sha(contract_path))
    candidate['source_snapshot']=dict(path=str(snapshot),sha256=sha(snapshot))
    # No kinematics, gains, saved ranges, normalization or plant settings change.
    expected=rebase(original['candidate'],manifest['original_root'],str(root))
    for field in ('source_contract','source_snapshot'):expected[field]=candidate[field]
    if candidate!=expected:raise AssertionError('Migration changed more than paths and sanitized provenance')
    geometry_deltas=None
    if args.recompute_platform_geometry:
        candidate['reset_placement'],geometry_deltas=reconcile_platform_geometry(
            candidate['reset_placement'],grounded_reset_placement(candidate))
    save_candidate(candidate,result/'candidate.json')
    import onnx
    baseline_source=root/'inputs/untrained_bounded_actor.onnx'
    model=onnx.load(str(baseline_source))
    graph_sha=hashlib.sha256(model.graph.SerializeToString()).hexdigest()
    metadata={p.key:p.value for p in model.metadata_props}
    if metadata['constrained_training_candidate_sha256']!=original['candidate_sha256']:
        raise ValueError('Baseline was not bound to original candidate')
    metadata.update(constrained_training_candidate_sha256=digest(candidate),
                    relocation_original_model_sha256=sha(baseline_source),
                    relocation_original_candidate_sha256=original['candidate_sha256'],
                    hardware_eligible='false',walking_verified='false')
    del model.metadata_props[:]
    for key,value in metadata.items():
        entry=model.metadata_props.add();entry.key=key;entry.value=value
    baseline=result/'untrained_bounded_actor.onnx'
    onnx.save(model,str(baseline))
    if hashlib.sha256(onnx.load(str(baseline)).graph.SerializeToString()).hexdigest()!=graph_sha:
        raise AssertionError('Relocation changed baseline graph')
    check_model_compatibility(candidate,baseline)
    receipt=dict(original_candidate_sha256=original['candidate_sha256'],candidate_sha256=digest(candidate),
                 baseline_original_sha256=sha(baseline_source),baseline_relocated_sha256=sha(baseline),
                 baseline_graph_sha256=graph_sha,plant_unchanged=geometry_deltas is None,
                 platform_geometry_roundoff_m=geometry_deltas,
                 checkpoint_resume=False,physical_parameters_unchanged=True,
                 migration_source_sha256=sha(__file__),source_checks_passed=True,
                 uploaded=False,gpu_execution_verified=False,hardware_eligible=False)
    (result/'relocation.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print(json.dumps(receipt,indent=2))


if __name__=='__main__':main()
