"""Audited actor-only transfer for a NEW task, with explicit path relocation."""
import copy,hashlib,json,math,re,tempfile
from pathlib import Path
import numpy as np
import torch
from tensordict import TensorDict
from constrained_training_candidate import digest
from onnx_actor_warmstart import warmstart_actor_from_onnx
from train_constrained_policy import assert_state_identical,tensor_state_sha

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def portable_candidate(c):
    """Only provenance paths/envelope hashes and <1e-12 reset roundoff differ.

    All calibration content, referenced-source content hashes, model/plant
    parameters and policy contracts remain in this actor-transfer comparison.
    This does not enable old optimizer/checkpoint continuation across paths.
    """
    c=copy.deepcopy(c)
    c.pop('source_contract');c.pop('source_snapshot')
    def walk(v,key=''):
        if isinstance(v,dict):return {k:walk(x,k) for k,x in v.items() if k not in ('path','xml_path')}
        if isinstance(v,list):return [walk(x,key) for x in v]
        if isinstance(v,float) and key in ('root_z_m','left','right'):return round(v,12)
        return v
    return walk(c)


def import_parent_actor(actor,c,parent,gain,observations,samples=256,snapshot=None):
    if not math.isfinite(gain) or gain<=0:raise ValueError('Invalid command coordinate gain')
    parent=Path(parent);report=json.loads((parent/'run.json').read_text())
    if report['stage']!='complete_requires_independent_walking_gate' or not report['full_checkpoint_reload']:raise ValueError('Incomplete parent')
    if sha(parent/'checkpoint.pt')!=report['checkpoint_sha256'] or sha(parent/'actor.onnx')!=report['export']['sha256']:raise ValueError('Parent artifact changed')
    previous=json.loads((parent/'environment.json').read_text())['actions']['joint_pos']['candidate']
    if digest(previous)!=report['candidate_sha256']:raise ValueError('Parent candidate envelope mismatch')
    if portable_candidate(previous)!=portable_candidate(c):raise ValueError('Parent physical/calibration/contract difference')
    checkpoint=torch.load(parent/'checkpoint.pt',map_location='cpu',weights_only=True)
    if tensor_state_sha(checkpoint['actor_state_dict'])!=report['actor_tensor_sha256']:raise ValueError('Parent actor tensor mismatch')
    if checkpoint['infos']['candidate_sha256']!=report['candidate_sha256'] or json.loads(json.dumps(checkpoint['infos']['experiment'],allow_nan=False))!=report['experiment']:raise ValueError('Parent checkpoint binding mismatch')
    warmstart_actor_from_onnx(actor,c['source_model']['path'],expected_sha256=c['source_model']['sha256'])
    actor.load_state_dict(checkpoint['actor_state_dict'],strict=True)
    assert_state_identical(checkpoint['actor_state_dict'],{k:v.detach().cpu() for k,v in actor.state_dict().items()})
    model=parent/'actor.onnx';snapshot_record=None
    if snapshot is not None:
        if not re.fullmatch(r'snapshot-\d{5}\.onnx',snapshot):raise ValueError('Invalid snapshot name')
        model=parent/snapshot;meta=json.loads(model.with_suffix('.json').read_text())
        if sha(model)!=meta['sha256'] or meta['candidate_sha256']!=report['candidate_sha256'] or meta['experiment']!=report['experiment']:raise ValueError('Snapshot binding mismatch')
        if not 0<meta['iterations']<=report['iterations'] or snapshot!=f"snapshot-{meta['iterations']:05d}.onnx":raise ValueError('Snapshot iteration mismatch')
        # Rename exported Sequential tensor labels to the original audited
        # layout. All operators/data flow then pass the existing strict loader;
        # original snapshot bytes are never rewritten. ORT parity below checks
        # against the original, unrenamed snapshot after import.
        import onnx
        graph=onnx.load_model(str(model),load_external_data=False)
        names={'0.onnx_mean':'obs_normalizer._mean','0.onnx_denominator':'onnx::Div_24'}
        for layer in (0,2,4,6):
            for suffix in ('weight','bias'):names[f'1.{layer}.{suffix}']=f'mlp.{layer}.{suffix}'
        if {x.name for x in graph.graph.initializer}!=set(names):raise ValueError('Unexpected snapshot tensor layout')
        for initializer in graph.graph.initializer:initializer.name=names[initializer.name]
        for node in graph.graph.node:
            for i,name in enumerate(node.input):
                if name in names:node.input[i]=names[name]
        with tempfile.TemporaryDirectory(prefix='duck-actor-transfer-') as directory:
            renamed=Path(directory)/'audited-actor.onnx';onnx.save(graph,str(renamed))
            snapshot_record=warmstart_actor_from_onnx(actor,renamed,expected_sha256=sha(renamed))
    # (gain*x - parent_mean)/parent_denominator, expressed with the original
    # 61D affine normalizer. No learned weight or physical command is increased.
    with torch.no_grad():
        actor.obs_normalizer.onnx_mean[:,48]/=gain
        actor.obs_normalizer.onnx_denominator[:,48]/=gain
    import onnxruntime as ort
    options=ort.SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    session=ort.InferenceSession(str(model),options,providers=['CPUExecutionProvider'])
    rng=np.random.default_rng(74)
    raw=np.repeat(observations[:1].detach().cpu().numpy(),samples,axis=0)
    raw+=rng.normal(0,.05,raw.shape).astype(np.float32)
    raw[:samples//2,48]=0.;raw[samples//2:,48]=rng.uniform(0,.06,samples-samples//2)
    transformed=raw.copy();transformed[:,48]*=gain
    expected=np.concatenate([session.run(None,{session.get_inputs()[0].name:r[None]})[0] for r in transformed])
    with torch.no_grad():actual=actor(TensorDict({actor.obs_groups[0]:torch.tensor(raw,device=next(actor.parameters()).device)},batch_size=[samples]),stochastic_output=False).cpu().numpy()
    np.testing.assert_allclose(actual,expected,atol=1e-5,rtol=1e-5)
    return dict(parent_run_sha256=sha(parent/'run.json'),parent_actor_sha256=sha(model),selected_snapshot=snapshot,snapshot_import=snapshot_record,parent_checkpoint_sha256=sha(parent/'checkpoint.pt'),
        parent_candidate_sha256=report['candidate_sha256'],new_candidate_sha256=digest(c),portable_candidate_sha256=digest(portable_candidate(c)),
        ignored_provenance='only file paths, source snapshot/contract envelope hashes, reset-position12decimal roundoff; calibration and content hashes retained',
        actor_only=True,critic_optimizer_kl_restored=False,command_coordinate_gain=gain,
        equivalence_observations=samples,max_output_error=float(abs(expected-actual).max()),zero_command_observations=samples//2,
        fresh_training_state=True,hardware_eligible=False)
