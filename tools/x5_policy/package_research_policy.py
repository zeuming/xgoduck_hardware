"""Package a separately verified research ONNX contract without deploying it.

Metadata compatibility is not a motion-release certificate. Original training
and evaluation artifacts are kept immutable and bound by SHA256 in a manifest.
"""
import argparse,hashlib,json
from pathlib import Path

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--candidate',type=Path,required=True);p.add_argument('--model',type=Path,required=True)
 p.add_argument('--gates',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 import numpy as np,onnx,onnxruntime as ort
 from constrained_training_candidate import load_candidate,digest
 from rl_core import policy_profile
 c=load_candidate(a.candidate);opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
 source=ort.InferenceSession(c['source_model']['path'],opts,providers=['CPUExecutionProvider']);profile=policy_profile(source)
 np.testing.assert_array_equal(profile['home_rad'],c['home_rad']);assert profile['scale']==c['action_scale']
 gates=[]
 for seed in range(4):
  f=a.gates/f'gate-{seed}/summary.json';s=json.loads(f.read_text())
  assert s['model_sha256']==sha(a.model) and s['candidate_sha256']==digest(c)
  assert s['gate']['source_domain_accepted'] and s['gate']['all_original_saved_ranges']
  gates.append(dict(path=str(f),sha256=sha(f),seed=seed,experiment=s['experiment'],gate=s['gate']))
 assert all(g['experiment']==gates[0]['experiment'] for g in gates)
 domain=gates[0]['experiment'];a.output.mkdir(parents=True,exist_ok=False)
 model=onnx.load(str(a.model));metadata={e.key:e.value for e in model.metadata_props};original=source.get_modelmeta().custom_metadata_map
 for k in ['joint_names','observation_names','default_joint_pos','action_scale']:metadata[k]=original[k]
 metadata['xgoduck_research_contract']=json.dumps(dict(hardware_eligible=False,physical_release_ready=False,production_loader_integrated=False,source_actor_sha256=sha(a.model),candidate_sha256=digest(c),domain=domain,held_head_rad=c['held_head_rad'],arbitrary_runtime_head_capture_compatible=False),sort_keys=True)
 onnx.helper.set_model_props(model,metadata);onnx.checker.check_model(model);target=a.output/'unvalidated_walk.onnx';onnx.save(model,str(target))
 before=ort.InferenceSession(str(a.model),opts,providers=['CPUExecutionProvider']);after=ort.InferenceSession(str(target),opts,providers=['CPUExecutionProvider']);loaded=policy_profile(after)
 np.testing.assert_array_equal(profile['home_rad'],loaded['home_rad']);assert profile['scale']==loaded['scale']
 observations=np.random.default_rng(612).normal(0,.5,(1024,61)).astype(np.float32)
 for x in observations:
  old=before.run(None,{before.get_inputs()[0].name:x[None]})[0];new=after.run(None,{after.get_inputs()[0].name:x[None]})[0];np.testing.assert_array_equal(old,new)
 manifest=dict(scope=__doc__,source_model=str(a.model),source_model_sha256=sha(a.model),packaged_model_sha256=sha(target),source_code_sha256=sha(__file__),candidate_sha256=digest(c),native_metadata_profile_passed=True,inference_bitwise_identical_samples=len(observations),independent_gates=gates,hardware_eligible=False,physical_release_ready=False,production_loader_integrated=False,fully_constrained_simulation_accepted=all(g['gate']['fully_constrained_simulation_accepted'] for g in gates))
 (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps({k:v for k,v in manifest.items() if k!='independent_gates'}))
if __name__=='__main__':main()
