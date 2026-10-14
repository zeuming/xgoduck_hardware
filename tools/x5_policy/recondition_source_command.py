"""Explicit offline ONNX command-coordinate transfer, never deployed automatically.

Reuses an already demonstrated walking response at a newly declared command
coordinate. Plant limits, normalizer, and remaining observations are untouched.
This is a NEW policy candidate, not unchanged checkpoint continuation or proof
of speed tracking. A subsequent independent plant gate is required.
"""
import argparse,hashlib,json
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--source-walking-command',type=float,required=True);p.add_argument('--target-command',type=float,required=True);a=p.parse_args()
 if not 0<a.target_command<=a.source_walking_command<=.3:raise ValueError('Command transfer outside research interval')
 import numpy as np,onnx,onnxruntime as ort
 from onnx import helper,numpy_helper
 m=onnx.load(str(a.source));assert len(m.graph.input)==len(m.graph.output)==1
 assert [d.dim_value for d in m.graph.input[0].type.tensor_type.shape.dim]==[1,61]
 name=m.graph.input[0].name;gain_name='declared_command_coordinate_scale';out_name='declared_command_reconditioned_observation'
 assert all(gain_name!=x.name for x in m.graph.initializer)
 for n in m.graph.node:
  for i,v in enumerate(n.input):
   if v==name:n.input[i]=out_name
 scale=np.ones((1,61),np.float32);scale[0,48]=a.source_walking_command/a.target_command
 m.graph.initializer.append(numpy_helper.from_array(scale,gain_name));m.graph.node.insert(0,helper.make_node('Mul',[name,gain_name],[out_name],name='explicit_command_coordinate_transfer'))
 onnx.checker.check_model(m);a.output.mkdir(parents=True,exist_ok=False);model=a.output/'actor.onnx';onnx.save(m,str(model))
 opts=ort.SessionOptions();opts.intra_op_num_threads=opts.inter_op_num_threads=1
 old=ort.InferenceSession(str(a.source),opts,providers=['CPUExecutionProvider']);new=ort.InferenceSession(str(model),opts,providers=['CPUExecutionProvider'])
 rng=np.random.default_rng(128);observations=rng.normal(0,.5,(4096,61)).astype(np.float32);observations[:,48:51]=0
 def run(s,v):return np.concatenate([s.run(None,{s.get_inputs()[0].name:x[None]})[0] for x in v])
 old_zero=run(old,observations);new_zero=run(new,observations);assert np.array_equal(old_zero,new_zero)
 observations[:,48]=rng.uniform(0,a.target_command,4096).astype(np.float32)
 expected=run(old,observations*scale);actual=run(new,observations);np.testing.assert_array_equal(expected,actual)
 info=dict(scope=__doc__,source=str(a.source),source_sha256=hashlib.sha256(a.source.read_bytes()).hexdigest(),model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),source_walking_command=a.source_walking_command,target_command=a.target_command,command_gain=float(scale[0,48]),command_obs_index=48,zero_command_observations_bitwise_identical=4096,command_transform_observations_bitwise_identical=4096,checkpoint_continuation=False,hardware_eligible=False,walking_accepted=False)
 (a.output/'manifest.json').write_text(json.dumps(info,indent=2));print(json.dumps(info))
if __name__=='__main__':main()
