"""Passive full initialized-plant fingerprint for matched baseline/candidate trials."""
def initial_fingerprint(env):
 """Passive physical and random-state identity; expected gain changes stay visible."""
 import torch,warp as wp,dataclasses
 import numpy as np,hashlib
 def digest(value):
  if isinstance(value,torch.Tensor):value=value.detach().cpu().numpy()
  elif isinstance(value,wp.array):value=value.numpy()
  arr=np.asarray(value)
  return dict(shape=list(arr.shape),dtype=str(arr.dtype),sha256=hashlib.sha256(arr.tobytes()).hexdigest())
 result={}
 # Hash every exposed tensor, rather than checking nominal mass alone.
 for prefix,obj in [('model',env.sim.model),('data',env.sim.data),
                    ('motor',env.scene['robot'].actuators[0])]:
  for key in sorted(dir(obj)):
   if key.startswith('__'):continue
   try:value=getattr(obj,key)
   except (AttributeError,RuntimeError):continue
   if isinstance(value,torch.Tensor):result[prefix+'.'+key]=digest(value)
 result['torch_rng']=digest(torch.get_rng_state())
 for field in dataclasses.fields(env.sim.wp_model):
  value=getattr(env.sim.wp_model,field.name)
  if isinstance(value,wp.array):result['warp_model.'+field.name]=digest(value)
 # Delay/BAM histories are distinct from the joint-state and physical model.
 motor=env.scene['robot'].actuators[0]
 for prefix,obj in [('delay',motor._delay_buffer),('bam',motor._bam_model),
                    ('bam_actuator',motor._bam_model.actuator),('robot_data',env.scene['robot'].data)]:
  for key,value in vars(obj).items():
   if isinstance(value,(torch.Tensor,wp.array,np.ndarray)):result[prefix+'.'+key]=digest(value)
 return result
