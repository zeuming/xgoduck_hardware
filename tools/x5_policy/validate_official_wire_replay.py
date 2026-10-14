"""Replay accepted simulator actions through the actual candidate decoder/wire validator."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();r=a.root
sys.path.insert(0,str(r/'candidate/host/policy'))
import motion
from rl_core import MOTOR_IDS
cdata=json.loads(Path('output/duck_sim/repair-20261011/model/training-adapter/generated-v2/candidate.json').read_text())['candidate']
c=object.__new__(motion.MotionController);c._config=json.loads((r/'saved-calibration.json').read_text())
c._policy_profiles={'walk':dict(home_rad=np.array(cdata['home_rad'],np.float32),scale=cdata['action_scale'])}
count=0;projections={};differences=[];cases=[]
for seed in range(4):
 root=r/f'head-range-projection-seed{seed}';summary=json.loads((root/'summary.json').read_text());assert summary['official_runtime_simulation_accepted']
 before=json.loads((r/f'simulation-v3-seed{seed}/summary.json').read_text())
 assert before['initial_fingerprint']==summary['initial_fingerprint']
 rows=[json.loads(x) for x in (root/'trace.jsonl').read_text().splitlines()]
 previous=np.zeros(14,np.float32);smooth=previous.copy()
 for row in rows:
  raw=np.array([row['action']],np.float32)
  target,previous,smooth,_=c._decode_policy_action('walk',raw,previous,smooth,0,0)
  out=c._execution_target_locked('walk',target);wire=c._raw_targets_locked(out,mode='walk')
  expected=row['wire_validation']['ticks']
  if wire!=expected:differences.append(dict(seed=seed,frame=row['frame'],expected=expected,actual=wire))
  for j in c._head_projection:projections[j['id']]=projections.get(j['id'],0)+1
  for i,sid in enumerate(MOTOR_IDS):
   if sid<30:assert out[i]==target[i]
  assert np.array_equal(previous,raw[0]);count+=1
 cases.append(dict(seed=seed,gate=summary['gate'],same_initial_state=True,trace_sha256=hashlib.sha256((root/'trace.jsonl').read_bytes()).hexdigest()))
result=dict(passed=not differences,frames=count,all_leg_targets_unchanged=True,raw_previous_action_unchanged=True,projected_frames_by_id=projections,wire_differences=differences[:20],cases=cases,hardware_verified=False,model_sha256=cdata['source_model']['sha256'])
(r/'wire-replay-result.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='cases'});raise SystemExit(bool(differences))
