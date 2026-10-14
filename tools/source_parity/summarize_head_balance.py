"""Regrade saved head-command comparisons without running physics/hardware."""
import hashlib,json,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/x5_policy'))
from evaluate_slow_gait import grade
r=ROOT/'output/duck_sim/source-parity-20261012'
p=r/'backward-causal-observation';out=r/'official-head-balance'
sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
patterns=[
 ('fixed_hold','robust-head-original-entry-*-hold-seed*-v10'),
 ('official_head_command_50Hz','official-head-command-original-entry-*-forward-seed*-v11'),
 ('official_head_command_zero','official-head-command-original-entry-*-zero-seed*-v12'),
 ('official_head_command_40Hz','official-head-command-original-entry-*-forward-seed*-hz40-v13'),
 ('direct_start_50Hz','official-head-command-direct-start-*-forward-seed*-v14'),
 ('recorded_entry_baseline','official-head-command-recorded-entry-*-baseline-v15'),
 ('recorded_entry_profile','official-head-command-recorded-entry-*-profile-v16'),
 ('static_support_objective_counterfactual','support-margin-head-*-forward-seed*-hz40-v17')]
rows=[]
for group,pattern in patterns:
 for directory in sorted(p.glob(pattern)):
  if not directory.is_dir():continue
  f=directory/'summary.json'
  if not f.exists():raise ValueError('Missing completed result: '+str(directory))
  d=json.loads(f.read_text());trace=directory/'trace.jsonl'
  samples=[json.loads(x) for x in trace.read_text().splitlines()]
  g=grade(samples,'zero' if d['speed_m_s']==0 else 'forward',sample_period=d['control_dt_s'])
  for key,value in g.items():
   if key=='scope':continue
   assert value==d['gate'][key],(directory.name,key,value,d['gate'][key])
  row=dict(group=group,name=directory.name,summary_sha256=sha(f),trace_sha256=sha(trace),
       actor_sha256=d['model_sha256'],calibration_sha256=d['calibration_overlay']['canonical_sha256'],
       initial_qpos_sha256=d['initial_qpos_sha256'],initial_qvel_sha256=d['initial_qvel_sha256'],
       head_command_rad=d.get('official_head_command_rad'),initial_hold_seconds=d.get('initial_hold_seconds'),
       control_hz=d['control_hz'],initial_compensated_pose=d.get('initial_compensated_pose'),
       gate=g,all_transmitted_ranges=d['all_transmitted_ranges'],all_actual_ranges=d['all_actual_ranges'])
  rows.append(row)
assert len(rows)==24,len(rows)
assert len({x['actor_sha256'] for x in rows})==1
assert len({x['calibration_sha256'] for x in rows})==1
a=next(x for x in rows if x['group']=='recorded_entry_baseline');b=next(x for x in rows if x['group']=='recorded_entry_profile')
assert a['initial_qpos_sha256']==b['initial_qpos_sha256'] and a['initial_qvel_sha256']==b['initial_qvel_sha256']
groups={group:dict(cases=sum(x['group']==group for x in rows),
    accepted=sum(x['group']==group and x['gate']['accepted'] and x['all_transmitted_ranges'] and x['all_actual_ranges'] for x in rows)) for group,_ in patterns}
report=dict(hardware_access=False,installed=False,hardware_eligible=False,real_walking_passed=False,
    profile='proposed-profile.json',candidate='candidate',groups=groups,cases=rows,
    input_contract='Head commands only at51:55; actual gyro/gravity/position/velocity unchanged, original raw-action history and action EMA/decode. HOLD head goals explicit; legs unchanged.',
    source_commit='8cdbbd84710d856581982c9eaf0d5e2970666232',
    load_scope='Two unmeasured same-total871g inertial hypotheses. Rear220g relocation is NOT measured board mass or 871-800. Not identified real COM/inertia bounds.',
    essential_failures=[
      '40Hz rear/high seed0 does lift/land alternately but net displacement is backward and final supported recovery fails; not walking acceptance.',
      'Direct-start reference seed3 walks forward but one repeated-right qualifying lift violates strict alternation; retained as failure.',
      'Recorded physical-entry rear/high pair: profile reduces backward drift44.8mm to31.4mm, still exceeds15mm zero-drift evaluation. Neither reproduces actual early fall.',
      'Maximizing static worst support margin improves rear/high40Hz but makes reference40Hz cross15degrees; static support alone is insufficient for choosing a real profile.'],
    operational_boundary='Optional head-command integration tested locally; no fixed compensation automatically selected for hardware. Two-file execution timing patch is separate and pending permission to restart. Do not alter operator connection, calibration, ranges or firmware.',
    tests=json.loads((out/'head-command-tests-v4.json').read_text()),
    current_calibration_stance_check=json.loads((out/'current-calibration-stance-check.json').read_text()))
target=out/'simulation-and-code-summary.json'
with target.open('x') as f:f.write(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(groups=groups,cases=len(rows),independent_regrade=True,installed=False,hardware_eligible=False),indent=2))
