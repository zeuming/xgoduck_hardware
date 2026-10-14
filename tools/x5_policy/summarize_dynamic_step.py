"""Summarize completed offline dynamic reference diagnostics without gait claims."""
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
BASE=ROOT/'output/duck_sim/overnight-20261011'


def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(p):
 r=read(p)
 if not r['completed'] or not r['source_hashes_unchanged']:raise ValueError(f'Incomplete/changed {p}')
 v=[x for g in r['generations'] for x in g['candidates']]
 return dict(path=str(p),sha256=sha(p),cases=len(v),qualified=sum(x['single_cycle_qualified'] for x in v),failed=sum(x['failed'] for x in v),warmup_failed=sum(x['warmup_failed'] for x in v),maximum_clearance_before_failure_m=max(x['maximum_swing_clearance_m'] for x in v),maximum_consecutive_air_frames=max(x['max_consecutive_air_frames'] for x in v),best=r['best'],initial_state_sha256=[g['initial_state_sha256'] for g in r['generations']],env0_effective_body_properties=[g['env0_effective_body_properties'] for g in r['generations']])
def parity(a,b):
 x=np.load(a);y=np.load(b);return {k:bool(np.array_equal(x[k],y[k])) for k in x.files}


def main():
 report=dict(scope=__doc__,hardware_access=False,cloud_machine_started=False,policy_weights_changed=False,physical_limits_changed=False,walking_qualified=False,simulation_single_cycle_qualified=False,sources={str(Path(__file__)):sha(__file__)})
 report['search']={n:run(BASE/f'dynamic-step-search-{n}-v1/search.json') for n in ('left','contact')}
 report['first_search_generation_exact_parity']=parity(BASE/'dynamic-step-search-left-v1/generation-00.npz',BASE/'dynamic-step-search-contact-v1/generation-00.npz')
 plan=read(BASE/'dynamic-step-phase-ablation-v1/plan.json');assert plan['completed']
 phase=[]
 for r in plan['runs']:
  value=run(Path(r['output'])/'search.json');value['delay_seconds']=r['delay_seconds'];phase.append(value)
 report['phase_only_ablation']=phase
 report['phase_all32_initial_root_joint_states_match']=len({r['initial_state_sha256'][0] for r in phase})==1
 report['phase_env0_mass_com_match']=all(x['env0_effective_body_properties']==phase[0]['env0_effective_body_properties'] for x in phase)
 report['preload_ablation']={str(s):run(BASE/f'dynamic-step-preload-ablation-v1/scale-{s}/search.json') for s in (0,1)}
 report['preload_control_exact_trace_parity_with_phase_control']=parity(BASE/'dynamic-step-preload-ablation-v1/scale-0/generation-00.npz',BASE/'dynamic-step-phase-ablation-v1/phase-00/generation-00.npz')
 report['preload_initial_state_match']=report['preload_ablation']['0']['initial_state_sha256']==report['preload_ablation']['1']['initial_state_sha256']
 following=read(BASE/'dynamic-step-following-v1/diagnosis.json');report['counterfactual_kinematics']=dict(path=str(BASE/'dynamic-step-following-v1/diagnosis.json'),sha256=sha(BASE/'dynamic-step-following-v1/diagnosis.json'),summary=following['summary'],reconstruction_error_m=following['kinematic_reconstruction_max_error_m'])
 report['interpretation']=[
  'No tested reference qualifies lift-land-recover. No alternation, net-forward or hardware validation.',
  'Contact objective produced one instantaneous zero swing load with only about 0.0005mm clearance, never a 3mm lift. Frozen reference did not reproduce that unloading across the fresh 32-world set.',
  'Two search arms have exact initial-generation trace parity; later optimization trajectories and reset states vary. Search scores are not a matched causal or statistical success claim.',
  'All phase tests start from identical 32-world qpos/qvel hashes; preload control reproduces original phase control arrays bitwise.',
  'Same-root target-angle kinematics are counterfactual: loaded servos need torque-producing angle offsets. Their below-ground target geometry is not a contact penetration or attainable motion proof.',
  'Fading the swing-leg standing bias removes three range failures in this 32-world set but still yields no lift. Not a demonstrated cause/fix of walking.',
  'All current references return the transfer after swing fall ends. Unloading observed during transfer return therefore motivates a new overlapped return-and-lift reference, not more iterations of the same family.',
  'Original 15deg, CAD+saved limits, EMA, 600ticks/s and BAM retained. Search explicitly zeroes interval push amplitudes while retaining timing/draws; this is not original task acceptance.',
  'Nominal model 800g / real robot 871g, actuator identification uncertainty remains. Finite failures cannot prove physical impossibility.'
 ]
 p=BASE/'dynamic-step-summary.json';p.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
 print(json.dumps(dict(search={k:{x:v[x] for x in ('cases','qualified','failed','maximum_clearance_before_failure_m','maximum_consecutive_air_frames')} for k,v in report['search'].items()},phase_qualified=sum(x['qualified'] for x in phase),preload_qualified=sum(x['qualified'] for x in report['preload_ablation'].values()),all_phase_states_match=report['phase_all32_initial_root_joint_states_match'])))

if __name__=='__main__':main()
