"""Offline commanded-speed transition diagnostic; original plant remains unchanged.

The brief higher command is part of this declared diagnostic, not a production
workaround. A successful bridge would not establish low-speed standing starts.
"""
import argparse,hashlib,json,sys
from pathlib import Path

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--candidate',required=True);p.add_argument('--model',required=True);p.add_argument('--output',required=True)
 p.add_argument('--measurement-source',required=True);p.add_argument('--seed',type=int,default=0)
 p.add_argument('--ankle-deg',type=float,required=True);p.add_argument('--slew',type=float,required=True)
 p.add_argument('--start-speed',type=float,required=True);p.add_argument('--cruise-speed',type=float,required=True)
 p.add_argument('--switch-s',type=float,default=1.5);p.add_argument('--transition-s',type=float,default=.5)
 a=p.parse_args()
 if not(0<a.cruise_speed<=a.start_speed<=.3 and .5<=a.switch_s<=3 and .1<=a.transition_s<=1):raise ValueError('Invalid bounded command bridge')
 import source_adaptation_experiment as ex
 original=ex.schedule
 def schedule(steps,dt,spec):
  base=original(steps,dt,spec)
  if a.start_speed==a.cruise_speed:return base
  t=steps.to(__import__('torch').float64)*dt-spec['warmup_s']
  ratio=(1-(1-a.cruise_speed/a.start_speed)*((t-a.switch_s)/a.transition_s).clamp(0,1)).to(base.dtype)
  return base*ratio
 ex.schedule=schedule
 import evaluate_source_adaptation as evaluate
 sys.argv=['evaluate_source_adaptation.py','--candidate',a.candidate,'--model',a.model,'--output',a.output,'--measurement-source',a.measurement_source,'--seed',str(a.seed),'--ankle-deg',str(a.ankle_deg),'--slew',str(a.slew),'--speed',str(a.start_speed)]
 evaluate.main()
 report=Path(a.output)/'summary.json';r=json.loads(report.read_text())
 r['command_bridge']=dict(start_speed_m_s=a.start_speed,cruise_speed_m_s=a.cruise_speed,switch_after_warmup_s=a.switch_s,transition_s=a.transition_s,production_controller=False,low_speed_standing_start_validated=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
 report.write_text(json.dumps(r,indent=2))
if __name__=='__main__':main()
