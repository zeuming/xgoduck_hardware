"""Explicit synchronous-mode regression runner; NOT an owner concurrency test."""
import argparse
from pathlib import Path
import runpy
import sys
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source',type=Path,required=True)
p.add_argument('--validator',choices=('causal','current-pose','async'),required=True)
a,rest=p.parse_known_args()
here=Path(__file__).resolve().parent
sys.path[:0]=[str(a.source.resolve()/'policy'),str(a.source.resolve()/'service'),str(here)]
import motion
original=motion.MotionController.__init__
def synchronous(self,*args,**kwargs):
    if kwargs.get('serial_owner') is True:raise ValueError('This wrapper only tests legacy synchronous semantics')
    kwargs['serial_owner']=False
    original(self,*args,**kwargs)
motion.MotionController.__init__=synchronous
filename={'causal':'validate_causal_candidate.py','current-pose':'validate_current_pose_probe.py','async':'validate_async_policy.py'}[a.validator]
sys.argv=[str(here/filename),'--source',str(a.source),*rest]
runpy.run_path(str(here/filename),run_name='__main__')
