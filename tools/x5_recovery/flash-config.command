#!/bin/zsh
set -eu
exec /Library/Developer/CommandLineTools/usr/bin/python3 - <<'PY_LAUNCH'
import os,pathlib
p=pathlib.Path((pathlib.Path('/Users/lo/Documents/project/xgoduck_hardware/output/x5_recovery/current-plan-path.txt')).read_text().strip()).parent/'flash-config.command'
os.execv('/bin/zsh',['/bin/zsh',str(p)])
PY_LAUNCH
