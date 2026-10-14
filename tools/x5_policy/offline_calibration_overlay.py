"""Use a newly saved range snapshot in offline checks, without editing hardware.

Only endpoints may differ from the frozen baseline. A changed zero, direction,
IMU or joint identity needs a separate model/observation audit.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
from model_adaptation_contract import saved_joint_contract


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def range_snapshot(candidate, path):
    baseline_path=Path(candidate['source_snapshot']['path'])
    if hashlib.sha256(baseline_path.read_bytes()).hexdigest()!=candidate['source_snapshot']['sha256']:
        raise ValueError('Original snapshot changed')
    baseline=json.loads(baseline_path.read_text())['calibration']
    if digest(baseline)!=candidate['calibration']['sha256']:
        raise ValueError('Original calibration binding changed')
    path=Path(path)
    current=json.loads(path.read_text())
    rows=saved_joint_contract(current)
    before=copy.deepcopy(baseline);after=copy.deepcopy(current)
    changes=[]
    for old,new in zip(before['joints'],after['joints']):
        endpoints={key:[old[key],new[key]] for key in ('min','max') if old[key]!=new[key]}
        if endpoints:changes.append(dict(id=new['id'],endpoints=endpoints))
        old.pop('min');old.pop('max');new.pop('min');new.pop('max')
    if before!=after:
        raise ValueError('Range-only comparison rejects zero/direction/IMU/identity changes')
    out=copy.deepcopy(candidate)
    out['calibration']=dict(candidate['calibration'],sha256=digest(current),joints=rows)
    by_id={row['id']:row for row in rows}
    out['lower_rad']=[math.radians(by_id[sid]['min_deg']) for sid in out['policy_joint_ids']]
    out['upper_rad']=[math.radians(by_id[sid]['max_deg']) for sid in out['policy_joint_ids']]
    provenance=dict(kind='offline_saved_range_overlay',path=str(path.resolve()),
        raw_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),canonical_sha256=digest(current),
        original_calibration_sha256=candidate['calibration']['sha256'],changes=changes,
        zero_direction_imu_identical=True,hardware_write=False,
        interpretation='User-saved operating endpoints, not independently verified physical hard stops')
    return out,provenance
