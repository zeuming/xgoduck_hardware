"""Offline zero → three-second forward pulse → zero screens on current hardware assumptions.

No controller, transport, flashing or deployment modules are imported. Every
initial joint and head reference comes from the selected snapshot; mass,
ground contact, zero initial velocity and delays remain simulation assumptions.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

import audit_policy_sim as sim
from audit_runtime_contract import assess_fixed_head_runtime_contract
from rl_core import POLICY_IDS


class ScheduledActor:
    def __init__(self, session, home, calibration, mapper, speed, gait_decoder):
        self.session, self.home, self.calibration = session, home, calibration
        self.speed, self.gait_decoder = speed, gait_decoder
        self.previous_raw = np.zeros(14, np.float32)
        thresholds = {11:16.,21:16.}
        if gait_decoder:
            thresholds.update({12:25.,22:25.,13:40.,23:40.,14:28.,24:28.})
        self.maps = {}
        for sid, threshold in thresholds.items():
            row = calibration[sid]
            lower,upper=sorted((tick-row['zero'])*360/4095*row['sign']
                for tick in (row['min']+3,row['max']-3))
            mapping=mapper(lower,upper,threshold)
            assert mapping(math.degrees(home[POLICY_IDS.index(sid)]))==math.degrees(home[POLICY_IDS.index(sid)])
            self.maps[sid]=mapping
        self.schedule, self.original_actions, self.original_previous = [], [], []

    def get_inputs(self):
        return self.session.get_inputs()

    def run(self, names, inputs):
        name=self.get_inputs()[0].name
        observation=np.asarray(inputs[name],np.float32).copy()
        at=math.ceil((len(self.schedule)/35.-1e-9)/.005)*.005
        command=[self.speed if 5.<=at<8. else 0.,0.,0.]
        observation[:,48:51]=command
        observation[:,34:48]=self.previous_raw
        outputs=self.session.run(names,{**inputs,name:observation})
        raw=np.asarray(outputs[0],np.float32)
        if raw.shape!=(1,14) or not np.isfinite(raw).all():
            raise ValueError('Invalid actor output')
        self.original_previous.append(self.previous_raw.tolist())
        self.previous_raw=raw[0].copy()
        self.original_actions.append(self.previous_raw.tolist())
        self.schedule.append(command)
        decoded=raw.copy()
        for sid,mapping in self.maps.items():
            index=POLICY_IDS.index(sid)
            scale=.95 if sid in (11,21) else 1.
            angle=math.degrees(self.home[index]+scale*float(raw[0,index]))
            decoded[0,index]=math.radians(mapping(angle))-self.home[index]
        return [decoded,*outputs[1:]]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--xml',type=Path,required=True)
    parser.add_argument('--decoder',type=Path,required=True)
    parser.add_argument('--root-height',type=float,required=True)
    parser.add_argument('--pitch',type=float,required=True)
    parser.add_argument('--roll',type=float,required=True)
    args=parser.parse_args()
    args.output.mkdir(mode=0o700,parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('offline_frozen_gait_math',args.decoder)
    math_module=importlib.util.module_from_spec(spec);spec.loader.exec_module(math_module)
    snapshot=json.loads(args.snapshot.read_text())
    cfg,hardware,source,angles=sim.validate_snapshot(snapshot)
    deps=sim.load_dependencies(sim.REFERENCE/'vendor/bam')
    options=deps[1].SessionOptions();options.intra_op_num_threads=options.inter_op_num_threads=1
    session=deps[1].InferenceSession(str(args.model),sess_options=options,providers=['CPUExecutionProvider'])
    original_profile=sim.policy_profile(session)
    profile=dict(home_rad=original_profile['home_rad'].copy(),scale=original_profile['scale'])
    assert profile['scale']==1.
    for sid,offset in {12:-1.,14:-1.,22:1.,24:1.}.items():
        profile['home_rad'][POLICY_IDS.index(sid)]+=math.radians(offset)
    report=dict(kind='current_600_actual_head_three_second_gait_screen',hardware_access=False,
        deployed=False,firmware_changed=False,calibration_changed=False,
        physical_standing_verified=False,physical_walking_verified=False,
        snapshot_source=source,
        source_sha256={str(path.resolve()):sim.digest(path) for path in
            (args.snapshot,args.model,args.xml,args.decoder,Path(__file__),Path(sim.__file__))},
        assumptions=dict(mass_kg=.873,kp=6.,feedback_delay_ms=20.,execution_delay_ms=20.,
            control_hz=35.,leg_slew_ticks_s=600.,head_slew_ticks_s=600.,initial_velocity=0.,
            root_height_m=args.root_height,initial_pitch_deg=args.pitch,initial_roll_deg=args.roll,
            captured_head=hardware['hold_head_deg'],head_target_source='captured saved reference',
            initial_head='at saved reference; initial takeover transient not modeled',
            contact_and_motor_dynamics='CAD floor and BAM model; not measured physical fidelity'),cases=[])
    cases=[(False,0.),(False,.05),(False,.18)]+[(True,v)for v in (.05,.1,.15,.18,.2)]
    for gait,speed in cases:
        name=('bounded-leg' if gait else 'current-roll')+'-speed'+str(speed)
        settings=sim.parser().parse_args(['--snapshot',str(args.snapshot),'--duration-s','30',
            '--kp','6','--hz','35','--feedback-delay-ms','20','--execution-delay-ms','20',
            '--slew-ticks-per-second','600','--initial-pose','snapshot','--head-source','captured',
            '--observation-filter','host','--mass-kg','.873',
            '--root-height-m',str(args.root_height),'--initial-pitch-deg',str(args.pitch),
            '--initial-roll-deg',str(args.roll),'--include-frames'])
        settings.xml=args.xml
        settings.model=args.model;settings.bam_params=args.xml.parent/'params/1910_m6.json'
        actor=ScheduledActor(session,profile['home_rad'],{j['id']:j for j in cfg['joints']},
            math_module.ExponentialRollMap,speed,gait)
        result=sim.rollout(settings,deps,actor,profile,cfg,hardware,angles,speed,'fixed-head')
        for index,frame in enumerate(result['frames']):
            frame.update(command=actor.schedule[index],original_neural_action=actor.original_actions[index],
                original_neural_previous_action=actor.original_previous[index])
        result['runtime_contract']=assess_fixed_head_runtime_contract(result)
        settled=[f for f in result['actual_motion_frames']if 20.<=f['time_s']<=30.]
        contacts={side:sum(f['feet'][side]['contact']for f in settled)/len(settled)if settled else 0.
            for side in ('left','right')}
        drift=float(np.linalg.norm(np.asarray(settled[-1]['base_position_m'])-settled[0]['base_position_m']))if settled else None
        result['final_zero_window']=dict(contact_fraction=contacts,drift_m=drift,
            passed=bool(settled)and drift<=.03 and min(contacts.values())>=.9)
        result['name']=name
        (args.output/(name+'.json')).write_text(json.dumps(result,separators=(',',':'),allow_nan=False)+'\n')
        compact={k:v for k,v in result.items()if k not in ('frames','actual_motion_frames')}
        report['cases'].append(compact)
        (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print(json.dumps(dict(case=name,ended=result['ended'],tilt=result['max_tilt_deg'],
            progress=result['displacement_m'],failures=result['runtime_contract']['failures'],
            settled=result['final_zero_window'])),flush=True)


if __name__=='__main__':main()
