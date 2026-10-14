"""Exact batched target/EMA parity against the reviewed production decoder."""
import argparse,json,sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--production',type=Path,required=True)
    p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    from train_constrained_policy import install_offline_guards
    install_offline_guards()
    import numpy as np,torch,onnxruntime as ort
    from current_body_training import load_contract,RuntimeDecoder,POLICY_IDS
    c=load_contract(a.contract)
    joints=json.loads(Path(c['files']['calibration']['resolved_path']).read_text())['joints']
    sys.path.insert(0,str(a.production.resolve()/'policy'))
    import motion
    sessions={mode:ort.InferenceSession(c['files'][mode]['resolved_path'],providers=['CPUExecutionProvider']) for mode in c['models']}
    controller=motion.MotionController(sessions,a.calibration,walk_head_command_rad=c['head_command_rad'])
    assert controller._transport is None
    rng=np.random.default_rng(17);result={}
    for mode in c['models']:
        controller._mode=mode
        seed=np.zeros(14,np.float32)
        if mode=='walk':seed[5:9]=c['head_command_rad']
        decoder=RuntimeDecoder(c['models'][mode],joints,1,'cpu',seed[5:9].tolist() if mode=='walk' else None)
        smooth=seed.copy();previous=np.zeros(14,np.float32)
        projections=0
        for step in range(256):
            raw=rng.normal(0,.9,(1,14)).astype(np.float32)
            target,previous,smooth,_=controller._decode_policy_action(mode,raw,previous,smooth,step*.02,0.)
            projected=controller._execution_target_locked(mode,target)
            ticks=controller._raw_targets_locked(projected,mode=mode)
            decoder.step(torch.from_numpy(raw))
            expected=np.array([ticks[motion.MOTOR_IDS.index(s)] for s in POLICY_IDS])
            np.testing.assert_array_equal(decoder.ticks[0].numpy(),expected)
            np.testing.assert_array_equal(decoder.ema[0].numpy(),smooth)
            np.testing.assert_array_equal(previous,raw[0])
            projections+=int(decoder.projected.sum())
        decoder.reset();np.testing.assert_array_equal(decoder.ema[0].numpy(),seed)
        result[mode]=dict(frames=256,integer_goals_exact=True,ema_exact=True,
            raw_history_unchanged=True,projected_joint_goals=projections,reset_seed_exact=True)
    out=dict(passed=True,models=result,hardware_access=False,
             scope='Actual reviewed decoder output parity; NOT complete sensor/filter or physical-body parity')
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out))


if __name__=='__main__':main()
