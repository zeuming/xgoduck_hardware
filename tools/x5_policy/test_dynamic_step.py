import unittest
import numpy as np
from search_dynamic_step import build_trajectory,LOW,HIGH
from diagnose_dynamic_step_phase import build_trajectory as phase_trajectory
from diagnose_dynamic_step_preload import build_trajectory as preload_trajectory


class TrajectoryTests(unittest.TestCase):
    def test_overlapping_phase_and_preload_remain_within_original_rate(self):
        rng=np.random.default_rng(853)
        p=rng.uniform(LOW,HIGH,(32,8))
        vectors=rng.normal(0,.1,(3,14));vectors[:,5:9]=0.
        extra=rng.normal(0,.07,(32,14));extra[:,5:]=0.
        limit=600*2*np.pi/4095
        for delay in (-.5,0.,1.):
            curve,end,_=preload_trajectory(p,*vectors,limit,delay,extra)
            np.testing.assert_array_equal(curve(0)[0],0.)
            np.testing.assert_allclose(curve(end.max()+.01)[0],0.,atol=1e-14)
            previous=curve(0)[0]
            for t in np.arange(.001,end.max()+.001,.001):
                current=curve(t)[0]
                self.assertLessEqual(float(np.abs(current-previous).max()/.001),limit+1e-6)
                np.testing.assert_array_equal(current[:,5:9],0.)
                previous=current

    def test_zero_diagnostic_change_preserves_original_reference(self):
        rng=np.random.default_rng(82)
        p=rng.uniform(LOW,HIGH,(8,8));vectors=rng.normal(0,.08,(3,14))
        limit=600*2*np.pi/4095
        original,end,_=build_trajectory(p,*vectors,limit)
        phase,pe,_=phase_trajectory(p,*vectors,limit,0.)
        preload,be,_=preload_trajectory(p,*vectors,limit,0.,np.zeros((8,14)))
        np.testing.assert_allclose(end,pe,atol=1e-14)
        np.testing.assert_allclose(end,be,atol=1e-14)
        for t in np.arange(0,end.max()+.1,.02):
            np.testing.assert_allclose(original(t)[0],phase(t)[0],rtol=0,atol=1e-14)
            np.testing.assert_allclose(original(t)[0],preload(t)[0],rtol=0,atol=1e-14)

    def test_retimed_curves_preserve_rate_continuity_and_head_hold(self):
        rng=np.random.default_rng(821)
        p=rng.uniform(LOW,HIGH,(32,8))
        transfer=rng.normal(0,.1,14);lift=rng.normal(0,.09,14);stance=rng.normal(0,.09,14)
        for vector in (transfer,lift,stance):vector[5:9]=0
        limit=600*2*np.pi/4095
        curve,end,_=build_trajectory(p,transfer,lift,stance,limit)
        np.testing.assert_array_equal(curve(0)[0],0.)
        np.testing.assert_allclose(curve(end.max()+.01)[0],0.,atol=1e-14)
        dt=.001;previous=curve(0)[0]
        for t in np.arange(dt,end.max()+dt,dt):
            current=curve(t)[0]
            self.assertLessEqual(float(np.abs(current-previous).max()/dt),limit+1e-6)
            np.testing.assert_array_equal(current[:,5:9],0.)
            previous=current


if __name__=='__main__':unittest.main()
