import unittest
import numpy as np
from cubic_reference import maximum_speed,evaluate,retime


class CubicReferenceTests(unittest.TestCase):
    def test_exact_extremum_and_cancellation(self):
        d=np.array([.3,-.4]);np.testing.assert_allclose(maximum_speed([(0.,.5,d)]),1.5*abs(d)/.5,atol=1e-14)
        p=[(0.,.5,d),(0.,.5,-d)];np.testing.assert_array_equal(maximum_speed(p),0.)
        np.testing.assert_array_equal(evaluate(p,.24),0.)

    def test_overlapping_four_ramps_respect_speed_bound(self):
        rng=np.random.default_rng(713);limit=600*2*np.pi/4095
        for _ in range(48):
            v=rng.normal(0,.15,(2,14));v[:,5:9]=0.
            start=rng.uniform(.05,.9);rise=rng.uniform(.1,.6);hold=rng.uniform(0,.1);back=rng.uniform(.15,.8)
            ret=start+rise*rng.uniform(0.,1.5)
            p=[(0.,start,v[0]),(ret,back,-v[0]),(start,rise,v[1]),(start+rise+hold,rise,-v[1])]
            q,m=retime(p,limit);end=max(s+d for s,d,_ in q)
            np.testing.assert_array_equal(evaluate(q,0.),0.)
            np.testing.assert_allclose(evaluate(q,end+.01),0.,atol=1e-14)
            previous=evaluate(q,0.)
            for t in np.arange(.002,end+.002,.002):
                current=evaluate(q,t)
                self.assertLessEqual(float(abs(current-previous).max()/.002),limit+1e-8)
                np.testing.assert_array_equal(current[5:9],0.)
                previous=current
            self.assertLessEqual(max(m['maximum_speed_after_rad_s']),limit+1e-9)


if __name__=='__main__':unittest.main()
