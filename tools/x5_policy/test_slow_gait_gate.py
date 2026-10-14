"""Independent acceptance must reject reward-like proxies and failed recovery."""
import copy
import math
import unittest
from evaluate_slow_gait import grade


def row(t,side=None):
    feet={s:dict(mesh_min_height_m=0.,vertical_load_n=4.) for s in ('left','right')}
    if side:
        feet[side]=dict(mesh_min_height_m=.004,vertical_load_n=0.)
        feet[('right' if side=='left' else 'left')]['vertical_load_n']=8.
    return dict(time_s=t,command_m_s=.03 if t<7 else 0.,feet=feet,
        tilt_deg=3.,failed_terms=[],root_position=[0. if t<=5 else .03,0.,.15],task_complete=False)


class GateTests(unittest.TestCase):
    def passing_trace(self):
        rows=[row(i*.02) for i in range(50)]
        for n,s in enumerate(['left','right','left','right']):
            rows += [row(5.02+n*.2+i*.02,s) for i in range(4)]
            rows += [row(5.10+n*.2+i*.02) for i in range(4)]
        rows += [row(7.+i*.02) for i in range(50)];rows[-1]['task_complete']=True
        return rows
    def test_supported_alternation_landing_and_recovery_required(self):
        rows=self.passing_trace();self.assertTrue(grade(rows,'forward')['accepted'])
        standing=copy.deepcopy(rows)
        for r in standing:
            for f in r['feet'].values():f.update(mesh_min_height_m=0.,vertical_load_n=4.)
        self.assertFalse(grade(standing,'forward')['accepted'])
        fallen=copy.deepcopy(rows);fallen[65]['tilt_deg']=16.
        self.assertFalse(grade(fallen,'forward')['accepted'])
        airborne=copy.deepcopy(rows)
        for r in airborne[-50:]:
            for f in r['feet'].values():f.update(mesh_min_height_m=.004,vertical_load_n=0.)
        self.assertFalse(grade(airborne,'forward')['accepted'])
        unloaded=copy.deepcopy(rows)
        for r in unloaded:
            for f in r['feet'].values():f['mesh_min_height_m']=0.
        self.assertFalse(grade(unloaded,'forward')['accepted'])

    def test_forward_distance_follows_initial_robot_heading(self):
        rows=self.passing_trace()
        for r in rows:
            r['heading_rad']=math.pi/2
            r['root_position'][1]=r['root_position'][0];r['root_position'][0]=0.
        result=grade(rows,'forward')
        self.assertTrue(result['accepted']);self.assertAlmostEqual(result['net_forward_m'],.03)
        for r in rows:r['heading_rad']=-math.pi/2
        self.assertFalse(grade(rows,'forward')['accepted'])

    def test_three_20ms_samples_do_not_span_60ms(self):
        rows=self.passing_trace()
        for side in ('left','right'):
            seen=0
            for r in rows:
                if r['feet'][side]['mesh_min_height_m']>=.003:
                    seen+=1
                    if seen%4==0:r['feet'][side].update(mesh_min_height_m=0.,vertical_load_n=4.)
        self.assertFalse(grade(rows,'forward')['accepted'])

    def test_40hz_keeps_60ms_duration_and_requires_contiguous_samples(self):
        rows=[row(i*.025) for i in range(40)]
        for n,s in enumerate(['left','right','left','right']):
            rows += [row(5.025+n*.25+i*.025,s) for i in range(4)]
            rows += [row(5.125+n*.25+i*.025) for i in range(4)]
        rows += [row(7.+i*.025) for i in range(40)];rows[-1]['task_complete']=True
        self.assertTrue(grade(rows,'forward',sample_period=.025)['accepted'])
        short=copy.deepcopy(rows)
        for s in ('left','right'):
            seen=0
            for r in short:
                if r['feet'][s]['mesh_min_height_m']>=.003:
                    seen+=1
                    if seen%4==0:r['feet'][s].update(mesh_min_height_m=0.,vertical_load_n=4.)
        self.assertFalse(grade(short,'forward',sample_period=.025)['accepted'])
        gaps=[r for r in rows if not any(abs(r['time_s']-(5.05+n*.25))<1e-6 for n in range(4))]
        self.assertFalse(grade(gaps,'forward',sample_period=.025)['accepted'])

    def test_invalid_observation_period_rejected(self):
        for dt in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):grade([], 'forward',sample_period=dt)


if __name__=='__main__':unittest.main()
