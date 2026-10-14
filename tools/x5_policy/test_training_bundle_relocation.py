import copy
import math
import unittest

from relocate_training_bundle import reconcile_platform_geometry


class PlatformGeometryTests(unittest.TestCase):
    def setUp(self):
        self.pinned=dict(root_z_m=.13878823121025205,
            root_zero_sole_min_z_m=dict(left=-.13778793972541503,right=-.13778823121025205),
            sole_clearance_m=.001,relative_reset_z_range_m=[0.,0.],
            xml_path='/bundle/robot.xml',xml_sha256='same-xml',
            foot_mesh_vertex_sha256=dict(left='same-left',right='same-right'),orientation='identity')

    def test_observed_linux_roundoff_rebinds_without_mutating_original(self):
        actual=copy.deepcopy(self.pinned)
        actual['root_z_m']=.13878823121025202
        actual['root_zero_sole_min_z_m']=dict(left=-.13778793972541506,right=-.13778823121025202)
        before=copy.deepcopy(self.pinned)
        result,deltas=reconcile_platform_geometry(self.pinned,actual)
        self.assertEqual(result,actual)
        self.assertEqual(self.pinned,before)
        self.assertAlmostEqual(deltas['root_z_m'],-2.7755575615628914e-17,places=30)

    def test_meaningful_geometry_changes_are_rejected(self):
        for change in ('height','sole','clearance','range','xml','mesh','orientation','fields'):
            with self.subTest(change=change):
                actual=copy.deepcopy(self.pinned)
                if change=='height':actual['root_z_m']+=1e-12
                elif change=='sole':actual['root_zero_sole_min_z_m']['left']+=1e-12
                elif change=='clearance':actual['sole_clearance_m']=.002
                elif change=='range':actual['relative_reset_z_range_m']=[0.,.001]
                elif change=='xml':actual['xml_sha256']='changed'
                elif change=='mesh':actual['foot_mesh_vertex_sha256']['left']='changed'
                elif change=='orientation':actual['orientation']='changed'
                else:actual['extra']=True
                with self.assertRaises(ValueError):reconcile_platform_geometry(self.pinned,actual)

    def test_nonfinite_and_missing_sides_rejected(self):
        for value in (math.nan,math.inf,-math.inf):
            actual=copy.deepcopy(self.pinned);actual['root_z_m']=value
            with self.assertRaises(ValueError):reconcile_platform_geometry(self.pinned,actual)
        actual=copy.deepcopy(self.pinned);del actual['root_zero_sole_min_z_m']['right']
        with self.assertRaises(ValueError):reconcile_platform_geometry(self.pinned,actual)

    def test_more_than_roundoff_rejected_even_below_absolute_ceiling(self):
        actual=copy.deepcopy(self.pinned)
        actual['root_z_m']+=33*math.ulp(actual['root_z_m'])
        with self.assertRaises(ValueError):reconcile_platform_geometry(self.pinned,actual)


if __name__=='__main__':unittest.main()
