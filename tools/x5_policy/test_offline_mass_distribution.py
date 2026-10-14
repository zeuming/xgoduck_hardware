"""Inertial correctness checks for the offline relocation experiment."""
import json
import tempfile
import unittest
from pathlib import Path

import mujoco
import numpy as np

from offline_mass_distribution import inertia_matrix, parallel_axis, relocate_point, scenario_spec, set_inertial


class MassDistributionTests(unittest.TestCase):
    def test_relocation_matches_direct_composite_inertia_about_any_origin(self):
        rng = np.random.default_rng(871)
        for _ in range(20):
            m, part = .26, .071
            com, dest, origin = rng.normal(size=(3, 3)) * .03
            tensor = np.diag([.0001, .0002, .00025])
            new_com, new_tensor = relocate_point(m, com, tensor, part, dest)
            np.testing.assert_allclose(m * new_com, (m-part)*com + part*dest, atol=1e-15)
            direct = tensor + parallel_axis(m-part, com-origin) + parallel_axis(part, dest-origin)
            np.testing.assert_allclose(new_tensor + parallel_axis(m, new_com-origin), direct, atol=1e-15)

    def test_zero_relocation_is_identity(self):
        com, tensor = np.array([.01, -.02, .03]), np.diag([.1, .2, .3])
        new_com, new_tensor = relocate_point(.26, com, tensor, 0., [.1, .2, .3])
        np.testing.assert_array_equal(new_com, com)
        np.testing.assert_array_equal(new_tensor, tensor)

    def test_dense_tensor_compilation_preserves_off_diagonal_precision(self):
        spec = mujoco.MjSpec.from_string('<mujoco><worldbody><body name="x"><freejoint/></body></worldbody></mujoco>')
        tensor = np.array([[1.1e-4, -2.6e-8, -1.6e-6], [-2.6e-8, 9.95e-5, -1e-7], [-1.6e-6, -1e-7, 5.78e-5]])
        set_inertial(spec.body('x'), .26, [.01, 0., -.02], tensor)
        compiled = spec.compile()
        np.testing.assert_allclose(inertia_matrix(compiled, 1), tensor, atol=1e-12, rtol=0)

    def test_compile_same_mass_different_location_and_no_source_mutation(self):
        def source():
            return mujoco.MjSpec.from_string('''<mujoco><worldbody>
              <body name="trunk_base"><freejoint/>
                <inertial mass=".3" pos="0 0 0" diaginertia=".001 .002 .002"/>
                <body name="yaw2roll" pos=".02 .01 0"><geom size=".01" mass=".1"/></body>
                <body name="bearing_roll" pos=".02 -.01 0"><geom size=".01" mass=".1"/></body>
              </body></worldbody></mujoco>''')
        original = source().compile()
        with tempfile.TemporaryDirectory() as temp:
            results = []
            for rear in (.03, .09):
                path = Path(temp) / f'{rear}.json'
                path.write_text(json.dumps(dict(total_mass_kg=.871, relocated_mass_kg=.071, rear_of_hip_m=rear)))
                fn, evidence = scenario_spec(source, path)
                model = fn().compile()
                results.append(model)
                self.assertTrue(evidence['physics_application']['noninertial_fields_identical'])
            np.testing.assert_array_equal(results[0].body_mass, results[1].body_mass)
            self.assertLess(results[1].body_ipos[1, 0], results[0].body_ipos[1, 0])
            self.assertGreater(inertia_matrix(results[1], 1)[1, 1], inertia_matrix(results[0], 1)[1, 1])
            np.testing.assert_array_equal(source().compile().body_mass, original.body_mass)

    def test_explicit_height_changes_full_inertia_without_changing_total_mass(self):
        def source():
            return mujoco.MjSpec.from_string('''<mujoco><worldbody>
              <body name="trunk_base"><freejoint/>
                <inertial mass=".3" pos="0 0 0" diaginertia=".001 .002 .002"/>
                <body name="yaw2roll" pos=".02 .01 0"><geom size=".01" mass=".1"/></body>
                <body name="bearing_roll" pos=".02 -.01 0"><geom size=".01" mass=".1"/></body>
              </body></worldbody></mujoco>''')
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'raised.json'
            path.write_text(json.dumps(dict(total_mass_kg=.871,relocated_mass_kg=.15,
                rear_of_hip_m=.09,height_above_original_trunk_com_m=.05)))
            fn,evidence=scenario_spec(source,path);model=fn().compile()
            np.testing.assert_allclose(model.body_mass.sum(),.871,atol=1e-12)
            np.testing.assert_allclose(model.body_ipos[1,2],.15/(.3*.871/.5)*.05,atol=1e-12)
            self.assertNotEqual(inertia_matrix(model,1)[0,2],0.)
            self.assertIn('0.05m',evidence['physics_application']['height_assumption'])


if __name__ == '__main__':
    unittest.main()
