import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from predict_policy_sensors import extrapolate


class PredictionContractTests(unittest.TestCase):
    def test_constant_velocity_matches_exact_rigid_rotation_and_each_read_age(self):
        rng=np.random.default_rng(40);obs=rng.normal(size=(1,61)).astype(np.float32)
        obs[0,3:6]=[.1,-.2,-.9746794];old=obs.copy()
        ages=np.linspace(.005,.04,14);dt=.025
        predicted=extrapolate(obs,ages,dt)
        rotation=Rotation.from_rotvec(-obs[0,:3].astype(float)*dt).as_matrix()
        np.testing.assert_allclose(predicted[0,3:6],rotation@obs[0,3:6],atol=1e-7)
        np.testing.assert_allclose(predicted[0,6:20],obs[0,6:20]+obs[0,20:34]*ages,atol=1e-7)
        np.testing.assert_array_equal(predicted[0,20:],obs[0,20:])
        np.testing.assert_array_equal(obs,old)

    def test_zero_age_is_bitwise_identity_and_negative_age_is_rejected(self):
        obs=np.random.default_rng(1).normal(size=(1,61)).astype(np.float32)
        np.testing.assert_array_equal(extrapolate(obs,0.,0.),obs)
        with self.assertRaises(ValueError):extrapolate(obs,-.001,0.)
        with self.assertRaises(ValueError):extrapolate(obs,0.,float('nan'))


if __name__=='__main__':unittest.main()
