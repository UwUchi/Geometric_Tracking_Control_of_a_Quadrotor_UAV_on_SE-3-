import numpy as np

from quad_se3_py.reference import compute_desired_attitude_from_state
from quad_se3_py.utils import quat_to_rotmat


def test_desired_attitude_hover_matches_heading():
    Rd, _, omega_d, _, A = compute_desired_attitude_from_state(
        x=np.array([0.5, 0.3, 5.0]),
        v=np.zeros(3),
        xd=np.array([0.5, 0.3, 5.0]),
        vd=np.zeros(3),
        xd_dd=np.zeros(3),
        b1d=np.array([1.0, 0.0, 0.0]),
        b1d_dot=np.zeros(3),
        current_Rd=np.eye(3),
        m=1.0,
        g=9.81,
        e3=np.array([0.0, 0.0, 1.0]),
        kx=8.0,
        kv=5.0,
    )
    np.testing.assert_allclose(Rd, np.eye(3), atol=1e-6)
    np.testing.assert_allclose(omega_d, np.zeros(3), atol=1e-6)
    np.testing.assert_allclose(A, np.array([0.0, 0.0, -9.81]), atol=1e-6)


def test_quaternion_rotation_round_trip_identity():
    R = quat_to_rotmat(np.array([0.0, 0.0, 0.0, 1.0]))
    np.testing.assert_allclose(R, np.eye(3), atol=1e-9)
