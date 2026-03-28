import numpy as np

from .utils import compute_Rd_and_derivatives, normalize


def compute_desired_force_vector(x, v, xd, vd, xdd, m, g, e3, kx, kv):
    ex = x - xd
    ev = v - vd
    return -kx * ex - kv * ev - m * g * e3 + m * xdd


def compute_desired_attitude_from_state(
    x,
    v,
    xd,
    vd,
    xdd,
    b1d,
    b1d_dot,
    current_Rd,
    m,
    g,
    e3,
    kx,
    kv,
):
    A = compute_desired_force_vector(
        x=x,
        v=v,
        xd=xd,
        vd=vd,
        xdd=xdd,
        m=m,
        g=g,
        e3=e3,
        kx=kx,
        kv=kv,
    )
    if np.linalg.norm(A) < 1e-6:
        b3d = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        b3d = -normalize(A)

    Rd, Rd_dot, Omega_d, Omega_dot_d = compute_Rd_and_derivatives(
        b3d=b3d,
        b1d=b1d,
        b1d_dot=b1d_dot,
        current_Rd=current_Rd,
    )
    return Rd, Rd_dot, Omega_d, Omega_dot_d, A
