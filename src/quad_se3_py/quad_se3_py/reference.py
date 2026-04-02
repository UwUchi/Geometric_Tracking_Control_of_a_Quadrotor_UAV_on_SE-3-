import numpy as np

from .utils import compute_Rd_and_derivatives, normalized_vec_and_derivative


def compute_desired_force_vector_and_derivative(
    m,
    g,
    e3,
    kx,
    kv,
    x,
    v,
    x_dd,
    xd,
    vd,
    xd_dd,
    xd_ddd=None,
):
    # 计算微分是为了计算 b3d_dot，进而计算 Rd_dot 和 Omega_d。
    if xd_ddd is None:
        xd_ddd = np.zeros(3)
    ex = x - xd
    ev = v - vd
    Fd = -kx * ex - kv * ev - m * g * e3 + m * xd_dd

    # Fd_dot = -kx ex_dot - kv ev_dot + m xd_dddot
    # ex_dot = ev
    # ev_dot = v_dot - xd_ddot
    Fd_dot = -kx * ev - kv * (x_dd - xd_dd) + m * xd_ddd
    return Fd, Fd_dot


def compute_desired_attitude_and_force_from_state(
    m,
    g,
    e3,
    kx,
    kv,
    x,
    v,
    x_dd,
    xd,
    vd,
    xd_dd,
    xd_ddd,
    b1d,
    b1d_dot,
    current_Rd,
    Omega_d_prev,
    dt=0.01,
):
    A, A_dot = compute_desired_force_vector_and_derivative(
        m=m,
        g=g,
        e3=e3,
        kx=kx,
        kv=kv,
        x=x,
        v=v,
        x_dd=x_dd,
        xd=xd,
        vd=vd,
        xd_dd=xd_dd,
        xd_ddd=xd_ddd,
    )
    if np.linalg.norm(A) < 1e-6:
        b3d = np.array([0.0, 0.0, 1.0], dtype=float)
        b3d_dot = np.zeros(3)
    else:
        b3d, b3d_dot, _ = normalized_vec_and_derivative(-A, -A_dot)

    Rd, Rd_dot, Omega_d, Omega_d_dot = compute_Rd_and_derivatives(
        b3d=b3d,
        b3d_dot=b3d_dot,
        b1d=b1d,
        b1d_dot=b1d_dot,
        current_Rd=current_Rd,
        Omega_d_prev=Omega_d_prev,
        dt=dt,
    )
    return Rd, Rd_dot, Omega_d, Omega_d_dot, A


def compute_desired_attitude_from_state(
    x,
    v,
    xd,
    vd,
    xd_dd,
    b1d,
    b1d_dot,
    current_Rd,
    m,
    g,
    e3,
    kx,
    kv,
    x_dd=None,
    xd_ddd=None,
):
    if x_dd is None:
        x_dd = np.zeros(3)
    if xd_ddd is None:
        xd_ddd = np.zeros(3)

    return compute_desired_attitude_and_force_from_state(
        m=m,
        g=g,
        e3=e3,
        kx=kx,
        kv=kv,
        x=x,
        v=v,
        x_dd=x_dd,
        xd=xd,
        vd=vd,
        xd_dd=xd_dd,
        xd_ddd=xd_ddd,
        b1d=b1d,
        b1d_dot=b1d_dot,
        current_Rd=current_Rd,
        Omega_d_prev=np.zeros(3),
        dt=0.01,
    )
