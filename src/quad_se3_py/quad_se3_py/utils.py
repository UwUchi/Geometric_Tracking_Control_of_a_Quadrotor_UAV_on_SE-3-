import numpy as np

def hat(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def vee(M):
    return np.array([M[2,1], M[0,2], M[1,0]])

def normalize(v, eps=1e-8):
    n = np.linalg.norm(v)
    if n < eps:
        return v.copy()
    return v / n


def project_to_so3(R):
    U, _, Vt = np.linalg.svd(R) #奇异值分解
    R_proj = U @ Vt
    if np.linalg.det(R_proj) < 0:
        U[:, -1] *= -1.0
        R_proj = U @ Vt
    return R_proj

def rotmat_to_quat(R):
    q = np.empty(4, dtype=float)  # x, y, z, w
    tr = np.trace(R)

    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q[3] = 0.25 * s
        q[0] = (R[2, 1] - R[1, 2]) / s
        q[1] = (R[0, 2] - R[2, 0]) / s
        q[2] = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax(np.diag(R))
        if i == 0:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

    q /= np.linalg.norm(q)
    return q

def quat_to_rotmat(q):
    x, y, z, w = q
    n = np.linalg.norm(q)
    if n < 1e-8:
        return np.eye(3)
    x, y, z, w = q / n

    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])
    return R

def compute_Rd_and_derivatives(b3d, b1d, b3d_dot=None, b1d_dot=None):
    b3d = normalize(b3d)
    b1c = b1d
    b2d = normalize(np.cross(b3d, b1c))
    b1d_orth = np.cross(b2d, b3d)
    Rd = np.column_stack((b1d_orth, b2d, b3d))

    # 先给一个可运行版本：若未提供导数，则近似为 0
    if b3d_dot is None:
        Rd_dot = np.zeros((3, 3))
        Omega_d = np.zeros(3)
        Omega_dot_d = np.zeros(3)
        return Rd, Rd_dot, Omega_d, Omega_dot_d
    
    # 这里给简化的一阶构造，够跑完整框架
    c = np.cross(b3d, b1c)
    c_norm = np.linalg.norm(c)
    if c_norm < 1e-8:
        Rd_dot = np.zeros((3, 3))
        Omega_d = np.zeros(3)
        Omega_dot_d = np.zeros(3)
        return Rd, Rd_dot, Omega_d, Omega_dot_d

    c_dot = np.cross(b3d_dot, b1c)
    b2d_dot = c_dot / c_norm - c * (np.dot(c, c_dot) / (c_norm**3))
    b1d_dot_orth = np.cross(b2d_dot, b3d) + np.cross(b2d, b3d_dot)

    Rd_dot = np.column_stack((b1d_dot_orth, b2d_dot, b3d_dot))
    Omega_hat_d = Rd.T @ Rd_dot
    Omega_d = vee(Omega_hat_d)

    Omega_dot_d = np.zeros(3)
    return Rd, Rd_dot, Omega_d, Omega_dot_d