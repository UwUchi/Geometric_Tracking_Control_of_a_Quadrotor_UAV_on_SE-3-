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