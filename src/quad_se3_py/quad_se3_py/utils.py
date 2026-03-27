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