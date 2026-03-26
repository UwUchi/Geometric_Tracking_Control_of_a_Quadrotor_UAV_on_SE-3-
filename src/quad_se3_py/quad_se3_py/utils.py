import numpy as np

def hat(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def vee(M):
    return np.array([M[2,1], M[0,2], M[1,0]])

def normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-6:
        return v
    return v / n