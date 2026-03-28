import numpy as np


def hat(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])


def vee(M):
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def normalize(v, eps=1e-8):
    n = np.linalg.norm(v)
    if n < eps:
        return v.copy()
    return v / n


def normalize_with_norm(v, eps=1e-8):
    n = np.linalg.norm(v)
    if n < eps:
        return v.copy(), n
    return v / n, n


def project_to_so3(R):
    U, _, Vt = np.linalg.svd(R)  # 奇异值分解
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
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return R


def euler_to_rotmat(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def normalized_vec_and_derivative(c, c_dot, eps=1e-8):
    """
    Compute a normalized vector and its derivative.

    b = c / ||c||
    b_dot = c_dot / ||c|| - c * (c·c_dot) / ||c||^3
    """
    c_norm = np.linalg.norm(c)
    if c_norm < eps:
        return np.zeros(3), np.zeros(3), False

    b = c / c_norm
    b_dot = c_dot / c_norm - c * (np.dot(c, c_dot) / (c_norm ** 3))
    return b, b_dot, True


def compute_Rd_and_derivatives(
    b3d,
    b1d,
    b3d_dot=None,
    b1d_dot=None,
    current_Rd=None,
    eps=1e-8,
):  # 这里没用差分近似，而是直接用解析式求导，更稳，更几何。
    """
    Compute the desired attitude matrix and related derivatives.

    输入:
        b3d      : 期望推力轴方向（通常是单位向量）
        b1d      : 用于确定航向的参考方向，不要求与 b3d 正交
        b3d_dot  : b3d 的导数，可为 None
        b1d_dot  : b1d 的导数，可为 None
        current_Rd : 当 b3d 与 b1d 近乎平行时的回退参考
    输出:
        Rd, Rd_dot, Omega_d, Omega_dot_d
    """
    # ---------- 1) 归一化输入 ----------
    b3d, n3 = normalize_with_norm(b3d, eps)
    if n3 < eps:
        # 完全非法输入，给个保底
        Rd = np.eye(3) if current_Rd is None else current_Rd.copy()
        return Rd, np.zeros((3, 3)), np.zeros(3), np.zeros(3)

    b1c, n1 = normalize_with_norm(b1d, eps)
    if n1 < eps:
        # 如果 b1d 非法，就从当前姿态取一个参考
        if current_Rd is not None:
            b1c = current_Rd[:, 0]
        else:
            b1c = np.array([1.0, 0.0, 0.0])

    if b3d_dot is None:
        b3d_dot = np.zeros(3)
    if b1d_dot is None:
        b1d_dot = np.zeros(3)

    # ---------- 2) 构造 b2d ----------
    # c = b3d x b1c
    c = np.cross(b3d, b1c)
    # 严格求导：c_dot = b3d_dot x b1c + b3d x b1c_dot
    c_dot = np.cross(b3d_dot, b1c) + np.cross(b3d, b1d_dot)

    b2d, b2d_dot, ok = normalized_vec_and_derivative(c, c_dot, eps)

    if not ok:
        # b3d 与 b1c 几乎平行，heading 不可定义
        # 用当前 Rd 的第二列/第一列回退，尽量保持连续
        if current_Rd is not None:
            Rd = current_Rd.copy()
        else:
            # 手工选一个不平行轴
            trial = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(trial, b3d)) > 0.9:
                trial = np.array([0.0, 1.0, 0.0])

            c = np.cross(b3d, trial)
            c_norm = np.linalg.norm(c)
            b2d = c / c_norm
            b1d_orth = np.cross(b2d, b3d)
            Rd = np.column_stack((b1d_orth, b2d, b3d))
            Rd = project_to_so3(Rd)

        return Rd, np.zeros((3, 3)), np.zeros(3), np.zeros(3)

    # ---------- 3) 构造第一列 ----------
    # 不是直接用 b1d，而是用它在垂直 b3d 平面内的投影方向
    # 论文写法：Rd = [b2d x b3d, b2d, b3d]
    b1d_orth = np.cross(b2d, b3d)
    b1d_orth_dot = np.cross(b2d_dot, b3d) + np.cross(b2d, b3d_dot)

    # ---------- 4) 拼 Rd ----------
    Rd = np.column_stack((b1d_orth, b2d, b3d))
    Rd = project_to_so3(Rd)

    Rd_dot = np.column_stack((b1d_orth_dot, b2d_dot, b3d_dot))

    # ---------- 5) 求 Omega_d ----------
    # 理论上 Rd^T Rd_dot 应该是反对称矩阵
    Omega_hat_d = Rd.T @ Rd_dot
    Omega_hat_d = 0.5 * (Omega_hat_d - Omega_hat_d.T)  # 数值对称化
    Omega_d = vee(Omega_hat_d)

    # ---------- 6) 先给简化版 Omega_dot_d ----------
    # 论文控制律里会用到 Omega_d_dot，但你当前复现阶段先置零通常能跑通框架。
    Omega_dot_d = np.zeros(3)

    return Rd, Rd_dot, Omega_d, Omega_dot_d
