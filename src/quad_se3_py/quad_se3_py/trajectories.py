import numpy as np


def _hover_reference(t):
    yaw_rate = 0.6
    yaw = yaw_rate * t
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    return {
        'position': (0.5*t, 0.3, 0.0),
        'velocity': (0.5, 0.0, 0.0),
        'acceleration': (0.0, 0.0, 0.0),
        'jerk': (0.0, 0.0, 0.0),
        'b1d': (float(cos_yaw), float(sin_yaw), 0.0),
        'b1d_dot': (
            float(-yaw_rate * sin_yaw),
            float(yaw_rate * cos_yaw),
            0.0,
        ),
    }


def _paper_case_1_helix_reference(t):
    pi_t = np.pi * t
    return {
        'position': (
            float(0.4 * t),
            float(0.4 * np.sin(pi_t)),
            float(-0.6 * np.cos(pi_t)),
        ),
        'velocity': (
            0.4,
            float(0.4 * np.pi * np.cos(pi_t)),
            float(0.6 * np.pi * np.sin(pi_t)),
        ),
        'acceleration': (
            0.0,
            float(-0.4 * (np.pi ** 2) * np.sin(pi_t)),
            float(0.6 * (np.pi ** 2) * np.cos(pi_t)),
        ),
        'jerk': (
            0.0,
            float(-0.4 * (np.pi ** 3) * np.cos(pi_t)),
            float(-0.6 * (np.pi ** 3) * np.sin(pi_t)),
        ),
        'b1d': (
            float(np.cos(pi_t)),
            float(np.sin(pi_t)),
            0.0,
        ),
        'b1d_dot': (
            float(-np.pi * np.sin(pi_t)),
            float(np.pi * np.cos(pi_t)),
            0.0,
        ),
    }


def _paper_case_2_recovery_reference(_t):
    return {
        'position': (0.0, 0.0, 0.0),
        'velocity': (0.0, 0.0, 0.0),
        'acceleration': (0.0, 0.0, 0.0),
        'jerk': (0.0, 0.0, 0.0),
        'b1d': (1.0, 0.0, 0.0),
        'b1d_dot': (0.0, 0.0, 0.0),
    }


def evaluate_trajectory(mode, t):
    if mode == 'hover':
        return _hover_reference(t)
    if mode == 'paper_case_1_helix':
        return _paper_case_1_helix_reference(t)
    if mode == 'paper_case_2_recovery_reference':
        return _paper_case_2_recovery_reference(t)
    raise ValueError(f'Unsupported trajectory_mode: {mode}')
