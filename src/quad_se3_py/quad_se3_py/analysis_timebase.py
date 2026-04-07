import numpy as np

from .timing import trajectory_time_from_stamp
from .trajectories import evaluate_trajectory


def choose_recorded_epoch_start_time(epoch_time_sec, first_state_time_sec):
    if epoch_time_sec is not None:
        return float(epoch_time_sec), 'trajectory_epoch_topic'
    if first_state_time_sec is not None:
        return float(first_state_time_sec), 'first_quad_state_stamp'
    raise RuntimeError(
        'No /quad_state sample found while resolving trajectory start time.'
    )


def resolve_reference_source(reference_source_arg, experiment_metadata, epoch_time_sec):
    if reference_source_arg != 'auto':
        return reference_source_arg
    if epoch_time_sec is not None:
        return 'time_function'
    if (
        experiment_metadata.get('trajectory_mode') is not None
        and experiment_metadata.get('trajectory_epoch_source') in (
            'first_quad_state_stamp',
            'trajectory_epoch_topic',
        )
        and experiment_metadata.get('trajectory_start_time_sec') is not None
    ):
        return 'time_function'
    return 'trajectory_topic'


def resolve_time_function_start_time(
    configured_start_time_sec,
    epoch_time_sec,
    experiment_metadata,
    trajectory_fallback_time_sec,
):
    if configured_start_time_sec is not None:
        return float(configured_start_time_sec), 'cli_argument'
    if epoch_time_sec is not None:
        return float(epoch_time_sec), 'trajectory_epoch_topic'

    metadata_start_time_sec = experiment_metadata.get('trajectory_start_time_sec')
    if metadata_start_time_sec is not None:
        return float(metadata_start_time_sec), 'experiment_metadata'

    if trajectory_fallback_time_sec is not None:
        return float(trajectory_fallback_time_sec), 'first_trajectory_stamp'

    raise RuntimeError(
        'Unable to resolve trajectory start time from CLI, /trajectory_epoch, '
        'experiment metadata, or /trajectory.'
    )


def resolve_reference_time_offset_sec(
    configured_reference_time_offset_sec,
    experiment_metadata,
):
    if configured_reference_time_offset_sec is not None:
        return float(configured_reference_time_offset_sec)
    return float(experiment_metadata.get('reference_time_offset_sec', 0.0))


def reconstruct_reference_from_stamps(
    stamps,
    trajectory_mode,
    trajectory_start_time_sec,
    reference_time_offset_sec=0.0,
):
    ref_position = []
    ref_velocity = []
    ref_acceleration = []
    ref_jerk = []
    ref_b1d = []
    ref_b1d_dot = []
    trajectory_time = []

    for stamp in stamps:
        t = trajectory_time_from_stamp(
            stamp,
            trajectory_start_time_sec,
            reference_time_offset_sec,
        )
        sample = evaluate_trajectory(trajectory_mode, t)
        trajectory_time.append(t)
        ref_position.append(sample['position'])
        ref_velocity.append(sample['velocity'])
        ref_acceleration.append(sample['acceleration'])
        ref_jerk.append(sample['jerk'])
        ref_b1d.append(sample['b1d'])
        ref_b1d_dot.append(sample['b1d_dot'])

    return {
        'trajectory_time': np.array(trajectory_time, dtype=float),
        'position': np.array(ref_position, dtype=float),
        'velocity': np.array(ref_velocity, dtype=float),
        'acceleration': np.array(ref_acceleration, dtype=float),
        'jerk': np.array(ref_jerk, dtype=float),
        'b1d': np.array(ref_b1d, dtype=float),
        'b1d_dot': np.array(ref_b1d_dot, dtype=float),
    }
