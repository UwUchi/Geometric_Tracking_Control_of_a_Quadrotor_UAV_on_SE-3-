#!/usr/bin/env python3

import argparse
import json
import os
import shlex
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

if os.environ.get('QUAD_SE3_ANALYZE_ENV_READY') != '1':
    setup_script = REPO_ROOT / 'install' / 'setup.bash'
    if setup_script.is_file():
        quoted_script = shlex.quote(str(Path(__file__).resolve()))
        quoted_args = ' '.join(shlex.quote(arg) for arg in sys.argv[1:])
        exec_command = (
            f'source {shlex.quote(str(setup_script))} && '
            'export QUAD_SE3_ANALYZE_ENV_READY=1 && '
            f'python3 {quoted_script}'
        )
        if quoted_args:
            exec_command += f' {quoted_args}'
        os.execv('/bin/bash', ['bash', '-lc', exec_command])

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.font_manager as font_manager
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PYTHON_VERSION = f'python{sys.version_info.major}.{sys.version_info.minor}'
QUAD_MSGS_PREFIX = REPO_ROOT / 'install' / 'quad_se3_msgs'
sys.path.insert(0, str(REPO_ROOT / 'src' / 'quad_se3_py'))
sys.path.insert(
    0,
    str(QUAD_MSGS_PREFIX / 'local' / 'lib' / PYTHON_VERSION / 'dist-packages'),
)
existing_ld_library_path = os.environ.get('LD_LIBRARY_PATH', '')
quad_msgs_lib_dir = str(QUAD_MSGS_PREFIX / 'lib')
os.environ['LD_LIBRARY_PATH'] = (
    f'{quad_msgs_lib_dir}:{existing_ld_library_path}'
    if existing_ld_library_path
    else quad_msgs_lib_dir
)

from quad_se3_py.reference import compute_desired_attitude_from_state
from quad_se3_py.timing import trajectory_time_from_stamp
from quad_se3_py.trajectories import evaluate_trajectory
from quad_se3_py.utils import quat_to_rotmat, vee


TOPICS = {
    '/quad_state': 'quad_se3_msgs/msg/QuadState',
    '/trajectory': 'quad_se3_msgs/msg/TrajectoryPoint',
    '/control_input': 'quad_se3_msgs/msg/ControlInput',
    '/trajectory_epoch': 'builtin_interfaces/msg/Time',
}


def parse_args():
    parser = argparse.ArgumentParser(description='Analyze quadrotor rosbag data.')
    parser.add_argument(
        'bag_path',
        nargs='?',
        help='Path to a rosbag2 directory, defaults to the latest bag under bags/',
    )
    parser.add_argument(
        '--output-dir',
        help='Directory to write plots and summary into',
    )
    parser.add_argument(
        '--case-name',
        help='Override output case name, defaults to the bag directory name',
    )
    parser.add_argument(
        '--arm-length',
        type=float,
        default=0.315,
        help='Rotor arm length d used to recover individual rotor thrusts.',
    )
    parser.add_argument(
        '--yaw-moment-coeff',
        type=float,
        default=8.004e-4,
        help='Yaw moment coefficient c_tau_f used to recover rotor thrusts.',
    )
    parser.add_argument(
        '--trajectory-mode',
        choices=['hover', 'paper_case_1_helix', 'paper_case_2_recovery_reference'],
        help='Trajectory mode for time-based reference reconstruction.',
    )
    parser.add_argument(
        '--trajectory-start-time-sec',
        type=float,
        help='Shared trajectory start time in ROS seconds. Defaults to the first /trajectory stamp.',
    )
    parser.add_argument(
        '--reference-source',
        choices=['auto', 'time_function', 'trajectory_topic'],
        default='auto',
        help='Reference reconstruction source. Defaults to auto-detecting bag metadata, otherwise falling back to /trajectory.',
    )
    return parser.parse_args()


def resolve_bag_path(bag_path_arg):
    if bag_path_arg:
        return Path(bag_path_arg).resolve()

    bags_dir = REPO_ROOT / 'bags'
    if not bags_dir.is_dir():
        raise RuntimeError(f'No bags directory found at {bags_dir}')

    bag_dirs = [path for path in bags_dir.iterdir() if path.is_dir()]
    if not bag_dirs:
        raise RuntimeError(f'No bag directories found under {bags_dir}')

    return max(bag_dirs, key=lambda path: path.stat().st_mtime).resolve()


def read_experiment_metadata(bag_path):
    metadata_path = Path(bag_path) / 'experiment_metadata.json'
    if not metadata_path.is_file():
        return {}
    with open(metadata_path, 'r', encoding='utf-8') as file:
        return json.load(file)


def read_bag_messages(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required_topics = {'/quad_state', '/trajectory', '/control_input'}
    missing = sorted(required_topics - set(topic_types))
    if missing:
        raise RuntimeError(f'Missing required topics in bag: {missing}')

    type_cache = {
        topic: get_message(topic_types[topic])
        for topic in topic_types
        if topic in TOPICS
    }
    bag_data = {topic: [] for topic in TOPICS}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in bag_data:
            continue
        message = deserialize_message(data, type_cache[topic])
        bag_data[topic].append((timestamp_ns, message))

    return bag_data


def stamp_to_sec(stamp, fallback_ns):
    sec = getattr(stamp, 'sec', 0)
    nanosec = getattr(stamp, 'nanosec', 0)
    if sec == 0 and nanosec == 0:
        return fallback_ns * 1e-9
    return float(sec) + float(nanosec) * 1e-9


def vector3_to_array(msg):
    return np.array([msg.x, msg.y, msg.z], dtype=float)


def sample_state(messages):
    times, stamps, positions, velocities, accelerations, rotations, omegas = [], [], [], [], [], [], []
    for timestamp_ns, msg in messages:
        stamp_sec = stamp_to_sec(msg.stamp, timestamp_ns)
        times.append(stamp_sec)
        stamps.append(msg.stamp)
        positions.append(vector3_to_array(msg.position))
        velocities.append(vector3_to_array(msg.velocity))
        accelerations.append(vector3_to_array(msg.acceleration))
        rotations.append(
            quat_to_rotmat(
                np.array([
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ], dtype=float)
            )
        )
        omegas.append(vector3_to_array(msg.angular_velocity))
    return {
        'time': np.array(times, dtype=float),
        'stamp': stamps,
        'position': np.array(positions, dtype=float),
        'velocity': np.array(velocities, dtype=float),
        'acceleration': np.array(accelerations, dtype=float),
        'rotation': np.array(rotations, dtype=float),
        'omega': np.array(omegas, dtype=float),
    }


def sample_trajectory(messages):
    times = []
    fields = {
        'position': [],
        'velocity': [],
        'acceleration': [],
        'jerk': [],
        'b1d': [],
        'b1d_dot': [],
    }
    for timestamp_ns, msg in messages:
        times.append(stamp_to_sec(msg.stamp, timestamp_ns))
        fields['position'].append(vector3_to_array(msg.position))
        fields['velocity'].append(vector3_to_array(msg.velocity))
        fields['acceleration'].append(vector3_to_array(msg.acceleration))
        fields['jerk'].append(vector3_to_array(msg.jerk))
        fields['b1d'].append(vector3_to_array(msg.b1d))
        fields['b1d_dot'].append(vector3_to_array(msg.b1d_dot))
    return {
        'time': np.array(times, dtype=float),
        **{key: np.array(value, dtype=float) for key, value in fields.items()},
    }


def sample_control(messages):
    times, thrusts, moments = [], [], []
    for timestamp_ns, msg in messages:
        times.append(timestamp_ns * 1e-9)
        thrusts.append(float(msg.thrust))
        moments.append(vector3_to_array(msg.moment))
    return {
        'time': np.array(times, dtype=float),
        'thrust': np.array(thrusts, dtype=float),
        'moment': np.array(moments, dtype=float),
    }


def sample_epoch(messages):
    if not messages:
        return None
    _, msg = messages[0]
    return float(msg.sec) + float(msg.nanosec) * 1e-9


def infer_trajectory_mode(bag_path, case_name_override):
    name = (case_name_override or bag_path.name).lower()
    if 'case1' in name or 'helix' in name:
        return 'paper_case_1_helix'
    if 'case2' in name or 'recovery' in name or 'upside_down' in name:
        return 'paper_case_2_recovery_reference'
    return 'hover'


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


def interp_vectors(source_times, source_values, target_times):
    if len(source_times) == 0:
        raise RuntimeError('Cannot interpolate empty source data.')
    if len(source_times) == 1:
        return np.repeat(source_values, len(target_times), axis=0)
    return np.column_stack([
        np.interp(target_times, source_times, source_values[:, dim])
        for dim in range(source_values.shape[1])
    ])


def reconstruct_reference_from_time_function(
    state_data,
    trajectory_mode,
    trajectory_start_time_sec,
):
    ref_position = []
    ref_velocity = []
    ref_acceleration = []
    ref_jerk = []
    ref_b1d = []
    ref_b1d_dot = []
    trajectory_time = []

    for stamp in state_data['stamp']:
        t = trajectory_time_from_stamp(stamp, trajectory_start_time_sec)
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


def reconstruct_reference_from_topic(state_data, trajectory_data):
    target_times = state_data['time']
    return {
        'trajectory_time': target_times - target_times[0],
        'position': interp_vectors(
            trajectory_data['time'], trajectory_data['position'], target_times
        ),
        'velocity': interp_vectors(
            trajectory_data['time'], trajectory_data['velocity'], target_times
        ),
        'acceleration': interp_vectors(
            trajectory_data['time'], trajectory_data['acceleration'], target_times
        ),
        'jerk': interp_vectors(
            trajectory_data['time'], trajectory_data['jerk'], target_times
        ),
        'b1d': interp_vectors(
            trajectory_data['time'], trajectory_data['b1d'], target_times
        ),
        'b1d_dot': interp_vectors(
            trajectory_data['time'], trajectory_data['b1d_dot'], target_times
        ),
    }


def analyze(state_data, reference_data):
    target_times = state_data['time']
    ref_position = reference_data['position']
    ref_velocity = reference_data['velocity']
    ref_acceleration = reference_data['acceleration']
    ref_jerk = reference_data['jerk']
    ref_b1d = reference_data['b1d']
    ref_b1d_dot = reference_data['b1d_dot']

    psi_values = []
    e_x_values = []
    e_v_values = []
    e_r_values = []
    e_omega_values = []
    desired_omega_values = []
    current_Rd = np.eye(3)
    current_Omega_d = np.zeros(3)
    e3 = np.array([0.0, 0.0, 1.0], dtype=float)

    for index in range(len(target_times)):
        Rd, _, Omega_d, _, _ = compute_desired_attitude_from_state(
            x=state_data['position'][index],
            v=state_data['velocity'][index],
            x_dd=state_data['acceleration'][index],
            xd=ref_position[index],
            vd=ref_velocity[index],
            xd_dd=ref_acceleration[index],
            xd_ddd=ref_jerk[index],
            b1d=ref_b1d[index],
            b1d_dot=ref_b1d_dot[index],
            current_Rd=current_Rd,
            m=4.34,
            g=9.81,
            e3=e3,
            kx=16*4.34,
            kv=5.6*4.34,
            Omega_d_prev=current_Omega_d,
        )
        current_Rd = Rd
        current_Omega_d = Omega_d

        R = state_data['rotation'][index]
        e_x = state_data['position'][index] - ref_position[index]
        e_v = state_data['velocity'][index] - ref_velocity[index]
        e_R = 0.5 * vee(Rd.T @ R - R.T @ Rd)
        e_Omega = state_data['omega'][index] - R.T @ Rd @ Omega_d
        psi = 0.5 * np.trace(np.eye(3) - Rd.T @ R)

        e_x_values.append(e_x)
        e_v_values.append(e_v)
        e_r_values.append(e_R)
        e_omega_values.append(e_Omega)
        desired_omega_values.append(Omega_d)
        psi_values.append(float(psi))

    return {
        'time': target_times,
        'reference_time': reference_data['trajectory_time'],
        'reference_position': ref_position,
        'reference_velocity': ref_velocity,
        'desired_omega': np.array(desired_omega_values, dtype=float),
        'e_x': np.array(e_x_values, dtype=float),
        'e_v': np.array(e_v_values, dtype=float),
        'e_R': np.array(e_r_values, dtype=float),
        'e_Omega': np.array(e_omega_values, dtype=float),
        'psi': np.array(psi_values, dtype=float),
    }


def recover_rotor_thrusts(control_data, arm_length, yaw_moment_coeff):
    if len(control_data['time']) == 0:
        return np.empty((0, 4), dtype=float)

    mixing_matrix = np.array([
        [1.0, 1.0, 1.0, 1.0],
        [0.0, -arm_length, 0.0, arm_length],
        [arm_length, 0.0, -arm_length, 0.0],
        [-yaw_moment_coeff, yaw_moment_coeff, -yaw_moment_coeff, yaw_moment_coeff],
    ], dtype=float)
    mixing_inv = np.linalg.inv(mixing_matrix)

    inputs = np.column_stack([
        control_data['thrust'],
        control_data['moment'][:, 0],
        control_data['moment'][:, 1],
        control_data['moment'][:, 2],
    ])
    return (mixing_inv @ inputs.T).T


def _apply_paper_axis_style(axis):
    axis.grid(False)
    axis.tick_params(direction='in', top=True, right=True, length=3)
    for spine in axis.spines.values():
        spine.set_linewidth(0.8)


def _set_component_limits(axis, actual_values, desired_values=None):
    stacked = [actual_values]
    if desired_values is not None:
        stacked.append(desired_values)
    combined = np.concatenate(stacked)
    data_min = float(np.min(combined))
    data_max = float(np.max(combined))
    if np.isclose(data_min, data_max):
        margin = 0.2 if np.isclose(data_min, 0.0) else max(0.1, 0.15 * abs(data_min))
    else:
        margin = 0.12 * (data_max - data_min)
    axis.set_ylim(data_min - margin, data_max + margin)


def plot_trajectory(output_dir, state_data, analysis_data):
    fig = plt.figure(figsize=(11.3, 12))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(
        state_data['position'][:, 0],
        state_data['position'][:, 1],
        state_data['position'][:, 2],
        color='#e0663f',
        linewidth=2.2,
        label='actual',
    )
    ax.plot(
        analysis_data['reference_position'][:, 0],
        analysis_data['reference_position'][:, 1],
        analysis_data['reference_position'][:, 2],
        color='#2f8fdd',
        linestyle='--',
        linewidth=2.0,
        label='desired',
    )
    ax.set_title('3D Trajectory')
    ax.set_xlabel('x [m]', labelpad=10)
    ax.set_ylabel('y [m]', labelpad=10)
    ax.set_zlabel('z [m]', labelpad=10)
    ax.legend()
    ax.grid(True)
    ax.view_init(elev=20, azim=-37)
    ax.set_box_aspect((
        np.ptp(state_data['position'][:, 0]) + 1e-6,
        np.ptp(state_data['position'][:, 1]) + 1e-6,
        np.ptp(state_data['position'][:, 2]) + 1e-6,
    ))

    if np.ptp(state_data['position'][:, 0]) < 1e-3: # 防止 x 轴范围过小导致刻度显示异常
        ax.set_xticks(np.array([0.0]))
        ax.xaxis.set_major_formatter(ticker.FormatStrFormatter('%.1f'))

    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.04, top=0.92)
    fig.savefig(output_dir / 'trajectory_3d.png', dpi=180)
    plt.close(fig)


def _plot_vector_components(axis_group, time, actual_values, desired_values, labels):
    for index, axis in enumerate(axis_group):
        axis.plot(
            time,
            actual_values[:, index],
            color='#1f4fb2',
            linewidth=1.1,
        )
        axis.plot(
            time,
            desired_values[:, index],
            color='#d74b4b',
            linewidth=0.9,
            linestyle=(0, (5, 4)),
        )
        axis.set_ylabel(f'{labels[index]}')
        axis.yaxis.set_label_coords(-0.08, 0.5)
        axis.set_xlim(time[0], time[-1])
        _set_component_limits(axis, actual_values[:, index], desired_values[:, index])
        _apply_paper_axis_style(axis)


def plot_errors(output_dir, state_data, control_data, analysis_data, arm_length, yaw_moment_coeff):
    time = analysis_data['time'] - analysis_data['time'][0]
    fig = plt.figure(figsize=(13, 9.6))
    outer = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.22)

    psi_axis = fig.add_subplot(outer[0, 0])
    psi_axis.plot(time, analysis_data['psi'], color='#1f4fb2', linewidth=1.1)
    psi_axis.set_title('(a) Attitude error function $\\Psi$', pad=10)
    psi_axis.set_xlim(time[0], time[-1])
    _set_component_limits(psi_axis, analysis_data['psi'])
    _apply_paper_axis_style(psi_axis)
    psi_axis.set_xlabel('time $[s]$')

    position_grid = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=outer[0, 1], hspace=0.12)
    position_axes = [fig.add_subplot(position_grid[index, 0]) for index in range(3)]
    _plot_vector_components(
        position_axes,
        time,
        state_data['position'],
        analysis_data['reference_position'],
        labels=['$x$ [m]', '$y$ [m]', '$z$ [m]'],
    )
    position_axes[0].set_title('(b) Position ($x$: solid, $x_d$: dotted, (m))', pad=10)
    position_axes[-1].set_xlabel('time [s]')
    for axis in position_axes[:-1]:
        axis.tick_params(labelbottom=False)

    omega_grid = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=outer[1, 0], hspace=0.12)
    omega_axes = [fig.add_subplot(omega_grid[index, 0]) for index in range(3)]
    _plot_vector_components(
        omega_axes,
        time,
        state_data['omega'],
        analysis_data['desired_omega'],
        labels=['$\\Omega_x$', '$\\Omega_y$', '$\\Omega_z$'],
    )
    omega_axes[0].set_title(
        '(c) Angular velocity ($\\Omega$: solid, $\\Omega_d$: dotted, (rad/s))',
        pad=12,
    )
    omega_axes[-1].set_xlabel('time [s]')
    for axis in omega_axes[:-1]:
        axis.tick_params(labelbottom=False)

    rotor_grid = gridspec.GridSpecFromSubplotSpec(4, 1, subplot_spec=outer[1, 1], hspace=0.12)
    rotor_axes = [fig.add_subplot(rotor_grid[index, 0]) for index in range(4)]
    rotor_thrusts = recover_rotor_thrusts(control_data, arm_length, yaw_moment_coeff)
    if len(control_data['time']) > 0:
        control_time = control_data['time'] - control_data['time'][0]
        for index, axis in enumerate(rotor_axes):
            axis.plot(
                control_time,
                rotor_thrusts[:, index],
                color='#1f4fb2',
                linewidth=1.1,
            )
            axis.set_ylabel(f'$f_{index + 1}$')
            axis.yaxis.set_label_coords(-0.08, 0.5)
            axis.set_xlim(control_time[0], control_time[-1])
            _set_component_limits(axis, rotor_thrusts[:, index])
            _apply_paper_axis_style(axis)
    rotor_axes[0].set_title('(d) Thrust of each rotor (N)', pad=12)
    rotor_axes[-1].set_xlabel('time [s]')
    for axis in rotor_axes[:-1]:
        axis.tick_params(labelbottom=False)

    fig.subplots_adjust(left=0.07, right=0.98, bottom=0.07, top=0.95)
    fig.savefig(output_dir / 'errors.png', dpi=180)
    plt.close(fig)

"""额外的调试图，显示位置误差在切向方向上的分量, 
可以帮助分析误差是控制器的问题还是rviz2可视化的时间没对齐。"""
def plot_debug(output_dir, state_data, analysis_data):
    target_times = state_data['time']
    ref_position = analysis_data['reference_position']
    ref_velocity = analysis_data['reference_velocity']
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111)

    position_error = state_data['position'] - ref_position
    tangent_speed = np.linalg.norm(ref_velocity, axis=1)
    tangent_error = np.divide(
        np.sum(position_error * ref_velocity, axis=1),
        tangent_speed,
        out=np.zeros_like(target_times, dtype=float),
        where=tangent_speed > 1e-6,
    )
    time = target_times - target_times[0]

    ax.plot(
        time,
        tangent_error,
        color='#e0663f',
        linewidth=1.2,
        label='$e_\\tau$',
    )

    ax.set_title('Tangential Position Error')
    ax.set_xlabel('time [s]', labelpad=10)
    ax.set_ylabel('$e_\\tau$ [m]', labelpad=10)
    ax.legend()
    _set_component_limits(ax, tangent_error)
    _apply_paper_axis_style(ax)

    fig.subplots_adjust(left=0.04, right=0.98, bottom=0.04, top=0.92)
    fig.savefig(output_dir / 'errors_debug.png', dpi=180)
    plt.close(fig)


def write_summary(output_dir, state_data, analysis_data):
    summary = {
        'duration_sec': float(analysis_data['time'][-1] - analysis_data['time'][0]),
        'num_state_samples': int(len(state_data['time'])),
        'max_position_error_norm': float(np.max(np.linalg.norm(analysis_data['e_x'], axis=1))),
        'final_position_error_norm': float(np.linalg.norm(analysis_data['e_x'][-1])),
        'max_attitude_error_function': float(np.max(analysis_data['psi'])),
        'final_attitude_error_function': float(analysis_data['psi'][-1]),
        'max_omega_error_norm': float(np.max(np.linalg.norm(analysis_data['e_Omega'], axis=1))),
        'final_omega_error_norm': float(np.linalg.norm(analysis_data['e_Omega'][-1])),
        'reference_source': analysis_data.get('reference_source'),
        'trajectory_mode': analysis_data.get('trajectory_mode'),
        'trajectory_start_time_sec': analysis_data.get('trajectory_start_time_sec'),
    }
    with open(output_dir / 'summary.json', 'w', encoding='utf-8') as file:
        json.dump(summary, file, indent=2)


def main():
    args = parse_args()
    bag_path = resolve_bag_path(args.bag_path)
    case_name = args.case_name or bag_path.name
    output_dir = (
        Path(args.output_dir).resolve()
        if args.output_dir
        else REPO_ROOT / 'plots' / bag_path.name
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    experiment_metadata = read_experiment_metadata(bag_path)
    bag_data = read_bag_messages(str(bag_path))
    state_data = sample_state(bag_data['/quad_state'])
    trajectory_data = sample_trajectory(bag_data['/trajectory'])
    control_data = sample_control(bag_data['/control_input'])
    epoch_time_sec = sample_epoch(bag_data['/trajectory_epoch'])

    reference_source = resolve_reference_source(
        args.reference_source,
        experiment_metadata,
        epoch_time_sec,
    )
    trajectory_mode = (
        args.trajectory_mode
        or experiment_metadata.get('trajectory_mode')
        or infer_trajectory_mode(bag_path, args.case_name)
    )
    if reference_source == 'time_function':
        trajectory_start_time_sec = (
            args.trajectory_start_time_sec
            if args.trajectory_start_time_sec is not None
            else (
                experiment_metadata.get('trajectory_start_time_sec')
                if experiment_metadata.get('trajectory_start_time_sec') is not None
                else epoch_time_sec
            )
        )
        if trajectory_start_time_sec is None:
            trajectory_start_time_sec = trajectory_data['time'][0]
            print(
                'Warning: time_function reference without recorded '
                'trajectory_start_time_sec uses the first /trajectory stamp as '
                'an approximation and may introduce a constant phase offset.'
            )
        reference_data = reconstruct_reference_from_time_function(
            state_data,
            trajectory_mode,
            trajectory_start_time_sec,
        )
    else:
        reference_data = reconstruct_reference_from_topic(state_data, trajectory_data)

    analysis_data = analyze(state_data, reference_data)
    analysis_data['reference_source'] = reference_source
    analysis_data['trajectory_mode'] = trajectory_mode
    analysis_data['trajectory_start_time_sec'] = (
        trajectory_start_time_sec
        if reference_source == 'time_function'
        else experiment_metadata.get('trajectory_start_time_sec')
    )
    font_path = "/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf"
    font_manager.fontManager.addfont(font_path)
    font_name = font_manager.FontProperties(fname=font_path).get_name()

    plt.rcParams.update({
        "font.family": font_name,
        "mathtext.fontset": "stix",
    })
    plot_trajectory(output_dir, state_data, analysis_data)
    plot_errors(
        output_dir,
        state_data,
        control_data,
        analysis_data,
        args.arm_length,
        args.yaw_moment_coeff,
    )
    plot_debug(output_dir, state_data, analysis_data)
    write_summary(output_dir, state_data, analysis_data)

    print(f'Wrote plots and summary to {output_dir}')


if __name__ == '__main__':
    main()
