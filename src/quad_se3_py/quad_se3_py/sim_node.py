from dataclasses import dataclass

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from quad_se3_msgs.msg import ControlInput, QuadState, TrajectoryPoint

from .config import make_control_gains
from .reference import compute_desired_attitude_and_force_from_state
from .timing import (
    resolve_trajectory_start_time,
    seconds_to_stamp,
    trajectory_time_from_stamp,
)
from .trajectories import evaluate_trajectory
from .utils import euler_to_rotmat, hat, normalize, project_to_so3, rotmat_to_quat, vee


@dataclass(frozen=True)
class SimTimebase:
    epoch_time_sec: float
    dt: float

    def stamp_for_step(self, step_index: int) -> Time:
        return seconds_to_stamp(self.epoch_time_sec + step_index * self.dt)


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')

        self.state_pub = self.create_publisher(QuadState, '/quad_state', 10)
        self.trajectory_pub = self.create_publisher(TrajectoryPoint, '/trajectory', 10)
        self.control_pub = self.create_publisher(ControlInput, '/control_input', 10)
        self.epoch_pub = self.create_publisher(
            Time,
            '/trajectory_epoch',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self.declare_parameter('trajectory_mode', 'hover')
        self.declare_parameter('trajectory_start_time_sec', 0.0)
        self.declare_parameter('reference_time_offset_sec', 0.0)
        self.declare_parameter('initial_position', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_velocity', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_roll_deg', 0.0)
        self.declare_parameter('initial_pitch_deg', 0.0)
        self.declare_parameter('initial_yaw_deg', 0.0)
        self.declare_parameter('initial_angular_velocity', [0.0, 0.0, 0.0])

        self.m = 4.34
        self.g = 9.81
        self.J = np.diag([0.0820, 0.0845, 0.1377])
        self.J_inv = np.linalg.inv(self.J)
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        configured_start_time_sec = float(
            self.get_parameter('trajectory_start_time_sec').value
        )
        self.reference_time_offset_sec = float(
            self.get_parameter('reference_time_offset_sec').value
        )
        self.dt = 0.002
        startup_time_sec = self.get_clock().now().nanoseconds * 1e-9
        self.trajectory_start_time_sec = resolve_trajectory_start_time(
            configured_start_time_sec,
            startup_time_sec,
        )
        self.timebase = SimTimebase(
            epoch_time_sec=self.trajectory_start_time_sec,
            dt=self.dt,
        )

        self.x = np.array(
            self.get_parameter('initial_position').value,
            dtype=float,
        )
        self.v = np.array(
            self.get_parameter('initial_velocity').value,
            dtype=float,
        )
        self.vdot = np.zeros(3)
        initial_rpy_deg = np.array([
            self.get_parameter('initial_roll_deg').value,
            self.get_parameter('initial_pitch_deg').value,
            self.get_parameter('initial_yaw_deg').value,
        ], dtype=float)
        self.R = euler_to_rotmat(np.deg2rad(initial_rpy_deg))
        self.Omega = np.array(
            self.get_parameter('initial_angular_velocity').value,
            dtype=float,
        )

        self.gains = make_control_gains(self.m)

        self.xd = np.zeros(3)
        self.vd = np.zeros(3)
        self.xd_dd = np.zeros(3)
        self.xd_ddd = np.zeros(3)
        self.b1d = np.array([1.0, 0.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.Rd = np.eye(3)
        self.Omega_d_prev = np.zeros(3)

        self.completed_steps = 0
        self.epoch_published = False
        self.initial_state_published = False
        self.log_counter = 0
        self.time_debug_counter = 0

        self.timer = self.create_timer(self.dt, self.update)

    def _sample_reference_from_stamp(self, stamp: Time):
        t = trajectory_time_from_stamp(
            stamp,
            self.trajectory_start_time_sec,
            self.reference_time_offset_sec,
        )
        return evaluate_trajectory(self.trajectory_mode, t), t

    def _load_reference_sample(self, sample):
        self.xd = np.array(sample['position'], dtype=float)
        self.vd = np.array(sample['velocity'], dtype=float)
        self.xd_dd = np.array(sample['acceleration'], dtype=float)
        self.xd_ddd = np.array(sample['jerk'], dtype=float)
        self.b1d = normalize(np.array(sample['b1d'], dtype=float))
        self.b1d_dot = np.array(sample['b1d_dot'], dtype=float)

    def _compute_control(self):
        Rd, _, Omega_d, Omega_d_dot, A = (
            compute_desired_attitude_and_force_from_state(
                x=self.x,
                v=self.v,
                x_dd=self.vdot,
                xd=self.xd,
                vd=self.vd,
                xd_dd=self.xd_dd,
                xd_ddd=self.xd_ddd,
                b1d=self.b1d,
                b1d_dot=self.b1d_dot,
                current_Rd=self.Rd,
                Omega_d_prev=self.Omega_d_prev,
                dt=self.dt,
                m=self.m,
                g=self.g,
                e3=self.e3,
                kx=self.gains.kx,
                kv=self.gains.kv,
            )
        )
        self.Rd = Rd
        self.Omega_d_prev = Omega_d

        e_R = 0.5 * vee(Rd.T @ self.R - self.R.T @ Rd)
        e_Omega = self.Omega - self.R.T @ Rd @ Omega_d

        thrust = float(-np.dot(A, self.R @ self.e3))
        moment = (
            -self.gains.kR * e_R
            - self.gains.kOmega * e_Omega
            + np.cross(self.Omega, self.J @ self.Omega)
            - self.J @ (
                hat(self.Omega) @ self.R.T @ Rd @ Omega_d
                - self.R.T @ Rd @ Omega_d_dot
            )
        )
        return thrust, moment, e_R, Omega_d_dot

    def _integrate_step(self, thrust: float, moment: np.ndarray):
        xdot = self.v
        self.vdot = self.g * self.e3 - (thrust / self.m) * (self.R @ self.e3)
        Rdot = self.R @ hat(self.Omega)
        Omegadot = self.J_inv @ (
            moment - np.cross(self.Omega, self.J @ self.Omega)
        )

        self.x += xdot * self.dt
        self.v += self.vdot * self.dt
        self.R += Rdot * self.dt
        self.R = project_to_so3(self.R)
        self.Omega += Omegadot * self.dt

    def _make_state_msg(self, stamp: Time) -> QuadState:
        q = rotmat_to_quat(self.R)
        msg = QuadState()
        msg.stamp = stamp
        msg.position.x, msg.position.y, msg.position.z = self.x
        msg.velocity.x, msg.velocity.y, msg.velocity.z = self.v
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = self.vdot
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            self.Omega
        )
        return msg

    def _make_trajectory_msg(self, stamp: Time, sample) -> TrajectoryPoint:
        msg = TrajectoryPoint()
        msg.stamp = stamp
        msg.position.x, msg.position.y, msg.position.z = sample['position']
        msg.velocity.x, msg.velocity.y, msg.velocity.z = sample['velocity']
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = (
            sample['acceleration']
        )
        msg.jerk.x, msg.jerk.y, msg.jerk.z = sample['jerk']
        msg.b1d.x, msg.b1d.y, msg.b1d.z = sample['b1d']
        msg.b1d_dot.x, msg.b1d_dot.y, msg.b1d_dot.z = sample['b1d_dot']
        return msg

    def _make_control_msg(self, thrust: float, moment: np.ndarray) -> ControlInput:
        msg = ControlInput()
        msg.thrust = thrust
        msg.moment.x = float(moment[0])
        msg.moment.y = float(moment[1])
        msg.moment.z = float(moment[2])
        return msg

    def _publish_epoch_if_needed(self):
        if self.epoch_published:
            return
        self.epoch_pub.publish(self.timebase.stamp_for_step(0))
        self.epoch_published = True

    def _publish_initial_state(self):
        stamp = self.timebase.stamp_for_step(0)
        trajectory_sample, _ = self._sample_reference_from_stamp(stamp)
        self.state_pub.publish(self._make_state_msg(stamp))
        self.trajectory_pub.publish(
            self._make_trajectory_msg(stamp, trajectory_sample)
        )
        self.initial_state_published = True

    def update(self):
        self._publish_epoch_if_needed()
        if not self.initial_state_published:
            self._publish_initial_state()
            return

        control_stamp = self.timebase.stamp_for_step(self.completed_steps)
        control_sample, t_ref = self._sample_reference_from_stamp(control_stamp)
        self._load_reference_sample(control_sample)
        thrust, moment, e_R, Omega_d_dot = self._compute_control()

        self._integrate_step(thrust, moment)
        self.completed_steps += 1

        publish_stamp = self.timebase.stamp_for_step(self.completed_steps)
        publish_sample, _ = self._sample_reference_from_stamp(publish_stamp)

        self.state_pub.publish(self._make_state_msg(publish_stamp))
        self.trajectory_pub.publish(
            self._make_trajectory_msg(publish_stamp, publish_sample)
        )
        self.control_pub.publish(self._make_control_msg(thrust, moment))

        self.log_counter += 1
        self.time_debug_counter += 1
        if self.log_counter % 250 == 0:
            self.get_logger().info(
                f'x=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f}), '
                f'xd=({self.xd[0]:.2f}, {self.xd[1]:.2f}, {self.xd[2]:.2f}), '
                f'eR=({e_R[0]:.2f}, {e_R[1]:.2f}, {e_R[2]:.2f}), '
                f'Omega_d_dot=({Omega_d_dot[0]:.2f}, '
                f'{Omega_d_dot[1]:.2f}, {Omega_d_dot[2]:.2f})'
            )
        if self.time_debug_counter % 500 == 0:
            self.get_logger().info(
                f'sim_time={self.completed_steps * self.dt:.4f}s, '
                f't_ref={t_ref:.4f}s'
            )


def main():
    rclpy.init()
    node = SimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
