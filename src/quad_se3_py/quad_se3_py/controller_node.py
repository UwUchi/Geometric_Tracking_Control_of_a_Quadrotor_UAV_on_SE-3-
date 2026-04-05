import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import numpy as np

from quad_se3_msgs.msg import QuadState, ControlInput
from .reference import compute_desired_attitude_and_force_from_state
from .timing import (
    stamp_to_seconds,
    trajectory_time_from_stamp,
)
from .trajectories import evaluate_trajectory
from .utils import hat, normalize, quat_to_rotmat, vee


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sub_state = self.create_subscription(
            QuadState, '/quad_state', self.state_cb, 10
        )
        self.sub_epoch = self.create_subscription(
            Time,
            '/trajectory_epoch',
            self.epoch_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.pub = self.create_publisher(ControlInput, '/control_input', 10)
        self.declare_parameter('trajectory_mode', 'hover')
        self.declare_parameter('trajectory_start_time_sec', 0.0)
        self.declare_parameter('reference_time_offset_sec', 0.0)

        self.m = 4.34
        self.g = 9.81
        self.J = np.diag([0.0820, 0.0845, 0.1377])
        self.e3 = np.array([0.0, 0.0, 1.0])
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.trajectory_start_time_sec = float(
            self.get_parameter('trajectory_start_time_sec').value
        )
        self.reference_time_offset_sec = float(
            self.get_parameter('reference_time_offset_sec').value
        )
        self.have_epoch = self.trajectory_start_time_sec > 0.0

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.vdot = np.zeros(3)  # 为了解析计算 Rd_dot 和 Omega_d，增加 vdot 状态。
        self.R = np.eye(3)
        self.Omega = np.zeros(3)
        self.state_stamp = None
        self.have_state = False
        self.time_debug_counter = 0

        self.xd = np.zeros(3)
        self.vd = np.zeros(3)
        self.xd_dd = np.zeros(3)
        self.xd_ddd = np.zeros(3)
        self.b1d = np.array([1.0, 0.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.Omega_d = np.zeros(3)
        self.Omega_d_prev = np.zeros(3)  # 用于数值差分计算 Omega_d_dot
        self.Omega_d_dot = np.zeros(3)
        self.Rd = np.eye(3)

        self.kx = 16.0 * self.m
        self.kv = 5.6 * self.m
        self.kR = 8.81
        self.kOmega = 2.54

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)
        self.log_counter = 0

    def epoch_cb(self, msg):
        self.trajectory_start_time_sec = stamp_to_seconds(msg)
        self.have_epoch = True

    def state_cb(self, msg):
        self.x = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        self.v = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        self.vdot = np.array(
            [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            dtype=float,
        )

        self.R = quat_to_rotmat(np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ], dtype=float))
        self.Omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=float)
        self.state_stamp = msg.stamp
        self.have_state = True

    def _update_reference_from_state_stamp(self):
        if self.state_stamp is None:
            return

        t = trajectory_time_from_stamp(
            self.state_stamp,
            self.trajectory_start_time_sec,
            self.reference_time_offset_sec,
        )
        sample = evaluate_trajectory(self.trajectory_mode, t)
        self.xd = np.array(sample['position'], dtype=float)
        self.vd = np.array(sample['velocity'], dtype=float)
        self.xd_dd = np.array(sample['acceleration'], dtype=float)
        self.xd_ddd = np.array(sample['jerk'], dtype=float)
        self.b1d = normalize(np.array(sample['b1d'], dtype=float))
        self.b1d_dot = np.array(sample['b1d_dot'], dtype=float)

    def update(self):
        if not self.have_state:
            if self.log_counter % 100 == 0:
                self.get_logger().warn('Waiting for the first /quad_state sample.')
            self.log_counter += 1
            return
        if not self.have_epoch:
            if self.log_counter % 100 == 0:
                self.get_logger().warn('Waiting for /trajectory_epoch before computing reference.')
            self.log_counter += 1
            return

        self._update_reference_from_state_stamp()
        Rd, _, Omega_d, Omega_d_dot, A = compute_desired_attitude_and_force_from_state(
            x=self.x,
            v=self.v,
            x_dd=self.vdot,  # 直接用 vdot 计算 Rd_dot 和 Omega_d，避免数值差分噪声。
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
            kx=self.kx,
            kv=self.kv,
        )
        self.Rd = Rd
        self.Omega_d_prev = Omega_d

        e_R = 0.5 * vee(Rd.T @ self.R - self.R.T @ Rd)
        e_Omega = self.Omega - self.R.T @ Rd @ Omega_d

        f = -np.dot(A, self.R @ self.e3)

        M = (
            -self.kR * e_R
            - self.kOmega * e_Omega
            + np.cross(self.Omega, self.J @ self.Omega)
            - self.J @ (
                hat(self.Omega) @ self.R.T @ Rd @ Omega_d
                - self.R.T @ Rd @ Omega_d_dot
            )
        )

        msg = ControlInput()
        # msg.thrust = float(max(0.0, f))
        msg.thrust = float(f)  # 直接使用 f，允许负值以测试控制器的行为。
        msg.moment.x = float(M[0])
        msg.moment.y = float(M[1])
        msg.moment.z = float(M[2])
        self.pub.publish(msg)

        self.log_counter += 1
        self.time_debug_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f'x=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f}), '
                f'xd=({self.xd[0]:.2f}, {self.xd[1]:.2f}, {self.xd[2]:.2f}), '
                f'b1d_dot=({self.b1d_dot[0]:.2f}, {self.b1d_dot[1]:.2f}, {self.b1d_dot[2]:.2f}), '
                f'eR=({e_R[0]:.2f}, {e_R[1]:.2f}, {e_R[2]:.2f}), '
                f'Omega_d_dot=({Omega_d_dot[0]:.2f}, {Omega_d_dot[1]:.2f}, {Omega_d_dot[2]:.2f}), '
            )
        if self.time_debug_counter % 200 == 0 and self.state_stamp is not None:
            state_stamp_sec = stamp_to_seconds(self.state_stamp)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            t_ref = trajectory_time_from_stamp(
                self.state_stamp,
                self.trajectory_start_time_sec,
                self.reference_time_offset_sec,
            )
            self.get_logger().info(
                f'state age={now_sec - state_stamp_sec:.4f}s, '
                f't_ref={t_ref:.4f}s'
            )


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
