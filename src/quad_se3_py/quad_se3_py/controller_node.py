import rclpy
from rclpy.node import Node
import numpy as np

from quad_se3_msgs.msg import QuadState, ControlInput, TrajectoryPoint
from .reference import compute_desired_attitude_and_force_from_state
from .utils import hat, normalize, quat_to_rotmat, vee


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sub_state = self.create_subscription(
            QuadState, '/quad_state', self.state_cb, 10
        )
        self.sub_traj = self.create_subscription(
            TrajectoryPoint, '/trajectory', self.traj_cb, 10
        )
        self.pub = self.create_publisher(ControlInput, '/control_input', 10)

        self.m = 1.0
        self.g = 9.81
        self.J = np.diag([0.02, 0.02, 0.04])
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.vdot = np.zeros(3)  # 为了解析计算 Rd_dot 和 Omega_d，增加 vdot 状态。
        self.R = np.eye(3)
        self.Omega = np.zeros(3)

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

        self.kx = 8.0
        self.kv = 5.0
        self.kR = 4.0
        self.kOmega = 0.8

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)
        self.log_counter = 0

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

    def traj_cb(self, msg):
        self.xd = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        self.vd = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        self.xd_dd = np.array(
            [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            dtype=float,
        )
        self.xd_ddd = np.array(
            [msg.jerk.x, msg.jerk.y, msg.jerk.z],
            dtype=float,
        )
        self.b1d = normalize(np.array([msg.b1d.x, msg.b1d.y, msg.b1d.z], dtype=float))
        self.b1d_dot = np.array([msg.b1d_dot.x, msg.b1d_dot.y, msg.b1d_dot.z], dtype=float)

    def update(self):
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
        msg.thrust = float(max(0.0, f))
        msg.moment.x = float(M[0])
        msg.moment.y = float(M[1])
        msg.moment.z = float(M[2])
        self.pub.publish(msg)

        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f'x=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f}), '
                f'xd=({self.xd[0]:.2f}, {self.xd[1]:.2f}, {self.xd[2]:.2f}), '
                f'b1d_dot=({self.b1d_dot[0]:.2f}, {self.b1d_dot[1]:.2f}, {self.b1d_dot[2]:.2f}), '
                f'eR=({e_R[0]:.2f}, {e_R[1]:.2f}, {e_R[2]:.2f}), '
                f'Omega_d_dot=({Omega_d_dot[0]:.2f}, {Omega_d_dot[1]:.2f}, {Omega_d_dot[2]:.2f}), '
            )


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
