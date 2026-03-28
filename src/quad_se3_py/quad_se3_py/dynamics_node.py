import rclpy
from rclpy.node import Node
import numpy as np

from quad_se3_msgs.msg import QuadState, ControlInput
from .utils import euler_to_rotmat, hat, project_to_so3, rotmat_to_quat


class DynamicsNode(Node):
    def __init__(self):
        super().__init__('dynamics_node')

        self.sub_u = self.create_subscription(
            ControlInput, '/control_input', self.control_cb, 10
        )

        self.pub = self.create_publisher(QuadState, '/quad_state', 10)

        self.m = 1.0
        self.g = 9.81
        self.J = np.diag([0.02, 0.02, 0.04])
        self.J_inv = np.linalg.inv(self.J)
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.declare_parameter('initial_position', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_velocity', [0.0, 0.0, 0.0])
        self.declare_parameter('initial_roll_deg', 0.0)
        self.declare_parameter('initial_pitch_deg', 0.0)
        self.declare_parameter('initial_yaw_deg', 0.0)
        self.declare_parameter('initial_angular_velocity', [0.0, 0.0, 0.0])

        self.x = np.array(
            self.get_parameter('initial_position').value,
            dtype=float,
        )
        self.v = np.array(
            self.get_parameter('initial_velocity').value,
            dtype=float,
        )
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

        self.M = np.zeros(3)
        self.f = self.m * self.g

        self.dt = 0.002
        self.log_counter = 0
        self.timer = self.create_timer(self.dt, self.update)

    def control_cb(self, msg):
        self.f = float(msg.thrust)
        self.M = np.array([
            msg.moment.x,
            msg.moment.y,
            msg.moment.z
        ], dtype=float)

    def update(self):
        xdot = self.v
        vdot = self.g * self.e3 - (self.f / self.m) * (self.R @ self.e3)
        Rdot = self.R @ hat(self.Omega)
        Omegadot = self.J_inv @ (
            self.M - np.cross(self.Omega, self.J @ self.Omega)
        )

        self.x += xdot * self.dt
        self.v += vdot * self.dt
        self.R += Rdot * self.dt
        self.R = project_to_so3(self.R)
        self.Omega += Omegadot * self.dt

        q = rotmat_to_quat(self.R)

        msg = QuadState()
        msg.stamp = self.get_clock().now().to_msg()

        msg.position.x, msg.position.y, msg.position.z = self.x
        msg.velocity.x, msg.velocity.y, msg.velocity.z = self.v
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = self.Omega

        self.pub.publish(msg)

        self.log_counter += 1
        if self.log_counter % 500 == 0:
            b3 = self.R @ self.e3
            self.get_logger().info(
                f'x=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f}), '
                f'b3=({b3[0]:.2f}, {b3[1]:.2f}, {b3[2]:.2f}), '
                f'Omega=({self.Omega[0]:.2f}, {self.Omega[1]:.2f}, {self.Omega[2]:.2f})'
            )


def main():
    rclpy.init()
    node = DynamicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
