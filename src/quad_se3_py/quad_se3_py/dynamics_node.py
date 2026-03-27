import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion

from .utils import hat, project_to_so3, rotmat_to_quat

class DynamicsNode(Node):
    def __init__(self):
        super().__init__('dynamics_node')

        self.sub_u = self.create_subscription(
            Vector3, '/force', self.force_cb, 10
        )

        self.pub_state = self.create_publisher(Vector3, '/state', 10)
        self.pub_velocity = self.create_publisher(Vector3, '/velocity', 10)
        self.pub_omega = self.create_publisher(Vector3, '/omega', 10)
        self.pub_orientation = self.create_publisher(Quaternion, '/orientation', 10)

        self.m = 1.0
        self.g = 9.81
        self.J = np.diag([0.02, 0.02, 0.04])
        self.J_inv = np.linalg.inv(self.J)
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.Omega = np.zeros(3)

        self.M = np.zeros(3)
        self.f = self.m * self.g

        self.dt = 0.002
        self.log_counter = 0
        self.timer = self.create_timer(self.dt, self.update)

    def force_cb(self, msg):
        self.M = np.array([msg.x, msg.y, 0.0], dtype=float)
        self.f = float(msg.z)

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

        msg = Vector3()
        msg.x, msg.y, msg.z = self.x
        self.pub_state.publish(msg)

        msg = Vector3()
        msg.x, msg.y, msg.z = self.v
        self.pub_velocity.publish(msg)

        msg = Vector3()
        msg.x, msg.y, msg.z = self.Omega
        self.pub_omega.publish(msg)

        q = rotmat_to_quat(self.R)
        qmsg = Quaternion()
        qmsg.x, qmsg.y, qmsg.z, qmsg.w = q
        self.pub_orientation.publish(qmsg)

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