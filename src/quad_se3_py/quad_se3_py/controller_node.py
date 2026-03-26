import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sub_state = self.create_subscription(
            Vector3, '/state', self.state_cb, 10
        )
        self.sub_velocity = self.create_subscription(
            Vector3, '/velocity', self.velocity_cb, 10
        )
        self.sub_xd = self.create_subscription(
            Vector3, '/xd', self.xd_cb, 10
        )
        self.sub_vd = self.create_subscription(
            Vector3, '/vd', self.vd_cb, 10
        )

        self.pub_force = self.create_publisher(Vector3, '/force', 10)

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.xd = np.zeros(3)
        self.vd = np.zeros(3)

        self.kx = 4.0
        self.kv = 3.0

        self.log_counter = 0
        self.timer = self.create_timer(0.01, self.update)

    def state_cb(self, msg):
        self.x = np.array([msg.x, msg.y, msg.z], dtype=float)

    def velocity_cb(self, msg):
        self.v = np.array([msg.x, msg.y, msg.z], dtype=float)

    def xd_cb(self, msg):
        self.xd = np.array([msg.x, msg.y, msg.z], dtype=float)

    def vd_cb(self, msg):
        self.vd = np.array([msg.x, msg.y, msg.z], dtype=float)

    def update(self):
        g = 9.81
        e3 = np.array([0.0, 0.0, 1.0])

        ex = self.x - self.xd
        ev = self.v - self.vd

        u = -self.kx * ex - self.kv * ev + g * e3

        msg = Vector3()
        msg.x, msg.y, msg.z = u
        self.pub_force.publish(msg)

        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f'ex=({ex[0]:.2f}, {ex[1]:.2f}, {ex[2]:.2f}), '
                f'ev=({ev[0]:.2f}, {ev[1]:.2f}, {ev[2]:.2f}), '
                f'u=({u[0]:.2f}, {u[1]:.2f}, {u[2]:.2f})'
            )


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()