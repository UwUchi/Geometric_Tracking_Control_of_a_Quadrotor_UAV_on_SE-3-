import rclpy
from rclpy.node import Node
import numpy as np

from quad_se3_msgs.msg import TrajectoryPoint


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.pub = self.create_publisher(TrajectoryPoint, '/trajectory', 10)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)
        self.t = 0.0

    def update(self):
        msg = TrajectoryPoint()
        msg.stamp = self.get_clock().now().to_msg()

        # 先用悬停点
        msg.position.x = 0.5
        msg.position.y = 0.3
        msg.position.z = 5.0

        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0

        msg.acceleration.x = 0.0
        msg.acceleration.y = 0.0
        msg.acceleration.z = 0.0

        msg.b1d.x = 1.0
        msg.b1d.y = 0.0
        msg.b1d.z = 0.0

        msg.omega_d.x = 0.0
        msg.omega_d.y = 0.0
        msg.omega_d.z = 0.0

        msg.omega_dot_d.x = 0.0
        msg.omega_dot_d.y = 0.0
        msg.omega_dot_d.z = 0.0

        self.pub.publish(msg)
        self.t += self.dt


def main():
    rclpy.init()
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()