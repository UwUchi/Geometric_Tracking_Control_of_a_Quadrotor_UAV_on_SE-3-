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
        self.yaw_rate = 0.6

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

        yaw = self.yaw_rate * self.t
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        msg.b1d.x = float(cos_yaw)
        msg.b1d.y = float(sin_yaw)
        msg.b1d.z = 0.0

        msg.b1d_dot.x = float(-self.yaw_rate * sin_yaw)
        msg.b1d_dot.y = float(self.yaw_rate * cos_yaw)
        msg.b1d_dot.z = 0.0

        msg.omega_d.x = 0.0
        msg.omega_d.y = 0.0
        msg.omega_d.z = float(self.yaw_rate)

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
