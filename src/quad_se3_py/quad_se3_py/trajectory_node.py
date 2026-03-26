import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.pub_xd = self.create_publisher(Vector3, '/xd', 10)
        self.pub_vd = self.create_publisher(Vector3, '/vd', 10)

        self.t = 0.0
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)

    def update(self):
        # 一个平滑的小轨迹
        xd = Vector3()
        vd = Vector3()

        xd.x = 0.5 * self.t
        xd.y = np.sin(self.t)
        xd.z = 1.0 + 0.5 * np.cos(self.t)

        vd.x = 0.5
        vd.y = np.cos(self.t)
        vd.z = -0.5 * np.sin(self.t)

        self.pub_xd.publish(xd)
        self.pub_vd.publish(vd)

        self.t += self.dt


def main():
    rclpy.init()
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()