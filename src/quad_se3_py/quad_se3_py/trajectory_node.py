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
        xd = Vector3()
        vd = Vector3()

        xd.x = 0.3 * np.sin(0.5 * self.t)
        xd.y = 0.3 * np.cos(0.5 * self.t)
        xd.z = 1.0 + 0.2 * np.sin(0.5 * self.t)

        vd.x = 0.15 * np.cos(0.5 * self.t)
        vd.y = -0.15 * np.sin(0.5 * self.t)
        vd.z = 0.1 * np.cos(0.5 * self.t)

        self.pub_xd.publish(xd)
        self.pub_vd.publish(vd)

        self.t += self.dt


def main():
    rclpy.init()
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()