import rclpy
from rclpy.node import Node

from quad_se3_msgs.msg import TrajectoryPoint
from .trajectories import evaluate_trajectory


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.pub = self.create_publisher(TrajectoryPoint, '/trajectory', 10)
        self.declare_parameter('trajectory_mode', 'hover')
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)
        self.t = 0.0
        self.trajectory_mode = self.get_parameter('trajectory_mode').value

    def update(self):
        msg = TrajectoryPoint()
        msg.stamp = self.get_clock().now().to_msg()

        sample = evaluate_trajectory(self.trajectory_mode, self.t)

        msg.position.x, msg.position.y, msg.position.z = sample['position']
        msg.velocity.x, msg.velocity.y, msg.velocity.z = sample['velocity']
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = sample['acceleration']
        msg.b1d.x, msg.b1d.y, msg.b1d.z = sample['b1d']
        msg.b1d_dot.x, msg.b1d_dot.y, msg.b1d_dot.z = sample['b1d_dot']
        msg.omega_d.x, msg.omega_d.y, msg.omega_d.z = sample['omega_d']
        msg.omega_dot_d.x, msg.omega_dot_d.y, msg.omega_dot_d.z = sample['omega_dot_d']

        self.pub.publish(msg)
        self.t += self.dt


def main():
    rclpy.init()
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
