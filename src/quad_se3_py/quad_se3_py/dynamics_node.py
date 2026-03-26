import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3


class DynamicsNode(Node):
    def __init__(self):
        super().__init__('dynamics_node')

        self.sub_force = self.create_subscription(
            Vector3, '/force', self.force_cb, 10
        )

        self.pub_state = self.create_publisher(Vector3, '/state', 10)
        self.pub_velocity = self.create_publisher(Vector3, '/velocity', 10)

        self.x = np.zeros(3)
        self.v = np.zeros(3)

        self.u = np.zeros(3)   # 控制器给出的“总加速度项”
        self.dt = 0.002

        self.log_counter = 0
        self.timer = self.create_timer(self.dt, self.update)

    def force_cb(self, msg):
        self.u = np.array([msg.x, msg.y, msg.z], dtype=float)

    def update(self):
        g = 9.81
        e3 = np.array([0.0, 0.0, 1.0])

        xdot = self.v
        vdot = self.u - g * e3 # 简单的点质量模型，u是总加速度项

        self.x += xdot * self.dt
        self.v += vdot * self.dt

        state_msg = Vector3()
        state_msg.x, state_msg.y, state_msg.z = self.x
        self.pub_state.publish(state_msg)

        vel_msg = Vector3()
        vel_msg.x, vel_msg.y, vel_msg.z = self.v
        self.pub_velocity.publish(vel_msg)

        self.log_counter += 1
        if self.log_counter % 500 == 0:
            self.get_logger().info(
                f'x=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f}), '
                f'v=({self.v[0]:.2f}, {self.v[1]:.2f}, {self.v[2]:.2f})'
            )


def main():
    rclpy.init()
    node = DynamicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()