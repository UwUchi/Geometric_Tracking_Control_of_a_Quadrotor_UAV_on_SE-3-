import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3

from .utils import normalize


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sub_state = self.create_subscription(Vector3, '/state', self.state_cb, 10)
        self.sub_velocity = self.create_subscription(Vector3, '/velocity', self.velocity_cb, 10)
        self.sub_xd = self.create_subscription(Vector3, '/xd', self.xd_cb, 10)
        self.sub_vd = self.create_subscription(Vector3, '/vd', self.vd_cb, 10)
        self.sub_b3 = self.create_subscription(Vector3, '/b3', self.b3_cb, 10)
        self.sub_omega = self.create_subscription(Vector3, '/omega', self.omega_cb, 10)

        self.pub_u = self.create_publisher(Vector3, '/force', 10)

        self.m = 1.0
        self.g = 9.81
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.xd = np.zeros(3)
        self.vd = np.zeros(3)
        self.b3 = np.array([0.0, 0.0, 1.0])
        self.Omega = np.zeros(3)

        self.kx = 4.0
        self.kv = 3.0
        self.kR_xy = 3.0
        self.kOmega_xy = 0.4

        self.timer = self.create_timer(0.01, self.update)
        self.log_counter = 0

    def state_cb(self, msg):
        self.x = np.array([msg.x, msg.y, msg.z], dtype=float)

    def velocity_cb(self, msg):
        self.v = np.array([msg.x, msg.y, msg.z], dtype=float)

    def xd_cb(self, msg):
        self.xd = np.array([msg.x, msg.y, msg.z], dtype=float)

    def vd_cb(self, msg):
        self.vd = np.array([msg.x, msg.y, msg.z], dtype=float)

    def b3_cb(self, msg):
        self.b3 = normalize(np.array([msg.x, msg.y, msg.z], dtype=float))

    def omega_cb(self, msg):
        self.Omega = np.array([msg.x, msg.y, msg.z], dtype=float)

    def update(self):
        ex = self.x - self.xd
        ev = self.v - self.vd

        # 论文里常写成 A = -kx ex - kv ev - m g e3 + m xdd
        # 这里先不加 xdd，写成 Fd 更直观
        Fd = -self.kx * ex - self.kv * ev + self.m * self.g * self.e3

        # 如果 Fd 太小，说明不需要太大推力，直接让 b3d 指向 z 轴，避免数值不稳定
        if np.linalg.norm(Fd) < 1e-6:
            b3d = np.array([0.0, 0.0, 1.0])
            f = self.m * self.g
        else:
            b3d = -normalize(Fd)
            f = -np.dot(Fd, self.b3)
            f = max(0.0, f)

        # 期望机体 z 轴方向
        b3d = -normalize(Fd)  # 注意这里是 -Fd，因为Fd 是期望的总推力，而 b3d 是机体 z 轴的方向，二者是反向的

        # 推力大小：让实际推力投影到当前 b3 上
        f = -np.dot(Fd, self.b3)
        f = max(0.0, f)

        # 先做简化的姿态控制：只对齐 b3 到 b3d
        # b3 × b3d 给出一个“倾斜误差”
        e_b3 = np.cross(self.b3, b3d)

        Mx = -self.kR_xy * e_b3[0] - self.kOmega_xy * self.Omega[0]
        My = -self.kR_xy * e_b3[1] - self.kOmega_xy * self.Omega[1]

        msg = Vector3()
        msg.x = Mx
        msg.y = My
        msg.z = f
        self.pub_u.publish(msg)

        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f'ex=({ex[0]:.2f}, {ex[1]:.2f}, {ex[2]:.2f}), '
                f'b3=({self.b3[0]:.2f}, {self.b3[1]:.2f}, {self.b3[2]:.2f}), '
                f'b3d=({b3d[0]:.2f}, {b3d[1]:.2f}, {b3d[2]:.2f}), '
                f'f={f:.2f}'
            )


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()