import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Vector3, Quaternion

from .utils import normalize, vee, quat_to_rotmat


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sub_state = self.create_subscription(Vector3, '/state', self.state_cb, 10)
        self.sub_velocity = self.create_subscription(Vector3, '/velocity', self.velocity_cb, 10)
        self.sub_xd = self.create_subscription(Vector3, '/xd', self.xd_cb, 10)
        self.sub_vd = self.create_subscription(Vector3, '/vd', self.vd_cb, 10)
        self.sub_omega = self.create_subscription(Vector3, '/omega', self.omega_cb, 10)
        self.sub_orientation = self.create_subscription(Quaternion, '/orientation', self.orientation_cb, 10)

        self.pub_u = self.create_publisher(Vector3, '/force', 10)

        self.m = 1.0
        self.g = 9.81
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.xd = np.zeros(3)
        self.vd = np.zeros(3)
        self.Omega = np.zeros(3)
        self.R = np.eye(3)  

        self.kx = 16.0
        self.kv = 6.0
        self.kR_xy = 3.0
        self.kOmega_xy = 0.5

        self.timer = self.create_timer(0.01, self.update)
        self.log_counter = 0
        self.waiting_log_counter = 0
        self.get_logger().info('controller_node started')

    def state_cb(self, msg):
        self.x = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.state_received = True

    def velocity_cb(self, msg):
        self.v = np.array([msg.x, msg.y, msg.z], dtype=float)

    def xd_cb(self, msg):
        self.xd = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.trajectory_received = True

    def vd_cb(self, msg):
        self.vd = np.array([msg.x, msg.y, msg.z], dtype=float)

    def omega_cb(self, msg):
        self.Omega = np.array([msg.x, msg.y, msg.z], dtype=float)

    def orientation_cb(self, msg):
        q = np.array([msg.x, msg.y, msg.z, msg.w], dtype=float)
        self.R = quat_to_rotmat(q)

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
            b3d = -normalize(Fd)        # 注意这里是 -Fd，因为Fd 是期望的总推力，而 b3d 是机体 z 轴的方向，二者是反向的
            f = -np.dot(Fd, self.R @ self.e3)    # 推力大小：让实际推力投影到当前 -b3 上
            f = max(0.0, f)

        # 固定期望航向
        b1d = np.array([1.0, 0.0, 0.0])

        # 构造 Rd
        c2 = normalize(np.cross(b3d, b1d))
        if np.linalg.norm(c2) < 1e-6:
            c2 = np.array([0.0, 1.0, 0.0])

        c1 = np.cross(c2, b3d)
        Rd = np.column_stack((c1, c2, b3d))

        e_R_mat = 0.5 * (Rd.T @ self.R - self.R.T @ Rd)
        e_R = vee(e_R_mat)

        e_Omega = self.Omega

        kR = np.array([self.kR_xy, self.kR_xy, 0.0])
        kOmega = np.array([self.kOmega_xy, self.kOmega_xy, 0.0])
        M = -kR * e_R - kOmega * e_Omega

        msg = Vector3()
        msg.x = M[0]
        msg.y = M[1]
        msg.z = f
        self.pub_u.publish(msg)

        self.log_counter += 1
        if self.log_counter % 100 == 0:
            self.get_logger().info(
                f'x=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f}), '
                f'xd=({self.xd[0]:.2f}, {self.xd[1]:.2f}, {self.xd[2]:.2f}), '
                f'ex=({ex[0]:.2f}, {ex[1]:.2f}, {ex[2]:.2f}), '
                f'eR=({e_R[0]:.2f}, {e_R[1]:.2f}, {e_R[2]:.2f}), '
                f'f={f:.2f}'
            )


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
