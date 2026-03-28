import rclpy
from rclpy.node import Node
import numpy as np

from quad_se3_msgs.msg import QuadState, ControlInput, TrajectoryPoint
from .utils import normalize, vee, quat_to_rotmat, hat, compute_Rd_and_derivatives


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.sub_state = self.create_subscription(
            QuadState, '/quad_state', self.state_cb, 10
        )
        self.sub_traj = self.create_subscription(
            TrajectoryPoint, '/trajectory', self.traj_cb, 10
        )
        self.pub = self.create_publisher(ControlInput, '/control_input', 10)

        self.m = 1.0
        self.g = 9.81
        self.J = np.diag([0.02, 0.02, 0.04])
        self.e3 = np.array([0.0, 0.0, 1.0])

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.R = np.eye(3)
        self.Omega = np.zeros(3)

        self.xd = np.zeros(3)
        self.vd = np.zeros(3)
        self.xdd = np.zeros(3)
        self.b1d = np.array([1.0, 0.0, 0.0])
        self.Omega_d_ref = np.zeros(3)
        self.Omega_dot_d_ref = np.zeros(3)

        self.kx = 8.0
        self.kv = 5.0
        self.kR = 4.0
        self.kOmega = 0.8

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)
        self.log_counter = 0

    def state_cb(self, msg):
        self.x = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        self.v = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        self.R = quat_to_rotmat(np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ], dtype=float))
        self.Omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=float)

    def traj_cb(self, msg):
        self.xd = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        self.vd = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        self.xdd = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], dtype=float)
        self.b1d = normalize(np.array([msg.b1d.x, msg.b1d.y, msg.b1d.z], dtype=float))
        self.Omega_d_ref = np.array([msg.omega_d.x, msg.omega_d.y, msg.omega_d.z], dtype=float)
        self.Omega_dot_d_ref = np.array([msg.omega_dot_d.x, msg.omega_dot_d.y, msg.omega_dot_d.z], dtype=float)


    def update(self):
        ex = self.x - self.xd
        ev = self.v - self.vd

        A = -self.kx * ex - self.kv * ev - self.m * self.g * self.e3 + self.m * self.xdd

        # 如果 A 太小，说明不需要太大推力，直接让 b3d 指向 z 轴，避免数值不稳定
        if np.linalg.norm(A) < 1e-6:
            b3d = np.array([0.0, 0.0, 1.0])
            f = self.m * self.g
        else:
            b3d = -normalize(A)        # 注意这里是 -A，因为A 是期望的总推力，而 b3d 是机体 z 轴的方向，二者是反向的


        Rd, Rd_dot, Omega_d_geom, Omega_dot_d_geom = compute_Rd_and_derivatives(
            b3d, self.b1d
        )

        # 这里先优先用几何构造结果；若你后面把参考角速度显式算好，也可以替换
        Omega_d = Omega_d_geom
        Omega_dot_d = Omega_dot_d_geom

        e_R = 0.5 * vee(Rd.T @ self.R - self.R.T @ Rd)
        e_Omega = self.Omega - self.R.T @ Rd @ Omega_d

        f = -np.dot(A, self.R @ self.e3)

        M = (
            -self.kR * e_R
            -self.kOmega * e_Omega
            + np.cross(self.Omega, self.J @ self.Omega)
            - self.J @ (
                hat(self.Omega) @ self.R.T @ Rd @ Omega_d
                - self.R.T @ Rd @ Omega_dot_d
            )
        )

        msg = ControlInput()
        msg.thrust = float(max(0.0, f))
        msg.moment.x = float(M[0])
        msg.moment.y = float(M[1])
        msg.moment.z = float(M[2])
        self.pub.publish(msg)

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
