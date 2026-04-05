import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from quad_se3_msgs.msg import TrajectoryPoint
from .timing import stamp_to_seconds, trajectory_time_from_stamp
from .trajectories import evaluate_trajectory


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        self.pub = self.create_publisher(TrajectoryPoint, '/trajectory', 10)
        self.epoch_sub = self.create_subscription(
            Time,
            '/trajectory_epoch',
            self.epoch_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.declare_parameter('trajectory_mode', 'hover')
        self.declare_parameter('trajectory_start_time_sec', 0.0)
        self.declare_parameter('reference_time_offset_sec', 0.0)
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.update)
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.trajectory_start_time_sec = float(
            self.get_parameter('trajectory_start_time_sec').value
        )
        self.reference_time_offset_sec = float(
            self.get_parameter('reference_time_offset_sec').value
        )
        self.have_epoch = self.trajectory_start_time_sec > 0.0

    def epoch_cb(self, msg):
        self.trajectory_start_time_sec = stamp_to_seconds(msg)
        self.have_epoch = True

    def update(self):
        if not self.have_epoch:
            return
        now = self.get_clock().now()
        t = trajectory_time_from_stamp(
            now.to_msg(),
            self.trajectory_start_time_sec,
            self.reference_time_offset_sec,
        )

        msg = TrajectoryPoint()
        msg.stamp = now.to_msg()

        sample = evaluate_trajectory(self.trajectory_mode, t)

        msg.position.x, msg.position.y, msg.position.z = sample['position']
        msg.velocity.x, msg.velocity.y, msg.velocity.z = sample['velocity']
        msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = sample['acceleration']
        msg.jerk.x, msg.jerk.y, msg.jerk.z = sample['jerk']
        msg.b1d.x, msg.b1d.y, msg.b1d.z = sample['b1d']
        msg.b1d_dot.x, msg.b1d_dot.y, msg.b1d_dot.z = sample['b1d_dot']

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
