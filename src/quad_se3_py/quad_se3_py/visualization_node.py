from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from quad_se3_msgs.msg import QuadState
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .reference import compute_desired_attitude_from_state
from .timing import (
    resolve_trajectory_start_time,
    stamp_to_seconds,
    trajectory_time_from_stamp,
)
from .trajectories import evaluate_trajectory
from .utils import quat_to_rotmat, rotmat_to_quat


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self.declare_parameter('path_max_points', 2000)
        self.declare_parameter('show_error_markers', True)
        self.declare_parameter('trajectory_mode', 'hover')
        self.declare_parameter('trajectory_start_time_sec', 0.0)
        self.declare_parameter('reference_time_offset_sec', 0.0)

        self.path_max_points = int(self.get_parameter('path_max_points').value)
        self.show_error_markers = bool(self.get_parameter('show_error_markers').value)
        self.trajectory_mode = self.get_parameter('trajectory_mode').value
        self.trajectory_start_time_sec = float(
            self.get_parameter('trajectory_start_time_sec').value
        )
        self.reference_time_offset_sec = float(
            self.get_parameter('reference_time_offset_sec').value
        )

        self.sub_state = self.create_subscription(
            QuadState, '/quad_state', self.state_cb, 10
        )

        self.actual_path_pub = self.create_publisher(Path, '/viz/actual_path', 10)
        self.desired_path_pub = self.create_publisher(Path, '/viz/desired_path', 10)
        self.actual_pose_pub = self.create_publisher(PoseStamped, '/viz/actual_pose', 10)
        self.desired_pose_pub = self.create_publisher(PoseStamped, '/viz/desired_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/markers', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.frame_id = 'world'
        self.actual_body_frame_id = 'quad_actual'
        self.m = 1.0
        self.g = 9.81
        self.kx = 8.0
        self.kv = 5.0
        self.e3 = np.array([0.0, 0.0, 1.0], dtype=float)

        self.x = np.zeros(3, dtype=float)
        self.v = np.zeros(3, dtype=float)
        self.x_dd = np.zeros(3, dtype=float)
        self.q = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.Rd = np.eye(3)

        self.xd = np.zeros(3, dtype=float)
        self.vd = np.zeros(3, dtype=float)
        self.xd_dd = np.zeros(3, dtype=float)
        self.xd_ddd = np.zeros(3, dtype=float)
        self.b1d = np.array([1.0, 0.0, 0.0], dtype=float)
        self.b1d_dot = np.zeros(3, dtype=float)

        self.actual_path_points = deque(maxlen=self.path_max_points)
        self.desired_path_points = deque(maxlen=self.path_max_points)
        self.time_debug_counter = 0

    def state_cb(self, msg):
        self.x = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        self.v = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        self.x_dd = np.array(
            [msg.acceleration.x, msg.acceleration.y, msg.acceleration.z],
            dtype=float,
        )
        self.q = np.array(
            [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ],
            dtype=float,
        )
        self._update_reference_from_state_stamp(msg.stamp)
        self.publish_visuals(msg.stamp)

    def _update_reference_from_state_stamp(self, stamp):
        if self.trajectory_start_time_sec <= 0.0:
            fallback_start_sec = stamp_to_seconds(stamp)
            self.trajectory_start_time_sec = resolve_trajectory_start_time(
                self.trajectory_start_time_sec,
                fallback_start_sec,
            )
            self.get_logger().warn(
                'trajectory_start_time_sec was not provided; '
                'falling back to the first state stamp.'
            )

        t = trajectory_time_from_stamp(
            stamp,
            self.trajectory_start_time_sec,
            self.reference_time_offset_sec,
        )
        sample = evaluate_trajectory(self.trajectory_mode, t)
        self.xd = np.array(sample['position'], dtype=float)
        self.vd = np.array(sample['velocity'], dtype=float)
        self.xd_dd = np.array(sample['acceleration'], dtype=float)
        self.xd_ddd = np.array(sample['jerk'], dtype=float)
        self.b1d = np.array(sample['b1d'], dtype=float)
        self.b1d_dot = np.array(sample['b1d_dot'], dtype=float)
        self.time_debug_counter += 1
        if self.time_debug_counter % 200 == 0:
            stamp_sec = stamp_to_seconds(stamp)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'state age={now_sec - stamp_sec:.4f}s, '
                f't_ref={t:.4f}s'
            )

    def publish_visuals(self, stamp):
        Rd, _, _, _, _ = compute_desired_attitude_from_state(
            x=self.x,
            v=self.v,
            xd=self.xd,
            vd=self.vd,
            xd_dd=self.xd_dd,
            b1d=self.b1d,
            b1d_dot=self.b1d_dot,
            current_Rd=self.Rd,
            m=self.m,
            g=self.g,
            e3=self.e3,
            kx=self.kx,
            kv=self.kv,
            x_dd=self.x_dd,
            xd_ddd=self.xd_ddd,
        )
        self.Rd = Rd
        desired_q = rotmat_to_quat(Rd)

        actual_pose = self._make_pose(stamp, self.x, self.q)
        desired_pose = self._make_pose(stamp, self.xd, desired_q)
        self.actual_pose_pub.publish(actual_pose)
        self.desired_pose_pub.publish(desired_pose)
        self._publish_actual_tf(stamp, self.x, self.q)

        self.actual_path_points.append(actual_pose)
        self.desired_path_points.append(desired_pose)

        self.actual_path_pub.publish(self._make_path(stamp, self.actual_path_points))
        self.desired_path_pub.publish(self._make_path(stamp, self.desired_path_points))
        self.marker_pub.publish(self._make_markers(stamp, Rd))

    def _publish_actual_tf(self, stamp, position, quaternion):
        msg = TransformStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.child_frame_id = self.actual_body_frame_id
        msg.transform.translation.x = float(position[0])
        msg.transform.translation.y = float(position[1])
        msg.transform.translation.z = float(position[2])
        msg.transform.rotation.x = float(quaternion[0])
        msg.transform.rotation.y = float(quaternion[1])
        msg.transform.rotation.z = float(quaternion[2])
        msg.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(msg)

    def _make_pose(self, stamp, position, quaternion):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])
        return msg

    def _make_path(self, stamp, path_points):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = stamp
        msg.poses = list(path_points)
        return msg

    def _make_markers(self, stamp, Rd):
        markers = MarkerArray()
        markers.markers.append(
            self._make_axis_marker(
                stamp=stamp,
                marker_id=0,
                origin=self.x,
                rotation_matrix=quat_to_rotmat(self.q),
                namespace='actual_axes',
                alpha=0.95,
            )
        )
        markers.markers.append(
            self._make_axis_marker(
                stamp=stamp,
                marker_id=1,
                origin=self.xd,
                rotation_matrix=Rd,
                namespace='desired_axes',
                alpha=0.75,
            )
        )
        markers.markers.append(self._make_error_marker(stamp, 2))
        return markers

    def _make_axis_marker(self, stamp, marker_id, origin, rotation_matrix, namespace, alpha):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.04

        axis_length = 0.45
        colors = (
            (1.0, 0.35, 0.2, alpha),
            (1.0, 0.75, 0.15, alpha),
            (1.0, 0.95, 0.3, alpha),
        ) if namespace == 'actual_axes' else (
            (0.2, 0.7, 1.0, alpha),
            (0.2, 1.0, 0.75, alpha),
            (0.55, 0.75, 1.0, alpha),
        )

        for axis_index in range(3):
            end_point = origin + axis_length * rotation_matrix[:, axis_index]
            marker.points.append(self._point_from_array(origin))
            marker.points.append(self._point_from_array(end_point))
            marker.colors.append(self._color_from_rgba(*colors[axis_index]))
            marker.colors.append(self._color_from_rgba(*colors[axis_index]))
        return marker

    def _make_error_marker(self, stamp, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = 'position_error'
        marker.id = marker_id
        marker.action = Marker.ADD
        if self.show_error_markers:
            marker.type = Marker.ARROW
            marker.scale.x = 0.05
            marker.scale.y = 0.09
            marker.scale.z = 0.12
            marker.color.r = 0.95
            marker.color.g = 0.15
            marker.color.b = 0.6
            marker.color.a = 0.95
            marker.points.append(self._point_from_array(self.xd))
            marker.points.append(self._point_from_array(self.x))
        else:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            marker.color.a = 0.0
        return marker

    def _point_from_array(self, xyz):
        point = Point()
        point.x = float(xyz[0])
        point.y = float(xyz[1])
        point.z = float(xyz[2])
        return point

    def _color_from_rgba(self, r, g, b, a):
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color


def main():
    rclpy.init()
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
