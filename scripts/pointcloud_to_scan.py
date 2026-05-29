#!/usr/bin/env python3
"""
LIGO /cloud_registered (world-frame PointCloud2) → /scan (LaserScan, aft_mapped frame).

TF를 통해 포인트클라우드를 aft_mapped 프레임으로 변환한 뒤,
지면 높이 부근의 포인트만 추출하여 2D LaserScan으로 발행한다.
stamp는 원본 PointCloud2와 동일하게 유지하여 TF 타이밍과 일치시킨다.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2

import tf2_ros


def _quat_to_rot(w, x, y, z):
    """Quaternion (w,x,y,z) → 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def _tf_to_matrix(tf_stamped):
    """geometry_msgs/TransformStamped → 4x4 numpy 변환 행렬."""
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = _quat_to_rot(q.w, q.x, q.y, q.z)
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_scan")

        self.declare_parameter("target_frame", "aft_mapped")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("z_min", -0.1)
        self.declare_parameter("z_max", 0.5)
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 20.0)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("tf_tolerance", 0.1)

        self._target_frame = self.get_parameter("target_frame").value
        self._z_min = self.get_parameter("z_min").value
        self._z_max = self.get_parameter("z_max").value
        self._range_min = self.get_parameter("range_min").value
        self._range_max = self.get_parameter("range_max").value
        self._angle_min = self.get_parameter("angle_min").value
        self._angle_max = self.get_parameter("angle_max").value
        self._angle_inc = self.get_parameter("angle_increment").value
        self._tf_tolerance = self.get_parameter("tf_tolerance").value

        self._n_beams = int(
            (self._angle_max - self._angle_min) / self._angle_inc
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._scan_pub = self.create_publisher(
            LaserScan,
            self.get_parameter("scan_topic").value,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_topic").value,
            self._cloud_cb,
            qos,
        )
        self.get_logger().info(
            f"pointcloud_to_scan: {self.get_parameter('cloud_topic').value} "
            f"-> {self.get_parameter('scan_topic').value} "
            f"(frame: {self._target_frame})"
        )

    def _cloud_cb(self, msg: PointCloud2):
        stamp = msg.header.stamp

        pts_structured = np.array(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )))
        if pts_structured.size == 0:
            return
        points = np.column_stack([
            pts_structured["x"].astype(np.float64),
            pts_structured["y"].astype(np.float64),
            pts_structured["z"].astype(np.float64),
        ])

        if msg.header.frame_id != self._target_frame:
            try:
                tf_stamped = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    msg.header.frame_id,
                    stamp,
                    rclpy.duration.Duration(seconds=self._tf_tolerance),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warn(
                    f"TF failed ({msg.header.frame_id} -> {self._target_frame}): {e}",
                    throttle_duration_sec=2.0,
                )
                return

            mat = _tf_to_matrix(tf_stamped)
            ones = np.ones((points.shape[0], 1), dtype=np.float64)
            pts_h = np.hstack([points, ones])
            transformed = (mat @ pts_h.T).T
            points = transformed[:, :3]

        mask = (points[:, 2] >= self._z_min) & (points[:, 2] <= self._z_max)
        pts = points[mask]
        if pts.shape[0] == 0:
            return

        ranges = np.full(self._n_beams, float("inf"), dtype=np.float32)
        dists = np.hypot(pts[:, 0], pts[:, 1]).astype(np.float32)
        angles = np.arctan2(pts[:, 1], pts[:, 0]).astype(np.float32)

        valid = (
            (dists >= self._range_min)
            & (dists <= self._range_max)
            & (angles >= self._angle_min)
            & (angles < self._angle_max)
        )
        dists = dists[valid]
        angles = angles[valid]

        indices = ((angles - self._angle_min) / self._angle_inc).astype(np.int32)
        np.clip(indices, 0, self._n_beams - 1, out=indices)
        np.minimum.at(ranges, indices, dists)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self._target_frame
        scan.angle_min = self._angle_min
        scan.angle_max = self._angle_max
        scan.angle_increment = self._angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = ranges.tolist()

        self._scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
