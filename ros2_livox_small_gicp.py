#!/usr/bin/env python3
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster

import small_gicp


def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    points_raw = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z"),
        skip_nans=True,
    )

    # ROS2 distro/version differences:
    # - some return iterator of tuples
    # - some return structured numpy array
    if isinstance(points_raw, np.ndarray):
        if points_raw.dtype.fields is not None:
            if not {"x", "y", "z"}.issubset(points_raw.dtype.fields):
                return np.empty((0, 3), dtype=np.float64)
            points = np.column_stack(
                (points_raw["x"], points_raw["y"], points_raw["z"])
            ).astype(np.float64, copy=False)
        else:
            points = np.asarray(points_raw, dtype=np.float64)
    else:
        points = np.asarray(list(points_raw), dtype=np.float64)

    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if points.ndim == 1:
        points = points.reshape(1, -1)
    return points[:, :3]


def rotmat_to_quaternion_xyzw(rot: np.ndarray) -> np.ndarray:
    q = np.empty(4, dtype=np.float64)
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (rot[2, 1] - rot[1, 2]) / s
        q[1] = (rot[0, 2] - rot[2, 0]) / s
        q[2] = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        q[3] = (rot[2, 1] - rot[1, 2]) / s
        q[0] = 0.25 * s
        q[1] = (rot[0, 1] + rot[1, 0]) / s
        q[2] = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        q[3] = (rot[0, 2] - rot[2, 0]) / s
        q[0] = (rot[0, 1] + rot[1, 0]) / s
        q[1] = 0.25 * s
        q[2] = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        q[3] = (rot[1, 0] - rot[0, 1]) / s
        q[0] = (rot[0, 2] + rot[2, 0]) / s
        q[1] = (rot[1, 2] + rot[2, 1]) / s
        q[2] = 0.25 * s

    norm = np.linalg.norm(q)
    if norm > 0:
        q /= norm
    return q  # x, y, z, w


class LivoxSmallGICPNode(Node):
    def __init__(self) -> None:
        super().__init__("livox_small_gicp_node")

        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("registration_type", "GICP")
        self.declare_parameter("downsampling_resolution", 0.20)
        self.declare_parameter("max_correspondence_distance", 1.50)
        self.declare_parameter("num_threads", 4)
        self.declare_parameter("min_points", 200)
        self.declare_parameter("log_every_n", 1)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("child_frame", "livox")
        self.declare_parameter("max_path_size", 2000)
        self.declare_parameter("use_cloud_frame_as_child", True)

        self.lidar_topic = self.get_parameter("lidar_topic").get_parameter_value().string_value
        self.registration_type = self.get_parameter("registration_type").get_parameter_value().string_value
        self.downsampling_resolution = (
            self.get_parameter("downsampling_resolution").get_parameter_value().double_value
        )
        self.max_correspondence_distance = (
            self.get_parameter("max_correspondence_distance").get_parameter_value().double_value
        )
        self.num_threads = self.get_parameter("num_threads").get_parameter_value().integer_value
        self.min_points = self.get_parameter("min_points").get_parameter_value().integer_value
        self.log_every_n = self.get_parameter("log_every_n").get_parameter_value().integer_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.child_frame = self.get_parameter("child_frame").get_parameter_value().string_value
        self.max_path_size = self.get_parameter("max_path_size").get_parameter_value().integer_value
        self.use_cloud_frame_as_child = (
            self.get_parameter("use_cloud_frame_as_child").get_parameter_value().bool_value
        )

        self.prev_points: np.ndarray | None = None
        self.pose = np.eye(4, dtype=np.float64)
        self.frame_idx = 0
        self.aligned_count = 0
        self.odom_pub = self.create_publisher(Odometry, "small_gicp/odom", 10)
        self.path_pub = self.create_publisher(Path, "small_gicp/path", 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("small_gicp ROS2 node started")
        self.get_logger().info(
            f"topic={self.lidar_topic}, type={self.registration_type}, "
            f"downsampling={self.downsampling_resolution:.3f}, max_corr={self.max_correspondence_distance:.3f}, "
            f"threads={self.num_threads}"
        )
        self.get_logger().info(
            f"publish odom=small_gicp/odom path=small_gicp/path frame={self.map_frame} child={self.child_frame}"
        )

    def lidar_callback(self, msg: PointCloud2) -> None:
        self.frame_idx += 1
        curr_points = pointcloud2_to_xyz(msg)

        if curr_points.shape[0] < self.min_points:
            self.get_logger().warn(
                f"frame={self.frame_idx}: too few points ({curr_points.shape[0]} < {self.min_points}), skip"
            )
            return

        if self.prev_points is None:
            self.prev_points = curr_points
            self.get_logger().info(f"frame={self.frame_idx}: initialized with {curr_points.shape[0]} points")
            self.publish_pose(msg)
            return

        try:
            result = small_gicp.align(
                self.prev_points,
                curr_points,
                init_T_target_source=np.eye(4, dtype=np.float64),
                registration_type=self.registration_type,
                downsampling_resolution=self.downsampling_resolution,
                max_correspondence_distance=self.max_correspondence_distance,
                num_threads=self.num_threads,
            )
        except Exception as exc:
            self.get_logger().warn(f"frame={self.frame_idx}: align failed: {exc}")
            self.prev_points = curr_points
            return

        # result.T_target_source is target<-source. target=prev, source=curr.
        self.pose = self.pose @ result.T_target_source
        self.aligned_count += 1

        if self.log_every_n <= 1 or (self.aligned_count % self.log_every_n == 0):
            delta_t = result.T_target_source[:3, 3]
            num_inliers = getattr(result, "num_inliers", -1)
            iterations = getattr(result, "iterations", -1)
            error_value = getattr(result, "e", None)
            if error_value is None:
                error_value = getattr(result, "error", None)
            if error_value is None:
                error_text = "n/a"
            else:
                error_text = f"{float(error_value):.6f}"
            self.get_logger().info(
                f"frame={self.frame_idx} aligned={self.aligned_count} "
                f"inliers={num_inliers} "
                f"iter={iterations} "
                f"error={error_text} "
                f"delta_xyz=({delta_t[0]:+.3f}, {delta_t[1]:+.3f}, {delta_t[2]:+.3f})"
            )

        self.publish_pose(msg)
        self.prev_points = curr_points

    def publish_pose(self, cloud_msg: PointCloud2) -> None:
        pos = self.pose[:3, 3]
        quat = rotmat_to_quaternion_xyzw(self.pose[:3, :3])
        child_frame_id = self.child_frame
        if self.use_cloud_frame_as_child and cloud_msg.header.frame_id:
            child_frame_id = cloud_msg.header.frame_id

        odom = Odometry()
        odom.header.stamp = cloud_msg.header.stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = child_frame_id
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        self.odom_pub.publish(odom)

        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.path_msg.header.stamp = cloud_msg.header.stamp
        self.path_msg.poses.append(pose_stamped)
        if len(self.path_msg.poses) > self.max_path_size:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_size :]
        self.path_pub.publish(self.path_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = cloud_msg.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = child_frame_id
        tf_msg.transform.translation.x = float(pos[0])
        tf_msg.transform.translation.y = float(pos[1])
        tf_msg.transform.translation.z = float(pos[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = LivoxSmallGICPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
