#!/usr/bin/env python3
"""Indoor scan-to-map GICP localizer using a pre-built PCD reference map.

This standalone ROS2 node is the Python counterpart to the C++ SmallGICPLocalizer.
It publishes:
  - /indoor/map_cloud        : reference map (latched, ENU frame)
  - /indoor/aligned_scan     : current scan aligned to map frame
  - /indoor/gicp_odom        : nav_msgs/Odometry in ENU frame
  - /indoor/gicp_path        : nav_msgs/Path

Usage (example):
  ros2 run ligo ros2_indoor_map_gicp.py \
      --ros-args \
      -p map_pcd_path:=/path/to/PCD/scans_3.pcd \
      -p lidar_topic:=/livox/lidar \
      -p map_frame:=enu

Dynamic indoor/outdoor control:
  ros2 topic pub /ligo/indoor_mode std_msgs/Bool "data: true"   # enter indoor
  ros2 topic pub /ligo/indoor_mode std_msgs/Bool "data: false"  # exit indoor
"""

import math
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool
import small_gicp


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def pointcloud2_to_xyz(msg: PointCloud2) -> np.ndarray:
    raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    if isinstance(raw, np.ndarray):
        if raw.dtype.fields is not None:
            pts = np.column_stack((raw["x"], raw["y"], raw["z"])).astype(np.float64)
        else:
            pts = np.asarray(raw, dtype=np.float64)
    else:
        pts = np.asarray(list(raw), dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if pts.ndim == 1:
        pts = pts.reshape(1, -1)
    return pts[:, :3]


def pcd_to_numpy(pcd_path: str) -> np.ndarray:
    """Minimal ASCII/binary PCD reader that returns Nx3 float64 array."""
    try:
        import pcl  # type: ignore
        cloud = pcl.load(pcd_path)
        return np.asarray(cloud, dtype=np.float64)[:, :3]
    except ImportError:
        pass

    # Fallback: parse PCD header manually
    points = []
    data_type = "ascii"
    width = height = 0
    fields = []
    with open(pcd_path, "rb") as f:
        for _ in range(50):  # header is < 50 lines
            line = f.readline()
            if not line:
                break
            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                break
            if text.startswith("FIELDS"):
                fields = text.split()[1:]
            elif text.startswith("WIDTH"):
                width = int(text.split()[1])
            elif text.startswith("HEIGHT"):
                height = int(text.split()[1])
            elif text.startswith("DATA"):
                data_type = text.split()[1].lower()
                body = f.read()
                break
        else:
            body = b""

    n_pts = width * height
    if n_pts == 0:
        return np.empty((0, 3), dtype=np.float64)

    try:
        xi = fields.index("x")
        yi = fields.index("y")
        zi = fields.index("z")
    except ValueError:
        xi, yi, zi = 0, 1, 2

    if data_type == "ascii":
        rows = [ln.split() for ln in body.decode("ascii", errors="ignore").split("\n") if ln.strip()]
        pts = np.array([[float(r[xi]), float(r[yi]), float(r[zi])] for r in rows if len(r) > max(xi, yi, zi)], dtype=np.float64)
    elif data_type in ("binary", "binary_compressed"):
        # 4 bytes per float field, assuming all-float32 (most common)
        n_fields = len(fields)
        row_bytes = n_fields * 4
        try:
            pts_raw = np.frombuffer(body[:n_pts * row_bytes], dtype=np.float32).reshape(n_pts, n_fields)
            pts = pts_raw[:, [xi, yi, zi]].astype(np.float64)
        except Exception:
            pts = np.empty((0, 3), dtype=np.float64)
    else:
        pts = np.empty((0, 3), dtype=np.float64)

    return pts


def rotmat_to_quaternion_xyzw(rot: np.ndarray) -> np.ndarray:
    q = np.empty(4, dtype=np.float64)
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q[3] = 0.25 * s
        q[0] = (rot[2, 1] - rot[1, 2]) / s
        q[1] = (rot[0, 2] - rot[2, 0]) / s
        q[2] = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        q[3] = (rot[2, 1] - rot[1, 2]) / s
        q[0] = 0.25 * s
        q[1] = (rot[0, 1] + rot[1, 0]) / s
        q[2] = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        q[3] = (rot[0, 2] - rot[2, 0]) / s
        q[0] = (rot[0, 1] + rot[1, 0]) / s
        q[1] = 0.25 * s
        q[2] = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        q[3] = (rot[1, 0] - rot[0, 1]) / s
        q[0] = (rot[0, 2] + rot[2, 0]) / s
        q[1] = (rot[1, 2] + rot[2, 1]) / s
        q[2] = 0.25 * s
    norm = np.linalg.norm(q)
    if norm > 1e-9:
        q /= norm
    return q


def xyz_to_pointcloud2(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    """Build a PointCloud2 from an Nx3 float32/float64 array."""
    from sensor_msgs.msg import PointField
    pts32 = pts.astype(np.float32)
    fields = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp    = stamp
    msg.height          = 1
    msg.width           = len(pts32)
    msg.fields          = fields
    msg.is_bigendian    = False
    msg.point_step      = 12
    msg.row_step        = 12 * len(pts32)
    msg.data            = pts32.tobytes()
    msg.is_dense        = True
    return msg


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class IndoorMapGICPNode(Node):
    def __init__(self) -> None:
        super().__init__("indoor_map_gicp_node")

        self.declare_parameter("lidar_topic",             "/livox/lidar")
        self.declare_parameter("map_pcd_path",            "")
        self.declare_parameter("map_frame",               "enu")
        self.declare_parameter("registration_type",       "GICP")
        self.declare_parameter("map_downsampling_res",    0.5)
        self.declare_parameter("scan_downsampling_res",   0.5)
        self.declare_parameter("max_correspondence_dist", 3.0)
        self.declare_parameter("num_threads",             4)
        self.declare_parameter("min_scan_points",         200)
        self.declare_parameter("max_path_size",           2000)
        self.declare_parameter("indoor_mode_topic",       "/ligo/indoor_mode")

        self.lidar_topic         = self.get_parameter("lidar_topic").value
        self.map_pcd_path        = self.get_parameter("map_pcd_path").value
        self.map_frame           = self.get_parameter("map_frame").value
        self.registration_type   = self.get_parameter("registration_type").value
        self.map_ds_res          = float(self.get_parameter("map_downsampling_res").value)
        self.scan_ds_res         = float(self.get_parameter("scan_downsampling_res").value)
        self.max_corr_dist       = float(self.get_parameter("max_correspondence_dist").value)
        self.num_threads         = int(self.get_parameter("num_threads").value)
        self.min_scan_points     = int(self.get_parameter("min_scan_points").value)
        self.max_path_size       = int(self.get_parameter("max_path_size").value)

        # State
        self.is_indoor   = False
        self.map_points  = None   # np.ndarray Nx3 in map frame
        self.T_map_lidar = np.eye(4, dtype=np.float64)  # accumulated pose
        self.map_published = False

        # Load map
        if self.map_pcd_path:
            self._load_map(self.map_pcd_path)

        # Publishers
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub_map_cloud    = self.create_publisher(PointCloud2, "/indoor/map_cloud",    latched_qos)
        self.pub_aligned_scan = self.create_publisher(PointCloud2, "/indoor/aligned_scan", 10)
        self.pub_odom         = self.create_publisher(Odometry,    "/indoor/gicp_odom",    10)
        self.pub_path         = self.create_publisher(Path,        "/indoor/gicp_path",    10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        # Subscriptions
        indoor_topic = str(self.get_parameter("indoor_mode_topic").value)
        self.create_subscription(Bool, indoor_topic, self._on_indoor_flag, 10)
        self.sub_lidar = self.create_subscription(
            PointCloud2, self.lidar_topic, self._on_lidar, qos_profile_sensor_data)

        self.get_logger().info(
            f"IndoorMapGICP started: map={self.map_pcd_path!r} frame={self.map_frame}"
        )

    # ------------------------------------------------------------------
    def _load_map(self, path: str) -> None:
        self.get_logger().info(f"[gicp] loading map: {path}")
        pts = pcd_to_numpy(path)
        if pts.shape[0] == 0:
            self.get_logger().error(f"[gicp] map load failed or empty: {path}")
            return
        self.map_points = pts
        self.get_logger().info(f"[gicp] map loaded: {pts.shape[0]} points")
        self.map_published = False

    def reset(self) -> None:
        self.T_map_lidar = np.eye(4, dtype=np.float64)
        self.map_published = False
        self.path_msg.poses.clear()
        self.get_logger().info("[gicp] state reset")

    # ------------------------------------------------------------------
    def _on_indoor_flag(self, msg: Bool) -> None:
        was_indoor = self.is_indoor
        self.is_indoor = msg.data
        if not was_indoor and msg.data:
            self.get_logger().info("[gicp] indoor mode ON")
            self.reset()
        elif was_indoor and not msg.data:
            self.get_logger().info("[gicp] indoor mode OFF — reset")
            self.reset()

    def _on_lidar(self, msg: PointCloud2) -> None:
        if not self.is_indoor:
            return
        if self.map_points is None:
            self.get_logger().warn_once("[gicp] no map loaded — skipping frame")
            return

        pts = pointcloud2_to_xyz(msg)
        if pts.shape[0] < self.min_scan_points:
            return

        # Publish map cloud once (latched)
        if not self.map_published:
            map_msg = xyz_to_pointcloud2(self.map_points, self.map_frame, msg.header.stamp)
            self.pub_map_cloud.publish(map_msg)
            self.map_published = True

        # Align scan to map
        try:
            result = small_gicp.align(
                target=self.map_points,
                source=pts,
                init_T_target_source=self.T_map_lidar,
                registration_type=self.registration_type,
                downsampling_resolution=self.scan_ds_res,
                max_correspondence_distance=self.max_corr_dist,
                num_threads=self.num_threads,
            )
        except Exception as exc:
            self.get_logger().warn(f"[gicp] align failed: {exc}")
            return

        if not result.converged:
            self.get_logger().warn("[gicp] not converged — keeping previous pose")
            return

        self.T_map_lidar = result.T_target_source

        # Publish aligned scan in map frame
        pts_map = (self.T_map_lidar[:3, :3] @ pts.T).T + self.T_map_lidar[:3, 3]
        scan_msg = xyz_to_pointcloud2(pts_map, self.map_frame, msg.header.stamp)
        self.pub_aligned_scan.publish(scan_msg)

        # Publish odometry
        self._publish_odom(msg.header.stamp)

    def _publish_odom(self, stamp) -> None:
        pos  = self.T_map_lidar[:3, 3]
        quat = rotmat_to_quaternion_xyzw(self.T_map_lidar[:3, :3])

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id  = "lidar"
        odom.pose.pose.position.x    = float(pos[0])
        odom.pose.pose.position.y    = float(pos[1])
        odom.pose.pose.position.z    = float(pos[2])
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        self.pub_odom.publish(odom)

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose   = odom.pose.pose
        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.max_path_size:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_size:]
        self.pub_path.publish(self.path_msg)


def main() -> None:
    rclpy.init()
    node = IndoorMapGICPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
