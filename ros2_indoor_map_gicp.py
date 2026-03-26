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

Grid-based map (same rule as scripts/grid_membership_monitor.py — free/occupied cell in ECEF):
  ros2 run ligo ros2_indoor_map_gicp.py --ros-args \\
      -p grid_map_dir:=/path/to/PCD \\
      -p ecef_topic:=/ligo/ecef_position
  (Subscribe to fused ECEF from ligo; source_pcd from the matching *_grid2d.yaml is loaded.)
"""

import logging
import math
import os
from dataclasses import dataclass
from glob import glob
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Bool
import small_gicp


def _default_grid_map_dir() -> str:
    """share/ligo/PCD after install, or <pkg>/PCD when running from source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        p = os.path.join(get_package_share_directory("ligo"), "PCD")
        if os.path.isdir(p):
            return p
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.normpath(os.path.join(here, "PCD"))
    return cand if os.path.isdir(cand) else ""


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
# Occupancy grid membership (aligned with grid_membership_monitor / ligo C++ registry)
# ---------------------------------------------------------------------------


def _parse_simple_yaml(path: str) -> dict:
    result = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            result[key.strip()] = value.strip()
    return result


def _parse_origin(origin_text: str) -> Tuple[float, float, float]:
    text = origin_text.strip()
    if not (text.startswith("[") and text.endswith("]")):
        raise ValueError(f"invalid origin: {origin_text}")
    parts = [p.strip() for p in text[1:-1].split(",")]
    if len(parts) != 3:
        raise ValueError(origin_text)
    return float(parts[0]), float(parts[1]), float(parts[2])


def _parse_float_list(text: str, n: int) -> Tuple[float, ...]:
    s = text.strip()
    if not (s.startswith("[") and s.endswith("]")):
        raise ValueError(text)
    parts = [p.strip() for p in s[1:-1].split(",")]
    nums = tuple(float(p) for p in parts if p)
    if len(nums) != n:
        raise ValueError(text)
    return nums


def _read_pgm_p5(path: str) -> Tuple[int, int, bytes]:
    with open(path, "rb") as f:
        if f.readline().strip() != b"P5":
            raise ValueError(f"not P5: {path}")

        def _next() -> bytes:
            while True:
                ln = f.readline()
                if not ln:
                    raise ValueError(path)
                if ln.startswith(b"#"):
                    continue
                return ln

        wh = _next().split()
        w, h = int(wh[0]), int(wh[1])
        if int(_next().strip()) != 255:
            raise ValueError(path)
        data = f.read(w * h)
        if len(data) != w * h:
            raise ValueError(path)
        return w, h, data


@dataclass
class _GridMapPy:
    map_id: str
    source_pcd: str
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str
    width: int
    height: int
    pixels: bytes
    enu_anchor_ecef_m: Optional[Tuple[float, float, float]] = None
    r_ecef_enu_row_major: Optional[Tuple[float, ...]] = None

    def _ecef_to_map_xy(self, x: float, y: float, z: float) -> Optional[Tuple[float, float]]:
        frame = self.frame_id.strip().lower()
        if frame == "ecef":
            return x, y
        if frame == "enu":
            if self.enu_anchor_ecef_m is None or self.r_ecef_enu_row_major is None:
                return None
            ax, ay, az = self.enu_anchor_ecef_m
            dx, dy, dz = x - ax, y - ay, z - az
            r = self.r_ecef_enu_row_major
            enu_x = r[0] * dx + r[3] * dy + r[6] * dz
            enu_y = r[1] * dx + r[4] * dy + r[7] * dz
            return enu_x, enu_y
        return None

    def classify(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        mxy = self._ecef_to_map_xy(x, y, z)
        if mxy is None:
            return False, "outside"
        mx, my = mxy
        col = int(math.floor((mx - self.origin_x) / self.resolution))
        row_from_bottom = int(math.floor((my - self.origin_y) / self.resolution))
        row = self.height - 1 - row_from_bottom
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return False, "outside"
        v = self.pixels[row * self.width + col]
        if v <= 50:
            return True, "occupied"
        if v >= 250:
            return True, "free"
        return True, "unknown"


def _load_grid_maps_from_dir(map_dir: str) -> List[_GridMapPy]:
    out: List[_GridMapPy] = []
    if not map_dir or not os.path.isdir(map_dir):
        return out
    yaml_paths = sorted(glob(os.path.join(map_dir, "*_grid2d.yaml")))
    for ypath in yaml_paths:
        try:
            kv = _parse_simple_yaml(ypath)
            image = kv.get("image", "")
            if not image:
                continue
            pgm_path = image if os.path.isabs(image) else os.path.join(os.path.dirname(ypath), image)
            if not os.path.isfile(pgm_path):
                continue
            src = kv.get("source_pcd", "")
            if not src:
                continue
            pcd_path = src if os.path.isabs(src) else os.path.join(os.path.dirname(ypath), src)
            pcd_path = os.path.normpath(pcd_path)
            if not os.path.isfile(pcd_path):
                continue
            ox, oy, _oz = _parse_origin(kv.get("origin", "[0,0,0]"))
            res = float(kv.get("resolution", "1.0"))
            frame_id = kv.get("frame_id", "ecef")
            stem = os.path.splitext(os.path.basename(ypath))[0]
            mid = stem[: -len("_grid2d")] if stem.endswith("_grid2d") else stem
            w, h, pixels = _read_pgm_p5(pgm_path)
            enu_a = None
            rmaj = None
            at = kv.get("anchor_ecef_m", kv.get("anchor_ecef", ""))
            rt = kv.get("R_ecef_enu_row_major", "")
            if at and rt and frame_id.strip().lower() == "enu":
                enu_a = _parse_float_list(at, 3)
                rmaj = _parse_float_list(rt, 9)
            out.append(
                _GridMapPy(
                    map_id=mid,
                    source_pcd=pcd_path,
                    resolution=res,
                    origin_x=ox,
                    origin_y=oy,
                    frame_id=frame_id,
                    width=w,
                    height=h,
                    pixels=pixels,
                    enu_anchor_ecef_m=enu_a,
                    r_ecef_enu_row_major=rmaj,
                )
            )
        except Exception as exc:  # noqa: BLE001
            logging.getLogger("indoor_map_gicp").warning("skip grid yaml %s: %s", ypath, exc)
    return out


def _lookup_known_pcd_from_grids(maps: List[_GridMapPy], x: float, y: float, z: float) -> Optional[Tuple[str, str]]:
    for gm in maps:
        inside, state = gm.classify(x, y, z)
        if inside and state in ("free", "occupied"):
            return gm.map_id, gm.source_pcd
    return None


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
        self.declare_parameter("grid_map_dir",            _default_grid_map_dir())
        self.declare_parameter("ecef_topic",              "/ligo/ecef_position")

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
        self.grid_map_dir        = str(self.get_parameter("grid_map_dir").value).strip()
        self.ecef_topic          = str(self.get_parameter("ecef_topic").value)

        # State
        self.is_indoor   = False
        self.map_points  = None   # np.ndarray Nx3 in map frame
        self.T_map_lidar = np.eye(4, dtype=np.float64)  # accumulated pose
        self.map_published = False
        self._grid_maps: List[_GridMapPy] = _load_grid_maps_from_dir(self.grid_map_dir)
        self._last_ecef: Optional[Tuple[float, float, float]] = None
        self._grid_resolved_pcd: Optional[str] = None
        self._no_map_warned = False

        # Load map (fixed path only when not using grid dir, or as sole config)
        if not self._grid_maps and self.map_pcd_path:
            self._load_map(self.map_pcd_path)
        elif self._grid_maps:
            self.get_logger().info(
                "[gicp] %d grid map(s) in %s — PCD from grid cell (need ECEF on %s)",
                len(self._grid_maps),
                self.grid_map_dir,
                self.ecef_topic,
            )

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
        self.create_subscription(PointStamped, self.ecef_topic, self._on_ecef, 10)
        self.sub_lidar = self.create_subscription(
            PointCloud2, self.lidar_topic, self._on_lidar, qos_profile_sensor_data)

        self.get_logger().info(
            "IndoorMapGICP started: map_pcd=%r grid_dir=%r frame=%s",
            self.map_pcd_path,
            self.grid_map_dir or None,
            self.map_frame,
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
        self._no_map_warned = False

    def reset(self) -> None:
        self.T_map_lidar = np.eye(4, dtype=np.float64)
        self.map_published = False
        self.path_msg.poses.clear()
        self.get_logger().info("[gicp] state reset")

    def _on_ecef(self, msg: PointStamped) -> None:
        self._last_ecef = (float(msg.point.x), float(msg.point.y), float(msg.point.z))

    def _try_resolve_map_from_grid(self) -> bool:
        """Return True if map_points is ready (from grid, fallback path, or earlier load)."""
        if not self._grid_maps:
            return self.map_points is not None
        if self._last_ecef is None:
            return self.map_points is not None
        x, y, z = self._last_ecef
        hit = _lookup_known_pcd_from_grids(self._grid_maps, x, y, z)
        if hit:
            _mid, pcd = hit
            if self._grid_resolved_pcd != pcd or self.map_points is None:
                self.get_logger().info("[gicp] grid cell -> map_id=%s pcd=%s", _mid, pcd)
                self._load_map(pcd)
                self._grid_resolved_pcd = pcd
                self.T_map_lidar = np.eye(4, dtype=np.float64)
                self.path_msg.poses.clear()
            return self.map_points is not None
        if self.map_pcd_path and self.map_points is None:
            self.get_logger().info("[gicp] no grid hit — fallback map_pcd_path")
            self._load_map(self.map_pcd_path)
        return self.map_points is not None

    # ------------------------------------------------------------------
    def _on_indoor_flag(self, msg: Bool) -> None:
        was_indoor = self.is_indoor
        self.is_indoor = msg.data
        if not was_indoor and msg.data:
            self.get_logger().info("[gicp] indoor mode ON")
            self.reset()
            self._try_resolve_map_from_grid()
        elif was_indoor and not msg.data:
            self.get_logger().info("[gicp] indoor mode OFF — reset")
            self.reset()

    def _on_lidar(self, msg: PointCloud2) -> None:
        if not self.is_indoor:
            return
        if self._grid_maps:
            self._try_resolve_map_from_grid()
        if self.map_points is None:
            if not self._no_map_warned:
                self.get_logger().warn(
                    "[gicp] no map loaded (set map_pcd_path or grid_map_dir + ECEF) — skipping"
                )
                self._no_map_warned = True
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
