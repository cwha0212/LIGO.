#!/usr/bin/env python3
import math
import os
from dataclasses import dataclass
from glob import glob
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import NavSatFix


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def lla_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def parse_simple_yaml(path: str) -> dict:
    result = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            result[key.strip()] = value.strip()
    return result


def parse_origin(origin_text: str) -> Tuple[float, float, float]:
    text = origin_text.strip()
    if not (text.startswith("[") and text.endswith("]")):
        raise ValueError(f"invalid origin format: {origin_text}")
    parts = [p.strip() for p in text[1:-1].split(",")]
    if len(parts) != 3:
        raise ValueError(f"origin must have 3 values: {origin_text}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def parse_float_list(text: str, expected_len: int) -> Tuple[float, ...]:
    s = text.strip()
    if not (s.startswith("[") and s.endswith("]")):
        raise ValueError(f"invalid list format: {text}")
    parts = [p.strip() for p in s[1:-1].split(",")]
    if len(parts) != expected_len:
        raise ValueError(f"invalid list length ({len(parts)} != {expected_len}): {text}")
    return tuple(float(p) for p in parts)


def read_pgm(path: str) -> Tuple[int, int, bytes]:
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"unsupported PGM magic in {path}: {magic}")

        def next_tokens() -> bytes:
            while True:
                line = f.readline()
                if not line:
                    raise ValueError(f"unexpected EOF while parsing PGM header: {path}")
                if line.startswith(b"#"):
                    continue
                return line

        wh = next_tokens().split()
        if len(wh) != 2:
            raise ValueError(f"invalid width/height line in {path}")
        width, height = int(wh[0]), int(wh[1])

        maxval_line = next_tokens().strip()
        maxval = int(maxval_line)
        if maxval != 255:
            raise ValueError(f"unsupported maxval {maxval} in {path}")

        data = f.read(width * height)
        if len(data) != width * height:
            raise ValueError(f"invalid PGM payload size in {path}")
        return width, height, data


@dataclass
class GridMap:
    map_id: str
    yaml_path: str
    pgm_path: str
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
            dx, dy, dz = (x - ax), (y - ay), (z - az)
            r = self.r_ecef_enu_row_major
            # p_enu = R_ecef_enu^T * (p_ecef - anchor_ecef)
            enu_x = r[0] * dx + r[3] * dy + r[6] * dz
            enu_y = r[1] * dx + r[4] * dy + r[7] * dz
            return enu_x, enu_y
        return None

    def classify(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        map_xy = self._ecef_to_map_xy(x, y, z)
        if map_xy is None:
            return False, "outside"
        mx, my = map_xy
        # Row mapping matches exporter: row = max_iy - iy.
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

    def distance_to_bbox(self, x: float, y: float, z: float) -> float:
        map_xy = self._ecef_to_map_xy(x, y, z)
        if map_xy is None:
            return float("inf")
        mx, my = map_xy
        max_x = self.origin_x + self.width * self.resolution
        max_y = self.origin_y + self.height * self.resolution
        dx = 0.0 if self.origin_x <= mx <= max_x else min(abs(mx - self.origin_x), abs(mx - max_x))
        dy = 0.0 if self.origin_y <= my <= max_y else min(abs(my - self.origin_y), abs(my - max_y))
        return math.hypot(dx, dy)


class GridMembershipMonitor(Node):
    def __init__(self) -> None:
        super().__init__("grid_membership_monitor")

        self.declare_parameter("map_dir", "/home/tae/test_NAVI/src/LIGO./PCD")
        self.declare_parameter("gps_quality_topic", "/ublox_driver/receiver_lla")
        self.declare_parameter("position_topic", "/ligo/global_position")
        self.declare_parameter("gps_cov_bad_threshold_m2", 50.0)
        self.declare_parameter("report_hz", 2.0)

        self.map_dir = str(self.get_parameter("map_dir").value)
        self.gps_quality_topic = str(self.get_parameter("gps_quality_topic").value)
        self.position_topic = str(self.get_parameter("position_topic").value)
        self.gps_cov_bad_threshold_m2 = float(self.get_parameter("gps_cov_bad_threshold_m2").value)
        report_hz = max(0.1, float(self.get_parameter("report_hz").value))

        self.maps: List[GridMap] = self._load_maps(self.map_dir)
        if not self.maps:
            self.get_logger().warn(f"no *_grid2d.yaml found in {self.map_dir}")
        else:
            self.get_logger().info(f"loaded {len(self.maps)} grid maps from {self.map_dir}")

        self.last_position_msg: Optional[NavSatFix] = None
        self.last_quality_msg: Optional[NavSatFix] = None

        self.create_subscription(NavSatFix, self.position_topic, self._on_position, 10)
        self.create_subscription(NavSatFix, self.gps_quality_topic, self._on_quality, 10)
        self.create_timer(1.0 / report_hz, self._on_timer)

    def _load_maps(self, map_dir: str) -> List[GridMap]:
        out: List[GridMap] = []
        yaml_paths = sorted(glob(os.path.join(map_dir, "*_grid2d.yaml")))
        for ypath in yaml_paths:
            try:
                meta = parse_simple_yaml(ypath)
                image = meta.get("image", "")
                if not image:
                    self.get_logger().warn(f"skip yaml without image: {ypath}")
                    continue
                pgm_path = image if os.path.isabs(image) else os.path.join(os.path.dirname(ypath), image)
                if not os.path.exists(pgm_path):
                    self.get_logger().warn(f"skip yaml image not found: {ypath} -> {pgm_path}")
                    continue

                resolution = float(meta.get("resolution", "1.0"))
                origin_x, origin_y, _ = parse_origin(meta.get("origin", "[0, 0, 0]"))
                frame_id = meta.get("frame_id", "ecef")
                enu_anchor_ecef_m: Optional[Tuple[float, float, float]] = None
                r_ecef_enu_row_major: Optional[Tuple[float, ...]] = None
                if frame_id.strip().lower() == "enu":
                    anchor_txt = meta.get("anchor_ecef_m", meta.get("anchor_ecef", ""))
                    rot_txt = meta.get("R_ecef_enu_row_major", "")
                    if anchor_txt and rot_txt:
                        enu_anchor_ecef_m = parse_float_list(anchor_txt, 3)
                        r_ecef_enu_row_major = parse_float_list(rot_txt, 9)
                    else:
                        self.get_logger().warn(
                            f"ENU grid missing ECEF transform metadata, map may be unusable: {ypath}"
                        )
                width, height, pixels = read_pgm(pgm_path)
                map_id = os.path.basename(ypath).replace("_grid2d.yaml", "")
                out.append(
                    GridMap(
                        map_id=map_id,
                        yaml_path=ypath,
                        pgm_path=pgm_path,
                        resolution=resolution,
                        origin_x=origin_x,
                        origin_y=origin_y,
                        frame_id=frame_id,
                        width=width,
                        height=height,
                        pixels=pixels,
                        enu_anchor_ecef_m=enu_anchor_ecef_m,
                        r_ecef_enu_row_major=r_ecef_enu_row_major,
                    )
                )
            except Exception as e:
                self.get_logger().error(f"failed to load grid map {ypath}: {e}")
        return out

    def _on_position(self, msg: NavSatFix) -> None:
        self.last_position_msg = msg

    def _on_quality(self, msg: NavSatFix) -> None:
        self.last_quality_msg = msg

    def _is_gps_bad(self) -> bool:
        q = self.last_quality_msg
        if q is None:
            return False
        cov = q.position_covariance
        max_diag = max(float(cov[0]), float(cov[4]), float(cov[8]))
        return max_diag >= self.gps_cov_bad_threshold_m2

    def _on_timer(self) -> None:
        if not self.maps:
            return
        if self.last_position_msg is None or self.last_quality_msg is None:
            return
        if not self._is_gps_bad():
            return

        p = self.last_position_msg
        x, y, z = lla_to_ecef(float(p.latitude), float(p.longitude), float(p.altitude))

        inside_known = []
        inside_unknown = []
        outside = []
        for gm in self.maps:
            inside, state = gm.classify(x, y, z)
            if inside and state in ("free", "occupied"):
                inside_known.append((gm, state))
            elif inside and state == "unknown":
                inside_unknown.append((gm, state))
            else:
                outside.append(gm)

        q = self.last_quality_msg
        cov = q.position_covariance
        max_diag = max(float(cov[0]), float(cov[4]), float(cov[8]))

        if inside_known:
            summary = ", ".join([f"{m.map_id}:{st}" for m, st in inside_known])
            self.get_logger().warn(
                f"[gps_bad][grid_check] GPS covariance high ({max_diag:.2f} m^2). "
                f"inside known cells: {summary}"
            )
            return

        if inside_unknown:
            summary = ", ".join([m.map_id for m, _ in inside_unknown])
            self.get_logger().warn(
                f"[gps_bad][grid_check] GPS covariance high ({max_diag:.2f} m^2). "
                f"inside map bounds but unknown cells: {summary}"
            )
            return

        nearest = sorted(outside, key=lambda gm: gm.distance_to_bbox(x, y, z))[:2]
        nearest_txt = ", ".join([f"{m.map_id}:{m.distance_to_bbox(x, y, z):.1f}m" for m in nearest])
        self.get_logger().warn(
            f"[gps_bad][grid_check] GPS covariance high ({max_diag:.2f} m^2). "
            f"outside all grids. nearest={nearest_txt}"
        )


def main() -> None:
    rclpy.init()
    node = GridMembershipMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
