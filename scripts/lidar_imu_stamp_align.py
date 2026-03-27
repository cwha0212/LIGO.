#!/usr/bin/env python3
"""Republish LiDAR (PointCloud2) and IMU with aligned header.stamp.

Subscribe to driver topics, add configurable constant offsets (seconds), publish
to output topics for LIGO or other nodes that assume a common time base.

Example (IMU가 LiDAR보다 0.05 s 앞서는 경우 IMU 스탬프에 -0.05 s):
  ros2 run ligo lidar_imu_stamp_align.py --ros-args \\
    -p imu_stamp_offset_sec:=-0.05

기본 토픽: in /lidar_points, /imu/data_raw → out /lidar_points_aligned, /imu/data_raw_aligned
(ligo는 lid_topic / imu_topic 을 *_aligned 쪽으로 맞추면 됨)

오프셋 파라미터 (초, 각 메시지 header.stamp에 더함):
  lidar_stamp_offset_sec, imu_stamp_offset_sec
  LiDAR가 상대시각(예: sec≈3088)·IMU가 Unix시각이면 큰 보정은 lidar_stamp_offset_sec 에만 주고
  imu_stamp_offset_sec 는 0 으로 둔다 (IMU에 epoch급 값을 더하면 sec가 int32 범위를 넘긴다).
"""

from __future__ import annotations

import copy
import sys

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, PointCloud2

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult

# laserMapping.cpp uses rclcpp::QoS qos_lidar(200000) for IMU/LiDAR (RELIABLE KEEP_LAST).
# qos_profile_sensor_data is BEST_EFFORT — incompatible with ligo's subscriber.
QOS_LIGO_MATCH = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=2000,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# builtin_interfaces/Time.sec 는 int32
_INT32_MIN = -(2**31)
_INT32_MAX = 2**31 - 1


def _shift_stamp_msg(stamp, offset_sec: float) -> tuple[TimeMsg | object, bool]:
    """offset_sec 만큼 이동한 TimeMsg. int32 sec 범위를 벗어나면 (원본 stamp, False)."""
    if abs(offset_sec) < 1e-12:
        return stamp, True
    total_ns = int(stamp.sec) * 10**9 + int(stamp.nanosec) + int(round(offset_sec * 1e9))
    sec, nsec = divmod(total_ns, 10**9)
    if sec < _INT32_MIN or sec > _INT32_MAX:
        return stamp, False
    out = TimeMsg()
    out.sec = int(sec)
    out.nanosec = int(nsec)
    return out, True


def _param_float(node: Node, name: str) -> float:
    """ROS2가 double 대신 integer/string으로 넘겨도 오프셋이 먹도록 읽는다."""
    raw = node.get_parameter(name).get_parameter_value()
    # rcl_interfaces: 일부 배포판은 type, 일부는 type_
    t = getattr(raw, "type", None)
    if t is None:
        t = getattr(raw, "type_", ParameterType.PARAMETER_NOT_SET)
    if t == ParameterType.PARAMETER_DOUBLE:
        return float(raw.double_value)
    if t == ParameterType.PARAMETER_INTEGER:
        return float(raw.integer_value)
    if t == ParameterType.PARAMETER_STRING:
        s = (raw.string_value or "").strip()
        if not s:
            return 0.0
        return float(s)
    return 0.0


class LidarImuStampAlignNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_imu_stamp_align")

        off_desc = ParameterDescriptor(description="Seconds added to outgoing message header.stamp (can be negative).")
        self.declare_parameter("lidar_in_topic", "/lidar_points")
        self.declare_parameter("imu_in_topic", "/imu/data_raw")
        self.declare_parameter("lidar_out_topic", "/lidar_points_aligned")
        self.declare_parameter("imu_out_topic", "/imu/data_raw_aligned")
        self.declare_parameter("lidar_stamp_offset_sec", 0.0, off_desc)
        self.declare_parameter("imu_stamp_offset_sec", 0.0, off_desc)

        self._stamp_overflow_warned = False

        lidar_in = self.get_parameter("lidar_in_topic").get_parameter_value().string_value
        imu_in = self.get_parameter("imu_in_topic").get_parameter_value().string_value
        lidar_out = self.get_parameter("lidar_out_topic").get_parameter_value().string_value
        imu_out = self.get_parameter("imu_out_topic").get_parameter_value().string_value
        self._lidar_off = _param_float(self, "lidar_stamp_offset_sec")
        self._imu_off = _param_float(self, "imu_stamp_offset_sec")

        self._pub_lidar = self.create_publisher(PointCloud2, lidar_out, QOS_LIGO_MATCH)
        self._pub_imu = self.create_publisher(Imu, imu_out, QOS_LIGO_MATCH)

        self.create_subscription(PointCloud2, lidar_in, self._on_lidar, qos_profile_sensor_data)
        self.create_subscription(Imu, imu_in, self._on_imu, qos_profile_sensor_data)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"lidar_imu_stamp_align: in [{lidar_in}] [{imu_in}] -> out [{lidar_out}] [{imu_out}] | "
            f"offsets (s) lidar={self._lidar_off:.9f} imu={self._imu_off:.9f}"
        )

    def _on_param_change(self, params):
        """ros2 param set 으로 오프셋 변경 시 즉시 반영."""
        try:
            for p in params:
                if p.name == "lidar_stamp_offset_sec":
                    self._lidar_off = float(p.value)
                elif p.name == "imu_stamp_offset_sec":
                    self._imu_off = float(p.value)
        except (TypeError, ValueError) as e:
            return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    def _shift(self, stamp, offset_sec: float):
        new_stamp, ok = _shift_stamp_msg(stamp, offset_sec)
        if not ok and not self._stamp_overflow_warned:
            self.get_logger().error(
                "stamp shift would exceed int32 sec (ROS Time). "
                "If LiDAR uses small relative time and IMU uses Unix time, set a large "
                "lidar_stamp_offset_sec only, and imu_stamp_offset_sec:=0. "
                "Passthrough without offset for this message."
            )
            self._stamp_overflow_warned = True
        return new_stamp if ok else stamp

    def _on_lidar(self, msg: PointCloud2) -> None:
        out = copy.deepcopy(msg)
        out.header.stamp = self._shift(msg.header.stamp, self._lidar_off)
        self._pub_lidar.publish(out)

    def _on_imu(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.header.stamp = self._shift(msg.header.stamp, self._imu_off)
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self._pub_imu.publish(out)


def main() -> None:
    # --ros-args -p ... 가 노드에 전달되려면 argv 필요 (미전달 시 CLI 오프셋이 무시될 수 있음)
    rclpy.init(args=sys.argv)
    node = LidarImuStampAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
