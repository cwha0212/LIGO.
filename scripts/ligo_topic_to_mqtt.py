#!/usr/bin/env python3
"""
LIGO ROS2 topic -> MQTT bridge.

MQTT topic별로 JSON을 분리 발행한다 (기본 prefix: navi1):
  - navi1/position   : lat, lon
  - navi1/heading    : deg_from_north_cw, cardinal
  - navi1/gps        : status, ntrip_connected (/receiver_pvt 기반)
  - navi1/init_heading_icp : success (/ligo/nmea_heading_align_status 기반)

Run:
  python3 scripts/ligo_topic_to_mqtt.py

Requires:
  pip install paho-mqtt
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from ligo.msg import NmeaHeadingAlignStatus
try:
    from gnss_comm.msg import GnssPVTSolnMsg
except Exception:
    GnssPVTSolnMsg = None

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("paho-mqtt가 필요합니다: pip install paho-mqtt") from exc


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Quaternion -> yaw(rad), Z-up convention."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def heading_cardinal(deg: float) -> str:
    """0~360 deg -> 16방위."""
    dirs = [
        "N", "NNE", "NE", "ENE",
        "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW",
        "W", "WNW", "NW", "NNW",
    ]
    idx = int((deg + 11.25) // 22.5) % 16
    return dirs[idx]


class LigoMqttBridge(Node):
    def __init__(self) -> None:
        super().__init__("ligo_topic_to_mqtt")

        # MQTT params
        self.declare_parameter("mqtt.host", "rms.bottle-tak.com")
        self.declare_parameter("mqtt.port", 80)
        self.declare_parameter("mqtt.topic_prefix", "navi1")
        self.declare_parameter("mqtt.use_websocket", True)
        self.declare_parameter("mqtt.ws_path", "/mqtt")
        self.declare_parameter("mqtt.username", "")
        self.declare_parameter("mqtt.password", "")
        self.declare_parameter("publish_period_sec", 0.5)

        # ROS topic params
        self.declare_parameter("topic.global_position", "/ligo/global_position")
        self.declare_parameter("topic.odom", "/aft_mapped_to_init")
        self.declare_parameter("topic.receiver_pvt", "/receiver_pvt")
        self.declare_parameter("topic.heading_align_status", "/ligo/nmea_heading_align_status")

        self.mqtt_host = str(self.get_parameter("mqtt.host").value)
        self.mqtt_port = int(self.get_parameter("mqtt.port").value)
        prefix = str(self.get_parameter("mqtt.topic_prefix").value).strip().strip("/")
        self.mqtt_topic_position = f"{prefix}/position"
        self.mqtt_topic_heading = f"{prefix}/heading"
        self.mqtt_topic_gps = f"{prefix}/gps"
        self.mqtt_topic_icp = f"{prefix}/init_heading_icp"
        self.mqtt_use_websocket = bool(self.get_parameter("mqtt.use_websocket").value)
        self.mqtt_ws_path = str(self.get_parameter("mqtt.ws_path").value)
        self.mqtt_username = str(self.get_parameter("mqtt.username").value)
        self.mqtt_password = str(self.get_parameter("mqtt.password").value)

        global_topic = str(self.get_parameter("topic.global_position").value)
        odom_topic = str(self.get_parameter("topic.odom").value)
        receiver_pvt_topic = str(self.get_parameter("topic.receiver_pvt").value)
        align_status_topic = str(self.get_parameter("topic.heading_align_status").value)

        period = float(self.get_parameter("publish_period_sec").value)

        # State cache
        self.lat: Optional[float] = None
        self.lon: Optional[float] = None
        self.heading_deg: Optional[float] = None
        self.heading_dir: Optional[str] = None
        self.gps_signal_status: Optional[str] = None  # "신호없음" | "신호미약" | "신호정상"
        self.ntrip_connected: Optional[bool] = None
        self.icp_heading_aligned: bool = False
        self._pvt_is_gps_fixed: Optional[bool] = None
        self._pvt_is_no_fix: Optional[bool] = None

        self.create_subscription(NavSatFix, global_topic, self.on_global_position, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        if GnssPVTSolnMsg is not None:
            self.create_subscription(GnssPVTSolnMsg, receiver_pvt_topic, self.on_receiver_pvt, 10)
        else:
            self.get_logger().warn("gnss_comm.msg.GnssPVTSolnMsg import 실패: NTRIP/정상 판정 정밀도 제한")
        self.create_subscription(NmeaHeadingAlignStatus, align_status_topic, self.on_heading_align_status, 10)

        self._mqtt_connected = False
        self._mqtt = self._create_mqtt_client()
        self._connect_mqtt()

        self.create_timer(period, self.publish_mqtt)
        self.get_logger().info(
            f"ROS->MQTT bridge started. mqtt={self.mqtt_host}:{self.mqtt_port} "
            f"topics={self.mqtt_topic_position}, {self.mqtt_topic_heading}, "
            f"{self.mqtt_topic_gps}, {self.mqtt_topic_icp}"
        )

    def _create_mqtt_client(self) -> mqtt.Client:
        if self.mqtt_use_websocket:
            client = mqtt.Client(transport="websockets")
            client.ws_set_options(path=self.mqtt_ws_path)
        else:
            client = mqtt.Client()

        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)

        client.on_connect = self._on_mqtt_connect
        client.on_disconnect = self._on_mqtt_disconnect
        return client

    def _connect_mqtt(self) -> None:
        try:
            self._mqtt.connect(self.mqtt_host, self.mqtt_port)
            self._mqtt.loop_start()
        except Exception as exc:
            self._mqtt_connected = False
            self.get_logger().warn(f"MQTT 연결 실패: {exc}")

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        self._mqtt_connected = (rc == 0)
        if self._mqtt_connected:
            self.get_logger().info("MQTT connected")
        else:
            self.get_logger().warn(f"MQTT connect failed, rc={rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        self.get_logger().warn(f"MQTT disconnected, rc={rc}")

    def on_global_position(self, msg: NavSatFix) -> None:
        self.lat = float(msg.latitude)
        self.lon = float(msg.longitude)

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        # yaw(rad): ENU 기준 (x=East, y=North). North 기준 heading으로 변환.
        heading_deg_from_north = (90.0 - math.degrees(yaw)) % 360.0
        self.heading_deg = heading_deg_from_north
        self.heading_dir = heading_cardinal(heading_deg_from_north)

    def on_receiver_pvt(self, msg) -> None:
        # wtrtk_driver fix_state_label:
        #  - GPS_fixed: valid_fix && fix_type!=0 && carr_soln==2
        #  - 그 외 valid fix는 미약으로 분류
        self._pvt_is_no_fix = (not bool(msg.valid_fix)) or int(msg.fix_type) == 0
        self._pvt_is_gps_fixed = (not self._pvt_is_no_fix) and int(msg.carr_soln) == 2
        # NTRIP 연결(실사용) 여부 근사: diff solution 또는 carrier solution 존재
        self.ntrip_connected = bool(msg.diff_soln) or int(msg.carr_soln) in (1, 2)
        self._refresh_gps_signal_status()

    def _refresh_gps_signal_status(self) -> None:
        # PVT 기반 3단계 판정
        if self._pvt_is_no_fix is True:
            self.gps_signal_status = "신호없음"
            return
        if self._pvt_is_gps_fixed is True:
            self.gps_signal_status = "신호정상"
            return
        # no_fix가 아니면서 GPS_fixed가 아니면 모두 미약 (2D_fix/GPS_single/GPS_float 포함)
        if self._pvt_is_no_fix is False:
            self.gps_signal_status = "신호미약"
        else:
            self.gps_signal_status = None

    def on_heading_align_status(self, msg: NmeaHeadingAlignStatus) -> None:
        self.icp_heading_aligned = bool(
            msg.icp_tf_ready and msg.status == NmeaHeadingAlignStatus.STATUS_LOCKED
        )

    def publish_mqtt(self) -> None:
        if not self._mqtt_connected:
            # 짧게 재시도
            self._connect_mqtt()
            return

        ts = time.time()
        try:
            pos_payload = {
                "timestamp_unix": ts,
                "lat": self.lat,
                "lon": self.lon,
            }
            heading_payload = {
                "timestamp_unix": ts,
                "deg_from_north_cw": self.heading_deg,
                "cardinal": self.heading_dir,
            }
            gps_payload = {
                "timestamp_unix": ts,
                "status": self.gps_signal_status,
                "ntrip_connected": self.ntrip_connected,
            }
            icp_payload = {
                "timestamp_unix": ts,
                "success": self.icp_heading_aligned,
            }
            self._mqtt.publish(
                self.mqtt_topic_position,
                json.dumps(pos_payload, ensure_ascii=False),
                qos=0,
                retain=False,
            )
            self._mqtt.publish(
                self.mqtt_topic_heading,
                json.dumps(heading_payload, ensure_ascii=False),
                qos=0,
                retain=False,
            )
            self._mqtt.publish(
                self.mqtt_topic_gps,
                json.dumps(gps_payload, ensure_ascii=False),
                qos=0,
                retain=False,
            )
            self._mqtt.publish(
                self.mqtt_topic_icp,
                json.dumps(icp_payload, ensure_ascii=False),
                qos=0,
                retain=False,
            )
        except Exception as exc:
            self._mqtt_connected = False
            self.get_logger().warn(f"MQTT publish 실패: {exc}")


def main() -> None:
    rclpy.init()
    node = LigoMqttBridge()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._mqtt.loop_stop()
            node._mqtt.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
