#!/usr/bin/env python3
"""
LIGO ROS2 topic -> MQTT bridge.

MQTT topic별로 JSON을 분리 발행한다 (기본 prefix: navi1):
  - navi1/position   : lat, lon
  - navi1/heading    : deg_from_north_cw, cardinal
  - navi1/gps        : status, ntrip_connected (/receiver_pvt 기반)
  - navi1/init_heading_icp : success/status (/ligo/nmea_heading_align_status 기반)
  - navi1/ligo_mode  : /ligo/mode(String JSON) 미러

Run:
  python3 scripts/ligo_topic_to_mqtt.py

Requires:
  pip install paho-mqtt
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
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


def _indoor_pcd_name(ros_obj: dict) -> str:
    v = ros_obj.get("pcd_name")
    if isinstance(v, str) and v.strip():
        return v.strip()
    v = ros_obj.get("map_pcd_basename")
    if isinstance(v, str) and v.strip():
        return v.strip()
    mp = ros_obj.get("map_pcd")
    if isinstance(mp, str) and mp.strip():
        return Path(mp).name
    return ""


def ligo_mode_payload_for_mqtt(ros_obj: dict) -> dict:
    mode = ros_obj.get("mode")
    if mode == "indoor":
        return {"mode": "indoor", "pcd_name": _indoor_pcd_name(ros_obj)}
    if mode == "outdoor":
        return {"mode": "outdoor"}
    return {"mode": str(mode), "raw": ros_obj}


def _mqtt_reason_failed(reason_code: object) -> bool:
    if reason_code is None:
        return False
    if hasattr(reason_code, "is_failure") and callable(reason_code.is_failure):
        try:
            return bool(reason_code.is_failure)
        except Exception:
            pass
    if isinstance(reason_code, int):
        return reason_code != 0
    try:
        return int(reason_code) != 0
    except (TypeError, ValueError):
        pass
    return str(reason_code).strip().lower() not in ("success", "0", "no error", "no_error")


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
        self.declare_parameter("reconnect_period_sec", 1.0)

        # ROS topic params
        self.declare_parameter("topic.global_position", "/ligo/global_position")
        self.declare_parameter("topic.odom", "/aft_mapped_to_init")
        self.declare_parameter("topic.receiver_pvt", "/ublox_driver/receiver_pvt")
        self.declare_parameter("topic.heading_align_status", "/ligo/nmea_heading_align_status")
        self.declare_parameter("topic.ligo_mode", "/ligo/mode")

        self.mqtt_host = str(self.get_parameter("mqtt.host").value)
        self.mqtt_port = int(self.get_parameter("mqtt.port").value)
        prefix = str(self.get_parameter("mqtt.topic_prefix").value).strip().strip("/")
        self.mqtt_topic_position = f"{prefix}/position"
        self.mqtt_topic_heading = f"{prefix}/heading"
        self.mqtt_topic_gps = f"{prefix}/gps"
        self.mqtt_topic_icp = f"{prefix}/init_heading_icp"
        self.mqtt_topic_ligo_mode = f"{prefix}/ligo_mode"
        self.mqtt_use_websocket = bool(self.get_parameter("mqtt.use_websocket").value)
        self.mqtt_ws_path = str(self.get_parameter("mqtt.ws_path").value)
        self.mqtt_username = str(self.get_parameter("mqtt.username").value)
        self.mqtt_password = str(self.get_parameter("mqtt.password").value)

        global_topic = str(self.get_parameter("topic.global_position").value)
        odom_topic = str(self.get_parameter("topic.odom").value)
        receiver_pvt_topic = str(self.get_parameter("topic.receiver_pvt").value)
        align_status_topic = str(self.get_parameter("topic.heading_align_status").value)
        ligo_mode_topic = str(self.get_parameter("topic.ligo_mode").value)

        reconnect_period = float(self.get_parameter("reconnect_period_sec").value)

        # State cache
        self.lat: Optional[float] = None
        self.lon: Optional[float] = None
        self.heading_deg: Optional[float] = None
        self.heading_dir: Optional[str] = None
        self.gps_signal_status: Optional[str] = None  # "신호없음" | "신호미약" | "신호정상"
        self.ntrip_connected: Optional[bool] = None
        self.icp_heading_aligned: bool = False
        self.icp_status_text: Optional[str] = None  # "UNALIGNED" | "COLLECTING" | "LOCKED"
        self._pvt_is_gps_fixed: Optional[bool] = None
        self._pvt_is_no_fix: Optional[bool] = None
        self._has_heading_sample: bool = False
        self._last_ligo_mode_payload: Optional[dict] = None
        self._ligo_mode_bootstrap_done: bool = False

        self.create_subscription(NavSatFix, global_topic, self.on_global_position, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        if GnssPVTSolnMsg is not None:
            self.create_subscription(GnssPVTSolnMsg, receiver_pvt_topic, self.on_receiver_pvt, 10)
        else:
            self.get_logger().warn("gnss_comm.msg.GnssPVTSolnMsg import 실패: NTRIP/정상 판정 정밀도 제한")
        self.create_subscription(NmeaHeadingAlignStatus, align_status_topic, self.on_heading_align_status, 10)
        mode_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(String, ligo_mode_topic, self.on_ligo_mode, mode_qos)

        self._mqtt_connected = False
        self._mqtt_loop_started = False
        self._reconnect_backoff_sec = 1.0
        self._reconnect_backoff_max_sec = 30.0
        self._next_reconnect_not_before = 0.0
        self._mqtt_publish_fail_count = 0
        self._last_publish_ok_unix = 0.0
        self._mqtt = self._create_mqtt_client()
        self._connect_mqtt()

        # 연결이 끊겼을 때만 주기적으로 재연결 시도.
        self.create_timer(reconnect_period, self._reconnect_tick)
        self.create_timer(10.0, self._log_mqtt_health)
        self.get_logger().info(
            f"ROS->MQTT bridge started. mqtt={self.mqtt_host}:{self.mqtt_port} "
            f"topics={self.mqtt_topic_position}, {self.mqtt_topic_heading}, "
            f"{self.mqtt_topic_gps}, {self.mqtt_topic_icp}, {self.mqtt_topic_ligo_mode}"
        )

    def _create_mqtt_client(self) -> mqtt.Client:
        kwargs = {"transport": "websockets" if self.mqtt_use_websocket else "tcp"}
        if hasattr(mqtt, "CallbackAPIVersion"):
            kwargs["callback_api_version"] = mqtt.CallbackAPIVersion.VERSION2
        if self.mqtt_use_websocket:
            client = mqtt.Client(**kwargs)
            client.ws_set_options(path=self.mqtt_ws_path)
        else:
            client = mqtt.Client(**kwargs)

        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)

        client.on_connect = self._on_mqtt_connect
        client.on_disconnect = self._on_mqtt_disconnect
        return client

    def _connect_mqtt(self) -> None:
        now = time.time()
        if now < self._next_reconnect_not_before:
            return
        try:
            self._mqtt.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
            if not self._mqtt_loop_started:
                self._mqtt.loop_start()
                self._mqtt_loop_started = True
        except Exception as exc:
            self._mqtt_connected = False
            self.get_logger().warn(f"MQTT 연결 실패: {exc}")
            self._schedule_next_reconnect()

    def _schedule_next_reconnect(self) -> None:
        self._next_reconnect_not_before = time.time() + self._reconnect_backoff_sec
        self._reconnect_backoff_sec = min(
            self._reconnect_backoff_sec * 2.0,
            self._reconnect_backoff_max_sec,
        )

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        self._mqtt_connected = not _mqtt_reason_failed(reason_code)
        if self._mqtt_connected:
            self._reconnect_backoff_sec = 1.0
            self._next_reconnect_not_before = 0.0
            self._mqtt_publish_fail_count = 0
            self.get_logger().info("MQTT connected")
            # 재연결 직후 현재 캐시 상태 1회 재발행.
            self._publish_position()
            self._publish_heading()
            self._publish_gps()
            self._publish_icp()
            self._publish_ligo_mode()
        else:
            self.get_logger().warn(f"MQTT connect failed, reason={reason_code!r}")
            self._schedule_next_reconnect()

    def _on_mqtt_disconnect(self, client, userdata, disconnect_flags, reason_code, properties=None):
        self._mqtt_connected = False
        self.get_logger().warn(f"MQTT disconnected, reason={reason_code!r}")
        self._schedule_next_reconnect()

    def on_global_position(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        if self.lat == lat and self.lon == lon:
            return
        self.lat = lat
        self.lon = lon
        self._publish_position()

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        # yaw(rad): ENU 기준 (x=East, y=North). North 기준 heading으로 변환.
        heading_deg_from_north = (90.0 - math.degrees(yaw)) % 360.0
        heading_dir = heading_cardinal(heading_deg_from_north)
        self._has_heading_sample = True
        self.heading_deg = heading_deg_from_north
        self.heading_dir = heading_dir
        # heading은 ICP lock 이후에만 publish
        if self.icp_heading_aligned:
            self._publish_heading()

    def on_receiver_pvt(self, msg) -> None:
        # wtrtk_driver fix_state_label:
        #  - GPS_fixed: valid_fix && fix_type!=0 && carr_soln==2
        #  - 그 외 valid fix는 미약으로 분류
        self._pvt_is_no_fix = (not bool(msg.valid_fix)) or int(msg.fix_type) == 0
        self._pvt_is_gps_fixed = (not self._pvt_is_no_fix) and int(msg.carr_soln) == 2
        # NTRIP 연결(실사용) 여부 근사: diff solution 또는 carrier solution 존재
        ntrip_connected = bool(msg.diff_soln) or int(msg.carr_soln) in (1, 2)
        self.ntrip_connected = ntrip_connected
        self._refresh_gps_signal_status()
        # gps는 PVT 메시지 수신 시마다 publish
        self._publish_gps()

    def _refresh_gps_signal_status(self) -> bool:
        prev = self.gps_signal_status
        # PVT 기반 3단계 판정
        if self._pvt_is_no_fix is True:
            self.gps_signal_status = "신호없음"
        elif self._pvt_is_gps_fixed is True:
            self.gps_signal_status = "신호정상"
        # no_fix가 아니면서 GPS_fixed가 아니면 모두 미약 (2D_fix/GPS_single/GPS_float 포함)
        elif self._pvt_is_no_fix is False:
            self.gps_signal_status = "신호미약"
        else:
            self.gps_signal_status = None
        return prev != self.gps_signal_status

    def on_heading_align_status(self, msg: NmeaHeadingAlignStatus) -> None:
        aligned = bool(
            msg.icp_tf_ready and msg.status == NmeaHeadingAlignStatus.STATUS_LOCKED
        )
        if msg.status == NmeaHeadingAlignStatus.STATUS_LOCKED:
            status_text = "LOCKED"
        elif msg.status == NmeaHeadingAlignStatus.STATUS_COLLECTING:
            status_text = "COLLECTING"
        else:
            status_text = "UNALIGNED"

        was_aligned = self.icp_heading_aligned
        was_status = self.icp_status_text
        if was_aligned == aligned and was_status == status_text:
            return
        self.icp_heading_aligned = aligned
        self.icp_status_text = status_text
        self._publish_icp()
        # lock 전환 시점에 heading 샘플이 이미 있으면 즉시 1회 publish
        if (not was_aligned) and aligned and self._has_heading_sample:
            self._publish_heading()

    def on_ligo_mode(self, msg: String) -> None:
        if not self.mqtt_topic_ligo_mode:
            return
        try:
            ros_obj = json.loads(msg.data)
            out = ligo_mode_payload_for_mqtt(ros_obj)
            self._last_ligo_mode_payload = out
            self._publish_json(self.mqtt_topic_ligo_mode, out)
        except json.JSONDecodeError:
            self._publish_raw(self.mqtt_topic_ligo_mode, msg.data.encode("utf-8"))

    def _reconnect_tick(self) -> None:
        if self._mqtt_connected:
            return
        self._connect_mqtt()

    def _log_mqtt_health(self) -> None:
        age_sec = None
        if self._last_publish_ok_unix > 0.0:
            age_sec = time.time() - self._last_publish_ok_unix
        self.get_logger().info(
            "MQTT health: "
            f"connected={self._mqtt_connected}, "
            f"publish_fail_count={self._mqtt_publish_fail_count}, "
            f"last_publish_ok_age_sec={age_sec if age_sec is not None else 'n/a'}"
        )

    def _publish_json(self, topic: str, body: dict) -> None:
        if not self._mqtt_connected:
            return
        payload = {"timestamp_unix": time.time(), **body}
        try:
            info = self._mqtt.publish(
                topic,
                json.dumps(payload, ensure_ascii=False),
                qos=0,
                retain=False,
            )
            if int(getattr(info, "rc", mqtt.MQTT_ERR_UNKNOWN)) != mqtt.MQTT_ERR_SUCCESS:
                self._mqtt_connected = False
                self._mqtt_publish_fail_count += 1
                self.get_logger().warn(
                    f"MQTT publish rc 실패: topic={topic}, rc={getattr(info, 'rc', None)}, "
                    f"fail_count={self._mqtt_publish_fail_count}"
                )
                self._schedule_next_reconnect()
                return
            self._mqtt_publish_fail_count = 0
            self._last_publish_ok_unix = time.time()
        except Exception as exc:
            self._mqtt_connected = False
            self._mqtt_publish_fail_count += 1
            self.get_logger().warn(f"MQTT publish 실패: {exc}")
            self._schedule_next_reconnect()

    def _publish_raw(self, topic: str, raw_payload: bytes) -> None:
        if not self._mqtt_connected:
            return
        try:
            info = self._mqtt.publish(topic, raw_payload, qos=0, retain=False)
            if int(getattr(info, "rc", mqtt.MQTT_ERR_UNKNOWN)) != mqtt.MQTT_ERR_SUCCESS:
                self._mqtt_connected = False
                self._mqtt_publish_fail_count += 1
                self.get_logger().warn(
                    f"MQTT raw publish rc 실패: topic={topic}, rc={getattr(info, 'rc', None)}, "
                    f"fail_count={self._mqtt_publish_fail_count}"
                )
                self._schedule_next_reconnect()
                return
            self._mqtt_publish_fail_count = 0
            self._last_publish_ok_unix = time.time()
        except Exception as exc:
            self._mqtt_connected = False
            self._mqtt_publish_fail_count += 1
            self.get_logger().warn(f"MQTT raw publish 실패: {exc}")
            self._schedule_next_reconnect()

    def _publish_position(self) -> None:
        self._publish_json(
            self.mqtt_topic_position,
            {"lat": self.lat, "lon": self.lon},
        )

    def _publish_heading(self) -> None:
        self._publish_json(
            self.mqtt_topic_heading,
            {"deg_from_north_cw": self.heading_deg, "cardinal": self.heading_dir},
        )

    def _publish_gps(self) -> None:
        self._publish_json(
            self.mqtt_topic_gps,
            {"status": self.gps_signal_status, "ntrip_connected": self.ntrip_connected},
        )

    def _publish_icp(self) -> None:
        self._publish_json(
            self.mqtt_topic_icp,
            {
                "success": self.icp_heading_aligned,
                "status": self.icp_status_text,
            },
        )

    def _publish_ligo_mode(self) -> None:
        # 최초 연결에서 /ligo/mode 미수신 상태면 null -> outdoor 순서로 부트스트랩.
        if (not self._ligo_mode_bootstrap_done) and self._last_ligo_mode_payload is None:
            self._publish_json(self.mqtt_topic_ligo_mode, {"mode": None})
            self._publish_json(self.mqtt_topic_ligo_mode, {"mode": "outdoor"})
            self._last_ligo_mode_payload = {"mode": "outdoor"}
            self._ligo_mode_bootstrap_done = True
            return

        payload = self._last_ligo_mode_payload if self._last_ligo_mode_payload is not None else {"mode": None}
        self._publish_json(self.mqtt_topic_ligo_mode, payload)


def main() -> None:
    rclpy.init()
    node = LigoMqttBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if getattr(node, "_mqtt_loop_started", False):
                node._mqtt.loop_stop()
            node._mqtt.disconnect()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
