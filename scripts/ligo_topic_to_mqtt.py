#!/usr/bin/env python3
"""
LIGO ROS2 topic -> MQTT bridge.

MQTT topic별로 JSON을 분리 발행한다. 토픽 이름·브로커 기본값은 config/mqtt_topics.yaml (또는 LIGO_MQTT_CONFIG).
  - {prefix}/position   : lat, lon
  - {prefix}/heading    : deg_from_north_cw, cardinal
  - {prefix}/gps        : status(신호없음=PVT 무픽스 유지 / 정상·미약=NavSatFix 위·경도 분산), ntrip_connected (PVT)
  - {prefix}/init_heading_icp : success/status (/ligo/nmea_heading_align_status 기반)
  - {prefix}/ligo_mode  : /ligo/mode(String JSON) 미러

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
from geometry_msgs.msg import PoseStamped
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

from mqtt_config import load_mqtt_config

# NavSatFix position_covariance 대각(위도·경도 분산) 중 최댓값이 이 값 이하면 신호정상.
GPS_LAT_LON_COV_MAX_NORMAL = 5.0


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

        _cfg = load_mqtt_config()
        _m = _cfg.get("mqtt", {}) or {}
        _topics = _m.get("topics") if isinstance(_m.get("topics"), dict) else {}
        _ka = _m.get("keepalive_sec", 5)
        try:
            _ka_i = int(_ka) if int(_ka) >= 1 else 5
        except (TypeError, ValueError):
            _ka_i = 5
        _pref = str(_m.get("topic_prefix", "navi1")).strip().strip("/") or "navi1"

        # MQTT params (기본값은 config/mqtt_topics.yaml)
        self.declare_parameter("mqtt.host", str(_m.get("host", "rms.bottle-tak.com")))
        self.declare_parameter("mqtt.port", int(_m.get("port", 80)))
        self.declare_parameter("mqtt.topic_prefix", _pref)
        self.declare_parameter("mqtt.use_websocket", bool(_m.get("use_websocket", True)))
        self.declare_parameter("mqtt.ws_path", str(_m.get("ws_path", "/mqtt")))
        self.declare_parameter("mqtt.username", str(_m.get("username", "") or ""))
        self.declare_parameter("mqtt.password", str(_m.get("password", "") or ""))
        self.declare_parameter("mqtt.nmea_enable", True)
        # paho-mqtt connect()의 keepalive는 정수(초)만 허용 — float 전달 시 "required argument is not an integer"
        self.declare_parameter("mqtt.keepalive_sec", _ka_i)
        self.declare_parameter("reconnect_period_sec", 1.0)

        # ROS topic params
        self.declare_parameter("topic.global_position", "/ligo/global_position")
        self.declare_parameter("topic.odom", "/aft_mapped_to_init")
        self.declare_parameter("topic.receiver_pvt", "/ublox_driver/receiver_pvt")
        self.declare_parameter("topic.heading_align_status", "/ligo/nmea_heading_align_status")
        self.declare_parameter("topic.ligo_mode", "/ligo/mode")
        self.declare_parameter("topic.local_pose", "/ligo/mqtt_pose")
        self.declare_parameter("topic.local_pose_meta", "/ligo/mqtt_pose_meta")

        self.mqtt_host = str(self.get_parameter("mqtt.host").value)
        self.mqtt_port = int(self.get_parameter("mqtt.port").value)
        prefix = str(self.get_parameter("mqtt.topic_prefix").value).strip().strip("/") or "navi1"

        def _mqtt_topic(key: str, fallback_tpl: str) -> str:
            tpl = str(_topics.get(key, "") or "").strip() or fallback_tpl
            return tpl.replace("{prefix}", prefix)

        self.mqtt_topic_position = _mqtt_topic("position", "{prefix}/position")
        self.mqtt_topic_heading = _mqtt_topic("heading", "{prefix}/heading")
        self.mqtt_topic_gps = _mqtt_topic("gps", "{prefix}/gps")
        self.mqtt_topic_icp = _mqtt_topic("init_heading_icp", "{prefix}/init_heading_icp")
        self.mqtt_topic_ligo_mode = _mqtt_topic("ligo_mode", "{prefix}/ligo_mode")
        self.mqtt_use_websocket = bool(self.get_parameter("mqtt.use_websocket").value)
        self.mqtt_ws_path = str(self.get_parameter("mqtt.ws_path").value)
        self.mqtt_username = str(self.get_parameter("mqtt.username").value)
        self.mqtt_password = str(self.get_parameter("mqtt.password").value)
        self.nmea_enable = bool(self.get_parameter("mqtt.nmea_enable").value)
        ka = int(self.get_parameter("mqtt.keepalive_sec").value)
        self._mqtt_keepalive_sec = ka if ka >= 1 else 60

        global_topic = str(self.get_parameter("topic.global_position").value)
        odom_topic = str(self.get_parameter("topic.odom").value)
        receiver_pvt_topic = str(self.get_parameter("topic.receiver_pvt").value)
        align_status_topic = str(self.get_parameter("topic.heading_align_status").value)
        ligo_mode_topic = str(self.get_parameter("topic.ligo_mode").value)
        local_pose_topic = str(self.get_parameter("topic.local_pose").value)
        local_pose_meta_topic = str(self.get_parameter("topic.local_pose_meta").value)

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
        self._lat_lon_cov_max: Optional[float] = None
        self._pvt_is_no_fix: Optional[bool] = None
        self._has_heading_sample: bool = False
        self.local_x: Optional[float] = None
        self.local_y: Optional[float] = None
        self.local_z: Optional[float] = None
        self.local_frame: Optional[str] = None
        self.local_valid: bool = False
        self.local_heading_deg: Optional[float] = None
        self._last_ligo_mode_payload: Optional[dict] = None
        self._ligo_mode_bootstrap_done: bool = False

        self.create_subscription(NavSatFix, global_topic, self.on_global_position, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        if GnssPVTSolnMsg is not None:
            self.create_subscription(GnssPVTSolnMsg, receiver_pvt_topic, self.on_receiver_pvt, 10)
        else:
            self.get_logger().warn(
                "gnss_comm.msg.GnssPVTSolnMsg import 실패: 신호없음(PVT)·ntrip_connected 판정 불가"
            )
        self.create_subscription(NmeaHeadingAlignStatus, align_status_topic, self.on_heading_align_status, 10)
        self.create_subscription(PoseStamped, local_pose_topic, self.on_local_pose, 10)
        self.create_subscription(String, local_pose_meta_topic, self.on_local_pose_meta, 10)
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
            self._mqtt.connect(
                self.mqtt_host, self.mqtt_port, keepalive=self._mqtt_keepalive_sec
            )
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

    def _on_mqtt_disconnect(self, client, userdata, *args):
        # paho-mqtt Callback API v1/v2 시그니처 호환:
        # v1: (client, userdata, rc)
        # v2: (client, userdata, disconnect_flags, reason_code, properties)
        reason_code = None
        if len(args) >= 2:
            reason_code = args[1]
        elif len(args) == 1:
            reason_code = args[0]
        self._mqtt_connected = False
        self.get_logger().warn(f"MQTT disconnected, reason={reason_code!r}")
        self._schedule_next_reconnect()

    def on_global_position(self, msg: NavSatFix) -> None:
        if not self.nmea_enable:
            return
        self.lat = float(msg.latitude)
        self.lon = float(msg.longitude)
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            c = msg.position_covariance
            v_lat = float(c[0])
            v_lon = float(c[4])
            if (
                math.isfinite(v_lat)
                and math.isfinite(v_lon)
                and v_lat >= 0.0
                and v_lon >= 0.0
                and (v_lat > 0.0 or v_lon > 0.0)
            ):
                self._lat_lon_cov_max = max(v_lat, v_lon)
            else:
                self._lat_lon_cov_max = None
        else:
            self._lat_lon_cov_max = None
        self._publish_position()
        if self._refresh_gps_signal_status():
            self._publish_gps()

    def on_odom(self, msg: Odometry) -> None:
        if not self.nmea_enable:
            return
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
        if not self.nmea_enable:
            return
        self._pvt_is_no_fix = (not bool(msg.valid_fix)) or int(msg.fix_type) == 0
        # NTRIP 연결(실사용) 여부 근사: diff solution 또는 carrier solution 존재
        ntrip_connected = bool(msg.diff_soln) or int(msg.carr_soln) in (1, 2)
        self.ntrip_connected = ntrip_connected
        self._refresh_gps_signal_status()
        # gps는 PVT 메시지 수신 시마다 publish
        self._publish_gps()

    def _refresh_gps_signal_status(self) -> bool:
        prev = self.gps_signal_status
        if not self.nmea_enable:
            self.gps_signal_status = "비활성"
            return prev != self.gps_signal_status
        # 신호없음: PVT 무픽스 (기존과 동일). 정상/미약: /ligo/global_position NavSatFix 위·경도 분산 대각 최댓값.
        if self._pvt_is_no_fix is True:
            self.gps_signal_status = "신호없음"
        elif self._pvt_is_no_fix is False:
            if (
                self._lat_lon_cov_max is not None
                and self._lat_lon_cov_max <= GPS_LAT_LON_COV_MAX_NORMAL
            ):
                self.gps_signal_status = "신호정상"
            else:
                self.gps_signal_status = "신호미약"
        else:
            self.gps_signal_status = None
        return prev != self.gps_signal_status

    def on_heading_align_status(self, msg: NmeaHeadingAlignStatus) -> None:
        if not self.nmea_enable:
            return
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

    def on_local_pose(self, msg: PoseStamped) -> None:
        if self.nmea_enable:
            return
        self.local_x = float(msg.pose.position.x)
        self.local_y = float(msg.pose.position.y)
        self.local_z = float(msg.pose.position.z)
        q = msg.pose.orientation
        self.local_heading_deg = (math.degrees(yaw_from_quaternion(q.x, q.y, q.z, q.w))) % 360.0
        self._publish_position()
        self._publish_heading()

    def on_local_pose_meta(self, msg: String) -> None:
        if self.nmea_enable:
            return
        try:
            obj = json.loads(msg.data)
            frame_raw = obj.get("frame")
            if isinstance(frame_raw, str) and frame_raw.strip():
                self.local_frame = frame_raw.strip()
            self.local_valid = bool(obj.get("valid", False))
        except json.JSONDecodeError:
            self.get_logger().warn("local_pose_meta JSON 파싱 실패")

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
        if not self.nmea_enable:
            if self.local_x is None:
                return
            self._publish_json(
                self.mqtt_topic_position,
                {
                    "x": round(self.local_x, 4),
                    "y": round(self.local_y, 4),
                    "z": round(self.local_z, 4),
                },
            )
            return
        if self.lat is None or self.lon is None:
            return
        self._publish_json(
            self.mqtt_topic_position,
            {"lat": self.lat, "lon": self.lon, "coordinate_type": "wgs84"},
        )

    def _publish_heading(self) -> None:
        if not self.nmea_enable:
            if self.local_heading_deg is None:
                return
            self._publish_json(
                self.mqtt_topic_heading,
                {"yaw_deg": round(self.local_heading_deg, 2)},
            )
            return
        if self.heading_deg is None:
            return
        self._publish_json(
            self.mqtt_topic_heading,
            {
                "deg_from_north_cw": self.heading_deg,
                "cardinal": self.heading_dir,
                "coordinate_type": "enu_north",
            },
        )

    def _publish_gps(self) -> None:
        if not self.nmea_enable:
            return
        self._publish_json(
            self.mqtt_topic_gps,
            {"status": self.gps_signal_status, "ntrip_connected": self.ntrip_connected},
        )

    def _publish_icp(self) -> None:
        if not self.nmea_enable:
            return
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
        # disconnect 먼저 → MQTT 네트워크 스레드가 즉시 종료되어 loop_stop 이 빨리 끝남.
        try:
            node._mqtt.disconnect()
        except Exception:
            pass
        try:
            if getattr(node, "_mqtt_loop_started", False):
                node._mqtt.loop_stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
