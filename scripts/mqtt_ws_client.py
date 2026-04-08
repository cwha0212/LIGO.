#!/usr/bin/env python3
"""
ROS /ligo/mode 구독 → MQTT 로 실내·실외 전환 알림.

  source .../install/setup.bash
  python3 scripts/mqtt_ws_client.py

MQTT 전송: ``mqtt_publish(TOPIC_LIGO_MODE, {"mode": "indoor", "pcd_name": "..."})`` 형태 (dict → JSON).

의존성: pip install paho-mqtt, ROS 2 rclpy

---------------------------------------------------------------------------
터미널에서 MQTT 구독해 보기
---------------------------------------------------------------------------

1) 브로커가 **일반 TCP**(예: 1883) 인 경우::

    mosquitto_sub -h rms.bottle-tak.com -p 1883 -t 'nav1/mode' -v

2) 이 스크립트와 동일 **WebSocket**(80, ``/mqtt``) 인 경우 (paho-mqtt)::

    python3 -c "
    import paho.mqtt.client as mqtt
    def on_connect(c, u, f, rc, p=None):
        if rc==0: c.subscribe('nav1/mode', qos=1)
    def on_message(c, u, m):
        print(m.topic, m.payload.decode('utf-8', errors='replace'), flush=True)
    cli = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1, transport='websockets')
    cli.ws_set_options(path='/mqtt')
    cli.on_connect, cli.on_message = on_connect, on_message
    cli.connect('rms.bottle-tak.com', 80)
    cli.loop_forever()
    "

토픽 이름은 스크립트 안 ``TOPIC_LIGO_MODE`` 와 맞출 것 (기본 ``nav1/mode``).
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


# =============================================================================
# 브로커 (WebSocket)
# =============================================================================

MQTT_HOST = "rms.bottle-tak.com"
MQTT_PORT = 80
WS_PATH = "/mqtt"

MQTT_RECONNECT_MIN_DELAY = 1
MQTT_RECONNECT_MAX_DELAY = 60
MQTT_INITIAL_RETRY_SEC = 3.0

# (선택) 이 스크립트가 MQTT 에서 구독할 토픽 — 필요 시 튜플에 문자열 추가
MQTT_SUBSCRIBE_TOPICS: tuple[str, ...] = ()


# =============================================================================
# ROS ↔ MQTT 토픽 이름
# =============================================================================

ROS_TOPIC_LIGO_MODE = "/ligo/mode"

# MQTT 쪽 토픽 (nav2 로 바꾸려면 여기만 수정)
TOPIC_LIGO_MODE = "nav1/mode"


# =============================================================================
# MQTT 클라이언트 (백그라운드 스레드)
# =============================================================================

mqtt_client: mqtt.Client | None = None
_mqtt_stop = threading.Event()
_mqtt_pub_lock = threading.Lock()


def mqtt_publish(topic: str, payload: dict, *, qos: int = 1) -> None:
    """
    MQTT 로 JSON 객체를 보낸다. 예전 스타일과 같이 토픽 + dict 만 보면 됨.

        mqtt_publish(TOPIC_LIGO_MODE, {"mode": "indoor", "pcd_name": "scans_3.pcd"})
        mqtt_publish(TOPIC_LIGO_MODE, {"mode": "outdoor"})
    """
    global mqtt_client
    if mqtt_client is None:
        return
    if hasattr(mqtt_client, "is_connected") and not mqtt_client.is_connected():
        return
    body = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    with _mqtt_pub_lock:
        mqtt_client.publish(topic, body.encode("utf-8"), qos=qos)


def mqtt_publish_raw_bytes(topic: str, payload: bytes, *, qos: int = 1) -> None:
    """JSON 이 아닌 바이트 그대로 보낼 때만 사용."""
    global mqtt_client
    if mqtt_client is None:
        return
    if hasattr(mqtt_client, "is_connected") and not mqtt_client.is_connected():
        return
    with _mqtt_pub_lock:
        mqtt_client.publish(topic, payload, qos=qos)


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
    """laserMapping 이 준 JSON 을 MQTT 용 dict 로 정리 (wire 는 mode / pcd_name 만)."""
    mode = ros_obj.get("mode")
    if mode == "indoor":
        return {"mode": "indoor", "pcd_name": _indoor_pcd_name(ros_obj)}
    if mode == "outdoor":
        return {"mode": "outdoor"}
    return {"mode": str(mode), "raw": ros_obj}


def _make_mqtt_client() -> mqtt.Client:
    try:
        return mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
            transport="websockets",
        )
    except AttributeError:
        return mqtt.Client(transport="websockets")


def mqtt_on_connect(client: mqtt.Client, userdata, flags, rc, properties=None):
    if rc != 0:
        print(f"[MQTT] 연결 실패 rc={rc}", file=sys.stderr, flush=True)
        return
    print("[MQTT] WebSocket 연결", flush=True)
    for t in MQTT_SUBSCRIBE_TOPICS:
        client.subscribe(t, qos=1)


def mqtt_on_message(client: mqtt.Client, userdata, msg):
    print(f"[MQTT] {msg.topic} {msg.payload.decode('utf-8', errors='replace')}", flush=True)


def mqtt_on_disconnect(client: mqtt.Client, userdata, *rest):
    rc = rest[0] if rest else "?"
    print(f"[MQTT] 끊김 rc={rc} (재연결)", flush=True)


def start_mqtt_loop() -> None:
    global mqtt_client
    mqtt_client = _make_mqtt_client()
    mqtt_client.ws_set_options(path=WS_PATH)
    mqtt_client.reconnect_delay_set(MQTT_RECONNECT_MIN_DELAY, MQTT_RECONNECT_MAX_DELAY)
    mqtt_client.on_connect = mqtt_on_connect
    mqtt_client.on_message = mqtt_on_message
    mqtt_client.on_disconnect = mqtt_on_disconnect

    while not _mqtt_stop.is_set():
        try:
            mqtt_client.connect(MQTT_HOST, MQTT_PORT)
            break
        except OSError as e:
            print(
                f"[MQTT] connect 실패, {MQTT_INITIAL_RETRY_SEC:.0f}s 후 재시도: {e}",
                file=sys.stderr,
                flush=True,
            )
            if _mqtt_stop.wait(timeout=MQTT_INITIAL_RETRY_SEC):
                mqtt_client = None
                return
    if _mqtt_stop.is_set():
        mqtt_client = None
        return

    mqtt_client.loop_start()
    _mqtt_stop.wait()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    mqtt_client = None


# =============================================================================
# ROS
# =============================================================================


class RosTap(Node):
    def __init__(self) -> None:
        super().__init__("mqtt_ws_client")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(String, ROS_TOPIC_LIGO_MODE, self._on_ligo_mode, qos)
        self.get_logger().info(f"ROS subscribe {ROS_TOPIC_LIGO_MODE}  →  MQTT publish {TOPIC_LIGO_MODE}")

    def _on_ligo_mode(self, msg: String) -> None:
        try:
            ros_obj = json.loads(msg.data)
        except json.JSONDecodeError:
            print(f"[ROS→MQTT] JSON 아님, raw 그대로 전달: {msg.data!r}", flush=True)
            mqtt_publish_raw_bytes(TOPIC_LIGO_MODE, msg.data.encode("utf-8"))
            return

        out = ligo_mode_payload_for_mqtt(ros_obj)

        # 로그: 한눈에 어떤 dict 가 나갔는지
        print(f"[ROS→MQTT] mqtt_publish({TOPIC_LIGO_MODE!r}, {out})", flush=True)

        mqtt_publish(TOPIC_LIGO_MODE, out)


# =============================================================================
# main
# =============================================================================


def main() -> int:
    global mqtt_client
    _mqtt_stop.clear()

    t = threading.Thread(target=start_mqtt_loop, daemon=True)
    t.start()

    for _ in range(100):
        if _mqtt_stop.is_set():
            break
        if mqtt_client is not None and hasattr(mqtt_client, "is_connected") and mqtt_client.is_connected():
            break
        time.sleep(0.05)

    rclpy.init()
    print(f"[mqtt_ws_client] 스크립트: {Path(__file__).resolve()}", flush=True)
    print(f"[mqtt_ws_client] MQTT {MQTT_HOST}:{MQTT_PORT} WS{WS_PATH}  →  topic {TOPIC_LIGO_MODE}", flush=True)
    print(f"[mqtt_ws_client] ROS {ROS_TOPIC_LIGO_MODE}  →  MQTT publish", flush=True)
    node = RosTap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        _mqtt_stop.set()
        t.join(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
