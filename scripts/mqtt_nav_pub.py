#!/usr/bin/env python3
"""
MQTT로 시작/목표 위경도 메시지를 1회 발행하는 유틸.

기본값은 본 프로젝트 브로커 설정(웹소켓 80 /mqtt)에 맞춰져 있다.

예시:
  ros2 run ligo mqtt_nav_pub.py --kind goal --lat 37.41200 --lon 127.09320
  ros2 run ligo mqtt_nav_pub.py --kind start_pose --lat 37.41150000 --lon 127.09310000 --heading 90.0
"""

from __future__ import annotations

import argparse
import json
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("paho-mqtt가 필요합니다: pip install paho-mqtt") from exc

from mqtt_config import load_mqtt_config, topic_prefix

_MQTT = load_mqtt_config().get("mqtt", {}) or {}


def _mqtt_reason_failed(reason_code: object) -> bool:
    """paho-mqtt v2 ReasonCode / 레거시 int 모두 대응."""
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
    name = str(reason_code).lower()
    return name not in ("success", "0", "no error", "no_error")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Publish start/goal lat-lon to MQTT")
    p.add_argument("--host", default=str(_MQTT.get("host", "rms.bottle-tak.com")), help="MQTT broker host")
    p.add_argument("--port", type=int, default=int(_MQTT.get("port", 80)), help="MQTT broker port")
    p.add_argument("--topic-prefix", default=topic_prefix(), help="Topic prefix")
    p.add_argument(
        "--kind",
        choices=["start", "goal", "current", "start_pose"],
        default="goal",
        help="Message kind",
    )
    p.add_argument("--topic", default="", help="Full topic override (if set, --kind ignored)")
    p.add_argument("--lat", type=float, default=None, help="Latitude")
    p.add_argument("--lon", type=float, default=None, help="Longitude")
    p.add_argument("--heading", type=float, default=None, help="Heading deg_from_north_cw")
    p.add_argument("--transport", choices=["websockets", "tcp"], default="websockets", help="MQTT transport")
    p.add_argument("--ws-path", default="/mqtt", help="WebSocket path")
    p.add_argument("--username", default="", help="MQTT username")
    p.add_argument("--password", default="", help="MQTT password")
    p.add_argument("--qos", type=int, choices=[0, 1, 2], default=0, help="MQTT QoS")
    p.add_argument("--retain", action="store_true", help="Publish with retain=true")
    return p


def main() -> int:
    args = build_parser().parse_args()
    prefix = args.topic_prefix.strip().strip("/")
    topic = args.topic if args.topic else f"{prefix}/nav/{args.kind}"

    if args.kind == "start_pose":
        if args.lat is None or args.lon is None:
            print("[ERROR] start_pose에는 --lat, --lon이 필요합니다.", file=sys.stderr)
            return 2
        if not (-90.0 <= args.lat <= 90.0 and -180.0 <= args.lon <= 180.0):
            print("[ERROR] lat/lon 범위를 확인하세요.", file=sys.stderr)
            return 2
        payload = {"lat": args.lat, "lon": args.lon}
        if args.heading is not None:
            payload["deg_from_north_cw"] = args.heading
    else:
        if args.lat is None or args.lon is None:
            print("[ERROR] 이 kind에는 --lat, --lon이 필요합니다.", file=sys.stderr)
            return 2
        if not (-90.0 <= args.lat <= 90.0 and -180.0 <= args.lon <= 180.0):
            print("[ERROR] lat/lon 범위를 확인하세요.", file=sys.stderr)
            return 2
        payload = {"lat": args.lat, "lon": args.lon}
        if args.heading is not None:
            payload["deg_from_north_cw"] = args.heading
    payload_text = json.dumps(payload, ensure_ascii=False)

    done = {"connected": False, "published": False, "failed": False}
    state = {"mid": None}

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        transport=args.transport,
    )
    if args.transport == "websockets":
        client.ws_set_options(path=args.ws_path)
    if args.username:
        client.username_pw_set(args.username, args.password)

    def on_connect(c, userdata, flags, reason_code, properties):
        if _mqtt_reason_failed(reason_code):
            print(f"[ERROR] MQTT connect failed: {reason_code!r}", file=sys.stderr)
            done["failed"] = True
            return
        done["connected"] = True
        info = c.publish(topic, payload_text, qos=args.qos, retain=bool(args.retain))
        state["mid"] = info.mid

    def on_publish(c, userdata, mid, reason_code, properties):
        if _mqtt_reason_failed(reason_code):
            print(f"[ERROR] MQTT publish failed: {reason_code!r}", file=sys.stderr)
            done["failed"] = True
            c.disconnect()
            return
        if state["mid"] is None or mid == state["mid"]:
            done["published"] = True
            c.disconnect()

    def on_disconnect(c, userdata, disconnect_flags, reason_code, properties):
        pass

    client.on_connect = on_connect
    client.on_publish = on_publish
    client.on_disconnect = on_disconnect

    try:
        client.connect(args.host, args.port, keepalive=30)
    except Exception as exc:
        print(f"[ERROR] connect 예외: {exc}", file=sys.stderr)
        if args.transport == "websockets":
            print("[HINT] 이 브로커는 websockets(/mqtt) 설정이 필요할 수 있습니다.", file=sys.stderr)
        return 3

    client.loop_start()
    t0 = time.time()
    while time.time() - t0 < 5.0:
        if done["published"]:
            break
        if done["failed"]:
            break
        time.sleep(0.05)
    client.loop_stop()

    if done["published"]:
        print(f"[OK] {topic} <- {payload_text}")
        return 0

    if done["connected"]:
        print("[ERROR] 연결은 되었지만 publish 확인에 실패했습니다.", file=sys.stderr)
    else:
        print("[ERROR] MQTT 연결에 실패했습니다.", file=sys.stderr)
    return 4


if __name__ == "__main__":
    raise SystemExit(main())

