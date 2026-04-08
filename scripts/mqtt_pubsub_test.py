#!/usr/bin/env python3
"""
Simple MQTT pub/sub connectivity test for LIGO topics.

What it does:
1) Connect to MQTT broker (websocket/tcp).
2) Subscribe to <topic_prefix>/#.
3) Publish test message to <topic_prefix>/test_ping.
4) Print incoming messages and basic counters.
5) Auto-verify required LIGO topics were received at least once.

Usage examples:
  python3 scripts/mqtt_pubsub_test.py
  python3 scripts/mqtt_pubsub_test.py --host rms.bottle-tak.com --port 80 --websocket --ws-path /mqtt
  python3 scripts/mqtt_pubsub_test.py --topic-prefix navi1 --duration 20
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import dataclass

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("paho-mqtt가 필요합니다: pip install paho-mqtt") from exc


@dataclass
class Stats:
    connected: bool = False
    recv_count: int = 0
    ping_echo_seen: bool = False


def main() -> int:
    ap = argparse.ArgumentParser(description="MQTT pub/sub test")
    ap.add_argument("--host", default="rms.bottle-tak.com", help="MQTT broker host")
    ap.add_argument("--port", type=int, default=80, help="MQTT broker port")
    ap.add_argument("--topic-prefix", default="navi1", help="Topic prefix (e.g. navi1)")
    ap.add_argument("--websocket", action="store_true", default=True, help="Use websocket transport (default: true)")
    ap.add_argument("--tcp", action="store_true", help="Use plain TCP instead of websocket")
    ap.add_argument("--ws-path", default="/mqtt", help="Websocket path")
    ap.add_argument("--username", default="", help="MQTT username")
    ap.add_argument("--password", default="", help="MQTT password")
    ap.add_argument("--qos", type=int, default=0, choices=[0, 1, 2], help="QoS for subscribe/publish")
    ap.add_argument("--duration", type=float, default=15.0, help="Test duration seconds")
    ap.add_argument(
        "--skip-required-check",
        action="store_true",
        help="Do not fail when required LIGO topics are not seen",
    )
    args = ap.parse_args()

    use_websocket = False if args.tcp else bool(args.websocket)
    prefix = args.topic_prefix.strip().strip("/")
    sub_topic = f"{prefix}/#"
    ping_topic = f"{prefix}/test_ping"
    ping_id = f"ping-{int(time.time() * 1000)}"
    required_topics = {
        f"{prefix}/position",
        f"{prefix}/heading",
        f"{prefix}/gps",
        f"{prefix}/init_heading_icp",
    }
    seen_required_topics: set[str] = set()

    stats = Stats()
    stop = False

    def _sig_handler(signum, frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    client = mqtt.Client(transport="websockets" if use_websocket else "tcp")
    if use_websocket:
        client.ws_set_options(path=args.ws_path)
    if args.username:
        client.username_pw_set(args.username, args.password)

    def on_connect(c, userdata, flags, rc):
        if rc == 0:
            stats.connected = True
            print(f"[OK] connected to {args.host}:{args.port}")
            c.subscribe(sub_topic, qos=args.qos)
            print(f"[SUB] {sub_topic} (qos={args.qos})")
            payload = {
                "kind": "mqtt_connectivity_test",
                "id": ping_id,
                "ts_unix": time.time(),
            }
            c.publish(ping_topic, json.dumps(payload, ensure_ascii=False), qos=args.qos, retain=False)
            print(f"[PUB] {ping_topic} -> id={ping_id}")
        else:
            print(f"[ERR] connect failed rc={rc}")

    def on_message(c, userdata, msg):
        stats.recv_count += 1
        text = msg.payload.decode("utf-8", errors="replace")
        print(f"[RECV] {msg.topic} {text}")
        if msg.topic in required_topics:
            seen_required_topics.add(msg.topic)
        if msg.topic == ping_topic and ping_id in text:
            stats.ping_echo_seen = True

    def on_disconnect(c, userdata, rc):
        stats.connected = False
        print(f"[INFO] disconnected rc={rc}")

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    try:
        client.connect(args.host, args.port, keepalive=30)
    except Exception as exc:
        print(f"[ERR] connect exception: {exc}")
        return 2

    client.loop_start()

    start = time.time()
    while not stop and (time.time() - start) < args.duration:
        time.sleep(0.2)

    client.loop_stop()
    try:
        client.disconnect()
    except Exception:
        pass

    print("\n=== SUMMARY ===")
    print(f"connected_once: {stats.connected or stats.recv_count > 0}")
    print(f"received_count: {stats.recv_count}")
    print(f"ping_echo_seen: {stats.ping_echo_seen}")
    print("required_topics_seen:")
    for t in sorted(required_topics):
        ok = t in seen_required_topics
        print(f"  - {t}: {'OK' if ok else 'MISSING'}")

    if stats.recv_count == 0:
        print("[WARN] 수신 메시지가 없습니다. 브로커/토픽/권한을 확인하세요.")
        return 1
    if not args.skip_required_check:
        missing = sorted(required_topics - seen_required_topics)
        if missing:
            print("[FAIL] 필수 LIGO MQTT 토픽 일부를 수신하지 못했습니다.")
            print("       (ligo_topic_to_mqtt.py 실행 여부 / topic_prefix / duration 확인)")
            return 3
        print("[PASS] 필수 LIGO MQTT 토픽 4종 수신 확인.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
