#!/usr/bin/env python3
import argparse
import datetime
import pathlib
import sys

import paho.mqtt.client as mqtt


def build_parser():
    parser = argparse.ArgumentParser(
        description="구독되는 모든 MQTT 토픽 메시지를 txt 파일로 저장합니다."
    )
    parser.add_argument("--host", default="rms.bottle-tak.com", help="MQTT 브로커 호스트")
    parser.add_argument("--port", type=int, default=80, help="MQTT 브로커 포트")
    parser.add_argument("--topic", default="navi1/+", help="구독 토픽 (기본: #)")
    parser.add_argument(
        "--transport",
        default="websockets",
        choices=["websockets", "tcp"],
        help="MQTT 전송 방식",
    )
    parser.add_argument(
        "--ws-path",
        default="/mqtt",
        help="WebSocket path (transport가 websockets일 때 사용)",
    )
    parser.add_argument(
        "--output",
        default="mqtt_messages.txt",
        help="메시지 저장 파일 경로",
    )
    return parser


class MqttAllTopicLogger:
    def __init__(self, args):
        self.args = args
        self.output_path = pathlib.Path(args.output).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.client = mqtt.Client(transport=args.transport)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        if args.transport == "websockets":
            self.client.ws_set_options(path=args.ws_path)

    def append_log(self, line):
        with self.output_path.open("a", encoding="utf-8") as file:
            file.write(line + "\n")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f"[INFO] MQTT 연결 성공: {self.args.host}:{self.args.port}")
            subscribe_result, _ = client.subscribe(self.args.topic)
            if subscribe_result == mqtt.MQTT_ERR_SUCCESS:
                print(f"[INFO] 구독 시작: {self.args.topic}")
            else:
                print(
                    f"[ERROR] 구독 실패 (코드: {subscribe_result}) - 토픽을 확인해주세요."
                )
        else:
            print(f"[ERROR] MQTT 연결 실패 (코드: {rc}) - 서버 상태를 확인해주세요.")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("[WARN] 연결이 끊어졌습니다. 자동 재연결을 시도합니다.")
        else:
            print("[INFO] MQTT 연결 종료")

    def on_message(self, client, userdata, msg):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            payload = msg.payload.decode("utf-8")
        except UnicodeDecodeError:
            payload = msg.payload.decode("utf-8", errors="replace")

        log_line = f"{timestamp} | topic={msg.topic} | payload={payload}"
        self.append_log(log_line)
        print(log_line)

    def run(self):
        try:
            print(f"[INFO] 로그 저장 파일: {self.output_path}")
            self.client.connect(self.args.host, self.args.port, keepalive=60)
            self.client.loop_forever()
        except KeyboardInterrupt:
            print("\n[INFO] 사용자 요청으로 종료합니다.")
            self.client.disconnect()
        except Exception as exc:
            print(f"[ERROR] MQTT 처리 중 예외 발생: {exc}")
            print("[HINT] host/port/ws-path/네트워크 상태를 확인해주세요.")
            sys.exit(1)


def main():
    args = build_parser().parse_args()
    logger = MqttAllTopicLogger(args)
    logger.run()


if __name__ == "__main__":
    main()
