#!/usr/bin/env python3
from __future__ import annotations

import json
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("paho-mqtt가 필요합니다: pip install paho-mqtt") from exc


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


def _is_valid_map_name(raw: object) -> bool:
    if not isinstance(raw, str):
        return False
    map_name = raw.strip()
    if not map_name:
        return False
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")
    return all(ch in allowed for ch in map_name)


@dataclass
class ModeState:
    mode: str = "idle"
    map_name: str = ""
    running_pid: Optional[int] = None


class ModeOrchestrator:
    def __init__(self) -> None:
        # MQTT
        self.mqtt_host = "rms.bottle-tak.com"
        self.mqtt_port = 80
        self.topic_prefix = "navi1"
        self.mqtt_control_topic = f"{self.topic_prefix}/control/mode"
        self.mqtt_status_topic = f"{self.topic_prefix}/control/mode_status"
        self.mqtt_use_websocket = True
        self.mqtt_ws_path = "/mqtt"
        self.mqtt_username = ""
        self.mqtt_password = ""
        self.keepalive_sec = 10

        # process control
        self.stop_timeout_sec = 10.0
        self._proc: Optional[subprocess.Popen] = None
        self._proc_mode: str = "idle"
        self._proc_map_name: str = ""
        self._lock = threading.Lock()
        self._running = True
        self._mqtt_connected = False
        self._mqtt_connect_requested = False

        self.state = ModeState()
        self._mqtt = self._create_mqtt_client()

    def _create_mqtt_client(self) -> mqtt.Client:
        kwargs = {"transport": "websockets" if self.mqtt_use_websocket else "tcp"}
        if hasattr(mqtt, "CallbackAPIVersion"):
            kwargs["callback_api_version"] = mqtt.CallbackAPIVersion.VERSION2
        client = mqtt.Client(**kwargs)
        if self.mqtt_use_websocket:
            client.ws_set_options(path=self.mqtt_ws_path)
        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)
        # 라이브 네트워크 환경에서 빠른 재연결을 위해 paho 내장 재연결 딜레이를 사용.
        client.reconnect_delay_set(min_delay=1, max_delay=5)

        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        return client

    def _publish_status(
        self,
        event: str,
        status: str,
        message: str,
        mode: Optional[str] = None,
        map_name: Optional[str] = None,
        extra: Optional[dict] = None,
    ) -> None:
        if not self._mqtt_connected:
            return
        payload = {
            "timestamp_unix": time.time(),
            "event": event,
            "status": status,
            "message": message,
            "mode": mode if mode is not None else self.state.mode,
            "map_name": map_name if map_name is not None else self.state.map_name,
        }
        if extra:
            payload.update(extra)
        try:
            info = self._mqtt.publish(
                self.mqtt_status_topic,
                json.dumps(payload, ensure_ascii=False),
                qos=0,
                retain=False,
            )
            if int(getattr(info, "rc", mqtt.MQTT_ERR_UNKNOWN)) != mqtt.MQTT_ERR_SUCCESS:
                self._mqtt_connected = False
                self._schedule_next_reconnect()
        except Exception:
            self._mqtt_connected = False
            self._schedule_next_reconnect()

    def _connect_mqtt(self) -> None:
        if self._mqtt_connect_requested:
            return
        try:
            # connect_async: DNS/네트워크 지연으로 인한 메인 루프 블로킹 방지.
            self._mqtt.connect_async(self.mqtt_host, self.mqtt_port, keepalive=self.keepalive_sec)
            self._mqtt_connect_requested = True
            print(f"[INFO] MQTT 연결 시도 중: {self.mqtt_host}:{self.mqtt_port}")
        except Exception as exc:
            print(f"[WARN] MQTT 연결 실패: {exc}")
            self._mqtt_connected = False
            self._mqtt_connect_requested = False

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        self._mqtt_connected = not _mqtt_reason_failed(reason_code)
        if not self._mqtt_connected:
            print(f"[WARN] MQTT connect 실패: reason={reason_code!r}")
            self._mqtt_connect_requested = False
            return
        self._mqtt_connect_requested = True
        print(f"[INFO] MQTT 연결됨: {self.mqtt_host}:{self.mqtt_port}")
        sub_info = client.subscribe(self.mqtt_control_topic, qos=0)
        if int(getattr(sub_info, "rc", mqtt.MQTT_ERR_UNKNOWN)) != mqtt.MQTT_ERR_SUCCESS:
            self._publish_status(
                event="ready",
                status="error",
                message=f"제어 토픽 구독 실패: rc={getattr(sub_info, 'rc', None)}",
            )
            print(f"[ERROR] MQTT subscribe 실패: topic={self.mqtt_control_topic}, rc={getattr(sub_info, 'rc', None)}")
            return
        print(f"[INFO] MQTT 구독 완료: {self.mqtt_control_topic}")
        self._publish_status(event="ready", status="ok", message="모드 제어 대기 중입니다.")

    def _on_disconnect(self, client, userdata, *args) -> None:
        reason_code = args[1] if len(args) >= 2 else (args[0] if len(args) == 1 else None)
        print(f"[WARN] MQTT 연결 해제: reason={reason_code!r}")
        self._mqtt_connected = False
        self._mqtt_connect_requested = False

    def _build_launch_command(self, mode: str, map_name: str) -> list[str]:
        if mode == "mapping":
            return [
                "ros2",
                "launch",
                "ligo",
                "nx_mapping.launch.py",
                f"map_name:={map_name}",
            ]
        return ["ros2", "launch", "ligo", "nx_odometry.launch.py"]

    def _start_mode(self, mode: str, map_name: str = "") -> None:
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                self._publish_status(
                    event="start",
                    status="error",
                    message="이미 실행 중인 모드가 있습니다. 먼저 stop 메시지를 보내주세요.",
                )
                return

            cmd = self._build_launch_command(mode=mode, map_name=map_name)
            try:
                proc = subprocess.Popen(cmd)
            except Exception as exc:
                self._publish_status(
                    event="start",
                    status="error",
                    message=f"모드 실행 실패: {exc}",
                    mode=mode,
                    map_name=map_name,
                )
                return

            self._proc = proc
            self._proc_mode = mode
            self._proc_map_name = map_name
            self.state.mode = mode
            self.state.map_name = map_name
            self.state.running_pid = proc.pid

        self._publish_status(
            event="start",
            status="ok",
            message=f"{mode} 모드 실행을 시작했습니다.",
            mode=mode,
            map_name=map_name,
            extra={"pid": proc.pid, "command": " ".join(cmd)},
        )
        print(f"[INFO] started: mode={mode}, map_name={map_name}, pid={proc.pid}")
        # 즉시 종료되는 경우(잘못된 launch 인자/환경) 사용자에게 빠르게 노출.
        time.sleep(0.2)
        early_rc = proc.poll()
        if early_rc is not None:
            with self._lock:
                self._proc = None
                self._proc_mode = "idle"
                self._proc_map_name = ""
                self.state = ModeState()
            self._publish_status(
                event="start",
                status="error",
                message="모드 프로세스가 시작 직후 종료되었습니다. journalctl 로그를 확인하세요.",
                mode=mode,
                map_name=map_name,
                extra={"exit_code": early_rc, "command": " ".join(cmd)},
            )
            print(f"[ERROR] early-exit: mode={mode}, map_name={map_name}, exit_code={early_rc}")

    def _stop_mode(self, reason: str) -> None:
        with self._lock:
            proc = self._proc
            mode = self._proc_mode
            map_name = self._proc_map_name
            if proc is None or proc.poll() is not None:
                self._proc = None
                self._proc_mode = "idle"
                self._proc_map_name = ""
                self.state = ModeState()
                self._publish_status(
                    event="stop",
                    status="ok",
                    message="종료할 실행 중 모드가 없습니다.",
                )
                return

        try:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=self.stop_timeout_sec)
            exit_code = proc.returncode
            stop_msg = "모드를 정상 종료했습니다."
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5.0)
            exit_code = proc.returncode
            stop_msg = "모드 종료가 지연되어 강제 종료했습니다."

        with self._lock:
            self._proc = None
            self._proc_mode = "idle"
            self._proc_map_name = ""
            self.state = ModeState()

        self._publish_status(
            event="stop",
            status="ok",
            message=stop_msg,
            mode=mode,
            map_name=map_name,
            extra={"exit_code": exit_code, "reason": reason},
        )
        print(f"[INFO] stopped: mode={mode}, map_name={map_name}, exit_code={exit_code}, reason={reason}")

    def _parse_control(self, payload: str) -> tuple[Optional[str], Optional[str], Optional[str]]:
        """
        Returns:
          (command, mode, error_message)
          command: "start_mapping" | "start_odometry" | "stop" | None
          mode: mapping/odometry/idle or None
        """
        try:
            obj = json.loads(payload)
        except json.JSONDecodeError:
            return None, None, "제어 메시지가 JSON 형식이 아닙니다."

        command = str(obj.get("command", "")).strip().lower()
        if command == "stop":
            return "stop", "idle", None

        if command not in ("", "start"):
            return None, None, "command는 start 또는 stop 이어야 합니다."

        mapping_mode = obj.get("mapping_mode")
        if not isinstance(mapping_mode, bool):
            return None, None, "mapping_mode(boolean) 필드가 필요합니다."

        if mapping_mode:
            map_name = obj.get("map_name")
            if not _is_valid_map_name(map_name):
                return None, None, "mapping_mode=true일 때 map_name은 영문/숫자/_/- 조합의 필수 문자열입니다."
            return "start_mapping", str(map_name).strip(), None

        return "start_odometry", "", None

    def _on_message(self, client, userdata, msg) -> None:
        payload = msg.payload.decode("utf-8", errors="replace")
        command, mode_or_map, error = self._parse_control(payload)
        if error:
            self._publish_status(event="control", status="error", message=error)
            print(f"[WARN] invalid control message: {error}, payload={payload}")
            return

        if command == "stop":
            self._stop_mode(reason="remote_stop_message")
            return
        if command == "start_odometry":
            self._start_mode(mode="odometry")
            return
        if command == "start_mapping":
            self._start_mode(mode="mapping", map_name=mode_or_map)
            return

        self._publish_status(event="control", status="error", message="알 수 없는 제어 명령입니다.")

    def _poll_process_exit(self) -> None:
        with self._lock:
            proc = self._proc
            mode = self._proc_mode
            map_name = self._proc_map_name
            if proc is None:
                return
            rc = proc.poll()
            if rc is None:
                return
            self._proc = None
            self._proc_mode = "idle"
            self._proc_map_name = ""
            self.state = ModeState()

        status = "ok" if rc == 0 else "error"
        message = "모드 실행이 종료되었습니다." if rc == 0 else "모드 실행이 비정상 종료되었습니다."
        self._publish_status(
            event="ended",
            status=status,
            message=message,
            mode=mode,
            map_name=map_name,
            extra={"exit_code": rc},
        )
        print(f"[INFO] ended: mode={mode}, map_name={map_name}, exit_code={rc}")

    def shutdown(self) -> None:
        self._running = False
        self._stop_mode(reason="service_shutdown")
        try:
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
        except Exception:
            pass

    def run(self) -> None:
        self._connect_mqtt()
        self._mqtt.loop_start()
        print(f"[INFO] control_topic={self.mqtt_control_topic}, status_topic={self.mqtt_status_topic}")
        try:
            while self._running:
                if not self._mqtt_connected:
                    self._connect_mqtt()
                self._poll_process_exit()
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("[INFO] 종료 신호를 받았습니다.")
        finally:
            self.shutdown()


def main() -> None:
    orchestrator = ModeOrchestrator()
    orchestrator.run()


if __name__ == "__main__":
    main()
