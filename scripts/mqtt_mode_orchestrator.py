#!/usr/bin/env python3
from __future__ import annotations

import collections
import json
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("paho-mqtt가 필요합니다: pip install paho-mqtt") from exc

from mqtt_config import (
    SHARED_MAP_ROOT,
    load_mqtt_config,
    local_map_root,
    resolve_topic,
    rsync_options,
    rsync_upload_options,
    topic_prefix,
)


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
    sub_map_name: str = ""
    indoor_map_names: List[str] = field(default_factory=list)
    running_pid: Optional[int] = None


@dataclass
class ParsedControl:
    kind: str
    map_name: str = ""
    sub_map_name: str = ""
    indoor_map_names: List[str] = field(default_factory=list)
    error: Optional[str] = None


class ModeOrchestrator:
    def __init__(self) -> None:
        self._cfg = load_mqtt_config()
        m = self._cfg.get("mqtt", {}) or {}

        self.mqtt_host = str(m.get("host", "rms.bottle-tak.com"))
        self.mqtt_port = int(m.get("port", 80))
        self.topic_prefix = topic_prefix(self._cfg)
        self.mqtt_control_topic = resolve_topic("control_mode", self._cfg)
        self.mqtt_status_topic = resolve_topic("control_status", self._cfg)
        self.mqtt_use_websocket = bool(m.get("use_websocket", True))
        self.mqtt_ws_path = str(m.get("ws_path", "/mqtt"))
        self.mqtt_username = str(m.get("username", "") or "")
        self.mqtt_password = str(m.get("password", "") or "")
        ka = m.get("keepalive_sec", 10)
        try:
            self.keepalive_sec = int(ka) if int(ka) >= 1 else 10
        except (TypeError, ValueError):
            self.keepalive_sec = 10

        orch = self._cfg.get("orchestrator", {}) or {}

        def _pos_float(value: object, fallback: float) -> float:
            try:
                v = float(value)
                return v if v > 0 else fallback
            except (TypeError, ValueError):
                return fallback

        self.stop_timeout_sec_mapping = _pos_float(orch.get("stop_timeout_sec_mapping"), 86400.0)
        self.stop_timeout_sec_odometry = _pos_float(orch.get("stop_timeout_sec_odometry"), 10.0)
        self.start_confirm_sec = _pos_float(orch.get("start_confirm_sec"), 1.5)
        self._proc: Optional[subprocess.Popen] = None
        self._proc_mode: str = "idle"
        self._proc_map_name: str = ""
        self._proc_sub_map_name: str = ""
        self._proc_indoor_map_names: List[str] = []
        self._sync_thread: Optional[threading.Thread] = None
        self._stop_thread: Optional[threading.Thread] = None
        # stop 진행 중에는 main loop 의 _poll_process_exit 가 중복 처리하지 않도록 함.
        self._stop_in_progress: bool = False
        self._lock = threading.Lock()
        # 연결 단절 동안 잃지 않도록 status 메시지를 보관하는 링 버퍼. 재연결되면 flush.
        self._pending_status: collections.deque = collections.deque(maxlen=200)
        self._pending_lock = threading.Lock()
        self._running = True
        self._mqtt_connected = False
        self._mqtt_connect_requested = False
        self._next_reconnect_not_before = 0.0
        self._reconnect_backoff_sec = 1.0
        self._reconnect_backoff_max_sec = 30.0

        self.state = ModeState()
        self._mqtt = self._create_mqtt_client()

    @staticmethod
    def _signal_proc_group(proc: subprocess.Popen, sig: signal.Signals) -> None:
        """proc(=ros2 launch)와 모든 자식 노드가 속한 process group 전체에 시그널 전송.
        start_new_session=True 로 띄웠다는 전제. 실패 시 단일 프로세스 시그널로 폴백."""
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, sig)
            return
        except (ProcessLookupError, PermissionError, OSError):
            pass
        try:
            proc.send_signal(sig)
        except Exception:
            pass

    def _busy_locked(self) -> bool:
        if self._stop_in_progress:
            return True
        if self._proc is not None and self._proc.poll() is None:
            return True
        t = self._sync_thread
        return t is not None and t.is_alive()

    def _busy_snapshot(self) -> Optional[dict]:
        """현재 점유 중인 작업의 종류·식별자를 반환. 점유 없음이면 None."""
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                snap: dict = {
                    "running": self._proc_mode,
                    "pid": self._proc.pid,
                    "map_name": self._proc_map_name,
                }
                if self._proc_mode == "mapping" and self._proc_sub_map_name:
                    snap["sub_map_name"] = self._proc_sub_map_name
                if self._proc_mode == "odometry" and self._proc_indoor_map_names:
                    snap["indoor_map_names"] = list(self._proc_indoor_map_names)
                return snap
            t = self._sync_thread
            if t is not None and t.is_alive():
                return {"running": "sync", "thread": t.name}
        return None

    def _schedule_next_reconnect(self) -> None:
        self._next_reconnect_not_before = time.time() + self._reconnect_backoff_sec
        self._reconnect_backoff_sec = min(
            self._reconnect_backoff_sec * 2.0,
            self._reconnect_backoff_max_sec,
        )

    def _reset_reconnect_backoff(self) -> None:
        self._reconnect_backoff_sec = 1.0
        self._next_reconnect_not_before = 0.0

    def _create_mqtt_client(self) -> mqtt.Client:
        kwargs = {"transport": "websockets" if self.mqtt_use_websocket else "tcp"}
        if hasattr(mqtt, "CallbackAPIVersion"):
            kwargs["callback_api_version"] = mqtt.CallbackAPIVersion.VERSION2
        client = mqtt.Client(**kwargs)
        if self.mqtt_use_websocket:
            client.ws_set_options(path=self.mqtt_ws_path)
        if self.mqtt_username:
            client.username_pw_set(self.mqtt_username, self.mqtt_password)
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
        sub_map_name: Optional[str] = None,
        extra: Optional[dict] = None,
    ) -> None:
        payload = {
            "timestamp_unix": time.time(),
            "event": event,
            "status": status,
            "message": message,
            "mode": mode if mode is not None else self.state.mode,
            "map_name": map_name if map_name is not None else self.state.map_name,
        }
        if sub_map_name is not None:
            payload["sub_map_name"] = sub_map_name
        if extra:
            payload.update(extra)
        payload_str = json.dumps(payload, ensure_ascii=False)
        # 연결 상태와 관계없이 큐에 넣어두고, 연결돼 있으면 즉시 flush.
        # 끊긴 사이에 발생한 stop/map_saved/sync 같은 결과 메시지를 잃지 않도록 한다.
        with self._pending_lock:
            self._pending_status.append(payload_str)
            if self._mqtt_connected:
                self._flush_pending_status_locked()

    def _flush_pending_status_locked(self) -> None:
        """_pending_lock 보유 상태에서 호출. publish 실패 시 큐는 유지하고 재연결 예약."""
        while self._pending_status:
            payload_str = self._pending_status[0]
            try:
                info = self._mqtt.publish(
                    self.mqtt_status_topic,
                    payload_str,
                    qos=0,
                    retain=False,
                )
                rc = int(getattr(info, "rc", mqtt.MQTT_ERR_UNKNOWN))
            except Exception:
                self._mark_mqtt_broken()
                return
            if rc != int(mqtt.MQTT_ERR_SUCCESS):
                self._mark_mqtt_broken()
                return
            self._pending_status.popleft()

    def _mark_mqtt_broken(self) -> None:
        self._mqtt_connected = False
        self._mqtt_connect_requested = False
        try:
            self._mqtt.disconnect()
        except Exception:
            pass
        self._schedule_next_reconnect()

    def _connect_mqtt(self) -> None:
        now = time.time()
        if now < self._next_reconnect_not_before:
            return
        if self._mqtt_connect_requested:
            return
        try:
            self._mqtt.connect_async(self.mqtt_host, self.mqtt_port, keepalive=self.keepalive_sec)
            self._mqtt_connect_requested = True
            print(f"[INFO] MQTT 연결 시도 중: {self.mqtt_host}:{self.mqtt_port}")
        except Exception as exc:
            print(f"[WARN] MQTT 연결 실패: {exc}")
            self._mqtt_connected = False
            self._mqtt_connect_requested = False
            self._schedule_next_reconnect()

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        self._mqtt_connected = not _mqtt_reason_failed(reason_code)
        if not self._mqtt_connected:
            print(f"[WARN] MQTT connect 실패: reason={reason_code!r}")
            self._mqtt_connect_requested = False
            self._schedule_next_reconnect()
            return
        self._mqtt_connect_requested = True
        self._reset_reconnect_backoff()
        print(f"[INFO] MQTT 연결됨: {self.mqtt_host}:{self.mqtt_port}")
        # paho-mqtt Client.subscribe 는 (result_code, mid) 튜플을 반환한다.
        sub_result = client.subscribe(self.mqtt_control_topic, qos=0)
        sub_rc = sub_result[0] if isinstance(sub_result, tuple) and sub_result else sub_result
        if int(sub_rc) != int(mqtt.MQTT_ERR_SUCCESS):
            self._publish_status(
                event="ready",
                status="error",
                message=f"제어 토픽 구독 실패: rc={sub_rc}",
            )
            print(f"[ERROR] MQTT subscribe 실패: topic={self.mqtt_control_topic}, rc={sub_rc}")
            return
        print(f"[INFO] MQTT 구독 완료: {self.mqtt_control_topic}")
        # 재연결이라면 끊긴 동안 쌓인 상태 메시지를 먼저 flush. 그 뒤 ready 발행.
        with self._pending_lock:
            pending_count = len(self._pending_status)
            self._flush_pending_status_locked()
        if pending_count > 0:
            print(f"[INFO] 끊김 동안 쌓인 상태 메시지 {pending_count}건 flush 시도")
        self._publish_status(event="ready", status="ok", message="모드 제어 대기 중입니다.")

    def _on_disconnect(self, client, userdata, *args) -> None:
        reason_code = args[1] if len(args) >= 2 else (args[0] if len(args) == 1 else None)
        print(f"[WARN] MQTT 연결 해제: reason={reason_code!r}")
        self._mqtt_connected = False
        self._mqtt_connect_requested = False

    def _build_launch_command(
        self, mode: str, map_name: str, sub_map_name: str, indoor_map_names: List[str]
    ) -> List[str]:
        if mode == "mapping":
            return [
                "ros2",
                "launch",
                "ligo",
                "nx_mapping.launch.py",
                f"map_name:={map_name}",
                f"sub_map_name:={sub_map_name}",
            ]
        cmd = ["ros2", "launch", "ligo", "nx_odometry.launch.py"]
        if indoor_map_names:
            cmd.append(f"indoor_map_name:={indoor_map_names[0]}")
        return cmd

    def _start_mode(
        self,
        mode: str,
        map_name: str = "",
        sub_map_name: str = "",
        indoor_map_names: Optional[List[str]] = None,
    ) -> None:
        with self._lock:
            if self._busy_locked():
                # 디스패치 레벨에서 1차 차단됨; 이중 안전망.
                self._publish_status(
                    event="start",
                    status="error",
                    message="이미 실행 중인 작업이 있습니다. stop 명령으로 먼저 종료하세요.",
                    mode=mode,
                    map_name=map_name,
                )
                return

            if indoor_map_names is None:
                indoor_map_names = []
            cmd = self._build_launch_command(
                mode=mode, map_name=map_name, sub_map_name=sub_map_name, indoor_map_names=indoor_map_names
            )
            try:
                # start_new_session=True: 자식을 새 process group의 leader로 만들어
                # stop 시 os.killpg 로 ros2 launch + 모든 노드(예: ligo_mapping, ligo_topic_to_mqtt)에
                # 동시에 SIGINT 를 보낼 수 있도록 한다(터미널 Ctrl+C 와 동일).
                proc = subprocess.Popen(cmd, start_new_session=True)
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
            self._proc_map_name = map_name if mode == "mapping" else (indoor_map_names[0] if indoor_map_names else "")
            self._proc_sub_map_name = sub_map_name if mode == "mapping" else ""
            self._proc_indoor_map_names = list(indoor_map_names) if mode == "odometry" else []
            self.state.mode = mode
            self.state.map_name = self._proc_map_name
            self.state.sub_map_name = self._proc_sub_map_name
            self.state.indoor_map_names = list(indoor_map_names) if mode == "odometry" else []
            self.state.running_pid = proc.pid

        time.sleep(self.start_confirm_sec)
        early_rc = proc.poll()
        if early_rc is not None:
            with self._lock:
                self._proc = None
                self._proc_mode = "idle"
                self._proc_map_name = ""
                self._proc_sub_map_name = ""
                self._proc_indoor_map_names = []
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
            return

        start_extra: dict = {"pid": proc.pid, "command": " ".join(cmd)}
        if mode == "mapping":
            start_extra["sub_map_name"] = sub_map_name
        if mode == "odometry" and indoor_map_names:
            start_extra["indoor_map_names"] = list(indoor_map_names)
        self._publish_status(
            event="start",
            status="ok",
            message=f"{mode} 모드 실행을 시작했습니다.",
            mode=mode,
            map_name=map_name if mode == "mapping" else self._proc_map_name,
            extra=start_extra,
        )
        print(
            f"[INFO] started: mode={mode}, map_name={map_name!r}, "
            f"indoor_map_names={indoor_map_names!r}, pid={proc.pid}"
        )

    @staticmethod
    def _summarize_rsync_failure(exit_code: int, stderr: str) -> str:
        tail = (stderr or "").strip().splitlines()
        last = tail[-1] if tail else ""
        if last:
            return f"rsync exit={exit_code}: {last[:200]}"
        return f"rsync exit={exit_code}"

    def _run_sync_in_thread(self, map_name: str, src: Path, dst: Path, opts: list[str]) -> None:
        t0 = time.time()
        ok = False
        reason: Optional[str] = None
        try:
            cmd = ["rsync", *opts, f"{src}/", f"{dst}/"]
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=7200)
            if completed.returncode == 0:
                ok = True
            else:
                reason = self._summarize_rsync_failure(completed.returncode, completed.stderr or "")
        except FileNotFoundError:
            reason = "rsync 실행 파일을 찾을 수 없습니다."
        except subprocess.TimeoutExpired:
            reason = "동기화가 시간 제한(7200초)을 초과했습니다."
        except Exception as exc:
            reason = f"동기화 중 예외: {exc}"

        elapsed_sec = round(time.time() - t0, 3)
        if ok:
            self._publish_status(
                event="sync",
                status="ok",
                message="맵 동기화가 완료되었습니다.",
                mode="idle",
                map_name=map_name,
                extra={"elapsed_sec": elapsed_sec},
            )
            print(f"[INFO] sync done: map_name={map_name}, elapsed={elapsed_sec}s")
        else:
            self._publish_status(
                event="sync",
                status="error",
                message="맵 동기화에 실패했습니다.",
                mode="idle",
                map_name=map_name,
                extra={"reason": reason or "알 수 없는 오류", "elapsed_sec": elapsed_sec},
            )
            print(f"[ERROR] sync failed: map_name={map_name}, reason={reason}, elapsed={elapsed_sec}s")
        with self._lock:
            self._sync_thread = None

    def _start_sync(self, map_name: str) -> None:
        started = False
        with self._lock:
            if self._busy_locked():
                # 디스패치 레벨에서 1차 차단됨; 이중 안전망.
                self._publish_status(
                    event="sync",
                    status="error",
                    message="이미 실행 중인 작업이 있습니다. stop 명령으로 먼저 종료하세요.",
                    mode="idle",
                    map_name=map_name,
                )
                return
            src = SHARED_MAP_ROOT / map_name
            if not src.is_dir():
                self._publish_status(
                    event="sync",
                    status="error",
                    message="맵 동기화에 실패했습니다.",
                    mode="idle",
                    map_name=map_name,
                    extra={"reason": f"공유 맵 경로에 해당 map이 없습니다: {src}"},
                )
                return
            dst = local_map_root() / map_name
            try:
                dst.mkdir(parents=True, exist_ok=True)
            except Exception as exc:
                self._publish_status(
                    event="sync",
                    status="error",
                    message="맵 동기화에 실패했습니다.",
                    mode="idle",
                    map_name=map_name,
                    extra={"reason": f"로컬 맵 디렉터리 생성 실패: {exc}"},
                )
                return
            opts = rsync_options(self._cfg)
            th = threading.Thread(
                target=self._run_sync_in_thread,
                args=(map_name, src, dst, opts),
                name=f"map_sync_{map_name}",
                daemon=True,
            )
            self._sync_thread = th
            th.start()
            started = True
        if started:
            self._publish_status(
                event="sync",
                status="ok",
                message="맵 동기화를 시작했습니다.",
                mode="idle",
                map_name=map_name,
            )
            print(f"[INFO] sync started: map_name={map_name}")

    def _stop_mode(self, reason: str) -> None:
        """원격 stop 처리. proc.wait 이 매핑에서 매우 길어질 수 있으므로 별도 워커 스레드로 실행해
        MQTT 콜백 스레드가 keepalive 를 계속 처리할 수 있도록 한다."""
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                self._proc = None
                self._proc_mode = "idle"
                self._proc_map_name = ""
                self._proc_sub_map_name = ""
                self._proc_indoor_map_names = []
                self.state = ModeState()
                self._publish_status(
                    event="stop",
                    status="ok",
                    message="종료할 실행 중 모드가 없습니다.",
                )
                return
            if self._stop_in_progress:
                self._publish_status(
                    event="stop",
                    status="ok",
                    message="이미 stop 요청이 진행 중입니다.",
                    mode=self._proc_mode,
                    map_name=self._proc_map_name,
                )
                return
            self._stop_in_progress = True
            th = threading.Thread(
                target=self._stop_mode_worker,
                args=(reason,),
                name="mode_stop_worker",
                daemon=True,
            )
            self._stop_thread = th
            th.start()

    def _stop_mode_worker(self, reason: str) -> None:
        with self._lock:
            proc = self._proc
            mode = self._proc_mode
            map_name = self._proc_map_name
            sub_map_name = self._proc_sub_map_name
            indoor_names = list(self._proc_indoor_map_names)

        if proc is None:
            with self._lock:
                self._stop_in_progress = False
                self._stop_thread = None
            return

        # 매핑은 종료 직전에 PCD/그리드 저장이 길게 걸릴 수 있어 mode별 timeout을 분리한다.
        stop_timeout = (
            self.stop_timeout_sec_mapping if mode == "mapping" else self.stop_timeout_sec_odometry
        )
        # SIGINT 발사 시점부터 프로세스 종료까지 — 매핑에서는 곧 PCD/그리드 저장 소요 시간.
        t_signal_sent = time.time()
        try:
            self._signal_proc_group(proc, signal.SIGINT)
            proc.wait(timeout=stop_timeout)
            exit_code = proc.returncode
            stop_msg = "모드를 정상 종료했습니다."
        except subprocess.TimeoutExpired:
            self._signal_proc_group(proc, signal.SIGTERM)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._signal_proc_group(proc, signal.SIGKILL)
                proc.wait(timeout=5.0)
            exit_code = proc.returncode
            stop_msg = "모드 종료가 지연되어 강제 종료했습니다."
        save_elapsed_sec = round(time.time() - t_signal_sent, 3)

        with self._lock:
            self._proc = None
            self._proc_mode = "idle"
            self._proc_map_name = ""
            self._proc_sub_map_name = ""
            self._proc_indoor_map_names = []
            self.state = ModeState()

        stop_extra: dict = {"exit_code": exit_code, "reason": reason}
        if mode == "mapping":
            stop_extra["save_elapsed_sec"] = save_elapsed_sec
            if sub_map_name:
                stop_extra["sub_map_name"] = sub_map_name
        if mode == "odometry" and indoor_names:
            stop_extra["indoor_map_names"] = indoor_names
        self._publish_status(
            event="stop",
            status="ok",
            message=stop_msg,
            mode=mode,
            map_name=map_name,
            sub_map_name=sub_map_name if mode == "mapping" else None,
            extra=stop_extra,
        )
        print(
            f"[INFO] stopped: mode={mode}, map_name={map_name}, exit_code={exit_code}, "
            f"reason={reason}, stop_timeout_sec={stop_timeout}, "
            f"save_elapsed_sec={save_elapsed_sec if mode == 'mapping' else 'n/a'}"
        )

        # 매핑이 정상 종료되었으면 저장 결과(map_saved) 발행 후 공유 경로로 업로드한다.
        if mode == "mapping" and exit_code == 0 and map_name and sub_map_name:
            self._publish_map_saved(map_name, sub_map_name, save_elapsed_sec=save_elapsed_sec)
            th_upload = threading.Thread(
                target=self._upload_map_to_shared,
                args=(map_name, sub_map_name),
                name=f"map_upload_{map_name}_{sub_map_name}",
                daemon=True,
            )
            th_upload.start()

        with self._lock:
            self._stop_in_progress = False
            self._stop_thread = None

    def _parse_control(self, payload: str) -> ParsedControl:
        try:
            obj = json.loads(payload)
        except json.JSONDecodeError:
            return ParsedControl(kind="error", error="제어 메시지가 JSON 형식이 아닩니다.")

        command = str(obj.get("command", "")).strip().lower()
        if command == "stop":
            return ParsedControl(kind="stop")

        if command == "synchronization":
            map_name = obj.get("map_name")
            if not _is_valid_map_name(map_name):
                return ParsedControl(
                    kind="error",
                    error="synchronization: map_name은 영문/숫자/_/- 조합의 필수 문자열입니다.",
                )
            return ParsedControl(kind="synchronization", map_name=str(map_name).strip())

        if command not in ("", "start"):
            return ParsedControl(
                kind="error",
                error="command는 start, stop, synchronization 중 하나여야 합니다.",
            )

        mapping_mode = obj.get("mapping_mode")
        if not isinstance(mapping_mode, bool):
            return ParsedControl(kind="error", error="mapping_mode(boolean) 필드가 필요합니다.")

        if mapping_mode:
            map_name = obj.get("map_name")
            sub_map_name = obj.get("sub_map_name")
            if not _is_valid_map_name(map_name):
                return ParsedControl(
                    kind="error",
                    error="mapping_mode=true일 때 map_name은 영문/숫자/_/- 조합의 필수 문자열입니다.",
                )
            if not _is_valid_map_name(sub_map_name):
                return ParsedControl(
                    kind="error",
                    error="mapping_mode=true일 때 sub_map_name은 영문/숫자/_/- 조합의 필수 문자열입니다.",
                )
            return ParsedControl(
                kind="start_mapping",
                map_name=str(map_name).strip(),
                sub_map_name=str(sub_map_name).strip(),
            )

        map_name = obj.get("map_name")
        if not _is_valid_map_name(map_name):
            return ParsedControl(
                kind="error",
                error="odometry: map_name은 영문/숫자/_/- 조합의 필수 문자열입니다.",
            )
        if obj.get("map_names") is not None:
            return ParsedControl(
                kind="error",
                error="odometry: map_names는 더 이상 사용하지 않습니다. map_name만 사용하세요.",
            )

        return ParsedControl(kind="start_odometry", indoor_map_names=[str(map_name).strip()])

    @staticmethod
    def _map_artifact_status(save_dir: Path, sub_map_name: str) -> tuple[str, list[dict], list[str]]:
        names = [
            f"{sub_map_name}.pcd",
            f"{sub_map_name}_ecef.pcd",
            f"{sub_map_name}_grid2d.pgm",
            f"{sub_map_name}_grid2d.yaml",
        ]
        files_out: list[dict] = []
        missing: list[str] = []
        for n in names:
            p = save_dir / n
            if p.is_file():
                try:
                    files_out.append({"name": n, "size": p.stat().st_size})
                except OSError:
                    files_out.append({"name": n, "size": None})
            else:
                missing.append(n)
        status = "ok" if not missing else "warn"
        return status, files_out, missing

    def _publish_map_saved(
        self,
        map_name: str,
        sub_map_name: str,
        save_elapsed_sec: Optional[float] = None,
    ) -> None:
        save_dir = local_map_root() / map_name / sub_map_name
        st, files_out, missing = self._map_artifact_status(save_dir, sub_map_name)
        msg = (
            "맵 저장이 완료되었습니다."
            if st == "ok"
            else f"일부 산출물이 없습니다: {', '.join(missing)}"
        )
        extra: dict = {"save_root": str(save_dir), "files": files_out, "missing": missing}
        if save_elapsed_sec is not None:
            extra["save_elapsed_sec"] = save_elapsed_sec
        self._publish_status(
            event="map_saved",
            status=st,
            message=msg,
            mode="mapping",
            map_name=map_name,
            sub_map_name=sub_map_name,
            extra=extra,
        )

    def _upload_map_to_shared(self, map_name: str, sub_map_name: str) -> None:
        """로컬 PCD → 공유 경로 rsync 업로드. _stop_mode_worker 완료 후 별도 스레드에서 실행."""
        src = local_map_root() / map_name / sub_map_name
        dst = SHARED_MAP_ROOT / map_name / sub_map_name
        opts = rsync_upload_options(self._cfg)

        t0 = time.time()
        self._publish_status(
            event="map_upload",
            status="ok",
            message="맵 업로드를 시작했습니다.",
            mode="mapping",
            map_name=map_name,
            sub_map_name=sub_map_name,
            extra={"src": str(src), "dst": str(dst)},
        )
        print(f"[INFO] map_upload start: {src} → {dst}")
        try:
            dst.mkdir(parents=True, exist_ok=True)
            cmd = ["rsync", *opts, f"{src}/", f"{dst}/"]
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=7200)
            elapsed_sec = round(time.time() - t0, 3)
            if completed.returncode == 0:
                self._publish_status(
                    event="map_upload",
                    status="ok",
                    message="맵 업로드가 완료되었습니다.",
                    mode="mapping",
                    map_name=map_name,
                    sub_map_name=sub_map_name,
                    extra={"elapsed_sec": elapsed_sec},
                )
                print(f"[INFO] map_upload done: map={map_name}/{sub_map_name}, elapsed={elapsed_sec}s")
            else:
                reason = self._summarize_rsync_failure(completed.returncode, completed.stderr or "")
                self._publish_status(
                    event="map_upload",
                    status="error",
                    message="맵 업로드에 실패했습니다.",
                    mode="mapping",
                    map_name=map_name,
                    sub_map_name=sub_map_name,
                    extra={"reason": reason, "elapsed_sec": elapsed_sec},
                )
                print(f"[ERROR] map_upload failed: map={map_name}/{sub_map_name}, reason={reason}")
        except FileNotFoundError:
            elapsed_sec = round(time.time() - t0, 3)
            self._publish_status(
                event="map_upload",
                status="error",
                message="맵 업로드에 실패했습니다.",
                mode="mapping",
                map_name=map_name,
                sub_map_name=sub_map_name,
                extra={"reason": "rsync 실행 파일을 찾을 수 없습니다.", "elapsed_sec": elapsed_sec},
            )
        except subprocess.TimeoutExpired:
            elapsed_sec = round(time.time() - t0, 3)
            self._publish_status(
                event="map_upload",
                status="error",
                message="맵 업로드에 실패했습니다.",
                mode="mapping",
                map_name=map_name,
                sub_map_name=sub_map_name,
                extra={"reason": "업로드가 시간 제한(7200초)을 초과했습니다.", "elapsed_sec": elapsed_sec},
            )
        except Exception as exc:
            elapsed_sec = round(time.time() - t0, 3)
            self._publish_status(
                event="map_upload",
                status="error",
                message="맵 업로드에 실패했습니다.",
                mode="mapping",
                map_name=map_name,
                sub_map_name=sub_map_name,
                extra={"reason": f"업로드 중 예외: {exc}", "elapsed_sec": elapsed_sec},
            )

    _EVENT_FOR_KIND = {
        "start_mapping": "start",
        "start_odometry": "start",
        "synchronization": "sync",
    }

    def _reject_when_busy(self, parsed: ParsedControl, busy: dict) -> None:
        running = str(busy.get("running", "unknown"))
        event = self._EVENT_FOR_KIND.get(parsed.kind, "control")
        target = parsed.kind.replace("start_", "")
        message = (
            f"현재 {running} 작업이 진행 중이라 {target} 명령을 받을 수 없습니다. "
            "먼저 {\"command\":\"stop\"}을 보내 종료하세요."
        )
        self._publish_status(
            event=event,
            status="error",
            message=message,
            mode=running if running in ("mapping", "odometry") else "idle",
            map_name=parsed.map_name or busy.get("map_name", ""),
            extra={"rejected_command": parsed.kind, "running": busy},
        )
        print(f"[WARN] rejected {parsed.kind!r}: busy={busy}")

    def _on_message(self, client, userdata, msg) -> None:
        payload = msg.payload.decode("utf-8", errors="replace")
        parsed = self._parse_control(payload)
        if parsed.error:
            self._publish_status(event="control", status="error", message=parsed.error)
            print(f"[WARN] invalid control message: {parsed.error}, payload={payload}")
            return

        if parsed.kind == "stop":
            self._stop_mode(reason="remote_stop_message")
            return

        # mapping/odometry/sync 진행 중에는 stop 이외의 어떤 시작 명령도 받지 않는다.
        busy = self._busy_snapshot()
        if busy is not None:
            self._reject_when_busy(parsed, busy)
            return

        if parsed.kind == "start_odometry":
            self._start_mode(mode="odometry", indoor_map_names=parsed.indoor_map_names)
            return
        if parsed.kind == "start_mapping":
            self._start_mode(
                mode="mapping",
                map_name=parsed.map_name,
                sub_map_name=parsed.sub_map_name,
            )
            return
        if parsed.kind == "synchronization":
            self._start_sync(parsed.map_name)
            return

        self._publish_status(event="control", status="error", message="알 수 없는 제어 명령입니다.")

    def _poll_process_exit(self) -> None:
        with self._lock:
            # stop 워커가 처리 중이면 해당 워커가 event 를 발행하도록 양보.
            if self._stop_in_progress:
                return
            proc = self._proc
            mode = self._proc_mode
            map_name = self._proc_map_name
            sub_map_name = self._proc_sub_map_name
            indoor_names = self._proc_indoor_map_names
            if proc is None:
                return
            rc = proc.poll()
            if rc is None:
                return
            self._proc = None
            self._proc_mode = "idle"
            self._proc_map_name = ""
            self._proc_sub_map_name = ""
            self._proc_indoor_map_names = []
            self.state = ModeState()

        status = "ok" if rc == 0 else "error"
        message = "모드 실행이 종료되었습니다." if rc == 0 else "모드 실행이 비정상 종료되었습니다."
        ended_extra: dict = {"exit_code": rc}
        if mode == "mapping" and sub_map_name:
            ended_extra["sub_map_name"] = sub_map_name
        if mode == "odometry" and indoor_names:
            ended_extra["indoor_map_names"] = indoor_names
        self._publish_status(
            event="ended",
            status=status,
            message=message,
            mode=mode,
            map_name=map_name,
            sub_map_name=sub_map_name if mode == "mapping" else None,
            extra=ended_extra,
        )
        print(f"[INFO] ended: mode={mode}, map_name={map_name}, exit_code={rc}")

        if mode == "mapping" and rc == 0 and map_name and sub_map_name:
            self._publish_map_saved(map_name, sub_map_name)
            th_upload = threading.Thread(
                target=self._upload_map_to_shared,
                args=(map_name, sub_map_name),
                name=f"map_upload_{map_name}_{sub_map_name}",
                daemon=True,
            )
            th_upload.start()

    def shutdown(self) -> None:
        self._running = False
        self._stop_mode(reason="service_shutdown")
        # 워커가 매핑 저장(최대 stop_timeout_sec_mapping)을 끝낼 때까지 대기.
        th = self._stop_thread
        if th is not None and th.is_alive():
            th.join()
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
