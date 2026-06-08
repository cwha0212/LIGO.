#!/usr/bin/env python3
from __future__ import annotations

import collections
import json
import math
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, List, Optional, Tuple

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("paho-mqtt가 필요합니다: pip install paho-mqtt") from exc

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    _RCLPY_AVAILABLE = True
except ImportError:
    _RCLPY_AVAILABLE = False

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


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """평면 yaw(rad)를 (x, y, z, w) quaternion으로 변환한다(roll=pitch=0)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


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
    nmea_enable: bool = True


@dataclass
class ParsedControl:
    kind: str
    map_name: str = ""
    sub_map_name: str = ""
    indoor_map_names: List[str] = field(default_factory=list)
    nmea_enable: bool = True
    error: Optional[str] = None


class Nav2TaskHandler:
    """rclpy 노드를 별도 스레드에서 실행하여 Nav2 NavigateToPose action을 처리한다.

    MQTT goal 수신 시 send_goal로 action을 보내고, goal 수락/feedback/result에 따라
    status_cb(task_name, status)로 진행 상태('processing'/'success'/'fail')를 알린다.
    """

    def __init__(
        self,
        status_cb: Callable[[str, str], None],
        action_name: str = "navigate_to_pose",
        server_wait_sec: float = 5.0,
    ) -> None:
        if not _RCLPY_AVAILABLE:
            raise RuntimeError(
                "rclpy/nav2_msgs를 import할 수 없습니다. ROS2 환경에서 실행하세요."
            )
        self._status_cb = status_cb
        self._server_wait_sec = server_wait_sec

        if not rclpy.ok():
            rclpy.init()
        self._node: Node = rclpy.create_node("ligo_mqtt_nav2_task_bridge")
        self._action_client = ActionClient(self._node, NavigateToPose, action_name)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="nav2_task_spin", daemon=True
        )
        self._spin_thread.start()
        # task_name별 진행 중인 goal handle (취소/중복 추적용)
        self._goal_handles: dict = {}
        self._lock = threading.Lock()

    def send_goal(
        self, task_name: str, x1: float, y1: float, x2: float, y2: float
    ) -> None:
        dx = x2 - x1
        dy = y2 - y1
        yaw = 0.0 if math.hypot(dx, dy) < 1e-3 else math.atan2(dy, dx)
        qx, qy, qz, qw = _quaternion_from_yaw(yaw)

        if not self._action_client.wait_for_server(timeout_sec=self._server_wait_sec):
            self._node.get_logger().warn(
                f"[task] Nav2 action server를 찾을 수 없습니다: task={task_name}"
            )
            self._status_cb(task_name, "fail")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x1)
        goal.pose.pose.position.y = float(y1)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=lambda fb, tn=task_name: self._on_feedback(tn, fb),
        )
        send_future.add_done_callback(
            lambda fut, tn=task_name: self._on_goal_response(tn, fut)
        )

    def _on_goal_response(self, task_name: str, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f"[task] goal 전송 실패: task={task_name}, err={exc}")
            self._status_cb(task_name, "fail")
            return

        if not goal_handle.accepted:
            self._node.get_logger().warn(f"[task] goal 거부됨: task={task_name}")
            self._status_cb(task_name, "fail")
            return

        with self._lock:
            self._goal_handles[task_name] = goal_handle
        self._status_cb(task_name, "processing")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, tn=task_name: self._on_result(tn, fut)
        )

    def _on_feedback(self, task_name: str, _feedback) -> None:
        self._status_cb(task_name, "processing")

    def _on_result(self, task_name: str, future) -> None:
        with self._lock:
            self._goal_handles.pop(task_name, None)
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().error(f"[task] result 수신 실패: task={task_name}, err={exc}")
            self._status_cb(task_name, "fail")
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._status_cb(task_name, "success")
        else:
            self._node.get_logger().warn(
                f"[task] goal 실패: task={task_name}, status={result.status}"
            )
            self._status_cb(task_name, "fail")

    def shutdown(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


class ModeOrchestrator:
    def __init__(self) -> None:
        self._cfg = load_mqtt_config()
        m = self._cfg.get("mqtt", {}) or {}

        self.mqtt_host = str(m.get("host", "rms.bottle-tak.com"))
        self.mqtt_port = int(m.get("port", 80))
        self.topic_prefix = topic_prefix(self._cfg)
        self.mqtt_control_topic = resolve_topic("control_mode", self._cfg)
        self.mqtt_status_topic = resolve_topic("control_status", self._cfg)
        self.mqtt_task_goal_topic = resolve_topic("task_goal", self._cfg)
        self.mqtt_task_status_topic = resolve_topic("task_status", self._cfg)
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
        self._proc_nmea_enable: bool = True
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
        self._nav2_handler: Optional["Nav2TaskHandler"] = None
        self._nav2_lock = threading.Lock()
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
                    "nmea_enable": self._proc_nmea_enable,
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
        # 끊긴 사이에 발생한 stop/map_saved/sync/sync_verify/map_upload_verify 같은 결과 메시지를 잃지 않도록 한다.
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
                event="control",
                status="fail",
                message=f"제어 토픽 구독 실패: rc={sub_rc}",
            )
            print(f"[ERROR] MQTT subscribe 실패: topic={self.mqtt_control_topic}, rc={sub_rc}")
            return
        print(f"[INFO] MQTT 구독 완료: {self.mqtt_control_topic}")
        if self.mqtt_task_goal_topic:
            task_sub = client.subscribe(self.mqtt_task_goal_topic, qos=0)
            task_rc = task_sub[0] if isinstance(task_sub, tuple) and task_sub else task_sub
            if int(task_rc) != int(mqtt.MQTT_ERR_SUCCESS):
                print(f"[ERROR] task 토픽 구독 실패: topic={self.mqtt_task_goal_topic}, rc={task_rc}")
            else:
                print(f"[INFO] MQTT 구독 완료: {self.mqtt_task_goal_topic}")
        with self._pending_lock:
            pending_count = len(self._pending_status)
            self._flush_pending_status_locked()
        if pending_count > 0:
            print(f"[INFO] 끊김 동안 쌓인 상태 메시지 {pending_count}건 flush 시도")

    def _on_disconnect(self, client, userdata, *args) -> None:
        reason_code = args[1] if len(args) >= 2 else (args[0] if len(args) == 1 else None)
        print(f"[WARN] MQTT 연결 해제: reason={reason_code!r}")
        self._mqtt_connected = False
        self._mqtt_connect_requested = False

    def _build_launch_command(
        self,
        mode: str,
        map_name: str,
        sub_map_name: str,
        indoor_map_names: List[str],
        nmea_enable: bool,
    ) -> List[str]:
        nmea_launch_arg = f"nmea_enable:={'true' if nmea_enable else 'false'}"
        if mode == "mapping":
            return [
                "ros2",
                "launch",
                "navi",
                "nx_mapping.launch.py",
                f"map_name:={map_name}",
                f"sub_map_name:={sub_map_name}",
                nmea_launch_arg,
            ]
        if not nmea_enable:
            hdl_map_name = indoor_map_names[0] if indoor_map_names else map_name
            if not hdl_map_name:
                raise ValueError("odometry 시작에 필요한 map_name이 없습니다.")
            pcd_path = self._resolve_hdl_global_pcd(hdl_map_name)
            return [
                "ros2",
                "launch",
                "hdl_localization",
                "hdl_localization_2.launch.py",
                f"global_pcd:={pcd_path}",
                "nmea_enable:=false",
            ]
        cmd = ["ros2", "launch", "navi", "nx_odometry.launch.py"]
        if indoor_map_names:
            cmd.append(f"indoor_map_name:={indoor_map_names[0]}")
        cmd.append(nmea_launch_arg)
        return cmd

    @staticmethod
    def _resolve_hdl_global_pcd(map_name: str) -> Path:
        map_dir = local_map_root() / map_name
        if not map_dir.is_dir():
            raise ValueError(
                f"odometry 맵 디렉터리가 없습니다: {map_dir}. synchronization 후 다시 시도하세요."
            )
        candidates = sorted(p for p in map_dir.rglob("*.pcd") if p.is_file())
        if not candidates:
            raise ValueError(f"odometry 맵에서 PCD 파일을 찾을 수 없습니다: {map_dir}")
        exact = map_dir / f"{map_name}.pcd"
        if exact.is_file():
            return exact
        for candidate in candidates:
            if not candidate.name.endswith("_orig.pcd"):
                return candidate
        return candidates[0]

    def _start_mode(
        self,
        mode: str,
        map_name: str = "",
        sub_map_name: str = "",
        indoor_map_names: Optional[List[str]] = None,
        nmea_enable: bool = True,
    ) -> None:
        with self._lock:
            if self._busy_locked():
                self._publish_status(
                    event=mode,
                    status="fail",
                    message="이미 실행 중인 작업이 있습니다. stop 명령으로 먼저 종료하세요.",
                    mode=mode,
                    map_name=map_name,
                )
                return

            if indoor_map_names is None:
                indoor_map_names = []
            try:
                cmd = self._build_launch_command(
                    mode=mode,
                    map_name=map_name,
                    sub_map_name=sub_map_name,
                    indoor_map_names=indoor_map_names,
                    nmea_enable=nmea_enable,
                )
            except Exception as exc:
                self._publish_status(
                    event=mode,
                    status="fail",
                    message=f"모드 실행 준비 실패: {exc}",
                    mode=mode,
                    map_name=map_name if mode == "mapping" else (indoor_map_names[0] if indoor_map_names else map_name),
                )
                return
            try:
                # start_new_session=True: 자식을 새 process group의 leader로 만들어
                # stop 시 os.killpg 로 ros2 launch + 모든 노드(예: ligo_mapping, ligo_topic_to_mqtt)에
                # 동시에 SIGINT 를 보낼 수 있도록 한다(터미널 Ctrl+C 와 동일).
                proc = subprocess.Popen(cmd, start_new_session=True)
            except Exception as exc:
                self._publish_status(
                    event=mode,
                    status="fail",
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
            self._proc_nmea_enable = bool(nmea_enable)
            self.state.mode = mode
            self.state.map_name = self._proc_map_name
            self.state.sub_map_name = self._proc_sub_map_name
            self.state.indoor_map_names = list(indoor_map_names) if mode == "odometry" else []
            self.state.running_pid = proc.pid
            self.state.nmea_enable = bool(nmea_enable)

        time.sleep(self.start_confirm_sec)
        early_rc = proc.poll()
        if early_rc is not None:
            with self._lock:
                self._proc = None
                self._proc_mode = "idle"
                self._proc_map_name = ""
                self._proc_sub_map_name = ""
                self._proc_indoor_map_names = []
                self._proc_nmea_enable = True
                self.state = ModeState()
            self._publish_status(
                event=mode,
                status="fail",
                message="모드 프로세스가 시작 직후 종료되었습니다. journalctl 로그를 확인하세요.",
                mode=mode,
                map_name=map_name,
                extra={"exit_code": early_rc, "command": " ".join(cmd)},
            )
            print(f"[ERROR] early-exit: mode={mode}, map_name={map_name}, exit_code={early_rc}")
            return

        start_extra: dict = {"pid": proc.pid, "command": " ".join(cmd), "nmea_enable": bool(nmea_enable)}
        if mode == "mapping":
            start_extra["sub_map_name"] = sub_map_name
        if mode == "odometry" and indoor_map_names:
            start_extra["indoor_map_names"] = list(indoor_map_names)
        self._publish_status(
            event=mode,
            status="start",
            message=f"{mode} 모드 실행을 시작했습니다.",
            mode=mode,
            map_name=map_name if mode == "mapping" else self._proc_map_name,
            extra=start_extra,
        )
        print(
            f"[INFO] started: mode={mode}, map_name={map_name!r}, "
            f"indoor_map_names={indoor_map_names!r}, nmea_enable={nmea_enable}, pid={proc.pid}"
        )

    @staticmethod
    def _summarize_rsync_failure(
        exit_code: int,
        stderr: str,
        stdout: str = "",
        *,
        max_chars: int = 900,
    ) -> str:
        """rsync 실패 시 MQTT reason 용 요약. exit=23 은 보통 NFS/CIFS 에서 owner/group 보존 실패."""
        err_lines = (stderr or "").strip().splitlines()
        out_lines = (stdout or "").strip().splitlines()
        lines = err_lines + out_lines
        lines = [ln.strip() for ln in lines if ln.strip()]
        if not lines:
            return f"rsync exit={exit_code}"
        key = (
            "permission denied",
            "operation not permitted",
            "read-only",
            "no space",
            "failed:",
            "error:",
            "rsync:",
        )
        important = [ln for ln in lines if any(k in ln.lower() for k in key)]
        tail = lines[-10:]
        merged: list[str] = []
        for ln in important[:6]:
            if ln not in merged:
                merged.append(ln)
        for ln in tail:
            if ln not in merged:
                merged.append(ln)
        text = " | ".join(merged)
        if len(text) > max_chars:
            text = "…" + text[-(max_chars - 1) :]
        return f"rsync exit={exit_code}: {text}"

    @staticmethod
    def _verify_rsync_mirror(reference: Path, mirror: Path) -> tuple[bool, dict[str, Any]]:
        """reference 아래의 모든 일반 파일이 mirror에 동일 크기로 존재하는지 검사한다."""
        out: dict[str, Any] = {
            "ok": False,
            "reference": str(reference),
            "mirror": str(mirror),
            "file_count": 0,
            "bytes_total": 0,
            "matched_samples": [],
            "mismatches": [],
        }
        max_samples = 40
        max_mismatch = 50

        if not reference.is_dir():
            out["mismatches"] = [
                {"path": ".", "reference_bytes": None, "mirror_bytes": None, "error": "기준 경로가 디렉터리가 아닙니다."}
            ]
            return False, out
        if not mirror.is_dir():
            out["mismatches"] = [
                {"path": ".", "reference_bytes": None, "mirror_bytes": None, "error": "대상 경로가 디렉터리가 아닙니다."}
            ]
            return False, out

        mismatches: list[dict[str, Any]] = []
        matched_samples: list[dict[str, Any]] = []
        bytes_total = 0
        file_count = 0

        try:
            paths = sorted(reference.rglob("*"))
        except OSError as exc:
            mismatches.append(
                {
                    "path": ".",
                    "reference_bytes": None,
                    "mirror_bytes": None,
                    "error": f"기준 경로 순회 실패: {exc}",
                }
            )
            out["mismatches"] = mismatches
            return False, out

        for p in paths:
            if not p.is_file():
                continue
            try:
                rel = p.relative_to(reference).as_posix()
            except ValueError:
                continue
            try:
                ref_sz = int(p.stat().st_size)
            except OSError as exc:
                mismatches.append(
                    {
                        "path": rel,
                        "reference_bytes": None,
                        "mirror_bytes": None,
                        "error": f"기준 파일 stat 실패: {exc}",
                    }
                )
                continue
            file_count += 1
            bytes_total += ref_sz
            dest = mirror / rel
            if not dest.is_file():
                mismatches.append({"path": rel, "reference_bytes": ref_sz, "mirror_bytes": None})
                continue
            try:
                mir_sz = int(dest.stat().st_size)
            except OSError as exc:
                mismatches.append(
                    {
                        "path": rel,
                        "reference_bytes": ref_sz,
                        "mirror_bytes": None,
                        "error": f"대상 파일 stat 실패: {exc}",
                    }
                )
                continue
            if mir_sz != ref_sz:
                mismatches.append({"path": rel, "reference_bytes": ref_sz, "mirror_bytes": mir_sz})
            elif len(matched_samples) < max_samples:
                matched_samples.append({"path": rel, "bytes": ref_sz})

        out["file_count"] = file_count
        out["bytes_total"] = bytes_total
        out["matched_samples"] = matched_samples
        total_mismatch = len(mismatches)
        matched_files = file_count - total_mismatch
        out["samples_truncated"] = matched_files > max_samples
        if total_mismatch > max_mismatch:
            out["mismatches_truncated"] = True
            out["mismatches"] = mismatches[:max_mismatch]
        else:
            out["mismatches"] = list(mismatches)
        out["ok"] = total_mismatch == 0
        return bool(out["ok"]), out

    @staticmethod
    def _verify_status_extra(
        verify_report: dict[str, Any],
        *,
        rsync_elapsed_sec: Optional[float] = None,
        verify_elapsed_sec: Optional[float] = None,
    ) -> dict[str, Any]:
        """MQTT 페이로드용: 검증 dict에서 ok 제외, 경과 시간 필드 추가."""
        extra: dict[str, Any] = {k: v for k, v in verify_report.items() if k != "ok"}
        if rsync_elapsed_sec is not None:
            extra["rsync_elapsed_sec"] = rsync_elapsed_sec
        if verify_elapsed_sec is not None:
            extra["verify_elapsed_sec"] = verify_elapsed_sec
        return extra

    def _run_sync_in_thread(self, map_name: str, src: Path, dst: Path, opts: list[str]) -> None:
        t0 = time.time()
        rsync_ok = False
        reason: Optional[str] = None
        try:
            cmd = ["rsync", *opts, f"{src}/", f"{dst}/"]
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=7200)
            if completed.returncode == 0:
                rsync_ok = True
            else:
                reason = self._summarize_rsync_failure(
                    completed.returncode,
                    completed.stderr or "",
                    completed.stdout or "",
                )
        except FileNotFoundError:
            reason = "rsync 실행 파일을 찾을 수 없습니다."
        except subprocess.TimeoutExpired:
            reason = "동기화가 시간 제한(7200초)을 초과했습니다."
        except Exception as exc:
            reason = f"동기화 중 예외: {exc}"

        t_after_rsync = time.time()
        rsync_elapsed_sec = round(t_after_rsync - t0, 3)

        if rsync_ok:
            self._publish_status(
                event="sync",
                status="finish",
                message="맵 동기화가 완료되었습니다.",
                mode="idle",
                map_name=map_name,
                extra={"elapsed_sec": rsync_elapsed_sec},
            )
            print(f"[INFO] sync rsync done: map_name={map_name}, elapsed={rsync_elapsed_sec}s")
            t_v0 = time.time()
            v_ok, verify_report = self._verify_rsync_mirror(src, dst)
            verify_elapsed_sec = round(time.time() - t_v0, 3)
            v_extra = self._verify_status_extra(
                verify_report,
                rsync_elapsed_sec=rsync_elapsed_sec,
                verify_elapsed_sec=verify_elapsed_sec,
            )
            if not v_ok:
                n_show = len(verify_report.get("mismatches") or [])
                truncated = bool(verify_report.get("mismatches_truncated"))
                v_extra["reason"] = (
                    f"불일치·누락 {n_show}건" + (" (일부만 표시)" if truncated else "")
                )
            self._publish_status(
                event="sync_verify",
                status="success" if v_ok else "fail",
                message=(
                    "맵 동기화 후 파일 용량 검증을 통과했습니다."
                    if v_ok
                    else "맵 동기화 후 파일 용량 검증에 실패했습니다."
                ),
                mode="idle",
                map_name=map_name,
                extra=v_extra,
            )
            print(
                f"[INFO] sync_verify: map_name={map_name}, ok={v_ok}, "
                f"verify_elapsed={verify_elapsed_sec}s"
            )
        else:
            self._publish_status(
                event="sync",
                status="fail",
                message="맵 동기화에 실패했습니다.",
                mode="idle",
                map_name=map_name,
                extra={"reason": reason or "알 수 없는 오류", "elapsed_sec": rsync_elapsed_sec},
            )
            print(f"[ERROR] sync failed: map_name={map_name}, reason={reason}, elapsed={rsync_elapsed_sec}s")
        with self._lock:
            self._sync_thread = None

    def _start_sync(self, map_name: str) -> None:
        started = False
        with self._lock:
            if self._busy_locked():
                self._publish_status(
                    event="sync",
                    status="fail",
                    message="이미 실행 중인 작업이 있습니다. stop 명령으로 먼저 종료하세요.",
                    mode="idle",
                    map_name=map_name,
                )
                return
            src = SHARED_MAP_ROOT / map_name
            if not src.is_dir():
                self._publish_status(
                    event="sync",
                    status="fail",
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
                    status="fail",
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
                status="start",
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
                    event="ready",
                    status="ok",
                    message="종료할 실행 중 모드가 없습니다.",
                )
                return
            if self._stop_in_progress:
                cur_mode = self._proc_mode
                self._publish_status(
                    event=cur_mode if cur_mode in ("mapping", "odometry") else "ready",
                    status="stop",
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
            nmea_enable = bool(self._proc_nmea_enable)

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
            self._proc_nmea_enable = True
            self.state = ModeState()

        stop_extra: dict = {"exit_code": exit_code, "reason": reason, "nmea_enable": nmea_enable}
        if mode == "mapping":
            stop_extra["save_elapsed_sec"] = save_elapsed_sec
            if sub_map_name:
                stop_extra["sub_map_name"] = sub_map_name
        if mode == "odometry" and indoor_names:
            stop_extra["indoor_map_names"] = indoor_names
        self._publish_status(
            event=mode if mode in ("mapping", "odometry") else "ready",
            status="stop",
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
            self._publish_map_saved(
                map_name,
                sub_map_name,
                save_elapsed_sec=save_elapsed_sec,
                nmea_enable=nmea_enable,
            )
            th_upload = threading.Thread(
                target=self._upload_map_to_shared,
                args=(map_name, sub_map_name, nmea_enable),
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

        if command == "ready":
            return ParsedControl(kind="ready")

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
                error="command는 start, stop, ready, synchronization 중 하나여야 합니다.",
            )

        mapping_mode = obj.get("mapping_mode")
        if not isinstance(mapping_mode, bool):
            return ParsedControl(kind="error", error="mapping_mode(boolean) 필드가 필요합니다.")
        nmea_enable = obj.get("nmea_enable", True)
        if not isinstance(nmea_enable, bool):
            return ParsedControl(kind="error", error="nmea_enable(boolean) 필드가 필요합니다.")

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
                nmea_enable=nmea_enable,
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

        return ParsedControl(
            kind="start_odometry",
            indoor_map_names=[str(map_name).strip()],
            nmea_enable=nmea_enable,
        )

    @staticmethod
    def _map_artifact_status(
        save_dir: Path,
        sub_map_name: str,
        nmea_enable: bool,
    ) -> tuple[str, list[dict], list[str]]:
        names = [f"{sub_map_name}.pcd"]
        if nmea_enable:
            names.extend(
                [
                    f"{sub_map_name}_orig.pcd",
                    f"{sub_map_name}_grid2d.pgm",
                    f"{sub_map_name}_grid2d.yaml",
                ]
            )
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
        status = "finish"
        return status, files_out, missing

    def _publish_map_saved(
        self,
        map_name: str,
        sub_map_name: str,
        save_elapsed_sec: Optional[float] = None,
        nmea_enable: bool = True,
    ) -> None:
        save_dir = local_map_root() / map_name / sub_map_name
        st, files_out, missing = self._map_artifact_status(save_dir, sub_map_name, nmea_enable)
        msg = (
            "맵 저장이 완료되었습니다."
            if not missing
            else f"일부 산출물이 없습니다: {', '.join(missing)}"
        )
        extra: dict[str, Any] = {
            "save_root": str(save_dir),
            "files": files_out,
            "missing": missing,
            "nmea_enable": bool(nmea_enable),
        }
        if save_elapsed_sec is not None:
            extra["save_elapsed_sec"] = save_elapsed_sec
        sz_sum = 0
        for item in files_out:
            s = item.get("size")
            if isinstance(s, int):
                sz_sum += s
        extra["artifact_bytes_total"] = sz_sum
        self._publish_status(
            event="map_saved",
            status=st,
            message=msg,
            mode="mapping",
            map_name=map_name,
            sub_map_name=sub_map_name,
            extra=extra,
        )

    def _upload_map_to_shared(self, map_name: str, sub_map_name: str, nmea_enable: bool) -> None:
        """로컬 PCD → 공유 경로 rsync 업로드. _stop_mode_worker 완료 후 별도 스레드에서 실행."""
        src = local_map_root() / map_name / sub_map_name
        dst = SHARED_MAP_ROOT / map_name / sub_map_name
        opts = rsync_upload_options(self._cfg)

        t0 = time.time()
        self._publish_status(
            event="map_upload",
            status="start",
            message="맵 업로드를 시작했습니다.",
            mode="mapping",
            map_name=map_name,
            sub_map_name=sub_map_name,
            extra={"src": str(src), "dst": str(dst), "nmea_enable": bool(nmea_enable)},
        )
        print(f"[INFO] map_upload start: {src} → {dst}")
        try:
            dst.mkdir(parents=True, exist_ok=True)
            cmd = ["rsync", *opts, f"{src}/", f"{dst}/"]
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=7200)
            t_after_rsync = time.time()
            rsync_elapsed_sec = round(t_after_rsync - t0, 3)
            if completed.returncode == 0:
                self._publish_status(
                    event="map_upload",
                    status="finish",
                    message="맵 업로드가 완료되었습니다.",
                    mode="mapping",
                    map_name=map_name,
                    sub_map_name=sub_map_name,
                    extra={"elapsed_sec": rsync_elapsed_sec, "nmea_enable": bool(nmea_enable)},
                )
                print(f"[INFO] map_upload rsync done: map={map_name}/{sub_map_name}, elapsed={rsync_elapsed_sec}s")
                t_v0 = time.time()
                v_ok, verify_report = self._verify_rsync_mirror(src, dst)
                verify_elapsed_sec = round(time.time() - t_v0, 3)
                v_extra = self._verify_status_extra(
                    verify_report,
                    rsync_elapsed_sec=rsync_elapsed_sec,
                    verify_elapsed_sec=verify_elapsed_sec,
                )
                if not v_ok:
                    n_show = len(verify_report.get("mismatches") or [])
                    truncated = bool(verify_report.get("mismatches_truncated"))
                    v_extra["reason"] = (
                        f"불일치·누락 {n_show}건" + (" (일부만 표시)" if truncated else "")
                    )
                self._publish_status(
                    event="map_upload_verify",
                    status="success" if v_ok else "fail",
                    message=(
                        "맵 업로드 후 공유 경로 파일 용량 검증을 통과했습니다."
                        if v_ok
                        else "맵 업로드 후 공유 경로 파일 용량 검증에 실패했습니다."
                    ),
                    mode="mapping",
                    map_name=map_name,
                    sub_map_name=sub_map_name,
                    extra={**v_extra, "nmea_enable": bool(nmea_enable)},
                )
                print(
                    f"[INFO] map_upload_verify: map={map_name}/{sub_map_name}, ok={v_ok}, "
                    f"verify_elapsed={verify_elapsed_sec}s"
                )
            else:
                elapsed_sec = rsync_elapsed_sec
                stderr = completed.stderr or ""
                stdout = completed.stdout or ""
                reason = self._summarize_rsync_failure(
                    completed.returncode,
                    stderr,
                    stdout,
                )
                self._publish_status(
                    event="map_upload",
                    status="fail",
                    message="맵 업로드에 실패했습니다.",
                    mode="mapping",
                    map_name=map_name,
                    sub_map_name=sub_map_name,
                    extra={
                        "reason": reason,
                        "elapsed_sec": elapsed_sec,
                        "exit_code": completed.returncode,
                        "nmea_enable": bool(nmea_enable),
                    },
                )
                print(
                    f"[ERROR] map_upload failed: map={map_name}/{sub_map_name}, rc={completed.returncode}\n"
                    f"  cmd: {' '.join(cmd)}\n"
                    f"  stderr:\n{stderr}\n"
                    f"  stdout:\n{stdout}"
                )
        except FileNotFoundError:
            elapsed_sec = round(time.time() - t0, 3)
            self._publish_status(
                event="map_upload",
                status="fail",
                message="맵 업로드에 실패했습니다.",
                mode="mapping",
                map_name=map_name,
                sub_map_name=sub_map_name,
                extra={
                    "reason": "rsync 실행 파일을 찾을 수 없습니다.",
                    "elapsed_sec": elapsed_sec,
                    "nmea_enable": bool(nmea_enable),
                },
            )
        except subprocess.TimeoutExpired:
            elapsed_sec = round(time.time() - t0, 3)
            self._publish_status(
                event="map_upload",
                status="fail",
                message="맵 업로드에 실패했습니다.",
                mode="mapping",
                map_name=map_name,
                sub_map_name=sub_map_name,
                extra={
                    "reason": "업로드가 시간 제한(7200초)을 초과했습니다.",
                    "elapsed_sec": elapsed_sec,
                    "nmea_enable": bool(nmea_enable),
                },
            )
        except Exception as exc:
            elapsed_sec = round(time.time() - t0, 3)
            self._publish_status(
                event="map_upload",
                status="fail",
                message="맵 업로드에 실패했습니다.",
                mode="mapping",
                map_name=map_name,
                sub_map_name=sub_map_name,
                extra={
                    "reason": f"업로드 중 예외: {exc}",
                    "elapsed_sec": elapsed_sec,
                    "nmea_enable": bool(nmea_enable),
                },
            )

    _EVENT_FOR_KIND = {
        "start_mapping": "mapping",
        "start_odometry": "odometry",
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
            status="fail",
            message=message,
            mode=running if running in ("mapping", "odometry") else "idle",
            map_name=parsed.map_name or busy.get("map_name", ""),
            extra={"rejected_command": parsed.kind, "running": busy},
        )
        print(f"[WARN] rejected {parsed.kind!r}: busy={busy}")

    def _build_ready_payload(self) -> dict:
        """보드 준비 상태 스냅샷. idle 이면 ready=true, 작업 중이면 ready=false."""
        busy = self._busy_snapshot()
        with self._lock:
            mode = self.state.mode
            map_name = self.state.map_name
        if busy is None:
            return {
                "timestamp_unix": time.time(),
                "event": "ready",
                "status": "ok",
                "message": "보드가 준비되었습니다.",
                "ready": True,
                "mode": mode,
                "map_name": map_name,
            }
        running = str(busy.get("running", "unknown"))
        return {
            "timestamp_unix": time.time(),
            "event": "ready",
            "status": "fail",
            "message": f"현재 {running} 작업이 진행 중입니다.",
            "ready": False,
            "mode": running if running in ("mapping", "odometry") else mode,
            "map_name": busy.get("map_name", map_name),
            "running": busy,
        }

    def _publish_ready_response(self) -> None:
        payload = self._build_ready_payload()
        extra: dict[str, Any] = {"ready": payload["ready"]}
        if "running" in payload:
            extra["running"] = payload["running"]
        self._publish_status(
            event="ready",
            status=payload["status"],
            message=payload["message"],
            mode=str(payload.get("mode", "idle")),
            map_name=str(payload.get("map_name", "")),
            extra=extra,
        )
        print(
            f"[INFO] ready 응답 발행: ready={payload.get('ready')}, status={payload.get('status')}"
        )

    def _on_message(self, client, userdata, msg) -> None:
        payload = msg.payload.decode("utf-8", errors="replace")
        if self.mqtt_task_goal_topic and msg.topic == self.mqtt_task_goal_topic:
            self._handle_task_goal(payload)
            return
        parsed = self._parse_control(payload)
        if parsed.error:
            self._publish_status(event="control", status="fail", message=parsed.error)
            print(f"[WARN] invalid control message: {parsed.error}, payload={payload}")
            return

        if parsed.kind == "ready":
            print(f"[INFO] ready 확인 요청 수신: payload={payload!r}")
            self._publish_ready_response()
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
            self._start_mode(
                mode="odometry",
                indoor_map_names=parsed.indoor_map_names,
                nmea_enable=parsed.nmea_enable,
            )
            return
        if parsed.kind == "start_mapping":
            self._start_mode(
                mode="mapping",
                map_name=parsed.map_name,
                sub_map_name=parsed.sub_map_name,
                nmea_enable=parsed.nmea_enable,
            )
            return
        if parsed.kind == "synchronization":
            self._start_sync(parsed.map_name)
            return

        self._publish_status(event="control", status="fail", message="알 수 없는 제어 명령입니다.")

    def _ensure_nav2_handler(self) -> Optional["Nav2TaskHandler"]:
        with self._nav2_lock:
            if self._nav2_handler is not None:
                return self._nav2_handler
            try:
                self._nav2_handler = Nav2TaskHandler(status_cb=self._publish_task_status)
            except Exception as exc:  # noqa: BLE001
                print(f"[ERROR] Nav2TaskHandler 초기화 실패: {exc}")
                self._nav2_handler = None
            return self._nav2_handler

    def _publish_task_status(self, task_name: str, status: str) -> None:
        if not self.mqtt_task_status_topic:
            return
        payload = {
            "task_name": task_name,
            "status": status,
            "timestamp_unix": time.time(),
        }
        payload_str = json.dumps(payload, ensure_ascii=False)
        try:
            self._mqtt.publish(self.mqtt_task_status_topic, payload_str, qos=0, retain=False)
        except Exception as exc:  # noqa: BLE001
            print(f"[WARN] task/status 발행 실패: {exc}")

    def _handle_task_goal(self, payload: str) -> None:
        try:
            obj = json.loads(payload)
        except json.JSONDecodeError:
            print(f"[WARN] task/goal 메시지가 JSON 형식이 아닙니다: {payload!r}")
            return
        if not isinstance(obj, dict):
            print("[WARN] task/goal 페이로드가 객체가 아닙니다.")
            return

        task_name = obj.get("task_name")
        if not isinstance(task_name, str) or not task_name.strip():
            print("[WARN] task/goal: task_name(문자열)이 필요합니다.")
            return
        task_name = task_name.strip()

        coords: dict = {}
        for key in ("x1", "y1", "x2", "y2"):
            try:
                coords[key] = float(obj[key])
            except (KeyError, TypeError, ValueError):
                self._publish_task_status(task_name, "fail")
                print(f"[WARN] task/goal: {key}(숫자)가 필요합니다. task={task_name}")
                return

        if not _RCLPY_AVAILABLE:
            self._publish_task_status(task_name, "fail")
            print("[ERROR] task/goal: rclpy/nav2_msgs를 사용할 수 없는 환경입니다.")
            return

        with self._lock:
            current_mode = self._proc_mode
        if current_mode != "odometry":
            print(
                f"[INFO] task/goal: orchestrator odometry 모드가 아님(current={current_mode}). "
                f"수동 launch(hdl/nav2) 가정하고 Nav2 action을 시도합니다. task={task_name}"
            )

        handler = self._ensure_nav2_handler()
        if handler is None:
            self._publish_task_status(task_name, "fail")
            return

        try:
            handler.send_goal(
                task_name, coords["x1"], coords["y1"], coords["x2"], coords["y2"]
            )
        except Exception as exc:  # noqa: BLE001
            self._publish_task_status(task_name, "fail")
            print(f"[ERROR] task/goal goal 전송 중 오류: task={task_name}, err={exc}")

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
            nmea_enable = bool(self._proc_nmea_enable)
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
            self._proc_nmea_enable = True
            self.state = ModeState()

        status = "ok" if rc == 0 else "fail"
        message = "모드 실행이 종료되었습니다." if rc == 0 else "모드 실행이 비정상 종료되었습니다."
        ended_extra: dict = {"exit_code": rc, "nmea_enable": nmea_enable}
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
            self._publish_map_saved(map_name, sub_map_name, nmea_enable=nmea_enable)
            th_upload = threading.Thread(
                target=self._upload_map_to_shared,
                args=(map_name, sub_map_name, nmea_enable),
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
        with self._nav2_lock:
            handler = self._nav2_handler
            self._nav2_handler = None
        if handler is not None:
            handler.shutdown()
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
