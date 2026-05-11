#!/usr/bin/env python3
"""공통 MQTT/동기화 설정 로드 — config/mqtt_topics.yaml (설치 시 share/ligo/config/)."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import yaml
except ImportError as exc:
    raise SystemExit("PyYAML이 필요합니다: pip install pyyaml 또는 rosdep install") from exc

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # noqa: BLE001 — ament 없이 스크립트만 실행하는 경우
    get_package_share_directory = None  # type: ignore[assignment,misc]

_DEFAULT_MQTT: Dict[str, Any] = {
    "host": "rms.bottle-tak.com",
    "port": 80,
    "use_websocket": True,
    "ws_path": "/mqtt",
    "username": "",
    "password": "",
    "keepalive_sec": 10,
    "topic_prefix": "navi1",
    "topics": {
        "control_mode": "{prefix}/control/mode",
        "control_status": "{prefix}/control/mode_status",
        "position": "{prefix}/position",
        "heading": "{prefix}/heading",
        "gps": "{prefix}/gps",
        "init_heading_icp": "{prefix}/init_heading_icp",
        "ligo_mode": "{prefix}/ligo_mode",
        "nav_prefix": "{prefix}/nav",
    },
}

_DEFAULT_SYNC: Dict[str, Any] = {
    "shared_map_root": "/mnt/rms_maps",
    "local_map_root": "${LIGO_ROOT}/PCD",
    "rsync_options": ["-a", "--delete", "--info=stats2"],
}

_DEFAULT_ORCH: Dict[str, Any] = {
    # 매핑은 종료 시 PCD/그리드 저장이 길어 launch sigterm_timeout 과 맞춤(24h).
    "stop_timeout_sec_mapping": 86400.0,
    "stop_timeout_sec_odometry": 10.0,
    "start_confirm_sec": 1.5,
}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    out = dict(base)
    for k, v in override.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)  # type: ignore[arg-type]
        else:
            out[k] = v
    return out


def _candidate_paths() -> list[Path]:
    paths: list[Path] = []
    env = os.environ.get("LIGO_MQTT_CONFIG", "").strip()
    if env:
        paths.append(Path(env).expanduser())
    if get_package_share_directory:
        try:
            paths.append(Path(get_package_share_directory("ligo")) / "config" / "mqtt_topics.yaml")
        except Exception:
            pass
    here = Path(__file__).resolve().parent
    paths.append(here.parent / "config" / "mqtt_topics.yaml")
    return paths


def load_mqtt_config() -> Dict[str, Any]:
    cfg: Dict[str, Any] = {
        "mqtt": dict(_DEFAULT_MQTT),
        "sync": dict(_DEFAULT_SYNC),
        "orchestrator": dict(_DEFAULT_ORCH),
    }
    for p in _candidate_paths():
        if not p.is_file():
            continue
        try:
            raw = yaml.safe_load(p.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(raw, dict):
            continue
        if "mqtt" in raw and isinstance(raw["mqtt"], dict):
            cfg["mqtt"] = _deep_merge(cfg["mqtt"], raw["mqtt"])
        if "sync" in raw and isinstance(raw["sync"], dict):
            cfg["sync"] = _deep_merge(cfg["sync"], raw["sync"])
        if "orchestrator" in raw and isinstance(raw["orchestrator"], dict):
            cfg["orchestrator"] = _deep_merge(cfg["orchestrator"], raw["orchestrator"])
        break
    return cfg


def topic_prefix(cfg: Optional[Dict[str, Any]] = None) -> str:
    c = cfg or load_mqtt_config()
    p = str(c.get("mqtt", {}).get("topic_prefix", "navi1")).strip().strip("/")
    return p or "navi1"


def resolve_topic(key: str, cfg: Optional[Dict[str, Any]] = None) -> str:
    c = cfg or load_mqtt_config()
    topics = c.get("mqtt", {}).get("topics") or {}
    if not isinstance(topics, dict):
        topics = {}
    tpl = str(topics.get(key) or _DEFAULT_MQTT["topics"].get(key) or "")
    if not tpl:
        return ""
    return tpl.replace("{prefix}", topic_prefix(c))


def _ligo_root_default() -> Path:
    env = os.environ.get("LIGO_ROOT", "").strip()
    if env:
        return Path(env).expanduser().resolve()
    return Path(__file__).resolve().parent.parent


def resolve_local_map_root(cfg: Optional[Dict[str, Any]] = None) -> Path:
    c = cfg or load_mqtt_config()
    raw = str(c.get("sync", {}).get("local_map_root", "") or "").strip()
    root = _ligo_root_default()
    if not raw:
        return (root / "PCD").resolve()
    expanded = raw.replace("${LIGO_ROOT}", str(root))
    p = Path(expanded).expanduser()
    return p.resolve() if p.is_absolute() else (root / p).resolve()


def shared_map_root(cfg: Optional[Dict[str, Any]] = None) -> Path:
    c = cfg or load_mqtt_config()
    s = str(c.get("sync", {}).get("shared_map_root", "/mnt/rms_maps")).strip()
    return Path(s).expanduser().resolve()


def rsync_options(cfg: Optional[Dict[str, Any]] = None) -> list[str]:
    c = cfg or load_mqtt_config()
    opts = c.get("sync", {}).get("rsync_options")
    if isinstance(opts, list) and all(isinstance(x, str) for x in opts):
        return list(opts)
    return list(_DEFAULT_SYNC["rsync_options"])  # type: ignore[list-item]
