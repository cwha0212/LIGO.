# LIGO MQTT 토픽 목록

현재 MQTT 브리지는 `scripts/ligo_topic_to_mqtt.py` **하나로 통합**되어 동작한다.
(`scripts/mqtt_ws_client.py` 기능 포함)

## 토픽 종류 요약

| MQTT 토픽 | 전달 내용(요약) |
|-----------|-----------------|
| `{prefix}/position` | 위도·경도 |
| `{prefix}/heading` | 북 기준 시계방향 방위각(도)·16방위 문자열 |
| `{prefix}/gps` | GPS 신호 3단계·NTRIP(RTK) 접속 여부 |
| `{prefix}/init_heading_icp` | 초기 heading 정합 성공/상태 |
| `{prefix}/ligo_mode` (기본 `navi1/ligo_mode`) | indoor, outdoor 및 지도 이름 |

---

## 발행 시점

**고정 주기 발행이 아니라 이벤트 기반 발행**이다.

| MQTT 토픽 | 발행 조건 |
|-----------|-----------|
| `{prefix}/position` | `NavSatFix` 수신 시 `lat/lon` 변경 때만 |
| `{prefix}/heading` | `init_heading_icp`가 `LOCKED`인 상태에서 odom 수신마다 |
| `{prefix}/gps` | `GnssPVTSolnMsg` 수신마다 |
| `{prefix}/init_heading_icp` | `success` 또는 `status` 변경 시만 |
| `{prefix}/ligo_mode` | `/ligo/mode` 수신마다 |
| (전체) | MQTT 재연결 직후 캐시 상태 1회 재발행 |

연결이 끊기면 발행은 생략되고 `reconnect_period_sec` 주기로 재연결을 시도한다.

---

## 토픽별 JSON 필드

### `{prefix}/position`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `lat` | float \| null | 위도(도) |
| `lon` | float \| null | 경도(도) |

### `{prefix}/heading`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `deg_from_north_cw` | float \| null | 북 기준 시계방향 방위각 |
| `cardinal` | string \| null | 16방위 (`N`, `NNE`, …) |

### `{prefix}/gps`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `status` | string \| null | `신호없음` \| `신호미약` \| `신호정상` |
| `ntrip_connected` | bool \| null |  NTRIP(RTK) 접속 여부 |

상태 판정:
- `valid_fix == false` 또는 `fix_type == 0` → `신호없음`
- 그 외 `carr_soln == 2` → `신호정상`
- 나머지 → `신호미약`

### `{prefix}/init_heading_icp`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `success` | bool | `icp_tf_ready` && `status==LOCKED` |
| `status` | string \| null | `UNALIGNED` \| `COLLECTING` \| `LOCKED` |

### `{prefix}/ligo_mode` (기본 `navi1/ligo_mode`)

입력: `/ligo/mode` (`std_msgs/String`, JSON 문자열)

- `mode == "indoor"` → `{ "mode": "indoor", "pcd_name": "..." }`
- `mode == "outdoor"` → `{ "mode": "outdoor" }`
- 그 외는 `{ "mode": "...", "raw": ... }`
- JSON 파싱 실패 시 원문 바이트 그대로 publish

---
