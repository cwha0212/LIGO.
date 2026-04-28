# LIGO MQTT Payload 스펙

이 문서는 현재 저장소의 MQTT payload를 아래 두 범주로 정리한다.

- ROS -> MQTT 브리지: `scripts/ligo_topic_to_mqtt.py`
- 모드 제어/상태: `scripts/mqtt_mode_orchestrator.py`

## 발행 동작

- **상시 주기 발행 아님**. ROS 입력 이벤트가 생길 때만 해당 MQTT 토픽을 발행한다.
- **재연결 타이머**(`reconnect_period_sec`, 기본 `1.0s`)는 MQTT 재접속 용도이며 데이터 주기 발행용이 아니다.
- MQTT 재연결 직후에는 캐시 상태로 주요 토픽(`position/heading/gps/init_heading_icp`)을 각각 1회 재발행한다.
- **MQTT QoS**: `0`, **retain**: `false`.
- **전송**: 기본 WebSocket (`mqtt.use_websocket=true`, `mqtt.ws_path=/mqtt`).
- **페이로드**: UTF-8 JSON 문자열 (`ensure_ascii=False`).

---

## MQTT 토픽 이름

### 접두사 기반 토픽

접두사는 파라미터 `mqtt.topic_prefix` (기본 `navi1`). 앞뒤 `/` 제거 후 아래 형태로 조합한다.

| MQTT topic (기본) | 내용 |
|-------------------|------|
| `{prefix}/position` | 위도·경도 + 타임스탬프 |
| `{prefix}/heading` | 북 기준 각도·16방위 + 타임스탬프 |
| `{prefix}/gps` | 신호 3단계·NTRIP + 타임스탬프 |
| `{prefix}/init_heading_icp` | 초기 heading 정합 성공/상태 + 타임스탬프 |

### 별도 토픽

| MQTT topic (기본) | 내용 |
|-------------------|------|
| `{prefix}/ligo_mode` | `/ligo/mode` 미러(실내/실외 모드 JSON) |

---

## ROS 파라미터 (기본값)

### MQTT / 재연결

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `mqtt.host` | `rms.bottle-tak.com` | MQTT 브로커 호스트 |
| `mqtt.port` | `80` | 포트 |
| `mqtt.topic_prefix` | `navi1` | MQTT 토픽 접두사 |
| `mqtt.use_websocket` | `true` | WebSocket 사용 여부 |
| `mqtt.ws_path` | `/mqtt` | WebSocket path |
| `mqtt.username` | `""` | (선택) 사용자명 |
| `mqtt.password` | `""` | (선택) 비밀번호 |
| `reconnect_period_sec` | `1.0` | MQTT 재연결 시도 주기(초) |

### 구독 ROS 토픽

| 파라미터 | 기본값 | 메시지 타입 |
|----------|--------|-------------|
| `topic.global_position` | `/ligo/global_position` | `sensor_msgs/NavSatFix` |
| `topic.odom` | `/aft_mapped_to_init` | `nav_msgs/Odometry` |
| `topic.receiver_pvt` | `/ublox_driver/receiver_pvt` | `gnss_comm/msg/GnssPVTSolnMsg` |
| `topic.heading_align_status` | `/ligo/nmea_heading_align_status` | `ligo/msg/NmeaHeadingAlignStatus` |
| `topic.ligo_mode` | `/ligo/mode` | `std_msgs/String` (JSON 문자열) |

`gnss_comm.msg.GnssPVTSolnMsg` import 실패 시 `/receiver_pvt` 구독을 생략하고 경고를 출력한다.

---

## 토픽별 JSON

모든 JSON payload에는 공통으로 `timestamp_unix`가 포함된다.

### `{prefix}/position`

```json
{
  "timestamp_unix": 1712551234.123,
  "lat": 37.1234567,
  "lon": 127.1234567
}
```

- 발행: `lat/lon` 값이 변경될 때만

### `{prefix}/heading`

```json
{
  "timestamp_unix": 1712551234.123,
  "deg_from_north_cw": 42.8,
  "cardinal": "NE"
}
```

- 발행: `init_heading_icp.success == true`(LOCKED) 이후 odom 수신마다

### `{prefix}/gps`

```json
{
  "timestamp_unix": 1712551234.123,
  "status": "신호정상",
  "ntrip_connected": true
}
```

- 발행: `receiver_pvt` 수신마다
- `status` 판정:
  1. `valid_fix == false` 또는 `fix_type == 0` → `신호없음`
  2. 위가 아니고 `carr_soln == 2` → `신호정상`
  3. 그 외 → `신호미약`
- `ntrip_connected`:
  - `diff_soln == true` 또는 `carr_soln in {1, 2}` → `true`

### `{prefix}/init_heading_icp`

```json
{
  "timestamp_unix": 1712551234.123,
  "success": true,
  "status": "LOCKED"
}
```

- 발행: `success` 또는 `status` 변경 시에만
- `status`: `UNALIGNED` | `COLLECTING` | `LOCKED`

### `{prefix}/ligo_mode` (기본 `navi1/ligo_mode`)

`/ligo/mode`(`std_msgs/String`)의 JSON을 MQTT용으로 축약해 발행한다.

입력 예:
```json
{"mode":"indoor","map_pcd":"/path/to/scans_3.pcd"}
```

출력 예:
```json
{"timestamp_unix":1712551234.123,"mode":"indoor","pcd_name":"scans_3.pcd"}
```

- `mode == "outdoor"`면 `{"mode":"outdoor"}` 형태
- 입력이 JSON이 아니면 원문 바이트를 그대로 publish

---

## 실행 예

```bash
python3 scripts/ligo_topic_to_mqtt.py --ros-args \
  -p mqtt.host:=127.0.0.1 \
  -p mqtt.port:=1883 \
  -p mqtt.use_websocket:=false \
  -p mqtt.topic_prefix:=navi1 \
```

의존성: `pip install paho-mqtt`

---

## 모드 제어 토픽 payload (`mqtt_mode_orchestrator.py`)

기본 토픽:

- 제어 입력: `navi1/control/mode`
- 상태 출력: `navi1/control/mode_status`

### 제어 입력 (`navi1/control/mode`)

#### 1) Mapping 시작 (`map_name`, `sub_map_name` 필수)

```json
{
  "command": "start",
  "mapping_mode": true,
  "map_name": "site_a",
  "sub_map_name": "floor_1"
}
```

규칙:

- `mapping_mode=true`일 때 `map_name`, `sub_map_name`은 필수.
- `map_name` 허용 문자: 영문/숫자/`_`/`-`.
- 누락/공백/형식 오류 시 실행하지 않고 에러 상태를 발행.

#### 2) Odometry 시작

```json
{
  "command": "start",
  "mapping_mode": false
}
```

odometry는 `map_name`(단일)만 입력받습니다. 이 이름을 기준으로 `PCD/<map_name>/`를 열고, 하위의 sub-map 디렉터리들(`*_grid2d.yaml`)을 전부 로드합니다.

```json
{
  "command": "start",
  "mapping_mode": false,
  "map_name": "building_a"
}
```

#### 3) 현재 모드 종료

```json
{
  "command": "stop"
}
```

### 상태 출력 (`navi1/control/mode_status`)

```json
{
  "timestamp_unix": 1776800100.123,
  "event": "start",
  "status": "ok",
  "message": "mapping 모드 실행을 시작했습니다.",
  "mode": "mapping",
  "map_name": "site_a_20260422",
  "pid": 12345
}
```

`event` 값:

- `ready`: 오케스트레이터 기동 완료
- `start`: 모드 실행 시작
- `stop`: stop 요청 또는 서비스 종료에 따른 종료
- `ended`: 하위 launch 프로세스 자체 종료
- `control` + `status=error`: 잘못된 제어 메시지
