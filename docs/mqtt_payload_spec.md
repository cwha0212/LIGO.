# LIGO MQTT Payload 스펙

`scripts/ligo_topic_to_mqtt.py`가 MQTT로 발행하는 **토픽별 JSON** 형식과 ROS 파라미터를 정리한다.

## 발행 동작

- **상시 주기 발행 아님**. ROS 입력 이벤트가 생길 때만 해당 MQTT 토픽을 발행한다.
- **재연결 타이머**(`reconnect_period_sec`, 기본 `1.0s`)는 MQTT 재접속 용도이며 데이터 주기 발행용이 아니다.
- MQTT 재연결 직후에는 캐시 상태로 4개 토픽을 각각 1회 발행한다.
- **MQTT QoS**: `0`, **retain**: `false` (코드 고정).
- **전송**: 기본 WebSocket (`mqtt.use_websocket=true`, `mqtt.ws_path=/mqtt`).
- **페이로드**: UTF-8 JSON 문자열 (`ensure_ascii=False`).

---

## MQTT 토픽 이름

접두사는 파라미터 `mqtt.topic_prefix` (기본 `navi1`). 앞뒤 `/`는 자동 제거 후 아래 형태로 조합한다.

| MQTT topic (기본) | 내용 |
|-------------------|------|
| `{prefix}/position` | 위도·경도 + 타임스탬프 |
| `{prefix}/heading` | 북 기준 각도·16방위 + 타임스탬프 |
| `{prefix}/gps` | 신호 3단계·NTRIP + 타임스탬프 |
| `{prefix}/init_heading_icp` | 초기 heading 정합 성공 여부 + 타임스탬프 |

예: `mqtt.topic_prefix:=fleet_a` → `fleet_a/position`, `fleet_a/heading`, …

---

## ROS 파라미터 (기본값)

### MQTT / 주기

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

### 구독할 ROS 토픽

| 파라미터 | 기본값 | 메시지 타입 |
|----------|--------|-------------|
| `topic.global_position` | `/ligo/global_position` | `sensor_msgs/NavSatFix` |
| `topic.odom` | `/aft_mapped_to_init` | `nav_msgs/Odometry` |
| `topic.receiver_pvt` | `/ublox_driver/receiver_pvt` | `gnss_comm/msg/GnssPVTSolnMsg` |
| `topic.heading_align_status` | `/ligo/nmea_heading_align_status` | `ligo/msg/NmeaHeadingAlignStatus` |

`gnss_comm.msg.GnssPVTSolnMsg` import에 실패하면 `/receiver_pvt` 구독을 생략하고 경고 로그만 남긴다. 이 경우 `gps.status`·`ntrip_connected`는 갱신되지 않을 수 있다.

### 실행 예

```bash
# 워크스페이스에서 ligo 메시지·의존성이 잡힌 뒤
python3 scripts/ligo_topic_to_mqtt.py --ros-args \
  -p mqtt.topic_prefix:=navi1 \
  -p mqtt.host:=rms.bottle-tak.com \
  -p mqtt.port:=80
```

의존성: `pip install paho-mqtt`

---

## 각 MQTT 토픽별 JSON

모든 페이로드에 공통으로 `timestamp_unix`(float, 초)가 포함된다.

### `{prefix}/position`

```json
{
  "timestamp_unix": 1712551234.123,
  "lat": 37.1234567,
  "lon": 127.1234567
}
```

- ROS 입력: `topic.global_position` (`NavSatFix`의 `latitude`, `longitude`).
- 발행 시점: `lat`/`lon`이 이전 값과 달라졌을 때만.

### `{prefix}/heading`

```json
{
  "timestamp_unix": 1712551234.123,
  "deg_from_north_cw": 42.8,
  "cardinal": "NE"
}
```

- `deg_from_north_cw`: 북을 0°로 한 시계방향 방위각 (0~360°, 도).
- `cardinal`: 16방위 문자열.
- ROS 입력: `topic.odom` 오도메트리 자세 쿼터니언 → yaw 변환 후 위 각도로 환산.
- 발행 시점: **`init_heading_icp.success == true`(LOCKED) 이후** odom 수신마다.

### `{prefix}/gps`

```json
{
  "timestamp_unix": 1712551234.123,
  "status": "신호정상",
  "ntrip_connected": true
}
```

- `status`: `신호없음` | `신호미약` | `신호정상` (문자열).
- ROS 입력: `topic.receiver_pvt` (`GnssPVTSolnMsg`).
- 발행 시점: `receiver_pvt` 수신마다(상시).

#### `status` 판정 (`receiver_pvt`만 사용)

1. `valid_fix == false` 또는 `fix_type == 0` → `신호없음`
2. 위가 아니고 `carr_soln == 2` (드라이버 기준 GPS_fixed) → `신호정상`
3. 그 외 → `신호미약`

#### `ntrip_connected`

- `diff_soln == true` 또는 `carr_soln in {1, 2}` → `true`
- 그 외 → `false`
- PVT 미수신·import 실패 시 `null` 가능

### `{prefix}/init_heading_icp`

```json
{
  "timestamp_unix": 1712551234.123,
  "success": true,
  "status": "LOCKED"
}
```

- `success`: `icp_tf_ready == true` 이고 `status == STATUS_LOCKED`일 때 `true`.
- `status`: `UNALIGNED` | `COLLECTING` | `LOCKED`.
- ROS 입력: `topic.heading_align_status` (`ligo/NmeaHeadingAlignStatus`).
- 발행 시점: `success` 또는 `status`가 **변경될 때만**.

---

## Null 가능성

초기 구간 또는 토픽 미수신 시 `lat`/`lon`, `deg_from_north_cw`/`cardinal`, `gps.status`/`ntrip_connected` 등이 JSON에서 `null`일 수 있다.
