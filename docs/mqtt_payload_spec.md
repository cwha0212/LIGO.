# LIGO MQTT Payload 스펙

`scripts/ligo_topic_to_mqtt.py`가 MQTT로 발행하는 JSON 포맷 정리.

## 발행 개요

- 발행 주기: 기본 `0.5s` (`publish_period_sec`)
- 기본 MQTT 토픽: `ligo/status` (`mqtt.topic`)
- 전송 방식: 기본 WebSocket (`mqtt.use_websocket=true`, `mqtt.ws_path=/mqtt`)

---

## Payload 구조

```json
{
  "timestamp_unix": 1712551234.123,
  "position": {
    "lat": 37.1234567,
    "lon": 127.1234567
  },
  "heading": {
    "deg_from_north_cw": 42.8,
    "cardinal": "NE"
  },
  "gps": {
    "status": "신호정상",
    "ntrip_connected": true
  },
  "init_heading_icp": {
    "success": true
  }
}
```

---

## 필드 설명

### `timestamp_unix`

- 타입: `number` (float)
- 의미: 메시지 생성 시각의 UNIX timestamp(초)

### `position`

- `lat`: 현재 위도 (WGS84, degree)
- `lon`: 현재 경도 (WGS84, degree)
- 입력 토픽: `/ligo/global_position` (`sensor_msgs/NavSatFix`)

> 참고: `alt_m`는 현재 요구사항에 따라 payload에서 제외(코드상 주석 처리)됨.

### `heading`

- `deg_from_north_cw`: 북쪽 기준 시계방향 heading (0~360 deg)
- `cardinal`: 16방위 문자열 (`N`, `NNE`, `NE`, ...)
- 입력 토픽: `/aft_mapped_to_init` (`nav_msgs/Odometry`)의 자세 쿼터니언

> 참고: `rad_local_yaw`는 현재 요구사항에 따라 payload에서 제외(코드상 주석 처리)됨.

### `gps`

- `status`: GPS 상태 (문자열)
  - `신호없음`
  - `신호미약`
  - `신호정상`
- `ntrip_connected`: NTRIP 보정 연결 상태 (bool)

#### `gps.status` 판정 로직

입력 토픽:

- `/receiver_pvt` (`gnss_comm/msg/GnssPVTSolnMsg`)

판정 규칙:

1. `receiver_pvt`에서 `No_fix` 조건이면 `신호없음`
   - `valid_fix == false` 또는 `fix_type == 0`
2. 그 외에서 `receiver_pvt`가 `GPS_fixed` 조건이면 `신호정상`
   - `valid_fix == true`
   - `fix_type != 0`
   - `carr_soln == 2`
3. 나머지는 `신호미약`
   - 예: `2D_fix`, `GPS_single`, `GPS_float`

#### `gps.ntrip_connected` 판정 로직

- `/receiver_pvt` 기준
- 아래 중 하나면 `true`
  - `diff_soln == true`
  - `carr_soln in {1, 2}`
- 그 외는 `false` 또는 수신 전 `null`

### `init_heading_icp`

- `success`: 초기 heading ICP 성공 여부 (`bool`)
- 입력 토픽: `/ligo/nmea_heading_align_status` (`ligo/msg/NmeaHeadingAlignStatus`)
- 판정:
  - `icp_tf_ready == true` 이고
  - `status == STATUS_LOCKED`

---

## Null 가능성

토픽이 아직 수신되지 않은 초기 구간에서는 일부 필드가 `null`일 수 있음:

- `position.lat`, `position.lon`
- `heading.deg_from_north_cw`, `heading.cardinal`
- `gps.status`, `gps.ntrip_connected`

---

## 관련 ROS 토픽(기본값)

- `/ligo/global_position` (`NavSatFix`)
- `/aft_mapped_to_init` (`Odometry`)
- `/receiver_pvt` (`GnssPVTSolnMsg`)
- `/ligo/nmea_heading_align_status` (`NmeaHeadingAlignStatus`)

