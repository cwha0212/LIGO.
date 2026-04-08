# LIGO MQTT Payload 스펙

`scripts/ligo_topic_to_mqtt.py`가 MQTT로 발행하는 JSON 포맷 정리.

## 발행 개요

- 발행 주기: 기본 `0.5s` (`publish_period_sec`)
- **MQTT 토픽은 항목별로 분리**한다. 접두사는 `mqtt.topic_prefix` (기본 `navi1`).
- 전송 방식: 기본 WebSocket (`mqtt.use_websocket=true`, `mqtt.ws_path=/mqtt`)

### MQTT 토픽 (기본 prefix `navi1`)

| MQTT topic | 내용 |
|------------|------|
| `navi1/position` | 위도·경도 |
| `navi1/heading` | 북 기준 각도·16방위 |
| `navi1/gps` | 신호 3단계·NTRIP |
| `navi1/init_heading_icp` | 초기 heading 정합 성공 여부 |

접두사 변경 예:

```bash
python3 scripts/ligo_topic_to_mqtt.py --ros-args -p mqtt.topic_prefix:=fleet_a
```

→ `fleet_a/position`, `fleet_a/heading`, …

---

## 각 MQTT 토픽별 JSON

### `…/position`

```json
{
  "timestamp_unix": 1712551234.123,
  "lat": 37.1234567,
  "lon": 127.1234567
}
```

- 입력 ROS: `/ligo/global_position` (`sensor_msgs/NavSatFix`)

### `…/heading`

```json
{
  "timestamp_unix": 1712551234.123,
  "deg_from_north_cw": 42.8,
  "cardinal": "NE"
}
```

- 입력 ROS: `/aft_mapped_to_init` (`nav_msgs/Odometry`) 자세 쿼터니언

### `…/gps`

```json
{
  "timestamp_unix": 1712551234.123,
  "status": "신호정상",
  "ntrip_connected": true
}
```

- `status`: `신호없음` | `신호미약` | `신호정상`
- 입력 ROS: `/receiver_pvt` (`gnss_comm/msg/GnssPVTSolnMsg`)

#### `status` 판정 (PVT만 사용)

1. `valid_fix == false` 또는 `fix_type == 0` → `신호없음`
2. `valid_fix == true`, `fix_type != 0`, `carr_soln == 2` (GPS_fixed) → `신호정상`
3. 나머지 → `신호미약`

#### `ntrip_connected`

- `diff_soln == true` 또는 `carr_soln in {1, 2}` → `true`, 그 외 `false` 또는 미수신 시 `null`

### `…/init_heading_icp`

```json
{
  "timestamp_unix": 1712551234.123,
  "success": true
}
```

- 입력 ROS: `/ligo/nmea_heading_align_status` (`ligo/msg/NmeaHeadingAlignStatus`)
- `success`: `icp_tf_ready == true` 이고 `status == STATUS_LOCKED`

---

## Null 가능성

초기 구간에서 `lat`/`lon`, heading, `gps.status` 등이 `null`일 수 있음.

---

## 관련 ROS 토픽(기본값)

- `/ligo/global_position` (`NavSatFix`)
- `/aft_mapped_to_init` (`Odometry`)
- `/receiver_pvt` (`GnssPVTSolnMsg`)
- `/ligo/nmea_heading_align_status` (`NmeaHeadingAlignStatus`)
