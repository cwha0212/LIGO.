# LIGO MQTT 토픽 목록


## 토픽 종류 요약

| MQTT 토픽 | 전달 내용(요약) |
|-----------|-----------------|
| `{prefix}/position` | 위도·경도 |
| `{prefix}/heading` | 북 기준 시계방향 방위각(도)·16방위 문자열 |
| `{prefix}/gps` | GPS 신호 3단계 문자열·NTRIP(실사용) 근사 여부 |
| `{prefix}/init_heading_icp` | heading추정 여부 |

## 발행 시점

**고정 주기 타이머로 네 토픽을 돌려가며 쏘는 방식은 아니다.** 아래 이벤트에서 각각 `publish` 한다.

| MQTT 토픽 | 발행 조건 |
|-----------|-----------|
| `{prefix}/position` | `NavSatFix` 수신 시, `lat`/`lon` 값이 이전과 달라졌을 때 (사실상 계속) |
| `{prefix}/heading` | heading추정이 완료된 뒤 매 odom마다 |
| `{prefix}/gps` | `GnssPVTSolnMsg` 수신마다 |
| `{prefix}/init_heading_icp` | `NmeaHeadingAlignStatus`에서 `success` 또는 `status`가 바뀔 때만 |
| (전체) | MQTT **재연결 직후** 캐시 상태로 네 토픽 각 1회 |

연결이 끊기면 발행은 생략되고, `reconnect_period_sec` 주기로 재연결을 시도한다.

## 토픽별 JSON 필드

### `{prefix}/position`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `lat` | float \| null | 위도(도) |
| `lon` | float \| null | 경도(도) |

**ROS 입력:** `topic.global_position`(기본 `/ligo/global_position`) — `sensor_msgs/NavSatFix`의 `latitude`, `longitude`.

---

### `{prefix}/heading`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `deg_from_north_cw` | float \| null | 북을 0°로 한 **시계방향** 방위각(0~360°, 도) |
| `cardinal` | string \| null | 16방위 (`N`, `NNE`, …) |

**ROS 입력:** `topic.odom`(기본 `/aft_mapped_to_init`) — `nav_msgs/Odometry` 자세 쿼터니언 → yaw → ENU 기준을 북 기준 heading으로 변환.

**참고:** ICP가 잠기기 전에는 오도메트리로 내부 상태만 갱신하고, **MQTT `heading`은 위 조건에서만** 나간다.

---

### `{prefix}/gps`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `status` | string \| null | `신호없음` \| `신호미약` \| `신호정상` |
| `ntrip_connected` | bool \| null | 차분/캐리어 솔루션 기반 NTRIP(실사용) 근사 |

**ROS 입력:** `topic.receiver_pvt`(기본 `/ublox_driver/receiver_pvt`) — `gnss_comm/msg/GnssPVTSolnMsg`.

- **`status`:** `valid_fix==false` 또는 `fix_type==0` → `신호없음`; 그렇지 않고 `carr_soln==2` → `신호정상`; 그 외 유효 픽스 → `신호미약`.
- **`ntrip_connected`:** `diff_soln==true` 또는 `carr_soln in {1,2}` → `true`, 아니면 `false`.

`GnssPVTSolnMsg` import 실패 시 해당 구독이 생략되면 `status`·`ntrip_connected`는 갱신·발행 흐름이 달라질 수 있다.

---

### `{prefix}/init_heading_icp`

| 필드 | 타입 | 의미 |
|------|------|------|
| `timestamp_unix` | float | 발행 시각(Unix 초) |
| `success` | bool | `icp_tf_ready`이고 `status == STATUS_LOCKED`이면 `true`, 아니면 `false` |
| `status` | string \| null | `UNALIGNED` \| `COLLECTING` \| `LOCKED` |

**ROS 입력:** `topic.heading_align_status`(기본 `/ligo/nmea_heading_align_status`) — `ligo/msg/NmeaHeadingAlignStatus`.

---

## JSON 예시

```json
{"timestamp_unix": 1712551234.123, "lat": 37.1234567, "lon": 127.1234567}
```

```json
{"timestamp_unix": 1712551234.123, "deg_from_north_cw": 42.8, "cardinal": "NE"}
```

```json
{"timestamp_unix": 1712551234.123, "status": "신호정상", "ntrip_connected": true}
```

```json
{"timestamp_unix": 1712551234.123, "success": true, "status": "LOCKED"}
```

초기 구간·토픽 미수신 시 `lat`/`lon`, `deg_from_north_cw`/`cardinal`, `status`/`ntrip_connected` 등이 `null`일 수 있다.

---

더 자세한 페이로드·파라미터 표는 `docs/mqtt_payload_spec.md`를 참고한다. (내용이 스크립트와 어긋나면 **소스 `ligo_topic_to_mqtt.py`를 기준**으로 본다.)
