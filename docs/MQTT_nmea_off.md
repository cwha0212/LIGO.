# MQTT 사양 (`nmea_enable=false`)

`control/mode`의 `start` 메시지에서 `nmea_enable=false`를 주면 전역 WGS84 대신 로컬 좌표 기반 MQTT를 사용한다.

## 1) 시작 제어 메시지

토픽: `{prefix}/control/mode`

### Mapping

```json
{
  "command": "start",
  "mapping_mode": true,
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "nmea_enable": false
}
```

### Odometry

```json
{
  "command": "start",
  "mapping_mode": false,
  "map_name": "building_a",
  "nmea_enable": false
}
```

## 2) Odometry 동작

- odometry는 `PCD/<map_name>/` 기준 indoor grid/PCD를 로드한다.
- small_gicp는 NMEA ICP 정렬 여부와 무관하게 실행된다.
- 위치 기준:
  - odometry: 지도 기준 `frame="map"`
  - mapping: 세션 시작점 기준 `frame="session"` (시작점을 `(0,0,0)`으로 사용)

## 3) ROS -> MQTT 데이터

브리지: `scripts/ligo_topic_to_mqtt.py`

- `{prefix}/position`
  - 필드: `x`, `y`, `z`
  - 소스: `/ligo/mqtt_pose`
- `{prefix}/heading`
  - 필드: `yaw_deg`
  - 로컬 frame yaw (북 기준 heading 아님)
- `{prefix}/gps`
  - **발행하지 않음** (`nmea_enable=false`에서는 불필요)
- `{prefix}/init_heading_icp`
  - **발행하지 않음** (`nmea_enable=false`에서는 불필요)

## 4) 저장/검증 상태 메시지

오케스트레이터: `scripts/mqtt_mode_orchestrator.py`

토픽: `{prefix}/control/mode_status`

`mapping` 종료 후 `map_saved`에서 필수 산출물은 다음 1개만 검증한다.

- `{sub_map_name}.pcd`

관련 이벤트는 모두 `nmea_enable` 필드를 포함한다. `event`는 task, `status`는 진행 상태.

- `mapping` (`start`, `stop`, `fail`)
- `odometry` (`start`, `stop`, `fail`)
- `ended` (`ok`, `fail`)
- `map_saved` (`start`, `finish`, `fail`)
- `map_upload` (`start`, `finish`, `fail`)
- `map_upload_verify` (`success`, `fail`)

## 5) MQTT 메시지 예시

### `{prefix}/position`

```json
{
  "timestamp_unix": 1779770100.123,
  "x": 12.3456,
  "y": -3.2100,
  "z": 0.1520
}
```

### `{prefix}/heading`

```json
{
  "timestamp_unix": 1779770100.456,
  "yaw_deg": 45.32
}
```

### `{prefix}/gps`

> `nmea_enable=false`에서는 발행하지 않음.

### `{prefix}/init_heading_icp`

> `nmea_enable=false`에서는 발행하지 않음.

### `{prefix}/ligo_mode`

```json
{
  "timestamp_unix": 1779770100.345,
  "mode": "indoor",
  "pcd_name": "building_a.pcd"
}
```

### `{prefix}/control/mode_status` (mapping start)

```json
{
  "event": "mapping",
  "status": "start",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "pid": 12345,
  "command": "ros2 launch ligo nx_mapping.launch.py ...",
  "nmea_enable": false
}
```

### `{prefix}/control/mode_status` (mapping stop)

```json
{
  "event": "mapping",
  "status": "stop",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "exit_code": 0,
  "reason": "user_stop",
  "save_elapsed_sec": 2.45,
  "nmea_enable": false
}
```

### `{prefix}/control/mode_status` (map_saved finish)

`nmea_enable=false`일 때 필수 산출물은 `.pcd` 1개만 검증한다.

```json
{
  "event": "map_saved",
  "status": "finish",
  "message": "맵 저장이 완료되었습니다.",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "save_root": "/home/user/ligo_maps/site_a/floor_1",
  "files": [
    {"name": "floor_1.pcd", "size": 10485760}
  ],
  "missing": [],
  "artifact_bytes_total": 10485760,
  "nmea_enable": false
}
```

### `{prefix}/control/mode_status` (map_upload finish)

```json
{
  "event": "map_upload",
  "status": "finish",
  "message": "맵 업로드가 완료되었습니다.",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "elapsed_sec": 2.10,
  "nmea_enable": false
}
```

### `{prefix}/control/mode_status` (map_upload_verify success)

```json
{
  "event": "map_upload_verify",
  "status": "success",
  "message": "업로드 검증이 완료되었습니다.",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "mismatches": [],
  "bytes_total": 10485760,
  "nmea_enable": false
}
```

## 6) 동기화(synchronization)

`synchronization` 명령은 `nmea_enable`과 무관하게 동일하게 동작한다.

- `sync` (`start`, `finish`, `fail`)
- `sync_verify` (`success`, `fail`)
