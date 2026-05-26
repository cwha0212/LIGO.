# MQTT 사양 (`nmea_enable=true`)

`control/mode`의 `start` 메시지에서 `nmea_enable`을 생략하거나 `true`로 주면 기존 GNSS/NMEA 융합 경로를 사용한다.

## 1) 시작 제어 메시지

토픽: `{prefix}/control/mode`

### Mapping

```json
{
  "command": "start",
  "mapping_mode": true,
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "nmea_enable": true
}
```

### Odometry

```json
{
  "command": "start",
  "mapping_mode": false,
  "map_name": "building_a",
  "nmea_enable": true
}
```

## 2) ROS -> MQTT 데이터

브리지: `scripts/ligo_topic_to_mqtt.py`

- `{prefix}/position`
  - 필드: `lat`, `lon`, `coordinate_type: "wgs84"`
  - 소스: `/ligo/global_position` (`NavSatFix`)
- `{prefix}/heading`
  - 필드: `deg_from_north_cw`, `cardinal`, `coordinate_type: "enu_north"`
  - 소스: `/aft_mapped_to_init` + `/ligo/nmea_heading_align_status`
  - 조건: ICP LOCK 이후 발행
- `{prefix}/gps`
  - 필드: `status`, `ntrip_connected`
  - 상태: `신호없음/신호미약/신호정상`
- `{prefix}/init_heading_icp`
  - 필드: `success`, `status`

## 3) 저장/검증 상태 메시지

오케스트레이터: `scripts/mqtt_mode_orchestrator.py`

토픽: `{prefix}/control/mode_status`

`mapping` 종료 후 `map_saved`에서 다음 4개 산출물을 검증한다.

- `{sub_map_name}.pcd`
- `{sub_map_name}_orig.pcd`
- `{sub_map_name}_grid2d.pgm`
- `{sub_map_name}_grid2d.yaml`

관련 이벤트:

- `stop`: 종료 결과 (`exit_code`, `save_elapsed_sec`, `nmea_enable`)
- `map_saved`: 로컬 저장 결과 (`files`, `missing`, `artifact_bytes_total`, `nmea_enable`)
- `map_upload`: 공유 경로 업로드 결과 (`elapsed_sec`, `nmea_enable`)
- `map_upload_verify`: 업로드 검증 결과 (`mismatches`, `bytes_total`, `nmea_enable`)

## 4) MQTT 메시지 예시

### `{prefix}/position`

```json
{
  "timestamp_unix": 1779770100.123,
  "lat": 37.3861,
  "lon": 127.1152,
  "coordinate_type": "wgs84"
}
```

### `{prefix}/heading`

```json
{
  "timestamp_unix": 1779770100.456,
  "deg_from_north_cw": 134.72,
  "cardinal": "SE",
  "coordinate_type": "enu_north"
}
```

### `{prefix}/gps`

```json
{
  "timestamp_unix": 1779770100.789,
  "status": "신호정상",
  "ntrip_connected": true
}
```

> `status` 값: `"신호없음"` / `"신호미약"` / `"신호정상"`

### `{prefix}/init_heading_icp`

```json
{
  "timestamp_unix": 1779770100.012,
  "success": true,
  "status": "LOCKED"
}
```

> `status` 값: `"UNALIGNED"` → `"COLLECTING"` → `"LOCKED"`

### `{prefix}/ligo_mode`

```json
{
  "timestamp_unix": 1779770100.345,
  "mode": "indoor",
  "pcd_name": "building_a.pcd"
}
```

### `{prefix}/control/mode_status` (start)

```json
{
  "event": "start",
  "status": "ok",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "pid": 12345,
  "command": "ros2 launch ligo nx_mapping.launch.py ...",
  "nmea_enable": true
}
```

### `{prefix}/control/mode_status` (stop)

```json
{
  "event": "stop",
  "status": "ok",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "exit_code": 0,
  "reason": "user_stop",
  "save_elapsed_sec": 3.21,
  "nmea_enable": true
}
```

### `{prefix}/control/mode_status` (map_saved)

```json
{
  "event": "map_saved",
  "status": "ok",
  "message": "맵 저장이 완료되었습니다.",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "save_root": "/home/user/ligo_maps/site_a/floor_1",
  "files": [
    {"name": "floor_1.pcd", "size": 10485760},
    {"name": "floor_1_orig.pcd", "size": 20971520},
    {"name": "floor_1_grid2d.pgm", "size": 524288},
    {"name": "floor_1_grid2d.yaml", "size": 256}
  ],
  "missing": [],
  "artifact_bytes_total": 31981824,
  "nmea_enable": true
}
```

### `{prefix}/control/mode_status` (map_upload)

```json
{
  "event": "map_upload",
  "status": "ok",
  "message": "맵 업로드가 완료되었습니다.",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "elapsed_sec": 5.234,
  "nmea_enable": true
}
```

### `{prefix}/control/mode_status` (map_upload_verify)

```json
{
  "event": "map_upload_verify",
  "status": "ok",
  "message": "업로드 검증이 완료되었습니다.",
  "mode": "mapping",
  "map_name": "site_a",
  "sub_map_name": "floor_1",
  "mismatches": [],
  "bytes_total": 31981824,
  "nmea_enable": true
}
```

## 5) 동기화(synchronization)

`synchronization` 명령은 `nmea_enable`과 무관하게 동일하게 동작한다.

- `sync`: rsync 실행 결과
- `sync_verify`: 공유 -> 로컬 파일 용량 검증 결과
