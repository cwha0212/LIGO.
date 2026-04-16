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

## MQTT 목표 직선 주행 노드

`scripts/mqtt_straight_goal_follower.py` 는 아래 목표 토픽만 구독한다.

- `{prefix}/nav/goal` : `{ "lat": <float>, "lon": <float> }`

`scripts/mqtt_nav2_straight_follower.py` 는 `start_pose` + `goal`을 구독하고,
`start_pose` 수신 시점을 ENU 원점으로 고정한 뒤 Nav2 `FollowPath` 액션으로 직선 경로를 추종한다.

- 구독: `{prefix}/nav/start_pose` : `{ "lat": <float>, "lon": <float>, "deg_from_north_cw": <float> }`
- 구독: `{prefix}/nav/goal` : `{ "lat": <float>, "lon": <float> }`
- 발행: `{prefix}/nav/reached` : `{ "status": "reached", "lat": <float>, "lon": <float>, "timestamp_unix": <float> }`

실행 예시:

- `ros2 run ligo mqtt_straight_goal_follower.py`
- `ros2 run ligo mqtt_nav2_straight_follower.py`
- `ros2 run ligo mqtt_nav_pub.py --kind goal --lat 37.41200 --lon 127.09320`
- Nav2 포함 통합 실행:
  - `ros2 launch ligo nav2_mqtt_straight.launch.py`
  - (선택) `path_frame`/`nav2_params_file` 지정:
    - `ros2 launch ligo nav2_mqtt_straight.launch.py path_frame:=map nav2_params_file:=/abs/path/nav2_params.yaml`
  - (선택) `start_pose` MQTT 토픽 오버라이드:
    - `ros2 launch ligo nav2_mqtt_straight.launch.py start_pose_topic:=myrobot/nav/start_pose`
  - (선택) ENU 원점 마커 토픽 오버라이드:
    - `ros2 launch ligo nav2_mqtt_straight.launch.py enu_origin_marker_topic:=/my_enu_origin_marker`
  - 기본은 LIGO 프레임(`map` + `aft_mapped`)에 맞춘 `config/nav2_ligo_params.yaml`을 사용한다.

실전 권장 실행(분리):

- 터미널 A (LIGO):
  - `ros2 launch ligo nx_mapping.launch.py`
- 터미널 B (Nav2 + follower):
  - `ros2 launch ligo nav2_mqtt_straight.launch.py use_ligo_mapping:=false use_mqtt_bridge:=false`

RViz 점검용 토픽:

- `/cmd_vel_arrow` (`visualization_msgs/Marker`)
  - follower 노드가 `/cmd_vel`을 구독해 현재 위치 기준 진행 방향 화살표를 발행
  - RViz에서 Marker 디스플레이 추가 후 topic을 `/cmd_vel_arrow`로 설정
- `/enu_origin_marker` (`visualization_msgs/Marker`)
  - `start_pose` 수신 후 ENU 원점(북기준 heading 포함)을 화살표로 표시
  - RViz에서 Marker 디스플레이 추가 후 topic을 `/enu_origin_marker`로 설정

가상 시뮬레이션 실행(Gazebo + RViz 선택):

- 기본(RViz + 가상 GPS/odom 폐루프):
  - `ros2 launch ligo nav2_mqtt_virtual_sim.launch.py`
- Gazebo도 함께 실행(설치된 경우):
  - `ros2 launch ligo nav2_mqtt_virtual_sim.launch.py use_gazebo:=true`
- 가상 기준 위경도 변경:
  - `ros2 launch ligo nav2_mqtt_virtual_sim.launch.py origin_lat:=37.4115 origin_lon:=127.0931`
- ENU 원점 마커 토픽 변경:
  - `ros2 launch ligo nav2_mqtt_virtual_sim.launch.py enu_origin_marker_topic:=/my_enu_origin_marker`

`start_pose + goal` MQTT 기반 테스트(현재 follower 코드 검증용):

1) 시뮬레이션 실행
- `ros2 launch ligo nav2_mqtt_virtual_sim.launch.py`

2) 시작 pose 입력 (`lat`, `lon`, `deg_from_north_cw`)
- topic: `{prefix}/nav/start_pose` (기본: `navi1/nav/start_pose`)
- 예시:
  - `ros2 run ligo mqtt_nav_pub.py --kind start_pose --lat 37.41150000 --lon 127.09310000 --heading 90.0`
- heading 규칙:
  - `deg_from_north_cw` = 북쪽 기준 시계방향 각도(도)
  - 예: 북쪽=0, 동쪽=90, 남쪽=180, 서쪽=270
  - `start_pose` 수신 시 해당 위경도를 ENU 원점(`x=0,y=0`)으로 재설정

3) 목표 위경도 입력 (`lat`, `lon`)
- topic: `{prefix}/nav/goal` (기본: `navi1/nav/goal`)
- 예시:
  - `ros2 run ligo mqtt_nav_pub.py --kind goal --lat 37.41151521 --lon 127.09310285`

RViz에서 확인할 핵심:
- `TF` 디스플레이: `map -> aft_mapped` (robot frame 이동)
- Marker topic `/cmd_vel_arrow`: follower가 출력 cmd_vel 방향 표시
- Marker topic `/sim_robot_marker`: 가상 로봇 형상
- Path topic `/sim_path`: 가상 로봇 이동 궤적

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
