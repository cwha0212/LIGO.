# LIGO MQTT 토픽 목록

현재 저장소에서 MQTT 관련 실행 파일은 아래 3개입니다.

- `scripts/ligo_topic_to_mqtt.py` (ROS -> MQTT 브리지)
- `scripts/mqtt_mode_orchestrator.py` (MQTT 제어 -> mapping/odometry 런치 오케스트레이션)
- `scripts/mqtt_nav_pub.py` (테스트용 MQTT 발행 유틸)

## 1) ROS -> MQTT 브리지 토픽 (`ligo_topic_to_mqtt.py`)

토픽 prefix 기본값은 `navi1`이며, 실제 토픽은 `{prefix}/...` 형태입니다.

| MQTT 토픽 | 전달 내용(요약) |
|-----------|-----------------|
| `{prefix}/position` | 위도/경도 |
| `{prefix}/heading` | 북기준 시계방향 heading(도), 16방위 |
| `{prefix}/gps` | GPS 상태(3단계), NTRIP 연결 상태 |
| `{prefix}/init_heading_icp` | ICP 초기 heading 정합 상태 |
| `{prefix}/ligo_mode` | `/ligo/mode` 미러 payload |

### 발행 특징

- 고정 주기 발행이 아니라 이벤트 기반 발행입니다.
- MQTT 재연결 직후 캐시 상태를 1회 재발행합니다.
- 연결 단절 시 publish를 건너뛰고 `reconnect_period_sec` 주기로 재연결합니다.

상세 payload는 `docs/mqtt_payload_spec.md`를 참고하세요.

## 2) 모드 제어 토픽 (`mqtt_mode_orchestrator.py`)

부팅 자동 실행(service) 환경에서 mapping/odometry 모드를 제어하는 토픽입니다.

| MQTT 토픽 | 역할 |
|-----------|------|
| `navi1/control/mode` | 모드 시작/종료 제어 메시지 입력 |
| `navi1/control/mode_status` | 오케스트레이터 상태/오류/종료 이벤트 출력 |

### 제어 메시지 예시

- Mapping 시작 (`map_name` 필수)
  - `{"command":"start","mapping_mode":true,"map_name":"site_a_20260422"}`
- Odometry 시작(선택: `map_names` — `PCD/<id>/<id>.pcd` 우선순위; `map_name` 미사용)
  - `{"command":"start","mapping_mode":false}`
  - `{"command":"start","mapping_mode":false,"map_names":["site_a","site_b"]}`
- 종료
  - `{"command":"stop"}`

상세 규칙과 운영 명령은 아래 문서를 참고하세요.

- `docs/systemd_mode_orchestrator.md`
- `docs/systemd_mode_orchestrator_commands.md`

## 3) 테스트 발행 유틸 (`mqtt_nav_pub.py`)

`mqtt_nav_pub.py`는 내비게이션 관련 테스트 payload를 1회 발행하는 도구입니다.

예시:

- `ros2 run ligo mqtt_nav_pub.py --kind goal --lat 37.41200 --lon 127.09320`
- `ros2 run ligo mqtt_nav_pub.py --kind start_pose --lat 37.41150000 --lon 127.09310000 --heading 90.0`
