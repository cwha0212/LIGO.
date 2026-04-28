# systemd MQTT 모드 오케스트레이터

부팅 직후 자동으로 시작되어 MQTT 제어 메시지로 LIGO 모드를 반복 실행/종료하는 서비스 구성 문서입니다.

## MQTT 토픽

- 제어 구독 토픽: `navi1/control/mode`
- 상태 발행 토픽: `navi1/control/mode_status`

## 제어 메시지 스키마

### 1) Mapping 시작 (`map_name`, `sub_map_name` 필수)

```json
{
  "command": "start",
  "mapping_mode": true,
  "map_name": "site_a",
  "sub_map_name": "floor_1"
}
```

- `map_name`은 영문/숫자/`_`/`-`만 허용됩니다.
- `mapping_mode=true`인데 `map_name` 또는 `sub_map_name`이 누락/공백/형식 오류면 실행하지 않고 에러 상태를 발행합니다.

### 2) Odometry 시작

지도를 쓰지 않고 기동만 할 때(설정의 `pcd_save.map_name` 등으로 참조 시도):

```json
{
  "command": "start",
  "mapping_mode": false
}
```

**odometry는 `map_name`(단일)만 사용합니다.** 이 이름의 하위 디렉터리 `PCD/<map_name>/` 아래에 있는 모든 sub-map(`*_grid2d.yaml`)을 로드합니다.

- 실제 매핑 저장 경로는 `PCD/<map_name>/<sub_map_name>/<sub_map_name>.pcd` 입니다.
- odometry는 `PCD/<map_name>/`를 기준으로 sub-map들을 전부 탐색합니다(재귀).

odometry 예시:

```json
{
  "command": "start",
  "mapping_mode": false,
  "map_name": "site_a"
}
```

### 3) 현재 모드 종료

```json
{
  "command": "stop"
}
```

## 상태 메시지 예시

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

추가 이벤트:

- `ready`: 서비스 기동 완료, 제어 대기 상태
- `start`: 모드 실행 시작
- `stop`: 원격 stop 또는 서비스 종료로 모드 종료
- `ended`: 하위 런치 프로세스가 자체 종료됨(정상/비정상)
- `control` + `status=error`: 잘못된 제어 메시지

## 실행되는 launch

- mapping: `ros2 launch ligo nx_mapping.launch.py map_name:=<map_name> sub_map_name:=<sub_map_name>` → `PCD/<map_name>/<sub_map_name>/`에 저장(동일 이름 시 덮어쓰기)
- odometry: `ros2 launch ligo nx_odometry.launch.py` — 선택 인자 `indoor_map_name:=<map_name>`로 map 그룹 지정

## 설치/운영 명령

명령은 별도 문서에 순서대로 정리되어 있습니다.

- `docs/systemd_mode_orchestrator_commands.md`

## 반복 동작 검증 핵심

1. Mapping 시작 메시지(`map_name` 포함) 발행
2. `mode_status`에서 `start` 이벤트 확인
3. `stop` 메시지 발행 후 `stop` 이벤트 확인
4. Odometry 시작 메시지 발행(`map_name` 단일) 후 `start` 이벤트 확인
5. 필요 시 반복
