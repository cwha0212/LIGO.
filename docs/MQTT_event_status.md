# MQTT Event / Status 요약

토픽: `{prefix}/control/mode_status`

`event`는 **작업 종류**, `status`는 해당 작업의 **진행 상태**를 나타낸다.

---

## 제어 명령 → 이벤트 매핑

| 제어 명령 (`control/mode`) | 트리거되는 `event` |
|---|---|
| `{"command": "ready"}` | `ready` |
| `{"command": "start", "mapping_mode": true, ...}` | `mapping` |
| `{"command": "start", "mapping_mode": false, ...}` | `odometry` |
| `{"command": "stop"}` | `mapping` 또는 `odometry` (실행 중인 모드) |
| `{"command": "synchronization", ...}` | `sync` |
| 잘못된 JSON / 알 수 없는 명령 | `control` |

---

## Event / Status 일람

| `event` | `status` | 의미 |
|---------|----------|------|
| **ready** | `ok` | 보드 준비됨 (idle) |
| | `fail` | 다른 작업 진행 중 (busy) |
| **mapping** | `start` | mapping 런치 시작 |
| | `stop` | mapping 정상 종료 |
| | `fail` | mapping 실패 (시작 실패, 거부 등) |
| **odometry** | `start` | odometry 런치 시작 |
| | `stop` | odometry 정상 종료 |
| | `fail` | odometry 실패 (시작 실패, 거부 등) |
| **map_saved** | `finish` | 로컬 맵 저장 검증 완료 |
| | `fail` | 로컬 맵 저장 검증 실패 |
| **map_upload** | `start` | 공유 경로 업로드 시작 |
| | `finish` | 공유 경로 업로드 완료 |
| | `fail` | 공유 경로 업로드 실패 |
| **map_upload_verify** | `success` | 업로드 파일 용량 검증 통과 |
| | `fail` | 업로드 파일 용량 검증 실패 |
| **sync** | `start` | 맵 동기화 시작 |
| | `finish` | 맵 동기화 완료 |
| | `fail` | 맵 동기화 실패 |
| **sync_verify** | `success` | 동기화 파일 용량 검증 통과 |
| | `fail` | 동기화 파일 용량 검증 실패 |
| **ended** | `ok` | 프로세스 자체 정상 종료 (원격 stop 없이) |
| | `fail` | 프로세스 자체 비정상 종료 |
| **control** | `fail` | 잘못된 제어 JSON |

---

## 정상 흐름 예시

### Mapping

```
mapping/start → mapping/stop → map_saved/finish → map_upload/start → map_upload/finish → map_upload_verify/success
```

### Odometry

```
odometry/start → odometry/stop
```

### 동기화

```
sync/start → sync/finish → sync_verify/success
```
