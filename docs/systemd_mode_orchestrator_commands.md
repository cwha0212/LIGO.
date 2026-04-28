# systemd MQTT 모드 오케스트레이터 명령 순서

아래 순서대로 실행하면 서비스 설치부터 반복 동작 검증까지 진행할 수 있습니다.

## 1) 빌드

```bash
cd /home/maum/last_navi
colcon build --symlink-install --packages-select ligo
```

## 2) 서비스 파일 설치

먼저 현재 장비 사용자/워크스페이스 경로에 맞게 unit 파일을 수정합니다.

```bash
id -un
echo $HOME
```

`/home/maum/last_navi/src/LIGO./systemd/ligo-mode-orchestrator.service`에서 아래 값을 반드시 환경에 맞게 바꿉니다.

- `User=maum`
- `WorkingDirectory=/home/maum/last_navi`
- `ExecStart` 내부의 `source /home/maum/last_navi/install/setup.bash`
- `.bashrc`에만 있는 라이브러리/경로가 있다면 `Environment=`로 명시:
  - `Environment=LD_LIBRARY_PATH=/home/maum/local/gtsam-4.1.1/lib:/usr/local/lib:/usr/lib/x86_64-linux-gnu`
  - `Environment=PATH=/usr/local/bin:/usr/bin:/bin`
- DDS 통신 일관성을 위해 ROS 환경을 고정:
  - `Environment=ROS_DOMAIN_ID=30`
  - `Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  - `Environment=ROS_LOCALHOST_ONLY=0`

필요하면 `ExecStart`의 `/bin/bash`를 `/usr/bin/bash`로 변경합니다.

그 다음 설치합니다.

```bash
sudo cp /home/maum/last_navi/src/LIGO./systemd/ligo-mode-orchestrator.service /etc/systemd/system/
sudo systemctl daemon-reload
```

## 3) 자동 시작 등록 + 즉시 시작

```bash
sudo systemctl enable ligo-mode-orchestrator.service
sudo systemctl start ligo-mode-orchestrator.service
```

## 4) 상태 확인

```bash
sudo systemctl status ligo-mode-orchestrator.service
sudo journalctl -u ligo-mode-orchestrator.service -f
```

## 5) MQTT 제어 메시지 발행 예시

브로커가 WebSocket(`:80`, `/mqtt`)을 사용하므로 아래 Python 예시를 권장합니다.

### 5-1) Mapping 시작 (`map_name`, `sub_map_name` 필수)

```bash
python3 - <<'PY'
import json
import paho.mqtt.client as mqtt

topic = "navi1/control/mode"
payload = {"command": "start", "mapping_mode": True, "map_name": "site_a", "sub_map_name": "floor_1"}

c = mqtt.Client(transport="websockets", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
c.ws_set_options(path="/mqtt")
c.connect("rms.bottle-tak.com", 80, keepalive=10)
c.loop_start()
c.publish(topic, json.dumps(payload, ensure_ascii=False), qos=0, retain=False)
c.loop_stop()
c.disconnect()
print("published:", payload)
PY
```

### 5-2) Stop

```bash
python3 - <<'PY'
import json
import paho.mqtt.client as mqtt

topic = "navi1/control/mode"
payload = {"command": "stop"}

c = mqtt.Client(transport="websockets", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
c.ws_set_options(path="/mqtt")
c.connect("rms.bottle-tak.com", 80, keepalive=10)
c.loop_start()
c.publish(topic, json.dumps(payload, ensure_ascii=False), qos=0, retain=False)
c.loop_stop()
c.disconnect()
print("published:", payload)
PY
```

### 5-3) Odometry 시작

```bash
python3 - <<'PY'
import json
import paho.mqtt.client as mqtt

topic = "navi1/control/mode"
payload = {"command": "start", "mapping_mode": False, "map_name": "site_a"}

c = mqtt.Client(transport="websockets", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
c.ws_set_options(path="/mqtt")
c.connect("rms.bottle-tak.com", 80, keepalive=10)
c.loop_start()
c.publish(topic, json.dumps(payload, ensure_ascii=False), qos=0, retain=False)
c.loop_stop()
c.disconnect()
print("published:", payload)
PY
```

## 6) 반복 검증 순서

1. Mapping 시작 메시지 발행
2. 상태 토픽에서 `start` 이벤트 확인
3. Stop 메시지 발행
4. 상태 토픽에서 `stop` 이벤트 확인
5. Odometry 시작 메시지 발행
6. 상태 토픽에서 `start` 이벤트 확인
7. 다시 Stop 후 `stop` 이벤트 확인
8. 1~7 반복

## 7) 운영 중 재시작/중지

```bash
sudo systemctl restart ligo-mode-orchestrator.service
sudo systemctl stop ligo-mode-orchestrator.service
```

## 8) 자주 발생하는 에러 해결

### 에러

- `Failed to determine user credentials: No such file or directory`
- `Failed at step USER spawning /bin/bash: No such file or directory`

### 원인

- `User=`에 지정한 계정이 실제 장비에 없음
- `WorkingDirectory` 또는 `ExecStart`의 홈 경로가 실제 경로와 다름
- 일부 환경에서 bash 경로가 다름

### 해결

1. `/etc/systemd/system/ligo-mode-orchestrator.service`에서 아래 확인/수정
   - `User=<실제 계정>`
   - `WorkingDirectory=<실제 워크스페이스 경로>`
   - `ExecStart`의 `source <실제 install/setup.bash 경로>`
   - 필요 시 `/bin/bash` -> `/usr/bin/bash`
2. 반영

```bash
sudo systemctl daemon-reload
sudo systemctl restart ligo-mode-orchestrator.service
sudo systemctl status ligo-mode-orchestrator.service
```
