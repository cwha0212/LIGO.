# systemd MQTT 모드 오케스트레이터 명령 순서

아래 순서대로 실행하면 서비스 설치부터 반복 동작 검증까지 진행할 수 있습니다.

## 1) 빌드

```bash
cd /home/chang/projects/NAVICOM/GPS_LIO_ws
colcon build --symlink-install --packages-select ligo
```

## 2) 서비스 파일 설치

```bash
sudo cp /home/chang/projects/NAVICOM/GPS_LIO_ws/src/LIGO./systemd/ligo-mode-orchestrator.service /etc/systemd/system/
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

### 5-1) Mapping 시작 (`map_name` 필수)

```bash
python3 - <<'PY'
import json
import paho.mqtt.client as mqtt

topic = "navi1/control/mode"
payload = {"command": "start", "mapping_mode": True, "map_name": "site_a_20260422"}

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
payload = {"command": "start", "mapping_mode": False}

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
