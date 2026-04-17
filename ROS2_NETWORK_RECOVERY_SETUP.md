# ROS2 네트워크 복구/안정화 설정 가이드

임베디드 보드에서 ROS2 + MQTT + 로봇 SDK를 함께 운영할 때, Wi-Fi 끊김/재연결 이후 `ddsi_udp_conn_write ... retcode -1` 문제가 반복되는 상황을 줄이기 위한 초기 설정 절차입니다.

## 1) 목표

- 네트워크 순간 단절 후 자동 복구
- ROS2(DDS), MQTT, SDK 간 장애 전파 최소화
- rosbag 기반 운용에서 실센서 기반 운용으로 무리 없이 전환

## 2) 네트워크 설계

- 보드, 센서, 로봇 SDK endpoint, MQTT 브로커 IP를 고정합니다.
- 가능하면 ROS 트래픽망과 인터넷망을 분리합니다.
  - ROS 트래픽: 센서/로봇 통신
  - 인터넷 트래픽: MQTT/외부 API
- NIC 분리가 어렵다면 DDS가 사용할 NIC를 반드시 고정합니다.

확인 명령:

```bash
ip a
ip route
```

## 3) ROS2 미들웨어 환경 통일

모든 ROS2 프로세스(launch/노드/서비스)에서 아래 값을 통일합니다.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
```

- 장비별 `ROS_DOMAIN_ID`가 다르면 통신 불안정 또는 미발견 문제가 발생할 수 있습니다.

## 4) CycloneDDS 인터페이스 고정

경로 예시: `/etc/ros/cyclonedds.xml`

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="wlan0"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

적용:

```bash
export CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml
```

- `wlan0`는 실제 ROS 트래픽 인터페이스로 변경합니다. 유선이면 `eth0`가 더 안정적일 수 있습니다.
- 자동 인터페이스 선택은 재연결 시 오동작 가능성이 있어 지양합니다.

## 5) Wi-Fi 안정화

일시 적용:

```bash
iw dev wlan0 set power_save off
```

권장:

- NetworkManager에서 절전 해제를 영구 설정
- 과도한 AP 로밍 환경 회피
- 가능하면 고정 SSID/채널 운영

## 6) systemd 서비스 구성 (자동 복구 핵심)

ROS2 launch를 systemd 서비스로 운영하고 아래 옵션을 사용합니다.

- `After=network-online.target`
- `Wants=network-online.target`
- `Restart=always`
- `RestartSec=2`
- `StartLimitIntervalSec=0`

서비스 환경변수 예시:

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `ROS_DOMAIN_ID=30`
- `CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml`

실행 예시:

```bash
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source /home/chang/projects/NAVICOM/GPS_LIO_ws/install/setup.bash && ros2 launch <your_package> <your_launch>.launch.py'
```

## 7) 네트워크 재연결 이벤트 처리

Wi-Fi down/up 이후 DDS가 이전 peer 정보를 잠시 유지할 수 있으므로, 이벤트 기반 재시작을 권장합니다.

- NetworkManager dispatcher 또는 watchdog에서 아래 수행:
  - `systemctl restart ros2-stack.service`
  - 필요 시 `systemctl restart mqtt-bridge.service`

## 8) MQTT/SDK 애플리케이션 복구 로직

네트워크 복구 시 DDS 외에도 MQTT/SDK 레이어가 반드시 재연결 가능해야 합니다.

- MQTT
  - reconnect + exponential backoff
  - reconnect 후 topic 재구독
- 로봇 SDK
  - heartbeat timeout 감지 후 재연결
  - 소켓 예외 발생 시 재시도 루프
- 사용자 메시지/로그
  - 예: "네트워크 재연결 중입니다. 자동 복구를 시도합니다."

## 9) rosbag -> 실센서 전환 시 체크

- 토픽 이름/메시지 타입을 bag 단계와 실센서 단계에서 동일하게 유지
- MQTT/SDK 브리지 노드는 분리 프로세스로 운영
- 핵심 노드별 watchdog(무수신 N초 시 restart) 적용

## 10) 검증 시나리오

1. 정상 부팅 후 ROS topic 확인
   - `ros2 topic list`
2. MQTT pub/sub 확인
3. Wi-Fi 10~30초 강제 단절
4. 재연결 후 30~60초 내 자동복구 확인
   - ROS topic 재수신
   - MQTT 재연결
   - SDK 명령 왕복
5. `ddsi_udp_conn_write ... retcode -1` 로그가 일시적으로 보여도 자동복구 완료 시 운영 허용

## 11) 운영 기준

- 목표는 "에러 로그 0회"가 아니라 "복구 성공률 100%"
- 연속 실패 횟수(예: 3회) 기준 프로세스 자동 재시작
- 장애 시점 메트릭(복구시간/재시작 횟수) 수집

---

이 문서는 운영 안정성을 우선하는 기본 설정서이며, 장비/NIC/AP 구조에 맞춰 인터페이스명, 도메인 ID, 서비스명은 환경에 맞게 조정해 사용합니다.
