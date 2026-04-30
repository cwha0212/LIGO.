# ligo 노드가 발행하는 ROS 2 인터페이스

`ligo_mapping` 실행 파일(`src/laserMapping.cpp` 중심) 및 초기화 보조(`src/li_initialization.cpp`)에서 **advertise**되는 토픽과, **실제로 메시지가 나가는 조건**을 정리했다.  
(TF는 `tf2_ros::TransformBroadcaster`로 별도 스트림이므로 마지막 절에 둔다.)

---

## 1. 요약 표

| 토픽 (기본 이름) | 메시지 타입 | 비고 |
|------------------|-------------|------|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 항상 발행(odom 루프에서) |
| `/path` | `nav_msgs/Path` | `publish.path_en`이 true일 때 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `publish.scan_publish_en` 또는 매핑 모드 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 스캔+바디 옵션 켜짐 시 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 초기 맵 포인트(초기화 시) |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | 퍼블리셔만 생성, **현재 코드에 `publish` 호출 없음** |
| `/nmea_aligned_to_init` | `nav_msgs/Odometry` | `NMEA_ENABLE` |
| `/nmea_aligned_path` | `nav_msgs/Path` | `NMEA_ENABLE` |
| `/ligo/nmea_lio_error_xy` | `std_msgs/Float64` | `NMEA_ENABLE`, ICP 후 융합 시 RMSE |
| `/ligo/nmea_03m_diag` | `std_msgs/Float64MultiArray` | `NMEA_ENABLE`, 0.3m 진단 1회 |
| `/ligo/nmea_heading_align_status` | `ligo/NmeaHeadingAlignStatus` | `NMEA_ENABLE` |
| `/ligo/enu_position` | `geometry_msgs/PointStamped` | `NMEA_ENABLE`, 파라미터로 토픽명 변경 가능 |
| `/ligo/global_position` | `sensor_msgs/NavSatFix` | 동일 |
| `/ligo/ecef_position` | `geometry_msgs/PointStamped` | 동일 |
| `/ligo/nmea_graph_anchor_marker` | `visualization_msgs/Marker` | `NMEA_ENABLE`, 그래프 앵커 시각화 |
| `/ligo/nmea_stamp_diag` | `std_msgs/Float64MultiArray` | `NMEA_ENABLE` 시(입력 NavSatFix 브리지 경로) |
| `/icp_pairs_marker` | `visualization_msgs/Marker` | `NMEA_ENABLE`, 여러 namespace |
| `/init_pairs_from_gps_move_marker` | `visualization_msgs/Marker` | 초기화 시각화 |
| `/planner_normal` | `visualization_msgs/Marker` | 퍼블리셔만 생성, **현재 `publish` 호출 없음** |
| `/indoor/map_cloud` | `sensor_msgs/PointCloud2` | latched QoS |
| `/indoor/map_2d` | `nav_msgs/OccupancyGrid` | 동일, PGM 그리드 있을 때 |
| `/indoor/aligned_scan` | `sensor_msgs/PointCloud2` | 실내 GICP 시각화 시 |

---

## 2. SLAM / 궤적 (항상·옵션)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 추정 포즈. `header.frame_id`는 NMEA ICP 준비 시 `map`, 아니면 `camera_init`. `child_frame_id`: `aft_mapped`. |
| `/path` | `nav_msgs/Path` | LIO 궤적. `publish.path_en`(YAML `publish.path_en`)이 켜져 있을 때만 누적 발행. |

---

## 3. 포인트 클라우드

| 토픽 | 타입 | 조건 |
|------|------|------|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 월드(또는 ENU) 정합 스캔. `publish.scan_publish_en` 또는 매핑 모드. |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | 바디 프레임 스캔. `publish.scan_bodyframe_pub_en`. |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 초기 IVox 맵 포인트(`publish_init_map`). |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | 노드가 퍼블리셔를 만들지만 **발행 로직 없음**(예약/미사용). |

---

## 4. NMEA / GNSS 융합 (`NMEA_ENABLE` + `gnss_comm`)

다음은 `parameters`에서 `nmea.enable`(등)으로 켠 경우에만 의미가 있다.

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/nmea_aligned_to_init` | `nav_msgs/Odometry` | 수신 GNSS(ENU) 오도메트리. `icp_tf_ready` 전에는 `nmea_unaligned` 등으로 표시. |
| `/nmea_aligned_path` | `nav_msgs/Path` | ICP 준비 후 GNSS 경로 누적. |
| `/ligo/nmea_lio_error_xy` | `std_msgs/Float64` | ICP 이후 LIO–GNSS 2D 오차 RMSE. |
| `/ligo/nmea_03m_diag` | `std_msgs/Float64MultiArray` | 0.3m 이동 시점 지연/변위 진단(한 번). |
| `/ligo/nmea_heading_align_status` | `ligo/NmeaHeadingAlignStatus` | 초기 TIME-PAIR 정합·실내 reloc 등 **로컬↔ENU heading 상태** (NMEA 빌드 시). |

### 파라미터로 바꿀 수 있는 토픽 이름 (기본값)

| 파라미터 | 기본 토픽 | 타입 |
|----------|-----------|------|
| `ligo.enu_position_topic` | `/ligo/enu_position` | `geometry_msgs/PointStamped` (`ligo.enu_position_frame_id`, 기본 `enu`) |
| `ligo.global_position_topic` | `/ligo/global_position` | `sensor_msgs/NavSatFix` |
| `ligo.ecef_position_topic` | `/ligo/ecef_position` | `geometry_msgs/PointStamped` (`ligo.ecef_position_frame_id`, 기본 `ecef`) |

융합 ENU/ECEF/WGS84는 `icp_tf_ready` 및 내부 조건을 만족할 때만 채워져 발행된다.

---

## 5. 진단 / 초기화 보조

| 토픽 | 타입 | 조건 |
|------|------|------|
| `/ligo/nmea_stamp_diag` | `std_msgs/Float64MultiArray` | `NMEA_ENABLE`일 때(`li_initialization.cpp`). LiDAR·GNSS 스탬프 정합 진단. |

---

## 6. 시각화 (RViz `Marker`)

| 토픽 | 용도 |
|------|------|
| `/icp_pairs_marker` | ICP 쌍 연결선, LIO/GPS 포인트, GPS path 등 (namespace 여러 개). `NMEA_ENABLE` 및 데이터 있을 때. |
| `/init_pairs_from_gps_move_marker` | GPS 이동 기준 초기 쌍 시각화. |
| `/ligo/nmea_graph_anchor_marker` | GTSAM 그래프 앵커 `E(0)` (자홍 구). |
| `/planner_normal` | 퍼블리셔만 있고 **발행 없음**. |

---

## 7. 실내 Small GICP

맵 로드·실내 세션 시 지도/정합 스캔을 내보낸다. 맵 클라우드·2D 그리드는 **latched** QoS.

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/indoor/map_cloud` | `sensor_msgs/PointCloud2` | 참조 맵(ENU/맵 프레임 처리 후). 최초 1회 latched. |
| `/indoor/map_2d` | `nav_msgs/OccupancyGrid` | PCD와 함께 로드된 `*_grid2d.yaml`/PGM이 있을 때. |
| `/indoor/aligned_scan` | `sensor_msgs/PointCloud2` | GICP에 맞춘 라이브 스캔(시각화). |

---

## 8. TF (`tf2_ros`)

토픽이 아니라 **TF 트리**로 브로드캐스트된다.

| 부모 → 자식 | 조건 |
|-------------|------|
| `camera_init` → `aft_mapped` | 항상 (`publish_odometry` 내 `sendTransform`). |
| `map` → `camera_init` | `NMEA_ENABLE`이고 `p_nmea->icp_tf_ready`일 때 (로컬→ENU 정렬 후). |

---

## 9. 빌드 플래그 요약

| 플래그 | 영향 |
|--------|------|
| `NMEA_ENABLE` (런타임 파라미터) | 위 표에서 “`NMEA_ENABLE`”이 붙은 토픽 실제 사용 |

---

*문서 기준: 저장소 내 `laserMapping.cpp`, `li_initialization.cpp`, `Indoor_Processing.cpp`.*
