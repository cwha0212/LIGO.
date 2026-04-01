# LIGO 파라미터 레퍼런스 (ROS 2)

설정은 표준 ROS 2 방식으로 로드한다. 패키지 프리셋 YAML은 루트에 `/**:` 및 `ros__parameters:` 네임스페이스가 있고, 일부 레거시 프리셋(`robosense.yaml` 등)은 최상위 키만 있는 형태일 수 있다. 노드는 **평탄한 점(`.`) 구분 이름**으로 파라미터를 읽는다 (예: `common.lid_topic`).

- **고급**: 대부분의 사용자가 바꿀 필요 없는 항목.
- **제거됨**: 이전 버전의 `gnss.gt_file_name`, `gnss.gt_file_type`는 코드에서 읽지 않으며 제거되었다. 외부 평가(궤적 vs GT)는 별도 스크립트로 처리한다.
- **제거됨 (미호출 함수)**: `compute_system_output_pose_enu` / `compute_system_output_pose_ecef`(`parameters`), `NMEAProcess::local2enu`, `TrajAlign`, 선언만 있던 `updateNMEAstatistics` — 어디에서도 호출되지 않아 삭제했다.

## 프리셋 파일 (`config/`)

| 파일 | 용도 |
|------|------|
| `avia.yaml` | Livox Avia / Mid360 등 기본, ROS 2 네임스페이스 포함 |
| `avia_septentrio.yaml` | Septentrio 등 GNSS 설정 예시 |
| `avia_deg.yaml` | Avia + 각도/중력 실험용 예시 |
| `hesai.yaml` | Hesai Pandar 등 (`lidar_type: 4`) |
| `velody.yaml` | Velodyne + 표준 PointCloud2 |
| `robosense.yaml` | Robosense + PointCloud2 |

실행 시 위 YAML 중 하나를 `ligo_mapping`에 넘기면 된다 ([README](../README.md) 실행 예시 참고).

## `common.*`

| 키 | 설명 | 고급 |
|----|------|------|
| `lid_topic` | LiDAR 토픽 | |
| `imu_topic` | IMU 토픽 | |
| `con_frame` | 여러 LiDAR 스캔 결합 여부 | |
| `con_frame_num` | 결합할 프레임 수 | |
| `scan_rate` | 초당 스캔 수(설정 참고) | |

## `preprocess.*`

| 키 | 설명 | 고급 |
|----|------|------|
| `lidar_type` | 1=Livox CustomMsg, 2=PointCloud2 등 | |
| `scan_line` | 스캔 라인 수 | |
| `timestamp_unit` | PointCloud2 시간 필드 단위 (0~3) | |
| `blind` | 근거리 블라인드 (m) | |
| `det_range` | 전처리 거리 상한 (m) | |

## `mapping.*`

| 키 | 설명 | 고급 |
|----|------|------|
| `mapping_mode` | 순수 매핑 시 실내 전환 경로 비활성화 | |
| `det_range` | 매핑 단계 LiDAR 유효 거리 | |
| `imu_en` | IMU 사용 | |
| `imu_time_inte`, `lidar_time_inte` | 적분/스캔 주기 관련 | ✓ |
| `satu_acc`, `satu_gyro` | IMU 포화 임계 | ✓ |
| `acc_norm` | 가속도 단위 (g vs m/s²) | |
| `lidar_meas_cov` | LiDAR 관측 노이즈 | ✓ |
| `acc_cov_output`, `gyr_cov_output`, `b_*_cov` | 출력/바이어스 공분산 | ✓ |
| `imu_meas_*_cov` | IMU 측정 노이즈 | ✓ |
| `plane_thr`, `match_s` | 평면 판별·매칭 판정 | ✓ |
| `ivox_grid_resolution` | IVox 격자 해상도 (m) | ✓ |
| `gravity`, `gravity_init`, `init_with_imu` | 중력·초기화 | |
| `extrinsic_T`, `extrinsic_R` | LiDAR–IMU 외부표정 | |
| `log_lidar_frame_time_ms` | 프레임 시각 로그 | ✓ |
| `dyn_filter`, `dyn_filter_resolution` | 동적 필터 | ✓ |

## `gnss.*` (NMEA/GTSAM 그래프·EKF 관련)

| 키 | 설명 | 고급 |
|----|------|------|
| `outlier_rejection` | 강건 M-estimator 사용 | |
| `gtsam_variable_thres`, `gtsam_marg_variable_thres` | 윈도/마진 변수 수 | ✓ |
| `gnss_sample_period` | GNSS 샘플 주기 (s) | |
| `outlier_thres`, `outlier_thres_init` | 아웃라이어 임계 | ✓ |
| `window_size` | 초기화에 쓰는 GNSS 윈도 크기 | ✓ |
| `prior_noise`, `marg_noise`, `odo_noise`, `grav_noise` | 각종 노이즈 | ✓ |
| `b_acc_noise`, `b_omg_noise`, `acc_noise`, `omg_noise` | 바이어스/관측 노이즈 | ✓ |
| `gnss_ekf_noise` | EKF 측정 노이즈 스케일 ([Estimator.cpp](../src/Estimator.cpp)) | ✓ |
| `gnss_extrinsic_T`, `gnss_extrinsic_R` | GNSS–IMU 외부표정 | |
| `nolidar` | LiDAR 없이 GNSS만 (특수 모드) | ✓ |

## `nmea.*` (`LIGO_WITH_NMEA` 빌드 시)

| 키 | 설명 |
|----|------|
| `force_indoor_on_high_cov` | NMEA 공분산이 `indoor_high_cov_threshold` 이상이면 실내 reloc 트리거 허용 (기본 `true`) |
| `indoor_high_cov_threshold` | 위 판정에 쓰는 공분산 임계 (대각 `pose.covariance` 스케일, 기본 `50`) |
| 기타 | `nmea_enable`, `posit_odo_topic`, `ppp_std_thres`, 앵커/ICP 초기화 등 — `config/avia.yaml` 및 `readParameters()` 참고 |

- **제거됨**: 미사용 프로파일 버퍼 `s_plot` / `s_plot3` / `MAXN` (런타임에 읽지 않음).

## `ligo.*`

토픽 이름·`frame_id` (`enu_position_topic`, `ecef_position_topic` 등).

## `odometry.*`, `publish.*`, `pcd_save.*`

다운샘플 없이 odometry 발행, path/scan 발행, PCD 저장 간격 등.

## `indoor.*`

실내 플래그, 그리드 맵 경로, GICP 임계·복셀·반복 횟수, GTSAM 실내 포즈 노이즈 등. `indoor.grid_map_dir`은 상대 경로일 때 패키지 소스 또는 `share/ligo`로 해석된다. 실내 scan-to-map GICP는 **`ligo_mapping` + `LIGO_WITH_SMALL_GICP` 빌드**에서 C++로 수행한다 (별도 Python 노드 없음).

## 루트 파라미터 (YAML에 평탄하게 둘 수 있음)

`prop_at_freq_of_imu`, `check_satu`, `init_map_size`, `space_down_sample`, `point_filter_num`, `filter_size_surf`, `filter_size_map`, `ivox_nearby_type`, `runtime_pos_log_enable` 등 — `readParameters()`에서 직접 읽는 항목.
