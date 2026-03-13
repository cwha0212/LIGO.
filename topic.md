# LIGO에서 사용하는 gnss_comm 토픽 정리

LIGO는 GNSS-LIO 통합 시 **gnss_comm** 패키지의 메시지 타입을 사용합니다.  
토픽 이름은 설정 파일(`config/*.yaml`)의 `gnss` 섹션에서 지정하며, 기본값은 U-Blox 드라이버 기준입니다.

---

## 1. GnssEphemMsg — 위성 궤력 (GPS / Galileo / BeiDou)


| 항목          | 내용                                         |
| ----------- | ------------------------------------------ |
| **메시지 타입**  | `gnss_comm::msg::GnssEphemMsg`             |
| **파라미터 이름** | `gnss.gnss_ephem_topic`                    |
| **기본 토픽**   | `/ublox_driver/ephem`                      |
| **구독 위치**   | `laserMapping.cpp` → `gnss_ephem_callback` |


### 처리 흐름

- 콜백에서 `msg2ephem(*ephem_msg)`로 ROS 메시지를 gnss_comm 내부 `Ephem` 구조로 변환.
- `p_gnss->p_assign->inputEphem(ephem)`으로 궤력 버퍼에 저장.

### LIGO에서 실제로 사용하는 데이터

- **궤력 전부**: `eph2pos()`, `eph2vel()`, `eph2svdt()` 등에 사용.
- 위성 위치/속도/발산시간 계산에 필요하므로, 메시지에 담긴 궤력 파라미터는 **전체**가 사용됨.
- **Ephem (GPS/Galileo/BeiDou)에서 추가로 참조하는 필드**:
  - `eph->tgd[0]`: 그룹 지연 보정.
  - `eph->ura`: URA 기반 측정 불확실성 가중치 계산 (PsrDopp factor의 `pr_uura`, `dp_uura`).

### 용도

- 위성 궤도/시계 계산 → Pseudo-range / Doppler / Carrier-phase 팩터의 위성 위치·속도·시계 보정.

---

## 2. GnssGloEphemMsg — GLONASS 위성 궤력


| 항목          | 내용                                             |
| ----------- | ---------------------------------------------- |
| **메시지 타입**  | `gnss_comm::msg::GnssGloEphemMsg`              |
| **파라미터 이름** | `gnss.gnss_glo_ephem_topic`                    |
| **기본 토픽**   | `/ublox_driver/glo_ephem`                      |
| **구독 위치**   | `laserMapping.cpp` → `gnss_glo_ephem_callback` |


### 처리 흐름

- `msg2glo_ephem(*glo_ephem_msg)`로 `GloEphem`으로 변환.
- `inputEphem(glo_ephem)`으로 동일 궤력 버퍼에 저장 (GLONASS용).

### LIGO에서 실제로 사용하는 데이터

- **궤력 전부**: `geph2pos()`, `geph2vel()`, `geph2svdt()`에 사용.
- GLONASS는 `tgd` 미사용 (코드에서 0.0 설정).

### 용도

- GLONASS 위성에 대한 위성 위치/속도/시계 보정.

---

## 3. StampedFloat64Array — 전리층 파라미터


| 항목          | 내용                                               |
| ----------- | ------------------------------------------------ |
| **메시지 타입**  | `gnss_comm::msg::StampedFloat64Array`            |
| **파라미터 이름** | `gnss.gnss_iono_params_topic`                    |
| **기본 토픽**   | `/ublox_driver/iono_params`                      |
| **구독 위치**   | `laserMapping.cpp` → `gnss_iono_params_callback` |


### 사용하는 상세 데이터


| 필드             | 타입          | 용도                               |
| -------------- | ----------- | -------------------------------- |
| `header.stamp` | time        | 파라미터 유효 시각 `ts`                  |
| `data`         | `float64[]` | **길이 8**의 전리층 파라미터 (Klobuchar 등) |


- `data` 8개 값 전체를 `iono_params`로 복사한 뒤 `p_gnss->inputIonoParams(ts, iono_params)`로 전달.
- `GNSS_Processing_fg.cpp`에서 `latest_gnss_iono_params`로 보관하고, PsrDopp 팩터에서 **values[15]~values[22]**로 넘겨 전리층 지연 `calculate_ion_delay()`에 사용.

### 용도

- 의사거리/도플러/반송파 위상 보정 시 전리층 지연 보정.

---

## 4. GnssPVTSolnMsg — RTK/수신기 PVT 해


| 항목          | 내용                                                                                                |
| ----------- | ------------------------------------------------------------------------------------------------- |
| **메시지 타입**  | `gnss_comm::msg::GnssPVTSolnMsg`                                                                  |
| **파라미터 이름** | `gnss.rtk_pvt_topic`                                                                              |
| **기본 토픽**   | `/ublox_driver/receiver_pvt`                                                                      |
| **구독 위치**   | `laserMapping.cpp` → `rtk_pvt_callback` (GNSS 사용 시) 또는 `rtk_pvt_callback` (GNSS 비활성 + NMEA만 사용 시) |


### 사용하는 상세 데이터


| 필드          | 타입      | 용도                                                   |
| ----------- | ------- | ---------------------------------------------------- |
| `time.week` | uint16  | GNSS 주(week) → `gst2time(week, tow)`로 시각 변환          |
| `time.tow`  | float64 | Time of week [s]                                     |
| `latitude`  | float64 | 위도 [rad] → LLA→ECEF→ENU 변환 후 prior/constraint        |
| `longitude` | float64 | 경도 [rad]                                             |
| `altitude`  | float64 | 고도 [m]                                               |
| `carr_soln` | int     | Carrier solution 상태 → `float_holder`에 저장 (초기화/품질 판단) |
| `diff_soln` | int     | Differential solution 상태 → `diff_holder`에 저장         |


- 시각: `ts = time2sec(gst2time(groundt_pvt->time.week, groundt_pvt->time.tow))`.
- `p_gnss->inputpvt(ts, latitude, longitude, altitude, carr_soln, diff_soln)` 호출로 `pvt_time`, `pvt_holder`(ENU), `diff_holder`, `float_holder` 갱신.

### 용도

- GNSS prior/제약, 초기화, 및 (옵션) ground truth로 사용.

---

## 5. GnssMeasMsg — GNSS 원시 측정치 (관측값)


| 항목          | 내용                                        |
| ----------- | ----------------------------------------- |
| **메시지 타입**  | `gnss_comm::msg::GnssMeasMsg`             |
| **파라미터 이름** | `gnss.gnss_meas_topic`                    |
| **기본 토픽**   | `/ublox_driver/range_meas`                |
| **구독 위치**   | `laserMapping.cpp` → `gnss_meas_callback` |


### 처리 흐름

- `msg2meas(*meas_msg)`로 `std::vector<ObsPtr>`(gnss_comm `Obs` 구조)로 변환.
- 첫 관측 시각 `time2sec(gnss_meas[0]->time)`을 `latest_gnss_time`에 저장.
- `time_diff_valid == true`일 때만 `gnss_meas_buf`에 넣고, 이후 `sync_packages()`에서 LiDAR/IMU와 동기화되어 `processGNSS()`로 전달.

### LIGO에서 사용하는 Obs 필드 (변환 후)


| Obs 필드                  | 용도                                      |
| ----------------------- | --------------------------------------- |
| `time`                  | 관측 시각; 동기화·프레임 구간·팩터 시각                 |
| `sat`                   | 위성 ID (satsys, L1_freq, 궤력 매칭)          |
| `freqs`                 | 반송파 주파수; L1 선택 및 파장 계산                  |
| `psr`                   | 의사거리; ToF·위성 위치 계산·Hatch 필터·PsrDopp 측정값 |
| `psr_std`               | 의사거리 표준편차; 품질 게이팅·pr_uura 계산            |
| `cp`                    | 반송파 위상 [cycle]; CP 팩터·Hatch 필터          |
| `cp_std`                | 반송파 위상 표준편차; CP 품질 게이팅                  |
| `dopp`                  | 도플러 [Hz]; PsrDopp 팩터 측정값                |
| `dopp_std`              | 도플러 표준편차; dp_uura 등                     |
| `CN0`                   | 신호 강도; (필요 시 품질/가중치)                    |
| `LLI`, `code`, `status` | 주로 품질/슬립 검출 등 (Assignment/필터에서 사용)      |


- 위 필드들은 `GNSS_Assignment::processGNSSBase`, `GNSSProcess::processGNSS`, `GnssPsrDoppMeas`, `SvPosCals` 등에서 사용됨.

### 용도

- Pseudo-range / Doppler / Carrier-phase 팩터 구성 및 GNSS-LIO 그래프 최적화.

---

## 6. GnssTimePulseInfoMsg — GNSS 시간 펄스( PPS ) 정보


| 항목          | 내용                                                                                |
| ----------- | --------------------------------------------------------------------------------- |
| **메시지 타입**  | `gnss_comm::msg::GnssTimePulseInfoMsg`                                            |
| **파라미터 이름** | `gnss.gnss_tp_info_topic`                                                         |
| **기본 토픽**   | `/ublox_driver/time_pulse_info`                                                   |
| **구독 위치**   | `laserMapping.cpp` → `gnss_tp_info_callback` (gnss_local_online_sync == true 일 때) |


### 사용하는 상세 데이터


| 필드          | 타입      | 용도                                            |
| ----------- | ------- | --------------------------------------------- |
| `time.week` | uint16  | 펄스의 GNSS 주                                    |
| `time.tow`  | float64 | Time of week [s]                              |
| `time_sys`  | int     | 시간 체계: SYS_GLO / SYS_GAL / SYS_BDS / SYS_NONE |
| `utc_based` | bool    | UTC 기준이면 GPST로 변환 시 `utc2gpst` 사용             |


- `time_sys`에 따라 `gpst2time`, `utc2gpst`, `gst2time`, `bdt2time` 중 하나로 `gtime_t`로 변환 후 `gnss_ts = time2sec(tp_time)`.
- `next_pulse_time = gnss_ts`, `next_pulse_time_valid = true`로 설정.

### 용도

- **GNSS–로컬 시계 동기화**: `local_trigger_info_callback`에서 로컬 트리거 타임스탬프와 `next_pulse_time`의 차이로 `time_diff_gnss_local`을 구하고 `p_gnss->inputGNSSTimeDiff(time_diff_gnss_local)` 호출.
- `gnss_local_online_sync == false`이면 이 토픽을 구독하지 않고, `gnss_local_time_diff` 상수값을 사용.

---

## 7. NavSatFix (rtk_lla_topic) — LLA 형태 위치


| 항목          | 내용                                                     |
| ----------- | ------------------------------------------------------ |
| **메시지 타입**  | `sensor_msgs::msg::NavSatFix` (표준 ROS 2, gnss_comm 아님) |
| **파라미터 이름** | `gnss.rtk_lla_topic`                                   |
| **기본 토픽**   | `/ublox_driver/receiver_lla`                           |
| **구독 위치**   | `laserMapping.cpp` → `rtk_lla_callback`                |


### 사용하는 상세 데이터


| 필드             | 타입      | 용도       |
| -------------- | ------- | -------- |
| `header.stamp` | time    | 시각 `ts`  |
| `latitude`     | float64 | 위도 [deg] |
| `longitude`    | float64 | 경도 [deg] |
| `altitude`     | float64 | 고도 [m]   |


- `p_gnss->inputlla(ts, latitude, longitude, altitude)`로 `lla_time`, `lla_holder`(ENU) 갱신.

### 용도

- LLA 형태의 보조 위치 소스 (PVT와 별도 채널).

---

## 토픽 구독 조건 요약


| 토픽                         | GNSS 사용 시  | 비고                                                                  |
| -------------------------- | ---------- | ------------------------------------------------------------------- |
| GnssEphemMsg               | ✓          | 항상                                                                  |
| GnssGloEphemMsg            | ✓          | 항상                                                                  |
| GnssMeasMsg                | ✓          | obs_from_rinex 아니고, UrbanNav 메시지 안 쓸 때                              |
| StampedFloat64Array (iono) | ✓          | 항상                                                                  |
| GnssTimePulseInfoMsg       | ✓ (옵션)     | gnss_local_online_sync == true 일 때                                  |
| GnssPVTSolnMsg             | ✓ 또는 NMEA만 | pvt_is_gt == true 이면 RTK 토픽 구독, 아니면 GT 파일 사용. GNSS 끄면 NMEA만 쓸 때도 구독 |
| NavSatFix (rtk_lla)        | ✓          | GnssMeasMsg 쓸 때 함께 구독                                               |
| LocalSensorExternalTrigger | ✓ (옵션)     | gnss_local_online_sync == true 일 때 (PPS 동기화)                        |


---

## 참고

- **gnss_comm** 메시지 정의 및 `msg2ephem` / `msg2glo_ephem` / `msg2meas` 변환 세부사항은 [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) 저장소를 참조.
- LIGO는 `LIGO_WITHOUT_GNSS`가 정의되면 GNSS 관련 구독을 하지 않고, `LIGO_WITH_URBANNAV_MSG`일 때는 RINEX/UrbanNav 메시지 경로를 사용할 수 있음.

