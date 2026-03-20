# IndoorLocalizationFactor 정리

이 문서는 `include/indoor_localization_factor.hpp`의 **사용 방법**과 **수식**을 정리한다.

---

## 1) 목적과 역할

- `IndoorLocalizationFactor`는 indoor localization / ICP 결과를 **ENU 절대 포즈 6-DoF** 관측으로 그래프에 추가한다.
- 잔차는 `position(3) + rotation(3)` 총 **6차원**이다.
- 속도 관측은 사용하지 않는다 (`A`의 velocity 성분은 이 factor에서 직접 제약하지 않음).

---

## 2) 그래프 변수(키) 구성

`NMEAFactor`와 동일한 4개 변수 블록을 쓴다.

- `j1`: `P(0)` 타입 `gtsam::Rot3` (외부 회전, local->ENU 정렬 회전)
- `j2`: `E(0)` 타입 `gtsam::Vector3` (ENU 기준 anchor/ref 위치)
- `j3`: `A(k)` 타입 `gtsam::Vector6` (`[pos(3), vel(3)]`)
- `j4`: `R(k)` 타입 `gtsam::Rot3` (body/local 자세)

클래스 선언:

- `NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>`

---

## 3) 입력 데이터 포맷 (`values_[17]`)

`NMEAFactor`와 동일한 packing을 사용한다.

- `[0:2]` `Tex_imu_r` : IMU->receiver(or sensor) translation
- `[3:5]` `anc_local` : local anchor
- `[6:8]` `pos_meas` : ENU 절대 위치 측정
- `[9:11]` 미사용(이 factor에서는 velocity 관측 미사용)
- `[12:15]` quaternion `(w,x,y,z)` : ENU 절대 자세 측정
- `[16]` `relative_sqrt_info` : 스칼라 가중치

추가 인자:

- `Rex_imu_r` : IMU->receiver 회전
- `hat_omg_T` : 시그니처 호환용 인자(현재 미사용)
- `invalid_lidar` : 시그니처 호환용 플래그(현재 잔차 계산에는 미사용)

---

## 4) 예측 포즈 수식 (코드와 동일)

아래는 `evaluateError()`에서 계산하는 예측식이다.

1. local 위치

# 
\mathbf{p}_{local}

\mathbf{R}\mathbf{T}*{ex}
\mathbf{p}*{body}
\left(-\mathbf{a}_{local}\right)


코드:

- `local_pos = rot * Tex_imu_r + pos_vel.head<3>() - anc_local`

1. ENU 위치

# 
\mathbf{p}_{enu}

\mathbf{R}*{enulocal}\mathbf{p}*{local}
\mathbf{p}_{ref}


코드:

- `P_enu = R_enu_local * local_pos + ref_enu`

1. ENU 자세

# 
\mathbf{R}_{enu}

\mathbf{R}*{enulocal}\mathbf{R}\mathbf{R}*{ex}


코드:

- `R_enu = R_enu_local * rot * Rex_imu_r`

---

## 5) residual 수식 (6D)

### 위치 residual (3D)

# 
\mathbf{r}_p

\left(\mathbf{p}*{enu}-\mathbf{p}*{meas}\right)w


### 회전 residual (3D)

# 
\mathbf{r}_R

\log\left(\mathbf{R}*{meas}^{\top}\mathbf{R}*{enu}\right)w


최종:

# 
\mathbf{r}

\begin{bmatrix}
\mathbf{r}_p 
\mathbf{r}_R
\end{bmatrix}
\in \mathbb{R}^{6}


여기서 w = \texttt{relativesqrtinfo}.

---

## 6) Jacobian 구조

- `H1` (`rot_ext`): 위치와 회전 모두 영향 (`6x3`)
- `H2` (`pos_ext`): 위치 항만 영향 (`6x3`, 아래 3x3만 non-zero)
- `H3` (`pos_vel`): 위치의 `pos` 성분만 영향 (`6x6`, velocity 열 0)
- `H4` (`rot`): 회전 항 중심 영향 (`6x3`)

즉, 이 factor는 `A(k)`의 velocity(뒤 3차원)를 직접 관측하지 않는다.

---

## 7) 노이즈 모델 설정

이 factor는 6차원 잔차이므로 노이즈도 6차원으로 설정해야 한다.

예시(개념):

- `diag([sigma_px^2, sigma_py^2, sigma_pz^2, sigma_rx^2, sigma_ry^2, sigma_rz^2])`

실무 팁:

- ICP 기반이면 보통 위치/회전 축별 신뢰도가 다르므로 축별 분산을 다르게 둔다.
- `relative_sqrt_info`(스칼라)와 noise model(6D)을 동시에 쓰고 있으므로 과가중되지 않게 튜닝한다.

---

## 8) 그래프 추가 예시

```cpp
double values[17] = {0};
values[0] = Tex_imu_r.x();
values[1] = Tex_imu_r.y();
values[2] = Tex_imu_r.z();
values[3] = anc_local.x();
values[4] = anc_local.y();
values[5] = anc_local.z();
values[6] = pos_meas_enu.x();
values[7] = pos_meas_enu.y();
values[8] = pos_meas_enu.z();
values[12] = q_meas_enu.w();
values[13] = q_meas_enu.x();
values[14] = q_meas_enu.y();
values[15] = q_meas_enu.z();
values[16] = relative_sqrt_info;

auto indoor_noise = gtsam::noiseModel::Diagonal::Variances(
    (gtsam::Vector(6) << sx2, sy2, sz2, srx2, sry2, srz2).finished());

graph.add(ligo::IndoorLocalizationFactor(
    P(0), E(0), A(frame_num), R(frame_num),
    false, values, hat_omg_T, Rex_imu_r, indoor_noise));
```

---

## 9) 기존 NMEAFactor와 차이

- 공통: ENU 변환 체인과 키 배치(`P,E,A,R`)가 동일
- 차이:
  - `NMEAFactor`: 위치/속도/자세(9D) 또는 position_only(3D)
  - `IndoorLocalizationFactor`: 위치/자세만(6D)

---

## 10) 현재 코드에서 실제 연결 위치

### 10.1 Placeholder 갱신 위치 (현재 활성)

- 파일: `src/laserMapping.cpp`
- 위치: `sync_packages(...)` 이후
- 호출:
  - `ligo::indoor::updateIndoorLocalizationPlaceholder(state_out.pos, state_out.rot, time_current);`

의미:

- 아직 실제 localization 모듈이 없어서, 임시로 현재 추정 `state_out`을 indoor 입력 슬롯(ENU pose)으로 저장한다.

### 10.2 실제 factor add 위치 (현재는 스텁, 주석 처리)

- 파일: `src/Indoor_Processing.cpp`
- 함수: `ligo::indoor::addIndoorFactorToGraphStubCommented()`
- 상태: 함수 본문은 no-op이고, 내부에 **주석된 샘플 코드**로 `IndoorLocalizationFactor` 삽입 예시를 제공

또한 호출부도 주석으로 준비:

- 파일: `src/laserMapping.cpp`
- `// ligo::indoor::addIndoorFactorToGraphStubCommented();`

---

## 11) localization 모듈이 생기면 수정할 곳

1) **입력 소스 교체**
- 파일: `src/Indoor_Processing.cpp`
- 함수: `updateIndoorLocalizationPlaceholder(...)`
- 현재 `state_out` 기반 갱신 대신, 실제 localization 결과(ENU pose + timestamp)로 `indoor_pos_enu_meas`, `indoor_rot_enu_meas`, `indoor_pose_time`, `indoor_pose_valid`를 갱신

2) **factor 추가 활성화**
- 파일: `src/Indoor_Processing.cpp`
- 함수: `addIndoorFactorToGraphStubCommented()`
- 주석 해제 후, 실제 구현으로 전환:
  - `indoor_flag && indoor_pose_valid` 가드
  - `values[17]`에 ENU pose 입력
  - `indoorPoseNoise / indoorPoseNoiseInit` 선택
  - `gtSAMgraph.add(ligo::IndoorLocalizationFactor(...))`

3) **호출 연결**
- 파일: `src/laserMapping.cpp`
- 주석된 호출 `addIndoorFactorToGraphStubCommented()`를 실제 호출로 변경
- 권장 시점: `sync_packages(...)`로 입력이 준비되고, graph update 전에 일관된 프레임 인덱스가 보장되는 구간

4) **파라미터 조정**
- 파일: `config/avia.yaml`
- `indoor.indoor_flag` 및 `indoor.*noise`, `indoor.outlier_*` 튜닝

