# IndoorLocalizationFactor 정리

이 문서는 `include/indoor_localization_factor.hpp`의 **사용 방법**과 **수식**을 정리한다.

---

## 1) 목적과 역할

- `IndoorLocalizationFactor`는 indoor localization / GICP 결과를 **system-ENU 절대 포즈 6-DoF** 관측으로 그래프에 추가한다.
- 잔차는 `position(3) + rotation(3)` 총 **6차원**이다.
- 속도 관측은 사용하지 않는다 (`A`의 velocity 성분은 이 factor에서 직접 제약하지 않음).
- 입력 포맷은 `NmeaLioGravRelFactor`, `NMEAFactor`와 통일된 **typed-인자** 방식을 사용한다.

---

## 2) 그래프 변수(키) 구성

`NMEAFactor`와 동일한 4개 변수 블록을 쓴다.

- `j1`: `P(0)` 타입 `gtsam::Rot3` (외부 회전, local → system-ENU 정렬 회전)
- `j2`: `E(0)` 타입 `gtsam::Vector3` (system-ENU 기준 anchor/ref 위치)
- `j3`: `A(k)` 타입 `gtsam::Vector6` (`[pos(3), vel(3)]`)
- `j4`: `R(k)` 타입 `gtsam::Rot3` (body/local 자세)

클래스 선언:

- `NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>`

---

## 3) 입력 데이터 포맷 (typed 인자, LIO factor와 통일)

```cpp
IndoorLocalizationFactor(
    gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
    const Eigen::Vector3d& Tex_imu_r,    // IMU → sensor translation
    const Eigen::Vector3d& anc_local,    // local-frame anchor
    const Eigen::Vector3d& pos_meas,     // system-ENU 절대 위치 측정
    const Eigen::Matrix3d& rot_meas,     // system-ENU 절대 자세 측정 (3x3 회전행렬)
    double relative_sqrt_info,           // 스칼라 가중치
    const Eigen::Matrix3d& Rex_imu_r,    // IMU → sensor 회전
    const gtsam::SharedNoiseModel& model);
```

- 회전 측정은 `NmeaLioGravRelFactor::rot_lio`와 동일하게 `Eigen::Matrix3d`로 직접 받는다.
- 호출 측에서 quaternion → matrix 변환을 수행한다(`q.normalized().toRotationMatrix()`).

---

## 4) 예측 포즈 수식 (코드와 동일)

`NMEAFactor`와 동일한 변환 체인을 사용한다.

1. local 위치

\[
\mathbf{p}_{local} = \mathbf{R}\,\mathbf{T}_{ex} + \mathbf{p}_{body} - \mathbf{a}_{local}
\]

코드: `local_pos = rot * Tex_imu_r + pos_vel.head<3>() - anc_local`

2. system-ENU 위치

\[
\mathbf{p}_{enu} = \mathbf{R}_{enu\_local}\,\mathbf{p}_{local} + \mathbf{p}_{ref}
\]

코드: `P_enu = R_enu_local * local_pos + ref_enu`

3. system-ENU 자세

\[
\mathbf{R}_{enu} = \mathbf{R}_{enu\_local}\,\mathbf{R}\,\mathbf{R}_{ex}
\]

코드: `R_enu = R_enu_local * rot * Rex_imu_r`

---

## 5) residual 수식 (6D)

### 위치 residual (3D)

\[
\mathbf{r}_p = (\mathbf{p}_{enu} - \mathbf{p}_{meas})\,w
\]

### 회전 residual (3D)

\[
\mathbf{r}_R = \log\left(\mathbf{R}_{meas}^{\top}\mathbf{R}_{enu}\right)\,w
\]

최종:

\[
\mathbf{r} = \begin{bmatrix}\mathbf{r}_p \\ \mathbf{r}_R\end{bmatrix} \in \mathbb{R}^{6}
\]

여기서 \(w = \texttt{relative\_sqrt\_info}\).

---

## 6) Jacobian 구조

- `H1` (`rot_ext`): 위치와 회전 모두 영향 (`6x3`)
- `H2` (`pos_ext`): 위치 항만 영향 (`6x3`, 아래 3x3만 non-zero)
- `H3` (`pos_vel`): 위치의 `pos` 성분만 영향 (`6x6`, velocity 열 0)
- `H4` (`rot`): 회전 항 중심 영향 (`6x3`)

이 factor는 `A(k)`의 velocity(뒤 3차원)를 직접 관측하지 않는다.

---

## 7) 노이즈 모델 설정

6차원 잔차이므로 노이즈도 6차원으로 설정한다.

예시:

```cpp
auto noise = gtsam::noiseModel::Diagonal::Variances(
    (gtsam::Vector(6) << sx2, sy2, sz2, srx2, sry2, srz2).finished());
```

`relative_sqrt_info`(스칼라)와 noise model(6D)을 동시에 적용하므로 과가중되지 않게 튜닝한다.

---

## 8) 그래프 추가 예시

```cpp
const Eigen::Matrix3d rot_meas = q_meas_enu.normalized().toRotationMatrix();
graph.add(ligo::IndoorLocalizationFactor(
    P(0), E(0), A(frame_num), R(frame_num),
    Tex_imu_r, anc_local,
    pos_meas_enu, rot_meas,
    relative_sqrt_info,
    Rex_imu_r, indoor_noise));
```

실제 코드 위치: `src/Indoor_Processing.cpp` 의 `addIndoorFactorToGraph(int frame_num)`.

---

## 9) NMEAFactor / NmeaLioGravRelFactor와의 관계

세 팩터 모두 **system-ENU** 기준에서 동일한 변환 체인을 사용한다.

| Factor | 입력 회전 | 잔차 차원 | 회전 측정 |
|---|---|---|---|
| `NmeaLioGravRelFactor` | `Eigen::Matrix3d rot_lio` | 27 | 사용 (`rot_lio` 항) |
| `NMEAFactor` (navsatfix) | `Eigen::Matrix3d rot_meas` (입력은 받지만 미사용) | 3 (position-only) | **미사용** (NavSatFix → twist/orientation 없음) |
| `NMEAFactor` (odometry) | `Eigen::Matrix3d rot_meas` | 9 | 사용 |
| `IndoorLocalizationFactor` | `Eigen::Matrix3d rot_meas` | 6 | 사용 |

> 정리: 세 팩터 모두 회전 입력은 `Eigen::Matrix3d`로 통일. 위치는 `Eigen::Vector3d`. NMEA(navsatfix)에서만 회전·속도 슬롯이 의미적으로 무효(브리지가 0을 채움)이므로 `position_only=true`로 잔차에서 제외한다.

---

## 10) 속도 처리

세 팩터 모두 속도 입력을 받지만,

- `NmeaLioGravRelFactor` → LIO 추정 속도(`vel_lio`)를 그래프 속도 변수와 비교(상태 천이 제약)
- `NMEAFactor` (odometry) → NMEA 메시지의 `twist.linear`를 ENU 속도와 비교
- `NMEAFactor` (navsatfix) → 입력 자체가 0(브리지가 채우지 못함). `position_only=true`에서 잔차로 들어가지 않음
- `IndoorLocalizationFactor` → 사용하지 않음(인터페이스에도 없음)

현재 외부 절대 속도 측정이 없으므로 navsatfix 경로와 indoor 경로에서는 속도 잔차는 사용하지 않는다.
