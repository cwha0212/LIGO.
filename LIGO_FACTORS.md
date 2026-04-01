# LIGO Factor 전체 정리 (설명 + 링크 + 수식)

이 문서는 LIGO 코드에서 실제로 최적화에 쓰이는 factor를 한 번에 확인할 수 있도록 정리한 레퍼런스다.

- GTSAM 기준: `gtSAMgraph.add(...)`로 활성 호출되는 factor
- Ceres 기준: `problem_->AddResidualBlock(...)`로 실제 residual block이 추가되는 factor
- 단순 정의/include만 있고 호출이 없는 항목은 "비활성/미사용 후보"로 별도 정리

---

## 문서/코드 링크 모음

### 프로젝트 내부 링크

- GNSS factor 폴더: `[include/gnss_factor](include/gnss_factor)`
- NMEA factor 폴더: `[include/nmea_factor](include/nmea_factor)`
- Ceres LiDAR factor: `[include/Curvefitter/lidar_feature_factor.h](include/Curvefitter/lidar_feature_factor.h)`
- NMEA/PPP factor 추가 로직: `[src/NMEA_Processing_fg.cpp](src/NMEA_Processing_fg.cpp)` (구 `GNSS_Processing_fg.cpp` raw 관측 경로는 제거됨)
- 슬라이딩 윈도우 prior 추가: `[src/NMEA_Assignment.cpp](src/NMEA_Assignment.cpp)`

### 외부 문서 링크

- GTSAM 공식 문서: [https://gtsam.org](https://gtsam.org)
- GTSAM `NoiseModelFactor` 개요: [https://borglab.github.io/gtsam/nonlinearfactor/](https://borglab.github.io/gtsam/nonlinearfactor/)
- Ceres Solver 공식 문서: [https://ceres-solver.readthedocs.io](https://ceres-solver.readthedocs.io)

---

## 1) 상태 변수 키/기호 정의

### 1.1 GTSAM 키

`src/NMEA_Assignment.h` 기준:

- `R`: `gtsam::Rot3` (body/local 자세)
- `P`: `gtsam::Rot3` (외부 파라미터 회전)
- `E`: `gtsam::Vector3` (외부 파라미터 위치/anchor)
- `A`: `gtsam::Vector6` (pos, vel)
- `F`: `gtsam::Vector12` (pos, vel, bias)
- `O`: `gtsam::Vector12` (omg, acc, bg, ba)
- `G`: `gtsam::Vector3` (중력 벡터)
- `B`: `gtsam::Vector4` (시스템별 clock bias)
- `C`: `gtsam::Vector1` (clock drift rate)

### 1.2 공통 표기

-  \mathbf{p} : 수신기 위치
-  \mathbf{v} : 수신기 속도
-  \mathbf{R} : 회전 행렬
-  dt : clock bias
-  \dot{dt} : clock drift rate
-  \rho : pseudorange
-  \phi : carrier phase 관련 거리량(코드에서는 차분식)
-  \Delta t : 두 epoch 사이 시간 간격

---

## 2) 레거시 GNSS raw 관측 factor (`include/gnss_factor`, 미빌드)

아래 표는 원 논문/레포의 pseudo-range·carrier-phase 계열 factor 정리다. **현재 패키지는 해당 소스 경로를 빌드에 포함하지 않으며**, NMEA/PPP 경로만 활성이다.

## 2.1 GNSS 관측 factor


| Factor                     | 코드 링크                                                                                      | Residual | 연결 변수                          | 목적                       |
| -------------------------- | ------------------------------------------------------------------------------------------ | -------- | ------------------------------ | ------------------------ |
| (제거됨) raw GNSS 관측 factor | NMEA-only 리팩터링으로 `include/gnss_factor/*` 제거됨 | - | - | pseudo-range / carrier-phase 경로 미사용 |


추가 위치(레거시): 이전에는 `GNSS_Processing_fg.cpp` — 제거됨.

### 핵심 수식

#### (a) PSR + Doppler (`GnssPsrDoppFactorNoR`, `Nolidar`)


r_{\rho} = (\rho_{\text{est}} - \rho_{\text{meas}})w_{\rho}


r_{d} = (d_{\text{est}} + d_{\text{meas}}\lambda)w_d


코드상 추정식 구성 요소:

- 기하학 거리  \mathbf{s}-\mathbf{p} 
- Sagnac 보정
- 수신기 clock bias/drift (dt, \dot{dt})
- 위성 시계 항(svdt, svddt)
- 전리층/대류권 보정

#### (b) CP 차분 (`GnssCpFactorNoR`, `Nolidar`)

# 
r_{\phi}

## \left(
\mathbf{s}_j-\mathbf{p}_j

\mathbf{s}_i-\mathbf{p}_i

- dt_j - dt_i

- \phi_{\text{meas}}
\right)w_{\phi}


---

## 2.2 Clock 동역학 factor


| Factor            | 코드 링크                                                                          | Residual | 연결 변수                | 목적                    |
| ----------------- | ------------------------------------------------------------------------------ | -------- | -------------------- | --------------------- |
| (제거됨) clock 동역학 factor | NMEA-only 리팩터링으로 `include/gnss_factor/*` 제거됨 | - | - | raw GNSS clock 상태 미사용 |


추가 위치(레거시): 이전에는 `GNSS_Processing_fg.cpp` — 제거됨.

### 핵심 수식

#### (a) `DdtSmoothFactor`


r_{\dot{dt}} = \dot{dt}*{j} - \dot{dt}*{i}


#### (b) `DtDdtFactor` (시스템별)

# 
r_{dt}^{(k)}

dt_j^{(k)} - dt_i^{(k)}

- \frac{1}{2}(\dot{dt}_i+\dot{dt}_j)\Delta t


코드에서는 4개 시스템 벡터 residual로 구성된다.

---

## 2.3 LIO 연계 factor (GNSS 경로)


| Factor                 | 코드 링크                                                                            | Residual | 연결 변수                | 목적                    |
| ---------------------- | -------------------------------------------------------------------------------- | -------- | -------------------- | --------------------- |
| (제거됨) `GnssLioFactor` | NMEA-only 리팩터링으로 제거됨 | - | - | legacy GNSS-LIO factor |
| `NmeaLioFactorNolidar` | `[nmea_lio_factor_nolidar.hpp](include/nmea_factor/nmea_lio_factor_nolidar.hpp)` | 15       | `R_i, F_i, R_j, F_j` | IMU preintegration 제약 |


추가 위치(레거시 GNSS-LIO 표): 이전 `GNSS_Processing_fg.cpp`. NMEA 융합 측은 `[src/NMEA_Processing_fg.cpp](src/NMEA_Processing_fg.cpp)`.

### 핵심 수식

#### (a) `GnssLioFactor` (6D)


\mathbf{r}_p=
\left(\mathbf{p}_j-\mathbf{p}_i-\mathbf{v}*i\Delta t-\frac{1}{2}\mathbf{g}\Delta t^2\right)-\Delta\mathbf{p}*{lio}


\mathbf{r}_v=
\left(\mathbf{v}_j-\mathbf{v}*i-\mathbf{g}\Delta t\right)-\Delta\mathbf{v}*{lio}



\mathbf{r}=[\mathbf{r}_p;\mathbf{r}_v]


#### (b) `NmeaLioFactorNolidar` (15D preintegration)


\mathbf{r}=
\begin{bmatrix}
\mathbf{r}*{\Delta p}
\mathbf{r}*{\Delta R}
\mathbf{r}*{\Delta v}
\mathbf{r}*{b_a}
\mathbf{r}_{b_g}
\end{bmatrix}


with preintegration covariance whitening:


\mathbf{r}_{white} = \mathbf{\Lambda}^{1/2}\mathbf{r}


---

## 3) NMEA 경로 활성 factor


| Factor                 | 코드 링크                                                                                    | Residual | 연결 변수                | 목적                 |
| ---------------------- | ---------------------------------------------------------------------------------------- | -------- | -------------------- | ------------------ |
| `NmeaLioGravRelFactor` | `[nmea_lio_gravity_rel_factor.hpp](include/nmea_factor/nmea_lio_gravity_rel_factor.hpp)` | 27       | `P, R, A, O, G`      | LIO 상태와 중력/외부회전 결합 |
| `NMEAFactor`           | `[nmea_factor.hpp](include/nmea_factor/nmea_factor.hpp)`                                 | 9 또는 3 | `P, E, A, R`         | `nmea_input_type=odometry`: P/V/R(9D). **`navsatfix**: 위치 3D만** (0 twist/quat 제약 방지). [`AddFactor`](src/NMEA_Processing_fg.cpp)에서 `position_only` 전달 |
| `NMEAFactorNolidar`    | `[nmea_factor_nolidar.hpp](include/nmea_factor/nmea_factor_nolidar.hpp)`                 | 9 또는 3 | `R, F`               | 위와 동일 분기 (`navsatfix` → 3D) |
| `NmeaLioFactorNolidar` | `[nmea_lio_factor_nolidar.hpp](include/nmea_factor/nmea_lio_factor_nolidar.hpp)`         | 15       | `R_i, F_i, R_j, F_j` | 상태 천이(재사용)         |


추가 위치: `[src/NMEA_Processing_fg.cpp](src/NMEA_Processing_fg.cpp)`

### 핵심 수식

#### (a) `NMEAFactor` / `NMEAFactorNolidar`

`nmea_input_type == "navsatfix"`일 때는 residual이 \(\mathbf{r}=\mathbf{r}_p \in \mathbb{R}^3\)만 사용한다.

\mathbf{r}*p = (\mathbf{p}*{est}-\mathbf{p}*{nmea})w


\mathbf{r}v = (\mathbf{v}{est}-\mathbf{v}*{nmea})w


\mathbf{r}*R = \log\left(\mathbf{R}*{nmea}^{\top}\mathbf{R}_{est}\right)w



\mathbf{r}=[\mathbf{r}_p;\mathbf{r}_v;\mathbf{r}_R] \in \mathbb{R}^9


#### (b) `NmeaLioGravRelFactor` (27D)

구성은 크게 두 블록:

- 중력/자세/외부회전 정합
- LIO 상태(위치, 속도, 각속도, 가속도, bias, gravity) 정합

코드에서 24D 상태 블록은  \sqrt{\Lambda}_{lidar} 로 whitening:


\mathbf{r}*{3:26} \leftarrow \sqrt{\Lambda}*{lidar}\mathbf{r}_{3:26}


---

## 4) 공통 GTSAM Prior factor 사용

커스텀 factor 외에 GTSAM 기본 `PriorFactor<T>`를 적극 사용한다.

### 4.1 퇴화/가중치 분기 priors

파일: `[src/NMEA_Processing_fg.cpp](src/NMEA_Processing_fg.cpp)`

- `PriorFactor<Vector3>(G)`
- `PriorFactor<Vector6>(A)`
- `PriorFactor<Rot3>(R)`
- `PriorFactor<Vector12>(O)`

`weight_lid_zero` 조건에서 `NmeaLioGravRelFactor` 대신 안정화 용도로 삽입된다.

### 4.2 마지널라이제이션 후 priors

파일: `[src/NMEA_Assignment.cpp](src/NMEA_Assignment.cpp)`

- 외부 파라미터: `PriorFactor<Rot3>(P)`, `PriorFactor<Vector3>(E)`
- 상태: `PriorFactor<Rot3>(R)`, `PriorFactor<Vector6>(A)`, `PriorFactor<Vector12>(F)`
- 시계 상태: `PriorFactor<Vector4>(B)`, `PriorFactor<Vector1>(C)`

즉, LIGO는 "관측 factor + 동역학 factor + 마지널 prior"의 구조로 윈도우를 유지한다.

---

## 5) Ceres 기반 LiDAR factor

GTSAM 트랙과 별도로, `Curvefitter`에서 Ceres residual을 사용한다.


| Factor/Functor        | 코드 링크                                                                  | Residual | 목적                           | 추가 위치                                                                      |
| --------------------- | ---------------------------------------------------------------------- | -------- | ---------------------------- | -------------------------------------------------------------------------- |
| `LiDARPoseFactor<_N>` | `[lidar_feature_factor.h](include/Curvefitter/lidar_feature_factor.h)` | 6        | spline pose vs LiDAR pose 정합 | `[trajectory_estimator.hpp](include/Curvefitter/trajectory_estimator.hpp)` |


### 수식


\mathbf{r}*R = w_R \log(\mathbf{R}*{spline}\mathbf{R}*{meas}^{-1})


\mathbf{r}p = w_p(\mathbf{p}{spline}-\mathbf{p}*{meas})


\mathbf{r}=[\mathbf{r}_R;\mathbf{r}_p]


- loss: `ceres::CauchyLoss(0.5)`
- 삽입 API: `problem_->AddResidualBlock(...)`

참고: `SO3KnotFactor<_N>`는 정의는 되어 있으나 현재 호출 경로에서 활성 residual 추가가 확인되지 않는다.

---

## 6) 비활성/미호출 factor 목록

다음 항목은 파일은 존재하지만 현재 기본 활성 경로에서 호출이 확인되지 않았다.

### GNSS 계열

- raw GNSS factor 헤더(`include/gnss_factor/*`)는 NMEA-only 리팩터링에서 제거됨.

### NMEA 계열

- NMEA-only 리팩터링으로 미사용 헤더(`nmea_lio_factor.hpp`, `nmea_pos_factor*.hpp`, `nmea_lio_gravity_hard_factor.hpp`) 제거됨.

---

## 7) 최종 요약

- 실사용 중심: **PSR/Doppler + CP + clock 동역학 + LIO 연계 + NMEA 관측 + prior**
- NMEA 경로는 공통 상태와 `NmeaLioFactorNolidar`를 사용해 상태 천이 제약을 유지한다.
- LiDAR spline 최적화는 별도의 Ceres 문제(`LiDARPoseFactor`)로 동작한다.
- 코드에는 실험/변형 factor가 다수 존재하므로, 신규 실험 시 "활성 경로 여부"를 먼저 확인하는 것이 안전하다.

---

## 8) `mapping_avia.launch.py` 실행 시 (NMEA-only) 실제 사용 factor

기준 파일/설정:

- launch: `[launch/mapping_avia.launch.py](launch/mapping_avia.launch.py)`
- 설정: `[config/avia.yaml](config/avia.yaml)`
- 조건: `gnss.gnss_enable=false`, `nmea.nmea_enable=true`

### 8.1 실행 경로 요약

- `mapping_avia.launch.py`는 `avia.yaml`을 파라미터로 `ligo_mapping` 노드를 실행한다.
- `parameters.cpp`에서 `NMEA_ENABLE`(및 `gnss.*` 네임스페이스의 GTSAM/윈도우 파라미터)를 로드한다. raw GNSS 관측(obs) 큐·동기화(`GNSS_ENABLE`) 경로는 코드에서 제거되어 있으며, NMEA만 사용한다.
- 실제 factor 추가는 `[src/NMEA_Processing_fg.cpp](src/NMEA_Processing_fg.cpp)`의 `SetInit()`와 `AddFactor()`에서 수행된다.

### 8.2 초기화 시점(`SetInit`)에 추가되는 factor

다음 GTSAM prior가 초기 그래프에 추가된다.

- `PriorFactor<Rot3>(P(0))` (외부 회전 초기화)
- `PriorFactor<Vector3>(E(0))` (anchor/외부 위치 초기화)
- `PriorFactor<Rot3>(R(0))` (상태 회전 초기화)
- `PriorFactor<Vector6>(A(0))` (상태 pos/vel 초기화)
- `PriorFactor<Vector12>(O(0))` (omg/acc/bg/ba 초기화)
- `PriorFactor<Vector3>(G(0))` (중력 초기화)

### 8.3 프레임 업데이트(`AddFactor`)에서 추가되는 factor

#### (a) LiDAR 사용 경로(`!nolidar`)

- 기본:
  - `NmeaLioGravRelFactor(P(0), R(k), A(k), O(k), G(k), ...)`
  - `NMEAFactor(P(0), E(0), A(k), R(k), ...)`
- 초기 윈도우 구간(`k < delete_thred`)에서는 `NMEAFactor_init` 노이즈 모델이 사용된다.

#### (b) LiDAR 비사용 경로(`nolidar`)

- `NmeaLioFactorNolidar(R(k-1), F(k-1), R(k), F(k), ...)`
- `NMEAFactorNolidar(R(k), F(k), ...)`

#### (c) LiDAR 가중치 퇴화 fallback (`weight_lid_zero`)

`NmeaLioGravRelFactor` 대신 안정화 prior를 추가:

- `PriorFactor<Vector3>(G(k))`
- `PriorFactor<Vector6>(A(k))`
- `PriorFactor<Rot3>(R(k))`
- `PriorFactor<Vector12>(O(k))`

### 8.4 주의: 이름과 실제 사용 경로

- 과거 `NmeaLioFactor` 경로는 미사용으로 정리되었고, 현재는 `NmeaLioFactorNolidar` 경로만 유지된다.
- `NmeaLioFactorNolidar`는 NMEA-only 경로의 상태 천이 제약으로 실제 사용된다.

### 8.5 NMEA-only 실제 사용 factor + 경로 일람

아래 표는 **`gnss_enable=false`, `nmea_enable=true`**일 때 `gtSAMgraph.add(...)`로 실제 추가되는 factor만 정리한 것이다.

| Factor | 사용 조건 | 실제 추가 코드(호출) | 클래스 정의 파일 | 이 문서 내 설명 위치 |
|---|---|---|---|---|
| `NmeaLioGravRelFactor` | `!nolidar` 이고 `!weight_lid_zero` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | [`include/nmea_factor/nmea_lio_gravity_rel_factor.hpp`](include/nmea_factor/nmea_lio_gravity_rel_factor.hpp) | `3) NMEA 경로 활성 factor`, `8.3(a)` |
| `NMEAFactor` (`NMEAFactor_init`) | `!nolidar` (초기 윈도우는 init 노이즈) | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | [`include/nmea_factor/nmea_factor.hpp`](include/nmea_factor/nmea_factor.hpp) | `3) NMEA 경로 활성 factor`, `8.3(a)` |
| `NmeaLioFactorNolidar` | `nolidar` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | [`include/nmea_factor/nmea_lio_factor_nolidar.hpp`](include/nmea_factor/nmea_lio_factor_nolidar.hpp) | `2.3) LIO 연계 factor`, `3) NMEA 경로 활성 factor`, `8.3(b)` |
| `NMEAFactorNolidar` | `nolidar` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | [`include/nmea_factor/nmea_factor_nolidar.hpp`](include/nmea_factor/nmea_factor_nolidar.hpp) | `3) NMEA 경로 활성 factor`, `8.3(b)` |
| `gtsam::PriorFactor<Rot3>(P(0))` | 초기화 `SetInit()` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`SetInit`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.2` |
| `gtsam::PriorFactor<Vector3>(E(0))` | 초기화 `SetInit()` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`SetInit`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.2` |
| `gtsam::PriorFactor<Rot3>(R(0))` | 초기화 `SetInit()` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`SetInit`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.2` |
| `gtsam::PriorFactor<Vector6>(A(0))` | 초기화 `SetInit()` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`SetInit`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.2` |
| `gtsam::PriorFactor<Vector12>(O(0))` | 초기화 `SetInit()` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`SetInit`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.2` |
| `gtsam::PriorFactor<Vector3>(G(0))` | 초기화 `SetInit()` | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`SetInit`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.2` |
| `gtsam::PriorFactor<Vector3>(G(k))` | `weight_lid_zero` fallback | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.3(c)` |
| `gtsam::PriorFactor<Vector6>(A(k))` | `weight_lid_zero` fallback | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.3(c)` |
| `gtsam::PriorFactor<Rot3>(R(k))` | `weight_lid_zero` fallback | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.3(c)` |
| `gtsam::PriorFactor<Vector12>(O(k))` | `weight_lid_zero` fallback | [`src/NMEA_Processing_fg.cpp`](src/NMEA_Processing_fg.cpp) (`AddFactor`) | GTSAM 템플릿(`gtsam::PriorFactor`) | `4) 공통 GTSAM Prior factor`, `8.3(c)` |

#### NMEA-only에서 "사용되지 않는" 대표 factor (혼동 방지)

- `NmeaLioFactor` 경로는 미사용 정리 단계에서 삭제됨.

