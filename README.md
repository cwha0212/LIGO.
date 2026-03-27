# LIGO

**LIGO: Tightly Coupled LiDAR-Inertial-GNSS Odometry based on a Hierarchy Fusion Framework for Global Localization with Real-time Mapping**

이 브랜치는 **raw GNSS 관측(eph/obs/RTK) 융합 코드 경로를 빌드에서 제외**하고, 전역 제약은 **`nmea.*`(NavSatFix/Odometry·PPP 파일)** 경로로만 사용한다. `gnss.*` YAML 키는 GTSAM/노이즈 등 NMEA 그래프 파라미터용으로 일부 유지된다.

- Paper: [TRO PDF](https://github.com/Joanna-HE/LIGO./blob/main/paper/LIGO_A_Tightly_Coupled_LiDAR-Inertial-GNSS_Odometry_Based_on_a_Hierarchy_Fusion_Framework_for_Global_Localization_With_Real-Time_Mapping.pdf)
- Datasets: [Google Drive](https://drive.google.com/drive/folders/1hNwl8u8Pg-SqKh2N808XFixj6PjPf091?usp=sharing)

# Developers

The codes of this repo are contributed by [Dongjiao He (贺东娇)](https://github.com/Joanna-HE).

# Properties

**LIGO is a multi-sensor fusion framework that maximizes the complementary properties of both LiDAR and GNSS systems**. This package achieves the following properties:

1. Competitive accuracy in trajectory estimation across large-scale scenarios.
2. Robustness to malfunctions of either GNSS or LiDAR sensors, enabling seamless handling of added or lost sensor signals during operation.
3. High-output-frequency odometry.
4. Capability of providing globally referenced pose estimations in both indoor and outdoor environments, suitable for ground vehicles and uncrewed aerial vehicles (UAVs).
5. No requirement for GNSS observations to be obtained exactly at the beginning or end time of LiDAR scans.
6. Robustness to large outliers and high noise levels in GNSS observations.

---

# 빌드 및 의존성 (ROS 2)

아래는 **C++ 노드 `ligo_mapping`과 indoor small_gicp 관련 기능까지 포함**해 패키지를 빌드하기 위한 요구사항과 절차이다.

## 1. 필수 환경

| 항목 | 요구 |
|------|------|
| OS | Ubuntu (ROS 2 지원 버전; **Humble / Jazzy** 등에서 테스트) |
| ROS 2 | `ament_cmake` 기반 빌드 |
| 컴파일러 | **C++17** |
| CMake | **≥ 3.8** (`ligo`), 번들 [small_gicp](https://github.com/koide3/small_gicp) 서브프로젝트는 **≥ 3.16** (상위 패키지가 처리) |

## 2. 시스템·라이브러리 의존성

### 2.1 ROS 2·빌드 도구

- `ros-<distro>-desktop` 또는 `ros-base` + 개발 패키지
- `python3`, `python3-dev` (CMake에서 `Python3` 컴포넌트 요구)
- `git`

### 2.2 일반적으로 apt로 설치 가능한 항목 (예: Ubuntu)

설치 패키지 이름은 배포판에 따라 다를 수 있다.

- **Eigen3** — `libeigen3-dev`
- **PCL** — `libpcl-dev` 등 (CMake: `PCL >= 1.8`)
- **OpenCV** — `libopencv-dev`
- **Sophus** — `libsophus-dev` 또는 소스 빌드 후 `CMAKE_PREFIX_PATH`에 등록
- **OpenMP** — `libomp-dev` (선택이지만 권장; 멀티스레드에 사용)
- **fmt** — `libfmt-dev` (링크 타겟 `fmt`)

### 2.3 버전 하한이 엄격한 항목 (소스 빌드가 필요한 경우가 많음)

| 라이브러리 | CMake 요구 | 비고 |
|------------|------------|------|
| **Ceres Solver** | **≥ 2.2.0** (`ceres/manifold.h`, `EigenQuaternionManifold`) | Ubuntu 기본 `libceres-dev`는 **2.0.x**인 경우가 많아 **부족할 수 있음**. [공식 릴리스 2.2.0+](https://github.com/ceres-solver/ceres-solver/releases) 소스 빌드 후 설치 권장. |
| **GTSAM** | **≥ 4.1.1** (`find_package(GTSAM 4.1.1 REQUIRED CONFIG)`) | [GTSAM 4.1.1+](https://github.com/borglab/gtsam/releases) 빌드·설치. |

설치 후 `CMAKE_PREFIX_PATH`에 Ceres/GTSAM **설치 루트**를 넣거나, `colcon build` 시 `-DCeres_DIR=...` `-DGTSAM_DIR=...` 로 `*Config.cmake` 위치를 지정한다.

### 2.4 ROS 2 패키지 의존성 (`package.xml` 기준)

빌드·실행에 포함되는 주요 패키지:

- `rclcpp`, `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `pcl_conversions`, `cv_bridge`
- `builtin_interfaces`, `ament_index_cpp`, `ament_index_python`
- `rosidl_default_generators` / `rosidl_default_runtime` (커스텀 메시지)
- **`livox_ros_driver2`** — 동일 워크스페이스에 소스로 두고 함께 빌드하거나, 이미 설치된 환경에서 `find_package` 가능해야 함.

### 2.5 선택 의존성

| 항목 | 역할 |
|------|------|
| **gnss_comm** | `find_package(gnss_comm)` 성공 시 NMEA/PPP 등 **GNSS 파이프라인** 빌드. 없으면 **LiDAR+IMU 스텁**만 사용. |
| **small_gicp (C++)** | `thirdparty/small_gicp`에 **[koide3/small_gicp](https://github.com/koide3/small_gicp)** 서브모듈이 있으면 `colcon` 빌드 시 **같이 빌드**되어 `ligo_mapping`에 정적 링크된다. |
| **small_gicp (Python)** | `ros2_livox_small_gicp.py`, `ros2_indoor_map_gicp.py` 등은 **`pip install small_gicp`** 로 별도 설치 (C++ 번들과 별개). |

---

## 3. 저장소 준비 (서브모듈)

C++용 **small_gicp**를 포함하려면 서브모듈을 받는다.

```bash
cd /path/to/LIGO   # 본 패키지 루트
git submodule update --init --recursive
```

`thirdparty/small_gicp`가 비어 있거나 없으면 indoor small_gicp C++ 모듈이 비활성화될 수 있다.

---

## 4. 워크스페이스에서 빌드

```bash
cd /path/to/your_ros2_ws
source /opt/ros/<distro>/setup.bash

# Ceres/GTSAM/Sophus 등을 소스 설치한 경우 예:
# export CMAKE_PREFIX_PATH="/opt/ceres_install:/opt/gtsam_install:${CMAKE_PREFIX_PATH}"

# gnss_comm을 쓰는 경우: 해당 install을 먼저 source
# source /path/to/gnss_comm_ws/install/setup.bash

colcon build --symlink-install --packages-select ligo
# livox_ros_driver2가 같은 ws에 있으면 함께 선택:
# colcon build --symlink-install --packages-select livox_ros_driver2 ligo

source install/setup.bash
```

- **`--symlink-install`** 을 권장한다 (Python 스크립트 수정 시 재빌드 부담 감소).
- Ceres/GTSAM을 표준 경로가 아닌 곳에 설치했다면 **`CMAKE_PREFIX_PATH`** 또는 **`--cmake-args`** 로 경로를 반드시 넘긴다.

---

## 5. 실행 예시

```bash
ros2 launch ligo mapping_avia.launch.py
```

또는:

```bash
ros2 run ligo ligo_mapping --ros-args -p __params:=/path/to/config/avia.yaml
```

설정 YAML은 ROS 1과 유사한 구조이며, 파라미터 이름은 점으로 구분한다 (예: `common.lid_topic`).

---

## 6. ROS 1 (Noetic) — 참고

Ubuntu 20.04, ROS Noetic, C++17, Eigen 3, GTSAM 4, OpenCV 4.2, PCL 1.10 환경에서 테스트되었다.

```bash
sudo apt-get install libboost-all-dev
```

- Livox: [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
- gnss_comm: [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) — [빌드 안내](https://github.com/HKUST-Aerial-Robotics/gnss_comm#2-build-gnss_comm-library)

```bash
cd ~/catkin_ws/src/
git clone https://github.com/Joanna-HE/LIGO..git
cd ~/catkin_ws/
source /PATH/TO/LIVOX_DRIVER/DEVEL/setup.bash
source /PATH/TO/GNSS_COMM/DEVEL/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

---

# Hardware setups for self-collected datasets

## Setup

<div align="left">
    <div align="left">
        <img src="https://github.com/Joanna-HE/LIGO./blob/main/image/hardware.jpg" width = 30% >
    </div>
</div>

Platform: DJI Matrice 300  
Onboard computer: DJI Manifold 2-c 256G, CPU: Intel i7-8550U  
LiDAR: Livox Mid360 and Livox Avia  
IMU: Built-in IMU of Livox LiDAR  
GNSS receiver: u-blox C099-F9P-2  
GNSS antenna: B4QA4GGGB  

## Recording rates

LiDAR: 10Hz  
IMU: 200Hz  
GNSS: 10Hz  
RTK: 10Hz  

## Recording software

Operating system: Ubuntu 20.04  
IMU and LiDAR driver: Livox driver  
GNSS driver: [ublox driver](https://github.com/Joanna-HE/ublox_driver)  

## ROS topics recorded

IMU: /livox/imu  
LiDAR: /livox/lidar  
RAW GNSS: /ublox_driver/range_meas  
GNSS EPHEM: /ublox_driver/ephem and /ublox_driver/glo_ephem  
IONO PARAMETER: /ublox_driver/iono_params  
Onboard pos solution of ublox: /ublox_driver/receiver_pvt and /ublox_driver/receiver_lla  
PPS time info: /ublox_driver/time_pulse_info  

## Time synchronization

PPS: Livox LiDARs can receive pps and gprmc given by the GNSS receiver  
The time difference between LiDAR and IMU is zero, and between LiDAR and GNSS message is 18.0 s  

## RTK solution

Please follow the *Differential GNSS* section shown in [ublox driver](https://github.com/Joanna-HE/ublox_driver) to get the differential GNSS solution online or offline. The self-collected datasets get the online RTK solution which are saved in the topic '/ublox_driver/receiver_pvt', the value of the 'carr_soln' as 1 and 'diff_soln' as 2 indicates the fix RTK solution.

---

# Demo

**Performance on a sequence with severe LiDAR degeneracy**

<div align="center">
    <div align="center">
        <img src="https://github.com/Joanna-HE/LIGO/blob/main/image/Sample.png" width = 75% >
    </div>
</div>

---

# 달라진 점

- v1.1: NMEA 입력을 NavSatMsg로 받아서 처리하는 부분을 다시 활성화함.
- v1.2: Z축에 대한 noise param 추가.
- v2: `indoor_localization_factor` 추가, NMEA에서 pose only 추가, `ring` 오류 처리, markdown 파일 추가.
