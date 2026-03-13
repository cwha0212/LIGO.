# LIGO 

**LIGO: Tightly Coupled LiDAR-Inertial-GNSS Odometry based on a Hierarchy Fusion Framework for Global Localization with Real-time Mapping**

Code, paper, video are coming soon......
instruction for using will be detailed soon

Our paper is published on [TRO](https://github.com/Joanna-HE/LIGO./blob/main/paper/LIGO_A_Tightly_Coupled_LiDAR-Inertial-GNSS_Odometry_Based_on_a_Hierarchy_Fusion_Framework_for_Global_Localization_With_Real-Time_Mapping.pdf)

Our datasets are uploaded on [Google Drive](https://drive.google.com/drive/folders/1hNwl8u8Pg-SqKh2N808XFixj6PjPf091?usp=sharing)

# Developers:
The codes of this repo are contributed by:
[Dongjiao He (贺东娇)](https://github.com/Joanna-HE)

# Properties

**LIGO is a multi-sensor fusion framework that maximizes the complementary properties of both LiDAR and GNSS systems**. This package achieves the following properties:

1. Competitive accuracy in trajectory estimation across large-scale scenarios.
2. Robustness to malfunctions of either GNSS or LiDAR sensors, enabling seamless handling of added or lost sensor signals during operation.
3. High-output-frequency odometry.
4. Capability of providing globally referenced pose estimations in both indoor and outdoor environments, suitable for ground vehicles and uncrewed aerial vehicles (UAVs).
5. No requirement for GNSS observations to be obtained exactly at the beginning or end time of LiDAR scans.
6. Robustness to large outliers and high noise levels in GNSS observations.

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

## Recording software:

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

# Build

## ROS 2 (Humble/Jazzy)

### Prerequisites
- ROS 2 (tested with Humble/Jazzy)
- C++17, Eigen3, PCL, GTSAM, Ceres, Sophus, OpenCV
- **livox_ros_driver2** (ROS 2 Livox driver)
- **gnss_comm**: GNSS support requires a ROS 2–compatible gnss_comm in the same workspace (e.g. build from [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) and port to ROS 2, or use a community port).

### Build
```bash
cd /path/to/your_ros2_ws/src
# clone this package (e.g. as LIGO./ or rename to ligo)
cd ..
source /opt/ros/<distro>/setup.bash
# If using gnss_comm: source /path/to/gnss_comm/install/setup.bash
colcon build --packages-select ligo
source install/setup.bash
```

### Run
```bash
ros2 launch ligo mapping_avia.launch.py
```
Or run the node with a config file:
```bash
ros2 run ligo ligo_mapping --ros-args -p __params:=/path/to/config/avia.yaml
```

Config YAML uses the same structure as ROS 1; parameter names use dots (e.g. `common.lid_topic`).

---

## ROS 1 (Noetic)

We test LIGO on ubuntu 20.04 with ROS noetic, and C++17 compiler & Eigen 3 & GTSAM 4 & opencv 4.2.0 & pcl 1.10  

### Install Boost using command sudo apt-get install libboost-all-dev

### Install [Livox Driver](https://github.com/Livox-SDK/livox_ros_driver)

### Install [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) with its [instuction](https://github.com/HKUST-Aerial-Robotics/gnss_comm#2-build-gnss_comm-library)

## Make

### clone the code to catkin_ws workspace
```
cd ~/catkin_ws/src/
git clone https://github.com/Joanna-HE/LIGO..git
```
### compile the package
```
cd ~/catkin_ws/
source /PATH/TO/LIVOX_DRIVER/DEVEL/.setup.bash
source /PATH/TO/GNSS_COMM/DEVEL/.setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Demo
**Performance on a sequence with severe LiDAR degeneracy**

<div align="center">
    <div align="center">
        <img src="https://github.com/Joanna-HE/LIGO/blob/main/image/Sample.png" width = 75% >
    </div>
</div>
