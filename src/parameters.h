/*
 * BSD 3-Clause License

 *  Copyright (c) 2025, Dongjiao He
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
/** Raw GNSS (ephemeris/obs) pipeline removed; keep macro for legacy conditionals. */
#ifndef GNSS_ENABLE
#define GNSS_ENABLE 0
#endif
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include "preprocess.h"
#ifdef LIGO_WITH_NMEA
#include "NMEA_Processing_fg.h"
#else
#include "ligo_gnss_nmea_stub.h"
#endif
#include "IMU_Processing.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mutex>
#include <omp.h>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ivox/ivox3d.h>
#include <Python.h>
#include <condition_variable>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl/common/transforms.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <Curvefitter/curvefitter.hpp>
#include "Indoor_NoiseModel.h"

using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;

extern std::vector<curvefitter::PoseData> pose_graph_key_pose;
extern std::vector<double> pose_time_vector;
extern std::vector<std::vector<Eigen::Vector3d> > LiDAR_points;
extern int points_num;
extern double map_time;
extern typename curvefitter::TrajectoryManager<4>::Ptr traj_manager;
extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;
extern IVoxType::Options ivox_options_;
extern int ivox_nearby_type;
extern state_output state_out;
extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame;
extern bool space_down_sample;
extern bool publish_odometry_without_downsample;
extern int  init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro;
extern float  plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
extern float  DET_RANGE;
extern bool   imu_en, init_with_imu;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov; 
extern int    lidar_type, pcd_save_interval;
extern int    gt_file_type;
extern std::vector<double> gravity_init, gravity;
extern std::vector<double> extrinT, extrinT_gnss;
extern std::vector<double> extrinR, extrinR_gnss;
extern std::vector<double> ppp_anc;
extern bool   runtime_pos_log, pcd_save_en, path_en;
extern bool   scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;
extern shared_ptr<NMEAProcess> p_nmea;
extern bool is_first_frame;
extern bool dyn_filter;
extern double dyn_filter_resolution;

extern std::string gt_fname, ephem_fname, ppp_fname;
extern std::string nmea_meas_topic;
extern std::string nmea_input_type;
/** If true, gpsHandler publishes stamp diagnostics on /ligo/nmea_stamp_diag (NavSatFix path). */
extern bool nmea_publish_stamp_diag;
extern std::string enu_position_topic;
extern std::string enu_position_frame_id;
extern std::string global_position_topic;
extern std::string ecef_position_topic;
extern std::string ecef_position_frame_id;
extern bool nmea_global_anchor_ready;
extern Eigen::Vector3d nmea_global_anchor_lla;
extern std::vector<double> default_gnss_iono_params;
extern double gnss_local_time_diff, gnss_ekf_noise;
extern bool next_pulse_time_valid, update_gnss, update_nmea;
extern bool time_diff_valid, is_first_gnss, is_first_nmea;
extern double latest_gnss_time, next_pulse_time, last_nmea_time;
extern double time_diff_gnss_local, time_diff_nmea_local;
extern double nmea_gps_latency;
extern bool gnss_local_online_sync, nolidar; 
extern double li_init_gyr_cov, li_init_acc_cov, lidar_time_inte, first_imu_time;
extern int orig_odom_freq;
extern double online_refine_time; //unit: s
extern bool NMEA_ENABLE;
extern bool mapping_mode;
extern bool indoor_flag;
extern double time_update_last, time_current, time_predict_last_const, t_last;
extern Eigen::Matrix3d Rot_gnss_init;
extern Eigen::Vector3d indoor_pos_enu_meas;
extern Eigen::Quaterniond indoor_rot_enu_meas;
extern bool indoor_pose_valid;
extern double indoor_pose_time;
extern gtsam::noiseModel::Base::shared_ptr indoorPoseNoise;
extern gtsam::noiseModel::Base::shared_ptr indoorPoseNoiseInit;

extern std::string indoor_map_pcd_path;
/** Non-empty: resolve GICP reference PCD via * _grid2d.yaml occupancy (same as indoor map membership). */
extern std::string indoor_grid_map_dir;
extern bool indoor_gicp_map_loaded;
extern Eigen::Isometry3d indoor_gicp_T_map_lidar;
extern bool indoor_flag_dynamic;
extern double indoor_gicp_max_factor_error;
extern int    indoor_gicp_min_factor_inliers;

extern MeasureGroup Measures;

extern std::vector<Eigen::Vector3d> est_poses;
extern std::vector<Eigen::Vector3d> local_poses;
extern std::vector<Eigen::Matrix3d> local_rots;
extern std::vector<double> time_frame;

extern ofstream fout_out, fout_rtk, fout_global, fout_ppp;
void readParameters(rclcpp::Node * node);
void open_file();
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &orient);
void cout_state_to_file(Eigen::Vector3d &pos_enu);
void cout_state_to_file_nmea();
/** Fused IMU position in ENU (same geometry as @ref cout_state_to_file_nmea). False if NMEA inactive or not ready. */
bool compute_fused_imu_position_enu(Eigen::Vector3d &pos_enu);
/** WGS84 geodetic (lat, lon deg; alt m) from fused ENU + anchor. False if no anchor data. */
bool compute_fused_imu_position_geo(Eigen::Vector3d &out_lla);
/** WGS84 ECEF (m) from fused ENU + anchor. False if no anchor data. */
bool compute_fused_imu_position_ecef(Eigen::Vector3d &out_ecef);
/** ENU position consistent with published /aft_mapped when ICP tf is ready; else fused ISAM ENU. */
bool compute_system_output_pose_enu(Eigen::Vector3d &pos_enu);
/** ECEF from @ref compute_system_output_pose_enu + anchor. */
bool compute_system_output_pose_ecef(Eigen::Vector3d &out_ecef);
void reset_cov_output(Eigen::Matrix<double, 24, 24> & P_init_output);