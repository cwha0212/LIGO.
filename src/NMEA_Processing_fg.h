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
// #include <mavros_msgs/GPSRAW.h>
#include <nav_msgs/msg/odometry.hpp>
#include "NMEA_Assignment.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#define WINDOW_SIZE (10) // should be 0

class NMEAProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NMEAProcess();
  ~NMEAProcess();
  
  void Reset();
  void processNMEA(const nav_msgs::msg::Odometry::SharedPtr &gnss_meas, state_output &state);
  void SetInitFromLocalization(const Eigen::Vector3d &indoor_pos_enu,
                               const Eigen::Matrix3d &indoor_rot_enu,
                               const state_output &seed_state,
                               double init_time_sec);
  bool NMEALIAlign();
  void TrajAlign(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&local_traj, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&enu_traj, Eigen::Vector3d &pos, Eigen::Matrix3d &rot);
  // bool TrajAlign(Eigen::Vector3d &pos, Eigen::Matrix3d &rot);
  void updateNMEAstatistics(Eigen::Vector3d &pos);
  Eigen::Vector3d local2enu(Eigen::Matrix3d enu_rot, Eigen::Vector3d anc, Eigen::Vector3d &pos);
  void SetInit();
  bool AddFactor(gtsam::Rot3 rel_rot_, gtsam::Point3 rel_pos_, gtsam::Vector3 rel_v_, Eigen::Vector3d state_gravity, double delta_t, double time_current,
                Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d omg, Eigen::Matrix3d rot);
  // std::vector<ObsPtr> gnss_meas_buf[WINDOW_SIZE+1]; //
  std::vector<nav_msgs::msg::Odometry::SharedPtr> nmea_meas_; //[WINDOW_SIZE+1]; //
  // Eigen::Matrix3d rot_window[WINDOW_SIZE+1]; //
  Eigen::Matrix3d rot_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d pos_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d vel_window[WINDOW_SIZE+1]; //
  // Init-phase buffers: accumulate all until 3m reached (no fixed size limit).
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> init_pos_buf;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> init_rot_buf;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> init_vel_buf;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> init_nmea_buf;
  std::vector<double> init_lio_time_buf;  // LIO 각 샘플의 시각(stamp). 지연보정 시 T-L 보간에 사용
  // Eigen::Vector3d pos_window[WINDOW_SIZE+1]; //
  // Eigen::Vector3d vel_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d Tex_imu_r;
  Eigen::Matrix3d Rex_imu_r;
  std::queue<nav_msgs::msg::Odometry::SharedPtr> nmea_msg;

  bool invalid_lidar = false;
  // double dt[4];
  // double ddt; 
  size_t id_accumulate = 0; // 
  size_t frame_delete = 0; // 

  int frame_num = 0; // 
  double last_nmea_time = 0.0; //
  double nmea_sample_period = 0.1;

  // double diff_t_gnss_local = 0.0;
  // Eigen::Vector3d ecef_pos, first_xyz_ecef_pvt, first_xyz_ecef_lla, first_lla_pvt, first_lla_lla;
  Eigen::Matrix3d Rot_nmea_init = Eigen::Matrix3d::Identity();
  bool nmea_ready = false;
  int frame_count = 0; //
  int delete_thred = 0;
  int wind_size = WINDOW_SIZE;
  int norm_vec_num = 0;
  bool nolidar = false;
  bool nolidar_cur = false;
  std::vector<Eigen::Vector3d> norm_vec_holder;
  // double para_yaw_enu_local[1];
  // double para_rcv_dt[(WINDOW_SIZE+1)*4] = {0}; //
  // double para_rcv_ddt[WINDOW_SIZE+1] = {0}; //
  Eigen::Vector3d anc_enu, gravity_init;
  Eigen::Vector3d anc_local = Eigen::Vector3d::Zero();
  // ICP rigid transform (local -> ENU) captured at initialization.
  Eigen::Matrix3d icp_R_local_to_enu = Eigen::Matrix3d::Identity();
  Eigen::Vector3d icp_t_local_to_enu = Eigen::Vector3d::Zero();
  bool icp_tf_ready = false;
  // ICP alignment pairs for RViz visualization (LIO[i] <-> NMEA[i] in camera_init frame).
  std::vector<Eigen::Vector3d> icp_pairs_lio;
  std::vector<Eigen::Vector3d> icp_pairs_nmea_local;
  // Eigen::Matrix3d R_ecef_enu;
  double yaw_enu_local = 0.0;
  
  void runISAM2opt(void);
  // void GnssPsrDoppMeas(const ObsPtr &obs_, const EphemBasePtr &ephem_);
  // void SvPosCals(const ObsPtr &obs_, const EphemBasePtr &ephem_);
  bool Evaluate(state_output &state);
  state_output state_const_;
  state_output state_const_last;
  double nmea_weight = 1.0;
  // NMEA-LIO init guard: run ICP only after cumulative motion from start.
  bool init_start_set = false;
  Eigen::Vector3d init_start_lio = Eigen::Vector3d::Zero();
  Eigen::Vector3d init_start_nmea = Eigen::Vector3d::Zero();
  static constexpr double GPS_MOVE_START_THRESH_M = 0.3;  // GPS 변위 이 값 초과 시 "이동 시작"으로 간주
  double nmea_gps_latency_estimated = 0.0;  // 초기 추정 수신지연(초). 0=미추정. Fusion phase 보정에 사용.
  double sum_nmea_lio_err_sq_xy = 0.0;      // ICP 이후 전체 경로 pair 오차² 누적
  int n_nmea_fusion_count = 0;               // fusion 횟수
  // 0.3m 시점 LIO-GPS 비교 (latency 진단용). icp_tf_ready 시 한 번 설정됨.
  bool diag_03m_valid = false;
  double diag_03m_latency_s = 0.0;           // t_gps - t_lio (초)
  double diag_03m_t_lio = 0.0, diag_03m_t_gps = 0.0;
  Eigen::Vector3d diag_03m_lio_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d diag_03m_gps_at_t_lio = Eigen::Vector3d::Zero();  // t_lio 시점 GPS 보간
  double diag_03m_lio_disp = 0.0, diag_03m_gps_disp_at_t_lio = 0.0;  // start 대비 변위
  double init_min_lio_total_move_m = 3.0;
  double init_min_nmea_total_move_m = 3.0;
  int init_icp_max_iterations = 80;
  double init_icp_max_fitness = 5.0;
  Eigen::Matrix<double, 24, 24> sqrt_lidar;
  double odo_weight1 = 1.0;
  double odo_weight2 = 1.0;
  double odo_weight3 = 1.0;
  // Eigen::Matrix<double, 3, 3> rot_weight = Eigen::Matrix3d::Identity();
  double odo_weight4 = 2.0;
  double odo_weight5 = 2.0;
  double odo_weight6 = 2.0;
  IntegrationBase* pre_integration = new IntegrationBase{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  NMEAAssignment* p_assign = new NMEAAssignment();
  // private:
    // int freq_idx = 0;
    // double freq = 0.0;
    // Eigen::Vector3d sv_pos;
    // Eigen::Vector3d sv_vel;
    // double svdt, svddt, tgd;
    // double pr_uura, dp_uura;
    // Eigen::Matrix3d rot_pos;
};

// # endif