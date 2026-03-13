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

#include "parameters.h"
#include <fstream>
#include <chrono>

typename curvefitter::TrajectoryManager<4>::Ptr traj_manager = std::make_shared<curvefitter::TrajectoryManager<4>>();
bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<curvefitter::PoseData> pose_graph_key_pose;
std::vector<double> pose_time_vector;
std::vector<std::vector<Eigen::Vector3d> > LiDAR_points;
int points_num;
double map_time;
state_output state_out;
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true, con_frame = false;
bool space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10, con_frame_num = 1;
double match_s = 81, satu_acc, satu_gyro;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000; 
float  DET_RANGE = 450;
bool   imu_en = true;
bool   init_with_imu = true;
double imu_time_inte = 0.005, gnss_ekf_noise = 0.01;
double laser_point_cov = 0.01, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
int    gt_file_type;
std::vector<double> gravity_init, gravity;
std::vector<double> extrinT(3, 0.0), extrinT_gnss(3, 0.0);
std::vector<double> extrinR(9, 0.0), extrinR_gnss(9, 0.0);
std::vector<double> ppp_anc(3, 0.0);
bool   runtime_pos_log, pcd_save_en, path_en;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
// shared_ptr<LI_Init> Init_LI;
shared_ptr<ImuProcess> p_imu;
shared_ptr<GNSSProcess> p_gnss;
shared_ptr<NMEAProcess> p_nmea;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

std::string gnss_ephem_topic, gnss_glo_ephem_topic, gnss_meas_topic, gnss_iono_params_topic;
std::string gt_fname, ephem_fname, ppp_fname;
std::string gnss_tp_info_topic, local_trigger_info_topic, rtk_pvt_topic, rtk_lla_topic;
std::string nmea_meas_topic;
std::vector<double> default_gnss_iono_params(8, 0.0);
double gnss_local_time_diff = 18.0;
bool next_pulse_time_valid = false, update_gnss = false, update_nmea = false;
bool time_diff_valid = false, is_first_gnss = true, is_first_nmea;
double latest_gnss_time = -1, next_pulse_time = 0.0, last_nmea_time = -1; 
double time_diff_gnss_local = 0.0, time_diff_nmea_local = 0.0;
bool gnss_local_online_sync = true, nolidar = false; 
double li_init_gyr_cov = 0.1, li_init_acc_cov = 0.1, lidar_time_inte = 0.1, first_imu_time = 0.0;
int orig_odom_freq = 10;
double online_refine_time = 20.0; //unit: s
bool GNSS_ENABLE = true;
bool NMEA_ENABLE = true;
bool dyn_filter = false;
double dyn_filter_resolution = 1.0;
Eigen::Matrix3d Rot_gnss_init(Eye3d);
std::vector<Eigen::Vector3d> est_poses;
std::vector<Eigen::Vector3d> local_poses;
std::vector<Eigen::Matrix3d> local_rots;
std::vector<double> time_frame;

MeasureGroup Measures;

ofstream fout_out, fout_rtk, fout_global, fout_ppp; 

void readParameters(rclcpp::Node * node)
{
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  p_gnss.reset(new GNSSProcess());
  p_nmea.reset(new NMEAProcess());

  auto get_param = [node](const std::string & name, auto default_val) -> decltype(default_val) {
    node->declare_parameter(name, default_val);
    return node->get_parameter(name).get_value<decltype(default_val)>();
  };
  prop_at_freq_of_imu = get_param("prop_at_freq_of_imu", true);
  check_satu = get_param("check_satu", true);
  init_map_size = get_param("init_map_size", 100);
  space_down_sample = get_param("space_down_sample", true);
  satu_acc = get_param("mapping.satu_acc", 3.0);
  satu_gyro = get_param("mapping.satu_gyro", 35.0);
  acc_norm = get_param("mapping.acc_norm", 1.0);
  plane_thr = get_param("mapping.plane_thr", 0.05f);
  p_pre->point_filter_num = get_param("point_filter_num", 2);
  lid_topic = get_param("common.lid_topic", std::string("/livox/lidar"));
  imu_topic = get_param("common.imu_topic", std::string("/livox/imu"));
  con_frame = get_param("common.con_frame", false);
  con_frame_num = get_param("common.con_frame_num", 1);
  filter_size_surf_min = get_param("filter_size_surf", 0.5);
  filter_size_map_min = get_param("filter_size_map", 0.5);
  DET_RANGE = get_param("mapping.det_range", 300.f);
  fov_deg = get_param("mapping.fov_degree", 180.0);
  imu_en = get_param("mapping.imu_en", true);
  init_with_imu = get_param("mapping.init_with_imu", true);
  imu_time_inte = get_param("mapping.imu_time_inte", 0.005);
  laser_point_cov = get_param("mapping.lidar_meas_cov", 0.1);
  acc_cov_input = get_param("mapping.acc_cov_input", 0.1);
  vel_cov = get_param("mapping.vel_cov", 20.0);
  gyr_cov_input = get_param("mapping.gyr_cov_input", 0.1);
  gyr_cov_output = get_param("mapping.gyr_cov_output", 0.1);
  acc_cov_output = get_param("mapping.acc_cov_output", 0.1);
  b_gyr_cov = get_param("mapping.b_gyr_cov", 0.0001);
  b_acc_cov = get_param("mapping.b_acc_cov", 0.0001);
  imu_meas_acc_cov = get_param("mapping.imu_meas_acc_cov", 0.1);
  imu_meas_omg_cov = get_param("mapping.imu_meas_omg_cov", 0.1);
  p_pre->blind = get_param("preprocess.blind", 1.0);
  p_pre->det_range = get_param("preprocess.det_range", 1.0);
  lidar_type = get_param("preprocess.lidar_type", 1);
  gt_file_type = get_param("gnss.gt_file_type", 1);
  p_pre->N_SCANS = get_param("preprocess.scan_line", 16);
  p_pre->SCAN_RATE = get_param("preprocess.scan_rate", 10);
  p_pre->time_unit = get_param("preprocess.timestamp_unit", 1);
  match_s = get_param("mapping.match_s", 81.0);
  gravity = get_param("mapping.gravity", std::vector<double>());
  gravity_init = get_param("mapping.gravity_init", std::vector<double>());
  extrinT = get_param("mapping.extrinsic_T", std::vector<double>());
  ppp_anc = get_param("nmea.ppp_anc", std::vector<double>());
  extrinR = get_param("mapping.extrinsic_R", std::vector<double>());
  publish_odometry_without_downsample = get_param("odometry.publish_odometry_without_downsample", false);
  path_en = get_param("publish.path_en", true);
  scan_pub_en = get_param("publish.scan_publish_en", true);
  scan_body_pub_en = get_param("publish.scan_bodyframe_pub_en", true);
  runtime_pos_log = get_param("runtime_pos_log_enable", false);
  pcd_save_en = get_param("pcd_save.pcd_save_en", false);
  pcd_save_interval = get_param("pcd_save.interval", -1);
  lidar_time_inte = get_param("mapping.lidar_time_inte", 0.1);
  dyn_filter = get_param("mapping.dyn_filter", true);
  dyn_filter_resolution = get_param("mapping.dyn_filter_resolution", 0.1);
  gnss_ekf_noise = get_param("gnss.gnss_ekf_noise", 0.01);
  extrinT_gnss = get_param("gnss.gnss_extrinsic_T", std::vector<double>());
  extrinR_gnss = get_param("gnss.gnss_extrinsic_R", std::vector<double>());
  ivox_options_.resolution_ = get_param("mapping.ivox_grid_resolution", 0.2f);
  ivox_nearby_type = get_param("ivox_nearby_type", 18);
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
#ifdef LIGO_WITHOUT_GNSS
  GNSS_ENABLE = false;
  NMEA_ENABLE = false;
#else
  GNSS_ENABLE = get_param("gnss.gnss_enable", false);
  cout << "gnss enable:" << GNSS_ENABLE << endl;
  // #region agent log
  { std::ofstream _f("/home/chang/projects/NAVICOM/GPS_LIO_ws/.cursor/debug-75b37d.log", std::ios::app); _f << "{\"sessionId\":\"75b37d\",\"location\":\"parameters.cpp:197\",\"message\":\"GNSS_ENABLE loaded\",\"data\":{\"value\":" << (GNSS_ENABLE ? "true" : "false") << "},\"timestamp\":" << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << ",\"hypothesisId\":\"H1\"}\n"; }
  // #endregion
  if (GNSS_ENABLE)
  {
    p_gnss->relative_sqrt_info = get_param("gnss.psr_dopp_weight", 10.0);
    p_gnss->cp_weight = get_param("gnss.cp_weight", 0.1);
    p_gnss->p_assign->outlier_rej = get_param("gnss.outlier_rejection", false);
    gnss_ephem_topic = get_param("gnss.gnss_ephem_topic", std::string("/ublox_driver/ephem"));
    gnss_glo_ephem_topic = get_param("gnss.gnss_glo_ephem_topic", std::string("/ublox_driver/glo_ephem"));
    gnss_meas_topic = get_param("gnss.gnss_meas_topic", std::string("/ublox_driver/range_meas"));
    ephem_fname = get_param("gnss.ephem_file_name", std::string("BRDM00DLR_S_20221870000_01D_MN.rnx"));
    gt_fname = get_param("gnss.gt_file_name", std::string("UrbanNav_TST_GT_raw.txt"));
    ppp_fname = get_param("nmea.ppp_file_name", std::string("TST.pos"));
    gnss_iono_params_topic = get_param("gnss.gnss_iono_params_topic", std::string("/ublox_driver/iono_params"));
    rtk_pvt_topic = get_param("gnss.rtk_pvt_topic", std::string("/ublox_driver/receiver_pvt"));
    rtk_lla_topic = get_param("gnss.rtk_lla_topic", std::string("/ublox_driver/receiver_lla"));
    gnss_tp_info_topic = get_param("gnss.gnss_tp_info_topic", std::string("/ublox_driver/time_pulse_info"));
    default_gnss_iono_params = get_param("gnss.gnss_iono_default_parameters", std::vector<double>(8, 0.0));
    p_gnss->gravity_init << VEC_FROM_ARRAY(gravity);
    gnss_local_online_sync = get_param("gnss.gnss_local_online_sync", true);
    if (gnss_local_online_sync)
      local_trigger_info_topic = get_param("gnss.local_trigger_info_topic", std::string("/external_trigger"));
    else
    {
      gnss_local_time_diff = get_param("gnss.gnss_local_time_diff", 18.0);
      time_diff_gnss_local = gnss_local_time_diff;
    }
    p_gnss->p_assign->gnss_elevation_threshold = get_param("gnss.gnss_elevation_thres", 30.0);
    p_gnss->p_assign->prior_noise = get_param("gnss.prior_noise", 0.010);
    p_gnss->p_assign->marg_noise = get_param("gnss.marg_noise", 0.010);
    p_gnss->pre_integration->acc_w = get_param("gnss.b_acc_noise", 0.10);
    p_gnss->pre_integration->gyr_w = get_param("gnss.b_omg_noise", 0.10);
    p_gnss->pre_integration->acc_n = get_param("gnss.acc_noise", 0.10);
    p_gnss->pre_integration->gyr_n = get_param("gnss.omg_noise", 0.10);
    p_gnss->p_assign->ddt_noise = get_param("gnss.ddt_noise", 0.10);
    p_gnss->p_assign->dt_noise = get_param("gnss.dt_noise", 0.10);
    p_gnss->p_assign->psr_dopp_noise = get_param("gnss.psr_dopp_noise", 0.1);
    p_gnss->p_assign->odo_noise = get_param("gnss.odo_noise", 0.1);
    p_gnss->p_assign->grav_noise = get_param("gnss.grav_noise", 0.1);
    p_gnss->p_assign->cp_noise = get_param("gnss.cp_noise", 0.1);
    p_gnss->p_assign->gnss_psr_std_threshold = get_param("gnss.gnss_psr_std_thres", 2.0);
    p_gnss->p_assign->gnss_dopp_std_threshold = get_param("gnss.gnss_dopp_std_thres", 2.0);
    p_gnss->p_assign->gnss_cp_std_threshold = get_param("gnss.gnss_cp_std_thres", 2.0);
    p_gnss->p_assign->gnss_cp_std_threshold /= 0.004;
    p_gnss->gnss_cp_time_threshold = get_param("gnss.gnss_cp_time_thres", 2.0);
    p_gnss->delete_thred = get_param("gnss.gtsam_variable_thres", 200);
    p_gnss->p_assign->marg_thred = get_param("gnss.gtsam_marg_variable_thres", 1);
    p_gnss->p_assign->outlier_thres = get_param("gnss.outlier_thres", 0.1);
    p_gnss->p_assign->outlier_thres_init = get_param("gnss.outlier_thres_init", 0.1);
    p_gnss->gnss_sample_period = get_param("gnss.gnss_sample_period", 0.1);
    nolidar = get_param("gnss.nolidar", false);
    p_gnss->p_assign->ephem_from_rinex = get_param("gnss.ephem_from_rinex", false);
    p_gnss->p_assign->obs_from_rinex = get_param("gnss.obs_from_rinex", false);
    p_gnss->p_assign->pvt_is_gt = get_param("gnss.pvt_is_gt", false);
    p_gnss->wind_size = get_param("gnss.window_size", 2);
    p_gnss->p_assign->initNoises();
  }
  else
  {
    rtk_pvt_topic = get_param("gnss.rtk_pvt_topic", std::string("/ublox_driver/receiver_pvt"));
  }
  NMEA_ENABLE = get_param("nmea.nmea_enable", false);
  cout << "nmea enable:" << NMEA_ENABLE << endl;
  if (NMEA_ENABLE)
  {
    p_nmea->p_assign->outlier_rej = get_param("gnss.outlier_rejection", false);
    p_nmea->nmea_weight = get_param("nmea.nmea_weight", 0.1);
    nmea_meas_topic = get_param("nmea.posit_odo_topic", std::string("/mavros/local_position/odom"));
    p_nmea->gravity_init << VEC_FROM_ARRAY(gravity);
    time_diff_nmea_local = get_param("nmea.nmea_local_time_diff", 0.0);
    p_nmea->p_assign->prior_noise = get_param("gnss.prior_noise", 0.010);
    p_nmea->p_assign->marg_noise = get_param("gnss.marg_noise", 0.010);
    p_nmea->pre_integration->acc_w = get_param("gnss.b_acc_noise", 0.10);
    p_nmea->pre_integration->gyr_w = get_param("gnss.b_omg_noise", 0.10);
    p_nmea->pre_integration->acc_n = get_param("gnss.acc_noise", 0.10);
    p_nmea->pre_integration->gyr_n = get_param("gnss.omg_noise", 0.10);
    p_nmea->p_assign->rot_noise = get_param("nmea.rot_noise", 1.0);
    p_nmea->p_assign->vel_noise = get_param("nmea.vel_noise", 1.0);
    p_nmea->p_assign->odo_noise = get_param("gnss.odo_noise", 0.1);
    p_nmea->p_assign->grav_noise = get_param("gnss.grav_noise", 0.1);
    p_nmea->p_assign->pos_noise = get_param("nmea.pos_noise", 0.1);
    p_nmea->delete_thred = get_param("gnss.gtsam_variable_thres", 200);
    p_nmea->p_assign->marg_thred = get_param("gnss.gtsam_marg_variable_thres", 1);
    p_nmea->p_assign->outlier_thres = get_param("gnss.outlier_thres", 0.1);
    p_nmea->p_assign->outlier_thres_init = get_param("gnss.outlier_thres_init", 0.1);
    p_nmea->nmea_sample_period = get_param("gnss.gnss_sample_period", 0.1);
    p_nmea->p_assign->ppp_std_threshold = get_param("nmea.ppp_std_thres", 20.0);
    nolidar = get_param("gnss.nolidar", false);
    p_nmea->wind_size = get_param("gnss.window_size", 2);
    p_nmea->p_assign->initNoises();
  }
#endif
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot) 
{
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

void open_file()
{
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
#ifndef LIGO_WITHOUT_GNSS
    if (GNSS_ENABLE)
    {
        fout_rtk.open(DEBUG_FILE_DIR("pos_rtk.txt"),ios::out);
        fout_rtk.setf(ios::fixed, ios::floatfield);
        fout_rtk.precision(6);
        fout_global.open(DEBUG_FILE_DIR("pos_est.txt"),ios::out);
        fout_global.setf(ios::fixed, ios::floatfield);
        fout_global.precision(6);
        fout_ppp.open(DEBUG_FILE_DIR("pos_ppp.txt"),ios::out);
        fout_ppp.setf(ios::fixed, ios::floatfield);
        fout_ppp.precision(6);
    }
#endif
    if (fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

}

void cout_state_to_file(Eigen::Vector3d &pos_lla)
{
#ifndef LIGO_WITHOUT_GNSS
    {
        Eigen::Vector3d pos_enu, pos_ecef;
        if (!nolidar)
        {
            Eigen::Vector3d pos_r = kf_output.x_.rot * p_gnss->Tex_imu_r + kf_output.x_.pos; // maybe improper.normalized()
            // Eigen::Vector3d truth_imu;
            // truth_imu << 0.0, 0.0, 0.14; // 0.0, 0.02, -0.43; // -0.16126, 0.35852, -0.30799; // deg // 
            // Eigen::Vector3d pos_r = kf_output.x_.rot * truth_imu + kf_output.x_.pos; // maybe improper.normalized()
            Eigen::Matrix3d enu_rot = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
            Eigen::Vector3d anc_cur = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
            pos_enu = p_gnss->local2enu(enu_rot, anc_cur, pos_r);
            pos_ecef = enu_rot * pos_r + anc_cur;
            local_poses.push_back(anc_cur);
            local_rots.push_back(enu_rot);
            pos_lla = ecef2geo(pos_ecef);
        }
        else
        {
            Eigen::Vector3d pos_r = kf_output.x_.rot * p_gnss->Tex_imu_r + kf_output.x_.pos; // .normalized()
            pos_enu = p_gnss->local2enu(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), pos_r);
            pos_lla = ecef2geo(pos_r);
        }
        // local_poses.push_back(kf_output.x_.pos);
        // local_rots.push_back(kf_output.x_.rot);
        est_poses.push_back(pos_enu);
        time_frame.push_back(time_predict_last_const);
    }
#endif
}

void cout_state_to_file_nmea()
{
#ifndef LIGO_WITHOUT_GNSS
    {
        Eigen::Vector3d pos_enu;
        if (!nolidar)
        {
            Eigen::Vector3d truth_imu;
            truth_imu << 0.0, 0.0, 0.14; // 0.0, 0.02, -0.43; // 
            // Eigen::Vector3d pos_r = kf_output.x_.rot * p_nmea->Tex_imu_r + kf_output.x_.pos; // maybe improper.normalized()
            Eigen::Vector3d pos_r = kf_output.x_.rot * truth_imu + kf_output.x_.pos; // maybe improper.normalized()
            Eigen::Matrix3d enu_rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
            Eigen::Vector3d anc_cur = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
            pos_enu = enu_rot * pos_r + anc_cur;
        }
        else
        {
            pos_enu = kf_output.x_.rot * p_nmea->Tex_imu_r + kf_output.x_.pos; // .normalized()
        }
        local_poses.push_back(kf_output.x_.pos);
        local_rots.push_back(kf_output.x_.rot);
        est_poses.push_back(pos_enu);
        time_frame.push_back(time_predict_last_const);
    }
#endif
}

void reset_cov_output(Eigen::Matrix<double, 24, 24> & P_init_output)
{
    P_init_output = MD(24, 24)::Identity() * 0.01;
    P_init_output.block<3, 3>(15, 15) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(18, 18) = MD(6,6)::Identity() * 0.001;
}
