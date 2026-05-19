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
#include "li_initialization.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <filesystem>
#include <rclcpp/exceptions.hpp>
#include <vector>

std::string map_folder = "/mnt/rms_maps";

namespace {
static std::string trim_ws(std::string s) {
  while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r' || s.back() == '\n')) s.pop_back();
  while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r' || s.front() == '\n')) s.erase(s.begin());
  return s;
}

static std::string resolve_map_folder_param(const std::string &raw)
{
  std::string s = trim_ws(raw);
  if (s.empty())
  {
    return std::string("/mnt/rms_maps");
  }
  namespace fs = std::filesystem;
  fs::path p(s);
  if (p.is_absolute())
  {
    std::error_code ec;
    const fs::path canon = fs::weakly_canonical(p, ec);
    return ec ? fs::absolute(p).lexically_normal().string() : canon.string();
  }
  std::string root(ROOT_DIR);
  if (!root.empty() && (root.back() == '/' || root.back() == '\\'))
  {
    root.pop_back();
  }
  const fs::path joined = (fs::path(root) / p).lexically_normal();
  std::error_code ec;
  const fs::path canon = fs::weakly_canonical(joined, ec);
  return ec ? joined.string() : canon.string();
}

/** If non-empty and relative: prefer package source ROOT_DIR/p when that directory exists, else share/ligo/p. */
static std::string resolve_indoor_grid_map_dir(const std::string& raw) {
  std::string s = trim_ws(raw);
  if (s.empty()) return s;
  namespace fs = std::filesystem;
  fs::path p(s);
  if (p.is_absolute()) return s;

  std::string root(ROOT_DIR);
  if (!root.empty() && (root.back() == '/' || root.back() == '\\')) root.pop_back();
  fs::path src_joined = (fs::path(root) / p).lexically_normal();
  std::error_code ec;
  if (fs::is_directory(src_joined, ec)) {
    fs::path canon = fs::weakly_canonical(src_joined, ec);
    return ec ? src_joined.string() : canon.string();
  }

  try {
    const std::string share = ament_index_cpp::get_package_share_directory("ligo");
    fs::path joined = fs::path(share) / p;
    fs::path canon = fs::weakly_canonical(joined, ec);
    return ec ? joined.lexically_normal().string() : canon.string();
  } catch (...) {
  }
  return src_joined.string();
}

static std::string sanitize_map_token(std::string raw, const std::string& fallback) {
  raw = trim_ws(raw);
  if (raw.empty()) {
    raw = fallback;
  }
  for (char &ch : raw) {
    if (ch == '/' || ch == '\\') {
      ch = '_';
    }
  }
  return raw;
}

static std::string resolve_indoor_map_group_dir(const std::string& map_name) {
  namespace fs = std::filesystem;
  const fs::path map_dir = fs::path(map_folder) / map_name;
  std::error_code ec;
  const fs::path canon = fs::weakly_canonical(map_dir, ec);
  return ec ? map_dir.string() : canon.string();
}

}  // namespace

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
std::vector<double> gravity_init, gravity;
std::vector<double> extrinT(3, 0.0), extrinT_gnss(3, 0.0);
std::vector<double> extrinR(9, 0.0), extrinR_gnss(9, 0.0);
std::vector<double> ppp_anc(3, 0.0);
bool   runtime_pos_log, log_lidar_frame_time_ms, path_en;
bool   scan_pub_en, scan_body_pub_en;
double pcd_save_grid2d_resolution_m = 0.2;
double pcd_save_downsample_voxel_m = 0.2;
std::string pcd_save_map_name = "map";
std::string pcd_save_sub_map_name = "sub_map";
bool pcd_tmp_map_enable = false;
double pcd_tmp_map_interval_sec = 1.0;
shared_ptr<Preprocess> p_pre;
// shared_ptr<LI_Init> Init_LI;
shared_ptr<ImuProcess> p_imu;
shared_ptr<NMEAProcess> p_nmea;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

std::string ppp_fname;
std::string nmea_meas_topic;
std::string nmea_input_type;
bool nmea_publish_stamp_diag = false;
std::string enu_position_topic = "/ligo/enu_position";
std::string enu_position_frame_id = "enu";
std::string enu_heading_topic = "/ligo/enu_heading_deg";
std::string global_position_topic = "/ligo/global_position";
std::string ecef_position_topic = "/ligo/ecef_position";
std::string ecef_position_frame_id = "ecef";
bool nmea_global_anchor_ready = false;
Eigen::Vector3d nmea_global_anchor_lla = Eigen::Vector3d::Zero();
bool nmea_last_raw_lla_valid = false;
Eigen::Vector3d nmea_last_raw_lla = Eigen::Vector3d::Zero();
bool nmea_use_fixed_anchor = false;
std::vector<double> nmea_fixed_anchor_lla_deg;
bool nmea_force_indoor_on_high_cov = true;
double nmea_indoor_high_cov_threshold = 50.0;
bool update_nmea = false;
bool time_diff_valid = false, is_first_nmea;
double last_nmea_time = -1;
double time_diff_nmea_local = 0.0;
double nmea_gps_latency = 0.0;
double lidar_time_inte = 0.1, first_imu_time = 0.0;
bool NMEA_ENABLE = true;
bool mapping_mode = false;
bool indoor_flag = false;
bool dyn_filter = false;
double dyn_filter_resolution = 1.0;
Eigen::Vector3d indoor_pos_enu_meas = Eigen::Vector3d::Zero();
Eigen::Quaterniond indoor_rot_enu_meas = Eigen::Quaterniond::Identity();
bool indoor_pose_valid = false;
double indoor_pose_time = 0.0;
gtsam::noiseModel::Base::shared_ptr indoorPoseNoise;
gtsam::noiseModel::Base::shared_ptr indoorPoseNoiseInit;
std::string indoor_map_pcd_path;
std::string indoor_grid_map_dir;
bool indoor_gicp_map_loaded = false;
Eigen::Isometry3d indoor_gicp_T_map_lidar = Eigen::Isometry3d::Identity();
bool indoor_flag_dynamic = false;
double indoor_gicp_max_factor_error = 250.0;
int    indoor_gicp_min_factor_inliers = 50;
double indoor_gicp_max_correspondence_m = 8.0;
double indoor_gicp_map_voxel_m = 0.5;
double indoor_gicp_scan_voxel_m = 0.5;
int    indoor_gicp_max_iterations_reg = 50;
bool indoor_gicp_align_reference_map_to_lio = true;
double indoor_gicp_factor_sqrt_info_scale = 1.0;
std::vector<Eigen::Vector3d> est_poses;
std::vector<Eigen::Vector3d> local_poses;
std::vector<Eigen::Matrix3d> local_rots;
std::vector<double> time_frame;

MeasureGroup Measures;

ofstream fout_out, fout_global, fout_ppp; 

void readParameters(rclcpp::Node * node)
{
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  p_nmea.reset(new NMEAProcess());

  auto get_param = [node](const std::string & name, auto default_val) -> decltype(default_val) {
    try {
      node->declare_parameter(name, default_val);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
      // Already declared (e.g. when both GNSS and NMEA read same param) — just get value
    }
    return node->get_parameter(name).get_value<decltype(default_val)>();
  };
  map_folder = resolve_map_folder_param(get_param("map_folder", std::string("")));
  RCLCPP_INFO(
      rclcpp::get_logger("ligo"),
      "[map] map_folder=%s",
      map_folder.c_str());
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
  mapping_mode = get_param("mapping.mapping_mode", false);
  cout << "mapping mode:" << mapping_mode << endl;
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
  log_lidar_frame_time_ms = get_param("mapping.log_lidar_frame_time_ms", false);
  pcd_save_interval = get_param("pcd_save.interval", -1);
  pcd_tmp_map_enable = get_param("pcd_save.tmp_map.enable", false);
  pcd_tmp_map_interval_sec = get_param("pcd_save.tmp_map.interval_sec", 1.0);
  if (pcd_tmp_map_interval_sec <= 0.0)
  {
    RCLCPP_WARN(
        rclcpp::get_logger("ligo"),
        "[tmp_map] invalid pcd_save.tmp_map.interval_sec=%.6f, fallback to 1.0",
        pcd_tmp_map_interval_sec);
    pcd_tmp_map_interval_sec = 1.0;
  }
  {
    pcd_save_map_name = sanitize_map_token(
        get_param("pcd_save.map_name", std::string("map")),
        "map");
    pcd_save_sub_map_name = sanitize_map_token(
        get_param("pcd_save.sub_map_name", std::string("sub_map")),
        "sub_map");
  }
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
  pcd_save_grid2d_resolution_m = get_param(
      "pcd_save.grid2d_resolution", static_cast<double>(ivox_options_.resolution_));
  if (pcd_save_grid2d_resolution_m <= 0.0)
    pcd_save_grid2d_resolution_m = static_cast<double>(ivox_options_.resolution_);
  pcd_save_downsample_voxel_m = get_param("pcd_save.downsample_voxel_m", 0.2);
  if (pcd_save_downsample_voxel_m < 0.0)
  {
    RCLCPP_WARN(
        rclcpp::get_logger("ligo"),
        "[pcd_save] invalid pcd_save.downsample_voxel_m=%.6f, fallback to 0.2",
        pcd_save_downsample_voxel_m);
    pcd_save_downsample_voxel_m = 0.2;
  }
  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
  time_diff_valid = true;
  ppp_fname = get_param("nmea.ppp_file_name", std::string("TST.pos"));
  NMEA_ENABLE = get_param("nmea.nmea_enable", false);
  cout << "nmea enable:" << NMEA_ENABLE << endl;
  indoor_flag = get_param("indoor.indoor_flag", false);
  if (mapping_mode)
  {
    indoor_flag = false;
  }
  cout << "indoor enable:" << indoor_flag << endl;
  if (NMEA_ENABLE)
  {
    p_nmea->p_assign->outlier_rej = get_param("gnss.outlier_rejection", false);
    {
      const double factor_scale = get_param("nmea.factor_sqrt_info_scale", -1.0);
      if (factor_scale >= 0.0)
      {
        p_nmea->nmea_weight = std::max(1e-9, factor_scale);
        RCLCPP_INFO(
            rclcpp::get_logger("ligo"),
            "nmea.factor_sqrt_info_scale: %.6f (NMEAFactor relative_sqrt_info; same role as indoor.gicp_factor_sqrt_info_scale)",
            p_nmea->nmea_weight);
      }
      else
      {
        p_nmea->nmea_weight = std::max(1e-9, get_param("nmea.nmea_weight", 0.1));
        RCLCPP_WARN(
            rclcpp::get_logger("ligo"),
            "nmea.factor_sqrt_info_scale unset or negative: using legacy nmea.nmea_weight=%.6f. "
            "Set nmea.factor_sqrt_info_scale (>=0) to match indoor.gicp_factor_sqrt_info_scale naming.",
            p_nmea->nmea_weight);
      }
    }
    nmea_meas_topic = get_param("nmea.posit_odo_topic", std::string("/mavros/local_position/odom"));
    nmea_input_type = get_param("nmea.nmea_input_type", std::string("odometry"));
    nmea_publish_stamp_diag = get_param("nmea.publish_stamp_diag", false);
    p_nmea->gravity_init << VEC_FROM_ARRAY(gravity);
    time_diff_nmea_local = get_param("nmea.nmea_local_time_diff", 0.0);
    nmea_gps_latency = get_param("nmea.nmea_gps_latency", 0.0);
    p_nmea->p_assign->prior_noise = get_param("gnss.prior_noise", 0.010);
    p_nmea->p_assign->marg_noise = get_param("gnss.marg_noise", 0.010);
    p_nmea->p_assign->rot_noise = get_param("nmea.rot_noise", 1.0);
    p_nmea->p_assign->vel_noise = get_param("nmea.vel_noise", 1.0);
    p_nmea->p_assign->odo_noise = get_param("gnss.odo_noise", 0.1);
    p_nmea->p_assign->grav_noise = get_param("gnss.grav_noise", 0.1);
    p_nmea->p_assign->pos_noise = get_param("nmea.pos_noise", 0.1);
    p_nmea->p_assign->pos_noise_z = get_param("nmea.pos_noise_z", p_nmea->p_assign->pos_noise);
    p_nmea->delete_thred = get_param("gnss.gtsam_variable_thres", 200);
    p_nmea->p_assign->marg_thred = get_param("gnss.gtsam_marg_variable_thres", 1);
    p_nmea->p_assign->outlier_thres = get_param("gnss.outlier_thres", 0.1);
    p_nmea->p_assign->outlier_thres_init = get_param("gnss.outlier_thres_init", 0.1);
    p_nmea->nmea_sample_period = get_param("gnss.gnss_sample_period", 0.1);
    p_nmea->p_assign->ppp_std_threshold = get_param("nmea.ppp_std_thres", 20.0);
    nmea_force_indoor_on_high_cov = get_param("nmea.force_indoor_on_high_cov", true);
    nmea_indoor_high_cov_threshold = get_param("nmea.indoor_high_cov_threshold", 50.0);
    const double legacy_lio_disp_m = get_param("nmea.init_min_lio_disp_m", 3.0);
    const double legacy_nmea_disp_m = get_param("nmea.init_min_nmea_disp_m", 3.0);
    p_nmea->init_min_lio_total_move_m = get_param("nmea.init_min_lio_total_move_m", legacy_lio_disp_m);
    p_nmea->init_min_nmea_total_move_m = get_param("nmea.init_min_nmea_total_move_m", legacy_nmea_disp_m);
    p_nmea->init_icp_max_iterations = get_param("nmea.init_icp_max_iterations", 80);
    p_nmea->init_icp_max_fitness = get_param("nmea.init_icp_max_fitness", 5.0);
    {
      const std::string lio_rot_mode = get_param("nmea.lio_align_rotation_mode", std::string("yaw_only"));
      p_nmea->lio_align_rotation_mode =
          (lio_rot_mode == "weighted_3d" || lio_rot_mode == "weighted3d") ? 1 : 0;
      p_nmea->lio_align_z_weight = get_param("nmea.lio_align_z_weight", 0.2);
      p_nmea->lio_align_max_tilt_deg = get_param("nmea.lio_align_max_tilt_deg", 25.0);
    }
    p_nmea->wind_size = get_param("gnss.window_size", 2);
    p_nmea->p_assign->initNoises();
    enu_position_topic = get_param("ligo.enu_position_topic", enu_position_topic);
    enu_position_frame_id = get_param("ligo.enu_position_frame_id", enu_position_frame_id);
    enu_heading_topic = get_param("ligo.enu_heading_topic", enu_heading_topic);
    global_position_topic = get_param("ligo.global_position_topic", global_position_topic);
    ecef_position_topic = get_param("ligo.ecef_position_topic", ecef_position_topic);
    ecef_position_frame_id = get_param("ligo.ecef_position_frame_id", ecef_position_frame_id);
    nmea_use_fixed_anchor = get_param("nmea.use_fixed_anchor", false);
    nmea_fixed_anchor_lla_deg = get_param("nmea.fixed_anchor_lla_deg", std::vector<double>());
    ligo_apply_fixed_nmea_anchor_if_configured();
  }

  // Indoor map assets + GICP/noise: always load from yaml even when indoor_flag is false
  // (e.g. mapping_mode forces indoor_flag off but /ligo/indoor_mode still needs grids under PCD/).
  {
    const bool indoor_outlier_rej = get_param("indoor.outlier_rejection", false);
    const double indoor_outlier_thres = get_param("indoor.outlier_thres", 0.1);
    const double indoor_outlier_thres_init = get_param("indoor.outlier_thres_init", 0.1);
    const double indoor_pos_noise = get_param("indoor.pos_noise", 0.1);
    const double indoor_pos_noise_z = get_param("indoor.pos_noise_z", indoor_pos_noise);
    const double indoor_rot_noise = get_param("indoor.rot_noise", 0.1);
    ligo::indoor::initIndoorPoseNoises(indoor_pos_noise, indoor_pos_noise_z, indoor_rot_noise,
                                       indoor_outlier_rej, indoor_outlier_thres, indoor_outlier_thres_init,
                                       indoorPoseNoise, indoorPoseNoiseInit);
    indoor_map_pcd_path = get_param("indoor.map_pcd_path", std::string(""));
    indoor_grid_map_dir = resolve_indoor_grid_map_dir(get_param("indoor.grid_map_dir", std::string("")));
    if (!mapping_mode) {
      if (!indoor_map_pcd_path.empty()) {
        RCLCPP_WARN(
            rclcpp::get_logger("ligo"),
            "[indoor/gicp] indoor.map_pcd_path is ignored in odometry mode; grid maps are loaded from <map_folder>/<map_name>/ "
            "(see map_folder in yaml).");
      }
      const std::string odom_map_name = sanitize_map_token(
          get_param("indoor.map_name_for_odometry", std::string("")),
          pcd_save_map_name);
      indoor_grid_map_dir = resolve_indoor_map_group_dir(odom_map_name);
      indoor_map_pcd_path.clear();
      RCLCPP_INFO(
          rclcpp::get_logger("ligo"),
          "[indoor/gicp] odometry map group: map_name=%s dir=%s",
          odom_map_name.c_str(),
          indoor_grid_map_dir.c_str());
    } else if (!indoor_grid_map_dir.empty()) {
      cout << "indoor.grid_map_dir (resolved): " << indoor_grid_map_dir << endl;
    }
    indoor_gicp_max_factor_error   = get_param("indoor.gicp_max_factor_error", 250.0);
    indoor_gicp_min_factor_inliers = get_param("indoor.gicp_min_factor_inliers", 50);
    indoor_gicp_max_correspondence_m = get_param("indoor.gicp_max_correspondence_m", 8.0);
    indoor_gicp_map_voxel_m   = get_param("indoor.gicp_map_voxel_m", 0.5);
    indoor_gicp_scan_voxel_m  = get_param("indoor.gicp_scan_voxel_m", 0.5);
    indoor_gicp_max_iterations_reg = get_param("indoor.gicp_max_iterations", 50);
    indoor_gicp_align_reference_map_to_lio =
        get_param("indoor.gicp_align_reference_map_to_lio", true);
    indoor_gicp_factor_sqrt_info_scale = get_param("indoor.gicp_factor_sqrt_info_scale", 1.0);
    if (indoor_gicp_factor_sqrt_info_scale < 1e-9) indoor_gicp_factor_sqrt_info_scale = 1e-9;
    cout << "indoor.gicp_factor_sqrt_info_scale: " << indoor_gicp_factor_sqrt_info_scale << endl;
  }
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
    cout << "~~~~ debug file logging disabled" << endl;
}

void cout_state_to_file_nmea()
{
    {
        Eigen::Vector3d pos_enu;
        if (p_nmea->icp_tf_ready)
        {
            pos_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.pos + p_nmea->icp_t_local_to_enu;
        }
        else
        {
            Eigen::Vector3d truth_imu;
            truth_imu << 0.0, 0.0, 0.14;
            Eigen::Vector3d pos_r = kf_output.x_.rot * truth_imu + kf_output.x_.pos;
            Eigen::Matrix3d enu_rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
            Eigen::Vector3d anc_cur = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
            pos_enu = enu_rot * pos_r + anc_cur;
        }
        local_poses.push_back(kf_output.x_.pos);
        local_rots.push_back(kf_output.x_.rot);
        est_poses.push_back(pos_enu);
        time_frame.push_back(time_predict_last_const);
    }
}

bool compute_fused_imu_position_enu(Eigen::Vector3d &pos_enu)
{
    if (!NMEA_ENABLE || !p_nmea)
        return false;
    // Same map-ENU as /aft_mapped_to_init during outdoor re-align gap (nmea_ready false, old ICP retained).
    if (p_nmea->icp_tf_ready)
    {
        pos_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.pos + p_nmea->icp_t_local_to_enu;
        return true;
    }
    if (!p_nmea->nmea_ready)
        return false;
    // ICP not ready but graph active: GTSAM E(0)/P(0) (can diverge from published LIO odom).
    {
        Eigen::Vector3d truth_imu;
        truth_imu << 0.0, 0.0, 0.14;
        Eigen::Vector3d pos_r = kf_output.x_.rot * truth_imu + kf_output.x_.pos;
        Eigen::Matrix3d enu_rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
        Eigen::Vector3d anc_cur = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
        pos_enu = enu_rot * pos_r + anc_cur;
    }
    return true;
}

bool compute_fused_imu_position_geo(Eigen::Vector3d &out_lla)
{
    Eigen::Vector3d p_enu;
    if (!compute_fused_imu_position_enu(p_enu))
        return false;
    if (!nmea_global_anchor_ready) {
        return false;
    }
    const Eigen::Vector3d anchor_ecef = gnss_comm::geo2ecef(nmea_global_anchor_lla);
    const Eigen::Matrix3d R_ecef_enu = gnss_comm::geo2rotation(nmea_global_anchor_lla);
    const Eigen::Vector3d p_ecef = anchor_ecef + R_ecef_enu * p_enu;
    out_lla = gnss_comm::ecef2geo(p_ecef);
    return true;
}

bool compute_fused_imu_position_ecef(Eigen::Vector3d &out_ecef)
{
    Eigen::Vector3d p_enu;
    if (!compute_fused_imu_position_enu(p_enu))
        return false;
    if (!nmea_global_anchor_ready) {
        return false;
    }
    const Eigen::Vector3d anchor_ecef = gnss_comm::geo2ecef(nmea_global_anchor_lla);
    const Eigen::Matrix3d R_ecef_enu = gnss_comm::geo2rotation(nmea_global_anchor_lla);
    out_ecef = anchor_ecef + R_ecef_enu * p_enu;
    return true;
}

namespace {
/** Raw GNSS ENU (not fused LIO): NavSatFix→ENU if anchor ready, else last Odometry ENU sample. */
static bool compute_prealign_gnss_enu(Eigen::Vector3d &pos_enu)
{
    if (nmea_last_raw_lla_valid && nmea_global_anchor_ready)
    {
        const Eigen::Vector3d p_ecef = gnss_comm::geo2ecef(nmea_last_raw_lla);
        const Eigen::Vector3d anchor_ecef = gnss_comm::geo2ecef(nmea_global_anchor_lla);
        const Eigen::Matrix3d R_ecef_enu = gnss_comm::geo2rotation(nmea_global_anchor_lla);
        pos_enu = R_ecef_enu.transpose() * (p_ecef - anchor_ecef);
        return true;
    }
    if (nmea_cur)
    {
        pos_enu << nmea_cur->pose.pose.position.x, nmea_cur->pose.pose.position.y, nmea_cur->pose.pose.position.z;
        return true;
    }
    return false;
}
}  // namespace

bool compute_ligo_global_topic_enu(Eigen::Vector3d &pos_enu)
{
    if (!NMEA_ENABLE || !p_nmea)
        return false;
    if (compute_fused_imu_position_enu(pos_enu))
        return true;
    return compute_prealign_gnss_enu(pos_enu);
}

bool compute_ligo_global_topic_geo(Eigen::Vector3d &out_lla)
{
    if (!NMEA_ENABLE || !p_nmea)
        return false;
    if (compute_fused_imu_position_geo(out_lla))
        return true;
    if (nmea_last_raw_lla_valid)
    {
        out_lla = nmea_last_raw_lla;
        return true;
    }
    Eigen::Vector3d p_enu;
    if (!compute_prealign_gnss_enu(p_enu))
        return false;
    if (!nmea_global_anchor_ready)
        return false;
    const Eigen::Vector3d anchor_ecef = gnss_comm::geo2ecef(nmea_global_anchor_lla);
    const Eigen::Matrix3d R_ecef_enu = gnss_comm::geo2rotation(nmea_global_anchor_lla);
    const Eigen::Vector3d p_ecef = anchor_ecef + R_ecef_enu * p_enu;
    out_lla = gnss_comm::ecef2geo(p_ecef);
    return true;
}

bool compute_ligo_global_topic_ecef(Eigen::Vector3d &out_ecef)
{
    if (!NMEA_ENABLE || !p_nmea)
        return false;
    if (compute_fused_imu_position_ecef(out_ecef))
        return true;
    if (nmea_last_raw_lla_valid)
    {
        out_ecef = gnss_comm::geo2ecef(nmea_last_raw_lla);
        return true;
    }
    Eigen::Vector3d p_enu;
    if (!compute_prealign_gnss_enu(p_enu))
        return false;
    if (!nmea_global_anchor_ready)
        return false;
    const Eigen::Vector3d anchor_ecef = gnss_comm::geo2ecef(nmea_global_anchor_lla);
    const Eigen::Matrix3d R_ecef_enu = gnss_comm::geo2rotation(nmea_global_anchor_lla);
    out_ecef = anchor_ecef + R_ecef_enu * p_enu;
    return true;
}

void reset_cov_output(Eigen::Matrix<double, 24, 24> & P_init_output)
{
    P_init_output = MD(24, 24)::Identity() * 0.01;
    P_init_output.block<3, 3>(15, 15) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(18, 18) = MD(6,6)::Identity() * 0.001;
}
