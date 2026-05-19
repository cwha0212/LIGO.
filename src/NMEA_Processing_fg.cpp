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

#include "NMEA_Processing_fg.h"
#include <algorithm>
#include <cmath>
#include <limits>

#include "parameters.h"
#include <rclcpp/rclcpp.hpp>
#include <gtsam/linear/linearExceptions.h>
#include <chrono>

namespace {
/** NMEA 전역 위치: 고도 미사용(0 m). xy만 관측으로 사용한다. */
static Eigen::Vector3d nmea_position_meas_xy_alt0(const nav_msgs::msg::Odometry &odom)
{
  return Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0);
}
}  // namespace

NMEAProcess::NMEAProcess()
{
  Reset();
  // initNoises();
}

NMEAProcess::~NMEAProcess() {}

bool NMEAProcess::graphAnchorEnu(Eigen::Vector3d &out) const
{
  if (nmea_ready && p_assign->isamCurrentEstimate.exists(E(0)))
  {
    const gtsam::Vector3 e = p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
    out = Eigen::Vector3d(e(0), e(1), e(2));
    return true;
  }
  if (icp_tf_ready)
  {
    out = anc_enu;
    return true;
  }
  return false;
}

void NMEAProcess::Reset() 
{
  RCLCPP_WARN(rclcpp::get_logger("ligo"), "Reset NMEAProcess");
  p_assign->change_ext = 1;
  p_assign->gtSAMgraph.resize(0); 
  p_assign->initialEstimate.clear();
  p_assign->isamCurrentEstimate.clear();
  frame_delete = 0;
  nmea_meas_.resize(WINDOW_SIZE+1);
  p_assign->factor_id_frame.clear();
  id_accumulate = 0;
  frame_num = 0;
  last_nmea_time = 0.0;
  frame_count = 0;
  invalid_lidar = false;
  Rot_nmea_init.setIdentity();
  icp_R_local_to_enu.setIdentity();
  icp_t_local_to_enu.setZero();
  icp_tf_ready = false;
  sum_nmea_lio_err_sq_xy = 0.0;
  n_nmea_fusion_count = 0;
  diag_03m_valid = false;
  icp_pairs_lio.clear();
  icp_pairs_nmea_local.clear();
  heading_align_source = 0;
  heading_align_post_rmse_m = std::numeric_limits<double>::quiet_NaN();
  init_start_set = false;
  init_start_lio.setZero();
  init_start_nmea.setZero();
  p_assign->process_feat_num = 0;
  nmea_ready = false;
  init_pos_buf.clear();
  init_rot_buf.clear();
  init_vel_buf.clear();
  init_nmea_buf.clear();
  init_lio_time_buf.clear();

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 5; // may matter? improtant!
  p_assign->isam = gtsam::ISAM2(parameters);
}

void NMEAProcess::ResetGraphClearingInitRetainIcp()
{
  RCLCPP_WARN(rclcpp::get_logger("ligo"),
              "Reset NMEA graph for outdoor re-align (retain ICP local→ENU until new alignment)");
  p_assign->change_ext = 1;
  p_assign->gtSAMgraph.resize(0);
  p_assign->initialEstimate.clear();
  p_assign->isamCurrentEstimate.clear();
  frame_delete = 0;
  nmea_meas_.resize(WINDOW_SIZE + 1);
  p_assign->factor_id_frame.clear();
  id_accumulate = 0;
  frame_num = 0;
  last_nmea_time = 0.0;
  frame_count = 0;
  invalid_lidar = false;
  // Keep: icp_R_local_to_enu, icp_t_local_to_enu, icp_tf_ready, anc_enu, anc_local, Rot_nmea_init, yaw_enu_local
  sum_nmea_lio_err_sq_xy = 0.0;
  n_nmea_fusion_count = 0;
  diag_03m_valid = false;
  icp_pairs_lio.clear();
  icp_pairs_nmea_local.clear();
  init_start_set = false;
  init_start_lio.setZero();
  init_start_nmea.setZero();
  p_assign->process_feat_num = 0;
  nmea_ready = false;
  init_pos_buf.clear();
  init_rot_buf.clear();
  init_vel_buf.clear();
  init_nmea_buf.clear();
  init_lio_time_buf.clear();

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 5;
  p_assign->isam = gtsam::ISAM2(parameters);
}

void NMEAProcess::processNMEA(const nav_msgs::msg::Odometry::SharedPtr &nmea_meas, state_output &state)
{
  if (!nmea_ready)
  {
    // Horizontal position diagonal [0],[7] only (lat/lon variance); altitude [14] excluded vs ppp_std_threshold.
    if (nmea_meas->pose.covariance[0] > p_assign->ppp_std_threshold ||
        nmea_meas->pose.covariance[7] > p_assign->ppp_std_threshold)
    {
      static int rej_cov_log_count = 0;
      if (++rej_cov_log_count <= 5 || rej_cov_log_count % 50 == 0)
      {
        RCLCPP_WARN(
            rclcpp::get_logger("ligo"),
            "[nmea/init] reject by covariance: cov_horiz=(%.3f, %.3f) th=%.3f",
            nmea_meas->pose.covariance[0], nmea_meas->pose.covariance[7],
            p_assign->ppp_std_threshold);
      }
      return;
    }
    {
      init_rot_buf.push_back(state.rot);
      init_pos_buf.push_back(state.pos + state.rot * Tex_imu_r);
      Eigen::Matrix3d omg_skew;
      omg_skew << SKEW_SYM_MATRX(state.omg);
      init_vel_buf.push_back(state.vel + state.rot * omg_skew * Tex_imu_r);
      init_nmea_buf.push_back(nmea_meas);
      init_lio_time_buf.push_back(rclcpp::Time(nmea_meas->header.stamp).seconds());
      frame_count = static_cast<int>(init_pos_buf.size());
    }
    if (!init_start_set)
    {
      init_start_set = true;
      init_start_lio = init_pos_buf.back();
      init_start_nmea = nmea_position_meas_xy_alt0(*nmea_meas);
      RCLCPP_INFO(
          rclcpp::get_logger("ligo"),
          "[nmea/init] start set: lio=(%.3f,%.3f,%.3f) nmea=(%.3f,%.3f,%.3f)",
          init_start_lio.x(), init_start_lio.y(), init_start_lio.z(),
          init_start_nmea.x(), init_start_nmea.y(), init_start_nmea.z());
    }
    frame_count ++;
    nmea_ready = NMEALIAlign();
    if (nmea_ready)
    {
      RCLCPP_INFO(rclcpp::get_logger("ligo"), "NMEA Initialization is done");
      state_const_ = state;
    }
  }
  else
  {  
    nmea_meas_[0] = nmea_meas;
  }
}

void NMEAProcess::runISAM2opt(void) //
{
  gtsam::FactorIndices delete_factor;
  gtsam::FactorIndices().swap(delete_factor);
  // Temporary safety: disable NMEA marginalization to avoid underconstrained
  // states during factor deletion (e.g., IndeterminantLinearSystemException at r31).
  // This keeps graph growing, but prioritizes runtime stability.
  const bool disable_nmea_marginalization = true;

  try
  {
    if (nmea_ready)
    {
      bool delete_happen = false;
      if (!disable_nmea_marginalization && frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 4000)
      {
        delete_happen = true;
      while (frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 3000)
      { 
        if (!p_assign->factor_id_frame.empty())       
        {
          // if (frame_delete > 0)
          {
          for (size_t i = 0; i < p_assign->factor_id_frame[0].size(); i++)
          {
            // if (p_assign->factor_id_frame[0][i] != 0 && p_assign->factor_id_frame[0][i] != 1)
            {
              delete_factor.push_back(p_assign->factor_id_frame[0][i]);
            }
          }
          // index_delete += p_assign->factor_id_frame[0].size();
          }
        
          p_assign->factor_id_frame.pop_front();
          frame_delete ++;
        }
        if (p_assign->factor_id_frame.empty()) break;
      }
      }

      if (delete_happen)
      {
        p_assign->delete_variables(frame_delete, frame_num, id_accumulate, delete_factor);
      }
      else
      {
        p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
        p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
        p_assign->initialEstimate.clear();
        p_assign->isam.update();
      }
      p_assign->isamCurrentEstimate = p_assign->isam.calculateEstimate();
    }
    else
    {
      p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
      p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
      p_assign->initialEstimate.clear();
      p_assign->isam.update();
      p_assign->isamCurrentEstimate = p_assign->isam.calculateEstimate();
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(
        rclcpp::get_logger("ligo"),
        "[nmea/isam] exception in runISAM2opt: %s. Drop current NMEA factors and keep running.",
        e.what());
    // Do not call Reset() here: it sets nmea_ready=false and triggers full pipeline reset
    // (including IMU re-initialization) from laserMapping. Instead, drop only current
    // pending factors/initial values and keep last valid estimate.
    p_assign->gtSAMgraph.resize(0);
    p_assign->initialEstimate.clear();
    return;
  }
}

bool NMEAProcess::NMEALIAlign()
{
  const auto logger = rclcpp::get_logger("ligo");
  const int n = static_cast<int>(init_pos_buf.size());
  if (n < 2)
  {
    static int init_wait_count = 0;
    if (++init_wait_count <= 5 || init_wait_count % 50 == 0)
    {
      RCLCPP_INFO(
          logger,
          "[nmea/init] waiting data: n=%d (min 2)",
          n);
    }
    return false;
  }

  // Drop leading frames if time gap too large (IMU propagation limit).
  for (size_t i = 0; i + 1 < init_nmea_buf.size(); )
  {
    const double dt = rclcpp::Time(init_nmea_buf[i + 1]->header.stamp).seconds() - rclcpp::Time(init_nmea_buf[i]->header.stamp).seconds();
    if (dt > 15 * nmea_sample_period)
    {
      init_pos_buf.erase(init_pos_buf.begin());
      init_rot_buf.erase(init_rot_buf.begin());
      init_vel_buf.erase(init_vel_buf.begin());
      init_nmea_buf.erase(init_nmea_buf.begin());
      init_lio_time_buf.erase(init_lio_time_buf.begin());
      frame_count = static_cast<int>(init_pos_buf.size());
      continue;
    }
    ++i;
  }
  const int n_valid = static_cast<int>(init_pos_buf.size());
  if (n_valid < 2) return false;

  const Eigen::Vector3d lio_cur = init_pos_buf.back();
  const Eigen::Vector3d nmea_cur = nmea_position_meas_xy_alt0(*init_nmea_buf.back());
  const double lio_total_move = (lio_cur - init_start_lio).norm();
  const double nmea_total_move = (nmea_cur - init_start_nmea).norm();
  if (lio_total_move < init_min_lio_total_move_m || nmea_total_move < init_min_nmea_total_move_m)
  {
    return false;
  }

  // 1) 지연 추정: 처음 이동(0.3m) LIO 시각 vs GPS 시각 차이
  std::vector<double> lio_disp(n_valid), gps_disp(n_valid);
  for (int i = 0; i < n_valid; ++i)
  {
    lio_disp[i] = (init_pos_buf[i] - init_start_lio).norm();
    const Eigen::Vector3d gv = nmea_position_meas_xy_alt0(*init_nmea_buf[i]);
    gps_disp[i] = (gv - init_start_nmea).norm();
  }
  int first_lio_03 = -1, first_gps_03 = -1;
  constexpr double THRESH_03 = 0.3;
  for (int i = 0; i < n_valid; ++i)
  {
    if (first_lio_03 < 0 && lio_disp[i] >= THRESH_03) first_lio_03 = i;
    if (first_gps_03 < 0 && gps_disp[i] >= THRESH_03) first_gps_03 = i;
  }
  // latency 없음 가정: 항상 0 사용
  double latency_est = 0.0;
  nmea_gps_latency_estimated = 0.0;
  if (first_lio_03 >= 0 && first_gps_03 >= 0)
  {
    const double t_lio = init_lio_time_buf[first_lio_03];
    const double t_gps = rclcpp::Time(init_nmea_buf[first_gps_03]->header.stamp).seconds();
    RCLCPP_INFO(logger, "[nmea/init] first_move: lio_0.3m=idx%d(t=%.1f) gps_0.3m=idx%d(t=%.1f) latency=0(assumed)",
                first_lio_03, t_lio, first_gps_03, t_gps);
  }
  // 0.3m 시점 진단: t_lio에 LIO vs GPS 비교 (latency 유무 확인용)
  if (first_lio_03 >= 0)
  {
    auto get_nmea_at_time = [&](double t_want) -> Eigen::Vector3d {
      if (n_valid < 1) return Eigen::Vector3d::Zero();
      if (t_want <= rclcpp::Time(init_nmea_buf.front()->header.stamp).seconds())
        return nmea_position_meas_xy_alt0(*init_nmea_buf.front());
      if (t_want >= rclcpp::Time(init_nmea_buf.back()->header.stamp).seconds())
        return nmea_position_meas_xy_alt0(*init_nmea_buf.back());
      for (int j = 0; j + 1 < n_valid; ++j)
      {
        const double t0 = rclcpp::Time(init_nmea_buf[j]->header.stamp).seconds();
        const double t1 = rclcpp::Time(init_nmea_buf[j + 1]->header.stamp).seconds();
        if (t0 <= t_want && t_want <= t1)
        {
          const double alpha = (t1 - t0) > 1e-9 ? (t_want - t0) / (t1 - t0) : 0.0;
          const Eigen::Vector3d p0 = nmea_position_meas_xy_alt0(*init_nmea_buf[j]);
          const Eigen::Vector3d p1 = nmea_position_meas_xy_alt0(*init_nmea_buf[j + 1]);
          return (1.0 - alpha) * p0 + alpha * p1;
        }
      }
      return nmea_position_meas_xy_alt0(*init_nmea_buf.back());
    };
    const double t_lio = init_lio_time_buf[first_lio_03];
    diag_03m_lio_pos = init_pos_buf[first_lio_03];
    diag_03m_gps_at_t_lio = get_nmea_at_time(t_lio);
    diag_03m_lio_disp = (diag_03m_lio_pos - init_start_lio).norm();
    diag_03m_gps_disp_at_t_lio = (diag_03m_gps_at_t_lio - init_start_nmea).norm();
    diag_03m_latency_s = latency_est;
    diag_03m_t_lio = t_lio;
    diag_03m_t_gps = first_gps_03 >= 0 ? rclcpp::Time(init_nmea_buf[first_gps_03]->header.stamp).seconds() : 0.0;
    diag_03m_valid = true;
    RCLCPP_INFO(logger, "[nmea/03m] t_lio=%.3fs lio_disp=%.3fm gps_disp_at_t_lio=%.3fm latency=%.3fs",
                t_lio, diag_03m_lio_disp, diag_03m_gps_disp_at_t_lio, diag_03m_latency_s);
  }

  // 2) 0.3m 이후: 보정된 시각(T-L)으로 비슷한 시간대 pair. GPS stamp T = 시각 T-L의 위치 → LIO도 T-L로 보간
  auto get_lio_at_time = [&](double t_want) -> Eigen::Vector3d {
    if (t_want <= init_lio_time_buf.front()) return init_pos_buf.front();
    if (t_want >= init_lio_time_buf.back()) return init_pos_buf.back();
    for (int j = 0; j + 1 < n_valid; ++j)
    {
      const double t0 = init_lio_time_buf[j], t1 = init_lio_time_buf[j + 1];
      if (t0 <= t_want && t_want <= t1)
      {
        const double alpha = (t1 - t0) > 1e-9 ? (t_want - t0) / (t1 - t0) : 0.0;
        return (1.0 - alpha) * init_pos_buf[j] + alpha * init_pos_buf[j + 1];
      }
    }
    return init_pos_buf.back();
  };
  auto get_nmea_pos = [&](int i) -> Eigen::Vector3d {
    return nmea_position_meas_xy_alt0(*init_nmea_buf[i]);
  };
  // 0.3m 이후 pair만: 보정된 시각(stamp-L)으로 LIO 보간 → 비슷한 시간대 (LIO(T-L), NMEA(stamp T))
  auto get_lio_corrected = [&](int i) -> Eigen::Vector3d {
    if (latency_est > 0.01)
    {
      const double s_i = rclcpp::Time(init_nmea_buf[i]->header.stamp).seconds();
      return get_lio_at_time(s_i - latency_est);
    }
    return init_pos_buf[i];
  };
  auto pair_usable = [&](int i) -> bool {
    return lio_disp[i] >= THRESH_03 && gps_disp[i] >= THRESH_03;
  };
  int n_used = 0;
  Eigen::Vector2d mu_lio = Eigen::Vector2d::Zero();
  Eigen::Vector2d mu_nmea = Eigen::Vector2d::Zero();
  double mu_lio_z = 0.0;
  double mu_nmea_z = 0.0;
  for (int i = 0; i < n_valid; ++i)
  {
    if (!pair_usable(i)) continue;
    const Eigen::Vector3d pl = get_lio_corrected(i);  // LIO at T-L (보정된 시각)
    const Eigen::Vector3d pn = get_nmea_pos(i);       // NMEA at stamp T (= 시각 T-L의 위치)
    mu_lio.x() += pl(0); mu_lio.y() += pl(1); mu_lio_z += pl(2);
    mu_nmea.x() += pn.x(); mu_nmea.y() += pn.y(); mu_nmea_z += pn.z();
    n_used++;
  }
  if (n_used < 2)
  {
    RCLCPP_WARN(logger, "[nmea/init] too few 0.3m+ pairs: n_used=%d", n_used);
    return false;
  }
  mu_lio /= n_used; mu_nmea /= n_used; mu_lio_z /= n_used; mu_nmea_z /= n_used;

  const Eigen::Vector3d mu_lio3(mu_lio.x(), mu_lio.y(), mu_lio_z);
  const Eigen::Vector3d mu_nmea3(mu_nmea.x(), mu_nmea.y(), mu_nmea_z);

  Eigen::Matrix3d R_align = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_align = Eigen::Vector3d::Zero();

  if (lio_align_rotation_mode != 1)
  {
    Eigen::Matrix2d sigma = Eigen::Matrix2d::Zero();
    for (int i = 0; i < n_valid; ++i)
    {
      if (!pair_usable(i)) continue;
      const Eigen::Vector3d pl3 = get_lio_corrected(i);
      const Eigen::Vector2d pl(pl3(0), pl3(1));
      const Eigen::Vector3d pn3 = get_nmea_pos(i);
      const Eigen::Vector2d pn(pn3.x(), pn3.y());
      sigma += (pn - mu_nmea) * (pl - mu_lio).transpose();
    }
    sigma /= n_used;

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d R2 = svd.matrixU() * svd.matrixV().transpose();
    if (R2.determinant() < 0.0)
    {
      Eigen::Matrix2d U = svd.matrixU();
      U.col(1) *= -1.0;
      R2 = U * svd.matrixV().transpose();
    }
    const Eigen::Vector2d t2 = mu_nmea - R2 * mu_lio;
    const double tz = mu_nmea_z - mu_lio_z;
    R_align.setIdentity();
    R_align.block<2, 2>(0, 0) = R2;
    t_align << t2.x(), t2.y(), tz;
  }
  else
  {
    const double pi = std::acos(-1.0);
    const double wz = std::max(1e-6, std::min(1.0, lio_align_z_weight));
    const double sz = std::sqrt(wz);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (int i = 0; i < n_valid; ++i)
    {
      if (!pair_usable(i)) continue;
      const Eigen::Vector3d pl3 = get_lio_corrected(i) - mu_lio3;
      const Eigen::Vector3d pn3 = get_nmea_pos(i) - mu_nmea3;
      const Eigen::Vector3d pls(pl3.x(), pl3.y(), sz * pl3.z());
      const Eigen::Vector3d pns(pn3.x(), pn3.y(), sz * pn3.z());
      H += pns * pls.transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd3(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d Rk = svd3.matrixU() * svd3.matrixV().transpose();
    if (Rk.determinant() < 0.0)
    {
      Eigen::Matrix3d U3 = svd3.matrixU();
      U3.col(2) *= -1.0;
      Rk = U3 * svd3.matrixV().transpose();
    }
    const double roll = std::atan2(Rk(2, 1), Rk(2, 2));
    const double cp = std::sqrt(Rk(0, 0) * Rk(0, 0) + Rk(1, 0) * Rk(1, 0));
    const double pitch = std::atan2(-Rk(2, 0), cp);
    const double max_t = lio_align_max_tilt_deg * pi / 180.0;
    if (std::abs(roll) > max_t || std::abs(pitch) > max_t)
    {
      const double yaw = std::atan2(Rk(1, 0), Rk(0, 0));
      R_align = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      RCLCPP_WARN(
          logger,
          "[nmea/init] weighted_3d: |roll|=%.2fdeg |pitch|=%.2fdeg exceed max_tilt_deg=%.1f → yaw-only yaw=%.2fdeg",
          roll * 180.0 / pi,
          pitch * 180.0 / pi,
          lio_align_max_tilt_deg,
          yaw * 180.0 / pi);
    }
    else
    {
      R_align = Rk;
    }
    t_align = mu_nmea3 - R_align * mu_lio3;
    RCLCPP_INFO(
        logger,
        "[nmea/init] weighted_3d Kabsch: z_weight=%.3f kabsch_roll=%.2fdeg kabsch_pitch=%.2fdeg kabsch_yaw=%.2fdeg",
        wz,
        roll * 180.0 / pi,
        pitch * 180.0 / pi,
        std::atan2(Rk(1, 0), Rk(0, 0)) * 180.0 / pi);
  }

  Eigen::Matrix4d sim_trans = Eigen::Matrix4d::Identity();
  sim_trans.block<3, 3>(0, 0) = R_align;
  sim_trans.block<3, 1>(0, 3) = t_align;

  auto rmse_after_transform = [&](const Eigen::Matrix4d &tf) -> double {
    double acc = 0.0;
    int cnt = 0;
    for (int i = 0; i < n_valid; ++i)
    {
      if (!pair_usable(i)) continue;
      const Eigen::Vector3d pl = get_lio_corrected(i);
      const Eigen::Vector4d p_in(pl(0), pl(1), pl(2), 1.0);
      const Eigen::Vector4d p_tf = tf * p_in;
      const Eigen::Vector3d pn = get_nmea_pos(i);
      acc += (p_tf.x() - pn.x()) * (p_tf.x() - pn.x()) + (p_tf.y() - pn.y()) * (p_tf.y() - pn.y()) + (p_tf.z() - pn.z()) * (p_tf.z() - pn.z());
      cnt++;
    }
    return cnt > 0 ? std::sqrt(acc / cnt) : 1e30;
  };

  const double pre_rmse = rmse_after_transform(Eigen::Matrix4d::Identity());
  const double post_rmse = rmse_after_transform(sim_trans);
  const bool align_ok = std::isfinite(post_rmse);
  const double yaw_deg = std::atan2(sim_trans(1, 0), sim_trans(0, 0)) * 180.0 / std::acos(-1.0);
  if (!align_ok || post_rmse > init_icp_max_fitness)
  {
    RCLCPP_WARN(
        logger,
        "[nmea/init] TIME-PAIR ALIGN rejected: ok=%d post_rmse=%.3f (max %.3f) pre_rmse=%.3f lio_total=%.3f nmea_total=%.3f",
        align_ok ? 1 : 0, post_rmse, init_icp_max_fitness, pre_rmse, lio_total_move, nmea_total_move);
    return false;
  }
  heading_align_source = 1;  // SOURCE_TIME_PAIR_ALIGN
  heading_align_post_rmse_m = post_rmse;
  RCLCPP_INFO(
      logger,
      "[nmea/init] TIME-PAIR ALIGN accepted: n=%d pre_rmse=%.3f post_rmse=%.3f improve=%.2fx yaw=%.2fdeg t=(%.3f,%.3f,%.3f) lio_total=%.3f nmea_total=%.3f latency_corrected=%s",
      n_valid,
      pre_rmse,
      post_rmse,
      (post_rmse > 1e-9) ? (pre_rmse / post_rmse) : 0.0,
      yaw_deg,
      sim_trans(0, 3),
      sim_trans(1, 3),
      sim_trans(2, 3),
      lio_total_move,
      nmea_total_move,
      "time_comp_0.3m+");
  anc_enu = sim_trans.block<3, 1>(0, 3);
  anc_local = init_pos_buf.back();
  Rot_nmea_init = R_align;
  yaw_enu_local = std::atan2(R_align(1, 0), R_align(0, 0));
  icp_R_local_to_enu = Rot_nmea_init;
  icp_t_local_to_enu = sim_trans.block<3, 1>(0, 3);
  icp_tf_ready = true;
  // Store alignment pairs for RViz: 0.3m 이후만, 보정된 시각(LIO at T-L) 적용
  icp_pairs_lio.clear();
  icp_pairs_nmea_local.clear();
  const Eigen::Matrix3d R = icp_R_local_to_enu;
  const Eigen::Vector3d t = icp_t_local_to_enu;
  for (int i = 0; i < n_valid; ++i)
  {
    if (!pair_usable(i)) continue;
    icp_pairs_lio.push_back(get_lio_corrected(i));
    icp_pairs_nmea_local.push_back(R.transpose() * (get_nmea_pos(i) - t));
  }
  // Prepare for Evaluate: nmea_meas_[0], pos_window, rot_window, vel_window.
  nmea_meas_.resize(1);
  nmea_meas_[0] = init_nmea_buf.back();
  pos_window[wind_size] = init_pos_buf.back();
  rot_window[wind_size] = init_rot_buf.back();
  vel_window[wind_size] = init_vel_buf.back();
  SetInit();
  frame_num = 1;
  last_nmea_time = rclcpp::Time(init_nmea_buf.back()->header.stamp).seconds();
  RCLCPP_INFO(logger, "[nmea/init] estimated_gps_latency=%.3fs (use for fusion)", latency_est);
  runISAM2opt();
  return true;
}

bool NMEAProcess::Evaluate(state_output &state)
{
  // After /ligo/indoor_mode: use GICP (IndoorLocalizationFactor) for global ENU — do not reject this step on GNSS covariance.
  const bool indoor_gicp_replaces_nmea = indoor_flag_dynamic;
  // Horizontal diagonal [0],[7] only vs ppp_std_threshold; altitude [14] excluded.
  if (!indoor_gicp_replaces_nmea &&
      (nmea_meas_[0]->pose.covariance[0] > p_assign->ppp_std_threshold ||
       nmea_meas_[0]->pose.covariance[7] > p_assign->ppp_std_threshold))
  {
    return false;
  }
  double time_current = rclcpp::Time(nmea_meas_[0]->header.stamp).seconds();
  double delta_t = time_current - last_nmea_time;

  Eigen::Matrix3d rot = state.rot;
  Eigen::Vector3d pos = state.pos;
  Eigen::Vector3d vel = state.vel;
  Eigen::Vector3d ba  = state.ba;
  Eigen::Vector3d bg  = state.bg;
  Eigen::Vector3d acc = state.acc;
  Eigen::Vector3d omg = state.omg;

  {
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector_imu;
    Eigen::Matrix<double, 12, 1> init_others_vector_imu;
    init_vel_bias_vector_imu.block<3,1>(0,0) = state.pos;
    init_vel_bias_vector_imu.block<3,1>(3,0) = state.vel;
    init_others_vector_imu.block<3,1>(0,0) = state.omg;
    init_others_vector_imu.block<3,1>(3,0) = state.acc;
    init_others_vector_imu.block<3,1>(6,0) = state.bg;
    init_others_vector_imu.block<3,1>(9,0) = state.ba;
    p_assign->initialEstimate.insert(A(frame_num), gtsam::Vector6(init_vel_bias_vector_imu));
    p_assign->initialEstimate.insert(O(frame_num), gtsam::Vector12(init_others_vector_imu));
    p_assign->initialEstimate.insert(G(frame_num), gtsam::Vector3(state.gravity));
    p_assign->initialEstimate.insert(R(frame_num), gtsam::Rot3(state.rot));
  }

  if (AddFactor(state.gravity, delta_t, time_current, ba, bg, pos, vel, acc, omg, rot))
  {
    frame_num ++;
    runISAM2opt();
  }
  else
  {
    return false;
  }

  if (frame_num <= 0)
  {
    return false;
  }

  {
    if (!p_assign->isamCurrentEstimate.exists(R(frame_num-1)) ||
        !p_assign->isamCurrentEstimate.exists(A(frame_num-1)) ||
        !p_assign->isamCurrentEstimate.exists(P(0)))
    {
      return false;
    }
    state_const_.rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix();
    state_const_.pos = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(0);
    state_const_.vel = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(3);
    state.gravity = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix().transpose() * gravity_init;
  }
  last_nmea_time = time_current;
  return true;
}

bool NMEAProcess::AddFactor(Eigen::Vector3d state_gravity, double delta_t, double time_current,
                Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d omg, Eigen::Matrix3d rot)
{
  (void)delta_t; (void)time_current; (void)ba; (void)bg;
  invalid_lidar = false;
  bool weight_lid_zero = false;
  {
    invalid_lidar = nolidar_cur;
    double weight_lid = 1;
    if (p_assign->process_feat_num < 10)
    {
      weight_lid = 0;
      weight_lid_zero = true;
    }
    else
    {
      weight_lid = 2 * double(norm_vec_num) / double(p_assign->process_feat_num);
    }
    norm_vec_num = 0;
    p_assign->process_feat_num = 0;
    double weight_check = (sqrt_lidar(0, 0) + sqrt_lidar(1, 1) + sqrt_lidar(2, 2)
                          + sqrt_lidar(6, 6) + sqrt_lidar(7, 7) + sqrt_lidar(8, 8)) / 6;
    sqrt_lidar *= weight_lid / weight_check;
    for (size_t j = 0; j < 9; j++)
    {
      if (sqrt_lidar(j, j) < 0.50)
      {
        sqrt_lidar(j, j) = 0.5;
        invalid_lidar = true;
      }
    }
  }
  if (nolidar_cur) nolidar_cur = false;

  std::vector<size_t> factor_id_cur;
  M3D omg_skew;
  omg_skew << SKEW_SYM_MATRX(omg);
  Eigen::Vector3d hat_omg_T = omg_skew * Tex_imu_r;
  {
    // Keep per-frame gravity state G(frame_num) anchored even when lidar branch is active.
    // Runtime logs showed underconstrained g199 during marginalization update.
    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector3>(G(frame_num), gtsam::Vector3(state_gravity), p_assign->priorGravNoise));
    factor_id_cur.push_back(id_accumulate);
    id_accumulate += 1;
    if (!weight_lid_zero)
    {
      p_assign->gtSAMgraph.add(ligo::NmeaLioGravRelFactor(P(0), R(frame_num), A(frame_num), O(frame_num), G(frame_num), gravity_init, state_gravity, pos, vel, rot, ba, bg, acc, omg, sqrt_lidar, p_assign->odomNoise));
      factor_id_cur.push_back(id_accumulate);
      id_accumulate += 1;
    }
    else
    {
      p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector3>(G(frame_num), gtsam::Vector3(state_gravity), p_assign->priorGravNoise));
      factor_id_cur.push_back(id_accumulate);
      id_accumulate += 1;

      Eigen::Matrix<double, 6, 1> pv;
      pv.block<3,1>(0,0) = pos;
      pv.block<3,1>(3,0) = vel;
      p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector6>(A(frame_num), gtsam::Vector6(pv), p_assign->priorNoise));
      factor_id_cur.push_back(id_accumulate);
      id_accumulate += 1;

      p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Rot3>(R(frame_num), gtsam::Rot3(rot), p_assign->priorrotNoise));
      factor_id_cur.push_back(id_accumulate);
      id_accumulate += 1;

      Eigen::Matrix<double, 12, 1> oth;
      oth.block<3,1>(0,0) = omg;
      oth.block<3,1>(3,0) = acc;
      oth.block<3,1>(6,0) = bg;
      oth.block<3,1>(9,0) = ba;
      p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector12>(O(frame_num), gtsam::Vector12(oth), p_assign->priorBiasNoise));
      factor_id_cur.push_back(id_accumulate);
      id_accumulate += 1;
    }
  }
  // Stabilize rotation DOF: always add a weak prior on R(frame_num)
  // to avoid occasional underconstrained rotation states (e.g., r31).
  {
    static gtsam::noiseModel::Base::shared_ptr weak_rot_prior = []() {
      gtsam::Vector v(3);
      v << 1e4, 1e4, 1e4;
      return gtsam::noiseModel::Diagonal::Variances(v);
    }();
    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Rot3>(
        R(frame_num), gtsam::Rot3(rot), weak_rot_prior));
    factor_id_cur.push_back(id_accumulate);
    id_accumulate += 1;
  }
  {
    const bool indoor_gicp_replaces_nmea = indoor_flag_dynamic;
    if (!indoor_gicp_replaces_nmea)
    {
      const bool nmea_navsatfix_pos_only = (nmea_input_type == "navsatfix");
      const Eigen::Vector3d pos_meas = nmea_position_meas_xy_alt0(*nmea_meas_[0]);
      const Eigen::Vector3d vel_meas(
          nmea_meas_[0]->twist.twist.linear.x,
          nmea_meas_[0]->twist.twist.linear.y,
          nmea_meas_[0]->twist.twist.linear.z);
      const Eigen::Quaterniond q_meas(
          nmea_meas_[0]->pose.pose.orientation.w,
          nmea_meas_[0]->pose.pose.orientation.x,
          nmea_meas_[0]->pose.pose.orientation.y,
          nmea_meas_[0]->pose.pose.orientation.z);
      const Eigen::Matrix3d rot_meas = q_meas.normalized().toRotationMatrix();
      RCLCPP_INFO(rclcpp::get_logger("ligo"), "[NMEA FACTOR INPUT]");
      const auto& nmea_noise = (frame_num < delete_thred) ? p_assign->robustnmeaNoise_init : p_assign->robustnmeaNoise;
      p_assign->gtSAMgraph.add(ligo::NMEAFactor(P(0), E(0), A(frame_num), R(frame_num), invalid_lidar,
                                                Tex_imu_r, anc_local, pos_meas, vel_meas, rot_meas, nmea_weight,
                                                hat_omg_T, Rex_imu_r, nmea_noise, nmea_navsatfix_pos_only));
      if (nmea_navsatfix_pos_only && invalid_lidar)
      {
        p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Rot3>(R(frame_num), gtsam::Rot3(rot), p_assign->priorrotNoise));
        factor_id_cur.push_back(id_accumulate);
        id_accumulate += 1;
        if (frame_num > 0 && p_assign->isamCurrentEstimate.exists(R(frame_num - 1)))
        {
          const gtsam::Rot3 prev_rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num - 1));
          const gtsam::Rot3 cur_rot = gtsam::Rot3(rot);
          const gtsam::Rot3 rel_meas = prev_rot.between(cur_rot);
          p_assign->gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Rot3>(R(frame_num - 1), R(frame_num), rel_meas, p_assign->margrotNoise));
          factor_id_cur.push_back(id_accumulate);
          id_accumulate += 1;
        }
        else if (frame_num > 0)
        {
          static std::chrono::steady_clock::time_point s_last_between_skip_log{};
          const auto now = std::chrono::steady_clock::now();
          if (now - s_last_between_skip_log > std::chrono::seconds(2))
          {
            s_last_between_skip_log = now;
            RCLCPP_WARN(
                rclcpp::get_logger("ligo"),
                "[nmea] BetweenFactor R(%zu)->R(%zu) skipped: previous rotation not in ISAM (e.g. after graph recovery).",
                static_cast<size_t>(frame_num - 1), static_cast<size_t>(frame_num));
          }
        }
      }
      factor_id_cur.push_back(id_accumulate);
      id_accumulate += 1;
    }
    else
    {
      RCLCPP_DEBUG(
          rclcpp::get_logger("ligo"),
          "[indoor] skip NMEAFactor: using GICP IndoorLocalizationFactor (frame=%d)", frame_num);
    }
  }

  {
    p_assign->factor_id_frame.push_back(factor_id_cur);
    std::vector<size_t>().swap(factor_id_cur);
  }
  return true;
}

void NMEAProcess::SetInitFromLocalization(const Eigen::Vector3d &indoor_pos_enu,
                                          const Eigen::Matrix3d &indoor_rot_enu,
                                          const state_output &seed_state,
                                          double init_time_sec)
{
  // Rebuild NMEA graph state from indoor localization anchor while keeping the
  // existing gravity prior path intact in SetInit().
  Reset();

  anc_enu = indoor_pos_enu;
  anc_local = seed_state.rot * Tex_imu_r + seed_state.pos;
  Rot_nmea_init.setIdentity();
  yaw_enu_local = 0.0;

  // Keep ENU/local conversion consistent with existing factor geometry:
  // R_enu = R_enu_local * R_local * Rex  =>  R_enu_local = R_enu * (R_local * Rex)^T
  Eigen::Matrix3d r_enu_local = indoor_rot_enu * (seed_state.rot * Rex_imu_r).transpose();

  {
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector;
    Eigen::Matrix<double, 12, 1> init_others_vector;
    init_vel_bias_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero();
    init_vel_bias_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero();
    init_others_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero();
    init_others_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero();
    init_others_vector.block<3,1>(6,0) = Eigen::Vector3d::Zero();
    init_others_vector.block<3,1>(9,0) = Eigen::Vector3d::Zero();

    p_assign->initialEstimate.insert(P(0), gtsam::Rot3(Rot_nmea_init));
    p_assign->initialEstimate.insert(A(0), gtsam::Vector6(init_vel_bias_vector));
    p_assign->initialEstimate.insert(O(0), gtsam::Vector12(init_others_vector));
    p_assign->initialEstimate.insert(E(0), gtsam::Vector3(anc_enu[0], anc_enu[1], anc_enu[2]));
    p_assign->initialEstimate.insert(R(0), gtsam::Rot3(r_enu_local));
    p_assign->initialEstimate.insert(G(0), gtsam::Vector3(gravity_init));

    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Rot3>(P(0), gtsam::Rot3(gtsam::Rot3(Rot_nmea_init)), p_assign->priorextrotNoise));
    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector3>(E(0), gtsam::Vector3(anc_enu[0], anc_enu[1], anc_enu[2]),
                            p_assign->priorextposNoise));
    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Rot3>(R(0), gtsam::Rot3(r_enu_local), p_assign->priorrotNoise));
    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector6>(A(0), gtsam::Vector6(init_vel_bias_vector), p_assign->priorNoise));
    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector12>(O(0), gtsam::Vector12(init_others_vector), p_assign->priorBiasNoise));
    p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector3>(G(0), gtsam::Vector3(gravity_init), p_assign->priorGravNoise));
    p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1, 2, 3, 4, 5});
    id_accumulate += 6;
  }

  nmea_ready = true;
  frame_num = 1;
  frame_count = 0;
  last_nmea_time = init_time_sec;
  runISAM2opt();

  // Derive the LIO-world → ENU transform from the indoor localization anchor so that
  // downstream consumers (GICP, odometry ENU publish) work immediately without waiting
  // for a separate LIO-NMEA ICP alignment pass.
  // Transform: p_enu = R_local_to_enu * p_local + t_local_to_enu
  //   where anc_enu = R_local_to_enu * anc_local + t_local_to_enu
  //   => t = anc_enu - R * anc_local
  icp_R_local_to_enu = r_enu_local;
  icp_t_local_to_enu = anc_enu - r_enu_local * anc_local;
  icp_tf_ready = true;
  heading_align_source = 2;  // SOURCE_INDOOR_RELOC
  heading_align_post_rmse_m = std::numeric_limits<double>::quiet_NaN();
  RCLCPP_INFO(rclcpp::get_logger("ligo"),
              "[nmea/init] ICP tf set from indoor reloc: "
              "t_enu=(%.2f, %.2f, %.2f)",
              icp_t_local_to_enu.x(), icp_t_local_to_enu.y(), icp_t_local_to_enu.z());
}

void NMEAProcess::SetInit()
{
  Eigen::Matrix3d R_enu_local_;
  R_enu_local_.setIdentity();
  Eigen::Matrix<double, 6, 1> init_vel_bias_vector;
  Eigen::Matrix<double, 12, 1> init_others_vector;
  init_vel_bias_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero();
  init_vel_bias_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero();
  init_others_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero();
  init_others_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero();
  init_others_vector.block<3,1>(6,0) = Eigen::Vector3d::Zero();
  init_others_vector.block<3,1>(9,0) = Eigen::Vector3d::Zero();

  p_assign->initialEstimate.insert(P(0), gtsam::Rot3(Rot_nmea_init));
  p_assign->initialEstimate.insert(A(0), gtsam::Vector6(init_vel_bias_vector));
  p_assign->initialEstimate.insert(O(0), gtsam::Vector12(init_others_vector));
  p_assign->initialEstimate.insert(E(0), gtsam::Vector3(anc_enu[0], anc_enu[1], anc_enu[2]));
  p_assign->initialEstimate.insert(R(0), gtsam::Rot3(R_enu_local_));
  p_assign->initialEstimate.insert(G(0), gtsam::Vector3(gravity_init));

  gtsam::PriorFactor<gtsam::Rot3> init_rot_ext(P(0), gtsam::Rot3(gtsam::Rot3(Rot_nmea_init)), p_assign->priorextrotNoise);
  gtsam::PriorFactor<gtsam::Vector3> init_pos_ext(E(0), gtsam::Vector3(anc_enu[0], anc_enu[1], anc_enu[2]), p_assign->priorextposNoise);
  gtsam::PriorFactor<gtsam::Rot3> init_rot_(R(0), gtsam::Rot3(R_enu_local_), p_assign->priorrotNoise);
  gtsam::PriorFactor<gtsam::Vector6> init_vel_(A(0), gtsam::Vector6(init_vel_bias_vector), p_assign->priorNoise);
  gtsam::PriorFactor<gtsam::Vector12> init_bias_(O(0), gtsam::Vector12(init_others_vector), p_assign->priorBiasNoise);
  gtsam::PriorFactor<gtsam::Vector3> init_grav_(G(0), gtsam::Vector3(gravity_init), p_assign->priorGravNoise);
  p_assign->gtSAMgraph.add(init_rot_ext);
  p_assign->gtSAMgraph.add(init_pos_ext);
  p_assign->gtSAMgraph.add(init_rot_);
  p_assign->gtSAMgraph.add(init_vel_);
  p_assign->gtSAMgraph.add(init_bias_);
  p_assign->gtSAMgraph.add(init_grav_);
  p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1, 2, 3, 4, 5});
  id_accumulate += 6;
}