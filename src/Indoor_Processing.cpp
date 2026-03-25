#include "Indoor_Processing.h"
#include "parameters.h"

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#ifdef LIGO_WITH_NMEA
#include "indoor_localization_factor.hpp"
#endif

#ifdef LIGO_WITH_SMALL_GICP
#include "Indoor_SmallGICP_Localizer.h"
#include <memory>

namespace {
  static std::shared_ptr<ligo::indoor::SmallGICPLocalizer> s_gicp_localizer;
  static bool s_map_published = false;
}
#endif  // LIGO_WITH_SMALL_GICP

namespace ligo {
namespace indoor {

// ---------------------------------------------------------------------------
// Original placeholder (kept functional for non-GICP builds)
// ---------------------------------------------------------------------------
void updateIndoorLocalizationPlaceholder(const Eigen::Vector3d& pos_enu,
                                         const Eigen::Matrix3d& rot_enu,
                                         double ts_sec) {
  if (!indoor_flag) {
    return;
  }
  // When GICP is available this is superseded by runIndoorGICPUpdate().
  // In non-GICP builds it still provides the LIO-derived fallback.
#ifndef LIGO_WITH_SMALL_GICP
  indoor_pos_enu_meas = pos_enu;
  indoor_rot_enu_meas = Eigen::Quaterniond(rot_enu).normalized();
  indoor_pose_time = ts_sec;
  indoor_pose_valid = true;
#else
  (void)pos_enu; (void)rot_enu; (void)ts_sec;
#endif
}

void addIndoorFactorToGraphStubCommented() {
  // No-op stub kept as design reference.
}

// ---------------------------------------------------------------------------
// GICP-backed localization (compiled only when LIGO_WITH_SMALL_GICP is ON)
// ---------------------------------------------------------------------------
#ifdef LIGO_WITH_SMALL_GICP

void initIndoorGICP(const std::string& map_pcd_path, const SmallGICPConfig& cfg) {
  s_gicp_localizer = std::make_shared<SmallGICPLocalizer>(cfg);
  s_map_published = false;
  if (s_gicp_localizer->loadMapFromPCD(map_pcd_path)) {
    indoor_gicp_map_loaded = true;
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[indoor/gicp] map loaded (%s)", map_pcd_path.c_str());
  } else {
    indoor_gicp_map_loaded = false;
    RCLCPP_ERROR(rclcpp::get_logger("ligo"),
                 "[indoor/gicp] failed to load map: %s", map_pcd_path.c_str());
  }
}

void resetIndoorGICP() {
  indoor_gicp_T_map_lidar = Eigen::Isometry3d::Identity();
  indoor_pose_valid = false;
  s_map_published = false;
  RCLCPP_INFO(rclcpp::get_logger("ligo"), "[indoor/gicp] state reset");
}

bool runIndoorGICPUpdate(const CloudT::ConstPtr& scan_world,
                         double timestamp,
                         const Eigen::Matrix3d& R_local_to_enu,
                         const Eigen::Vector3d& t_local_to_enu) {
  if (!s_gicp_localizer || !s_gicp_localizer->hasMap()) return false;
  if (!scan_world || scan_world->empty()) return false;

  // Convert scan from LIO world frame to map (ENU) frame
  auto scan_map = pcl::make_shared<SmallGICPLocalizer::LidarCloud>();
  scan_map->reserve(scan_world->size());
  for (const auto& p : scan_world->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const Eigen::Vector3d p_enu = R_local_to_enu * Eigen::Vector3d(p.x, p.y, p.z) + t_local_to_enu;
    SmallGICPLocalizer::LidarPoint q;
    q.x = static_cast<float>(p_enu.x());
    q.y = static_cast<float>(p_enu.y());
    q.z = static_cast<float>(p_enu.z());
    q.intensity = p.intensity;
    q.normal_x = 0.0f; q.normal_y = 0.0f; q.normal_z = 0.0f; q.curvature = 0.0f;
    scan_map->push_back(q);
  }
  if (static_cast<int>(scan_map->size()) < 50) return false;

  const SmallGICPResult result = s_gicp_localizer->localize(scan_map, indoor_gicp_T_map_lidar);
  if (!result.success || !result.converged) {
    RCLCPP_WARN(rclcpp::get_logger("ligo"),
                "[indoor/gicp] not converged (iter=%zu inliers=%zu err=%.4f)",
                result.iterations, result.num_inliers, result.error);
    return false;
  }

  indoor_gicp_T_map_lidar = result.T_map_lidar;
  indoor_pos_enu_meas     = result.T_map_lidar.translation();
  indoor_rot_enu_meas     = Eigen::Quaterniond(result.T_map_lidar.rotation()).normalized();
  indoor_pose_time        = timestamp;
  indoor_pose_valid       = true;
  return true;
}

void publishIndoorViz(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_map,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_scan,
    const CloudT::ConstPtr& scan_world,
    const Eigen::Matrix3d& R_local_to_enu,
    const Eigen::Vector3d& t_local_to_enu,
    double timestamp_sec) {
  if (!s_gicp_localizer || !s_gicp_localizer->hasMap()) return;

  const int32_t  sec     = static_cast<int32_t>(std::floor(timestamp_sec));
  const uint32_t nanosec = static_cast<uint32_t>(std::round((timestamp_sec - sec) * 1e9));

  // --- map cloud (publish once; map is static) ---
  if (!s_map_published && pub_map && pub_map->get_subscription_count() > 0) {
    // Retrieve internal map by running a dummy localize; use a helper approach instead:
    // We publish the aligned scan only (map retrieval would require exposing impl_).
    s_map_published = true;
  }

  // --- aligned current scan in ENU (map frame) ---
  if (pub_scan && pub_scan->get_subscription_count() > 0 && scan_world && !scan_world->empty()) {
    CloudT scan_enu;
    scan_enu.reserve(scan_world->size());
    for (const auto& p : scan_world->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      const Eigen::Vector3d pe = R_local_to_enu * Eigen::Vector3d(p.x, p.y, p.z) + t_local_to_enu;
      PointT q = p;
      q.x = static_cast<float>(pe.x());
      q.y = static_cast<float>(pe.y());
      q.z = static_cast<float>(pe.z());
      scan_enu.push_back(q);
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(scan_enu, msg);
    msg.header.stamp.sec     = sec;
    msg.header.stamp.nanosec = nanosec;
    msg.header.frame_id      = "enu";
    pub_scan->publish(msg);
  }
}

#endif  // LIGO_WITH_SMALL_GICP

// ---------------------------------------------------------------------------
// GTSAM factor insertion (compiled only with NMEA)
// ---------------------------------------------------------------------------
void addIndoorFactorToGraph(int frame_num) {
#ifdef LIGO_WITH_NMEA
  if (!indoor_flag || !indoor_pose_valid) return;
  if (!p_nmea || !p_nmea->nmea_ready) return;
  if (!indoorPoseNoise || !indoorPoseNoiseInit) return;

  double values[17] = {};
  values[0]  = p_nmea->Tex_imu_r[0];
  values[1]  = p_nmea->Tex_imu_r[1];
  values[2]  = p_nmea->Tex_imu_r[2];
  values[3]  = p_nmea->anc_local[0];
  values[4]  = p_nmea->anc_local[1];
  values[5]  = p_nmea->anc_local[2];
  values[6]  = indoor_pos_enu_meas[0];
  values[7]  = indoor_pos_enu_meas[1];
  values[8]  = indoor_pos_enu_meas[2];
  // values[9:11]: velocity slot unused in IndoorLocalizationFactor
  values[12] = indoor_rot_enu_meas.w();
  values[13] = indoor_rot_enu_meas.x();
  values[14] = indoor_rot_enu_meas.y();
  values[15] = indoor_rot_enu_meas.z();
  values[16] = 1.0;  // relative_sqrt_info weight

  const auto& noise = (frame_num < p_nmea->delete_thred) ? indoorPoseNoiseInit : indoorPoseNoise;
  p_nmea->p_assign->gtSAMgraph.add(ligo::IndoorLocalizationFactor(
      P(0), E(0), A(frame_num), R(frame_num),
      false, values, Eigen::Vector3d::Zero(), p_nmea->Rex_imu_r, noise));
#else
  (void)frame_num;
#endif
}

}  // namespace indoor
}  // namespace ligo
