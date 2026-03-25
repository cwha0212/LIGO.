#include "Indoor_Processing.h"
#include "parameters.h"
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#ifdef LIGO_WITH_NMEA
#include "indoor_localization_factor.hpp"
#endif

#ifdef LIGO_WITH_SMALL_GICP
#include "Indoor_SmallGICP_Localizer.h"
#include "Indoor_GridMapRegistry.h"
#include <memory>
#include <pcl/io/pcd_io.h>

namespace {
  static std::shared_ptr<ligo::indoor::SmallGICPLocalizer> s_gicp_localizer;
  static bool s_map_published = false;
  static ligo::indoor::SmallGICPConfig s_gicp_cfg_grid;
  static std::string s_gicp_grid_resolved_pcd;
  // Display-resolution cached map cloud (published latched to RViz)
  static sensor_msgs::msg::PointCloud2 s_map_cloud_msg;
  static bool s_map_cloud_ready = false;
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

void setIndoorGICPConfigForGridSelection(const SmallGICPConfig& cfg) {
  s_gicp_cfg_grid = cfg;
}

// Called at session start: loads map without resetting indoor_gicp_T_map_lidar.
// Caller (laserMapping reset block) seeds T_map_lidar immediately after this returns.
bool loadIndoorGICPMapForSession(const Eigen::Vector3d& ecef_m) {
  if (!indoorGridMapsLoaded()) return false;

  const auto opt = lookupIndoorGridKnownPcd(ecef_m);
  std::string pcd;
  std::string map_id;

  if (opt) {
    pcd    = opt->second;
    map_id = opt->first;
    RCLCPP_WARN(rclcpp::get_logger("ligo"),
                "[indoor/gicp] session map selected via grid membership: map_id=%s  file=%s",
                map_id.c_str(), pcd.c_str());
  } else {
    // Grid lookup failed (ENU→ECEF approximate, or rover outside cell boundary).
    // Fallback: if only one map is loaded, use it unconditionally.
    //           If multiple maps exist, fall through and report failure.
    const size_t n = indoorGridMapCount();
    if (n == 1) {
      const auto first = getFirstGridMapEntry();
      pcd    = first->second;
      map_id = first->first;
      RCLCPP_WARN(rclcpp::get_logger("ligo"),
                  "[indoor/gicp] grid lookup failed; using sole loaded map as fallback: map_id=%s  file=%s",
                  map_id.c_str(), pcd.c_str());
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ligo"),
                   "[indoor/gicp] grid lookup failed (%zu maps loaded) — ENU→ECEF mismatch? "
                   "Rover position may be outside all grid cells.", n);
      // Last resort: try map_pcd_path from parameters
      if (!indoor_map_pcd_path.empty()) {
        pcd    = indoor_map_pcd_path;
        map_id = "map_pcd_path_fallback";
        RCLCPP_WARN(rclcpp::get_logger("ligo"),
                    "[indoor/gicp] falling back to indoor.map_pcd_path: %s", pcd.c_str());
      } else {
        return false;
      }
    }
  }

  if (indoor_gicp_map_loaded && pcd == s_gicp_grid_resolved_pcd) return true;
  initIndoorGICP(pcd, s_gicp_cfg_grid);
  // Intentionally NOT calling resetIndoorGICP() — caller seeds T_map_lidar right after.
  return indoor_gicp_map_loaded;
}

bool ensureIndoorGICPMapFromGridEcef(const Eigen::Vector3d& ecef_m) {
  if (!indoorGridMapsLoaded())
    return indoor_gicp_map_loaded;
  const auto opt = lookupIndoorGridKnownPcd(ecef_m);
  if (!opt)
    return indoor_gicp_map_loaded;
  const std::string& pcd = opt->second;
  if (indoor_gicp_map_loaded && pcd == s_gicp_grid_resolved_pcd)
    return true;
  // Map changed mid-session (different indoor area): reload without resetting pose.
  // Keeping current T_map_lidar is better than resetting to Identity here.
  initIndoorGICP(pcd, s_gicp_cfg_grid);
  s_map_published = false;
  RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/gicp] map switched mid-session: map_id=%s  file=%s",
              opt->first.c_str(), pcd.c_str());
  return indoor_gicp_map_loaded;
}

void initIndoorGICP(const std::string& map_pcd_path, const SmallGICPConfig& cfg) {
  s_gicp_localizer = std::make_shared<SmallGICPLocalizer>(cfg);
  s_map_published = false;
  s_map_cloud_ready = false;
  if (s_gicp_localizer->loadMapFromPCD(map_pcd_path)) {
    indoor_gicp_map_loaded = true;
    s_gicp_grid_resolved_pcd = map_pcd_path;
    RCLCPP_WARN(rclcpp::get_logger("ligo"),
                "==== [indoor/gicp] REFERENCE MAP LOADED ==== path=%s", map_pcd_path.c_str());
    // Build display-resolution PCD cloud for RViz (reload raw, no downsampling limit)
    pcl::PointCloud<pcl::PointXYZI> raw;
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_pcd_path, raw) == 0 && !raw.empty()) {
      sensor_msgs::msg::PointCloud2 tmp;
      pcl::toROSMsg(raw, tmp);
      tmp.header.frame_id = "map";   // ENU frame in this system is called "map"
      s_map_cloud_msg = std::move(tmp);
      s_map_cloud_ready = true;
    }
  } else {
    indoor_gicp_map_loaded = false;
    RCLCPP_ERROR(rclcpp::get_logger("ligo"),
                 "[indoor/gicp] failed to load map: %s", map_pcd_path.c_str());
  }
}

std::string getIndoorGicpMapPath() {
  return s_gicp_grid_resolved_pcd;
}

void resetIndoorGICP() {
  indoor_gicp_T_map_lidar = Eigen::Isometry3d::Identity();
  indoor_pose_valid = false;
  s_map_published = false;
  // Keep s_map_cloud_ready so the map re-publishes to new RViz subscribers after map change.
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

  static size_t s_gicp_fail_count = 0;
  static size_t s_gicp_ok_count   = 0;

  const SmallGICPResult result = s_gicp_localizer->localize(scan_map, indoor_gicp_T_map_lidar);

  if (!result.success || !result.converged) {
    ++s_gicp_fail_count;
    s_gicp_ok_count = 0;
    // First failure and every 30: WARN without throttle; otherwise throttled
    if (s_gicp_fail_count == 1 || s_gicp_fail_count % 30 == 0) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"),
                  "[indoor/gicp] NOT CONVERGED #%zu  iter=%zu inliers=%zu err=%.4f  map=%s",
                  s_gicp_fail_count, result.iterations, result.num_inliers, result.error,
                  s_gicp_grid_resolved_pcd.c_str());
    }
    // (silent on other failures — counter throttles the output via %30 check above)
    return false;
  }

  // Converged
  s_gicp_fail_count = 0;
  ++s_gicp_ok_count;
  if (s_gicp_ok_count == 1 || s_gicp_ok_count % 100 == 0) {
    const Eigen::Vector3d t = result.T_map_lidar.translation();
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[indoor/gicp] converged #%zu  iter=%zu inliers=%zu err=%.4f  "
                "pos_enu=(%.2f, %.2f, %.2f)",
                s_gicp_ok_count, result.iterations, result.num_inliers, result.error,
                t.x(), t.y(), t.z());
  }

  indoor_gicp_T_map_lidar = result.T_map_lidar;
  indoor_pos_enu_meas     = result.T_map_lidar.translation();
  indoor_rot_enu_meas     = Eigen::Quaterniond(result.T_map_lidar.rotation()).normalized();
  indoor_pose_time        = timestamp;
  indoor_pose_valid       = true;
  return true;
}

void publishIndoorMapCloudOnly(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_map,
    double timestamp_sec) {
  if (!pub_map || !s_map_cloud_ready || s_map_published) return;
  const int32_t  sec     = static_cast<int32_t>(std::floor(timestamp_sec));
  const uint32_t nanosec = static_cast<uint32_t>(std::round((timestamp_sec - sec) * 1e9));
  s_map_cloud_msg.header.stamp.sec     = sec;
  s_map_cloud_msg.header.stamp.nanosec = nanosec;
  pub_map->publish(s_map_cloud_msg);
  s_map_published = true;
  RCLCPP_INFO(rclcpp::get_logger("ligo"),
              "[indoor/gicp] map cloud published (%u pts) -> /indoor/map_cloud",
              s_map_cloud_msg.width * s_map_cloud_msg.height);
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

  // --- map cloud (latched): delegate to dedicated function ---
  publishIndoorMapCloudOnly(pub_map, timestamp_sec);

  // --- GICP-aligned scan in ENU: apply current indoor_gicp_T_map_lidar to each point ---
  if (pub_scan && scan_world && !scan_world->empty()) {
    const Eigen::Matrix3d R_gicp = indoor_gicp_T_map_lidar.rotation();
    const Eigen::Vector3d t_gicp = indoor_gicp_T_map_lidar.translation();
    CloudT scan_aligned;
    scan_aligned.reserve(scan_world->size());
    for (const auto& p : scan_world->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      // world frame -> ENU via ICP tf, then refine with GICP T
      const Eigen::Vector3d p_enu_raw = R_local_to_enu * Eigen::Vector3d(p.x, p.y, p.z) + t_local_to_enu;
      const Eigen::Vector3d p_aligned = R_gicp * p_enu_raw + t_gicp;
      PointT q = p;
      q.x = static_cast<float>(p_aligned.x());
      q.y = static_cast<float>(p_aligned.y());
      q.z = static_cast<float>(p_aligned.z());
      scan_aligned.push_back(q);
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(scan_aligned, msg);
    msg.header.stamp.sec     = sec;
    msg.header.stamp.nanosec = nanosec;
    msg.header.frame_id      = "map";   // ENU frame in this system is called "map"
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
