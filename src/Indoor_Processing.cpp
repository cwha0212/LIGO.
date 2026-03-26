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
  // Map cloud transformed to system-global-ENU for outdoor-aligned visualization
  static sensor_msgs::msg::PointCloud2 s_map_cloud_sys_msg;
  static bool s_map_cloud_sys_ready = false;
  // ECEF→ENU transform of the currently loaded map (from grid YAML)
  static Eigen::Vector3d s_map_anchor_ecef = Eigen::Vector3d::Zero();
  static Eigen::Matrix3d s_map_R_ecef_enu  = Eigen::Matrix3d::Identity();
  static bool s_map_transform_valid = false;
  // ECEF→ENU transform of the system GNSS anchor
  static Eigen::Vector3d s_sys_anchor_ecef = Eigen::Vector3d::Zero();
  static Eigen::Matrix3d s_sys_R_ecef_enu  = Eigen::Matrix3d::Identity();
  static bool s_sys_anchor_valid = false;
  // Precomputed system-ENU → map-local-ENU transform
  static Eigen::Matrix3d s_R_sys_to_map = Eigen::Matrix3d::Identity();
  static Eigen::Vector3d s_t_sys_to_map = Eigen::Vector3d::Zero();
  static bool s_sys_to_map_valid = false;

  static void recomputeSysToMapTransform() {
    if (s_map_transform_valid && s_sys_anchor_valid) {
      // p_map = R_map^T * R_sys * p_sys + R_map^T * (anc_sys - anc_map)
      s_R_sys_to_map = s_map_R_ecef_enu.transpose() * s_sys_R_ecef_enu;
      s_t_sys_to_map = s_map_R_ecef_enu.transpose() * (s_sys_anchor_ecef - s_map_anchor_ecef);
      s_sys_to_map_valid = true;
    }
  }
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

  // Store the map's ECEF→ENU transform for GICP seed computation.
  s_map_transform_valid = false;
  auto tf = lookupIndoorGridTransformByMapId(map_id);
  if (!tf && indoorGridMapCount() == 1)
    tf = getFirstGridMapTransform();
  if (tf) {
    s_map_anchor_ecef     = tf->anchor_ecef_m;
    s_map_R_ecef_enu      = tf->R_ecef_enu;
    s_map_transform_valid = true;
    recomputeSysToMapTransform();
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[indoor/gicp] map ECEF transform cached: anchor=(%.2f,%.2f,%.2f)  sys_to_map=%s",
                s_map_anchor_ecef.x(), s_map_anchor_ecef.y(), s_map_anchor_ecef.z(),
                s_sys_to_map_valid ? "ready" : "pending_sys_anchor");
  }

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

std::optional<Eigen::Vector3d> ecefToLoadedMapEnu(const Eigen::Vector3d& ecef) {
  if (!s_map_transform_valid) return std::nullopt;
  return s_map_R_ecef_enu.transpose() * (ecef - s_map_anchor_ecef);
}

void setSystemEcefAnchor(const Eigen::Vector3d& anc_ecef,
                         const Eigen::Matrix3d& R_ecef_enu) {
  s_sys_anchor_ecef = anc_ecef;
  s_sys_R_ecef_enu  = R_ecef_enu;
  s_sys_anchor_valid = true;
  recomputeSysToMapTransform();
}

void resetIndoorGICP() {
  indoor_gicp_T_map_lidar = Eigen::Isometry3d::Identity();
  indoor_pose_valid = false;
  s_map_published = false;
  s_map_cloud_sys_ready = false;
  RCLCPP_INFO(rclcpp::get_logger("ligo"), "[indoor/gicp] state reset");
}

bool runIndoorGICPUpdate(const CloudT::ConstPtr& scan_world,
                         double timestamp,
                         const Eigen::Matrix3d& R_local_to_enu,
                         const Eigen::Vector3d& t_local_to_enu) {
  if (!s_gicp_localizer || !s_gicp_localizer->hasMap()) return false;
  if (!scan_world || scan_world->empty()) return false;

  // Convert scan from LIO world frame → system-ENU → map-local-ENU
  // so that it matches the reference PCD coordinate system.
  auto scan_map = pcl::make_shared<SmallGICPLocalizer::LidarCloud>();
  scan_map->reserve(scan_world->size());
  for (const auto& p : scan_world->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    Eigen::Vector3d p_sys_enu = R_local_to_enu * Eigen::Vector3d(p.x, p.y, p.z) + t_local_to_enu;
    Eigen::Vector3d p_final = s_sys_to_map_valid
        ? (s_R_sys_to_map * p_sys_enu + s_t_sys_to_map)
        : p_sys_enu;
    SmallGICPLocalizer::LidarPoint q;
    q.x = static_cast<float>(p_final.x());
    q.y = static_cast<float>(p_final.y());
    q.z = static_cast<float>(p_final.z());
    q.intensity = p.intensity;
    q.normal_x = 0.0f; q.normal_y = 0.0f; q.normal_z = 0.0f; q.curvature = 0.0f;
    scan_map->push_back(q);
  }
  if (static_cast<int>(scan_map->size()) < 50) return false;

  static size_t s_gicp_total = 0;
  static size_t s_gicp_converged = 0;

  const SmallGICPResult result = s_gicp_localizer->localize(scan_map, indoor_gicp_T_map_lidar);
  ++s_gicp_total;

  if (!result.success) {
    if (s_gicp_total <= 3) {
      RCLCPP_WARN(rclcpp::get_logger("ligo"),
                  "[indoor/gicp] registration failed (no result)  map=%s",
                  s_gicp_grid_resolved_pcd.c_str());
    }
    return false;
  }

  // Always update the pose tracker so the next GICP iteration starts from the
  // best available guess, regardless of quality.
  indoor_gicp_T_map_lidar = result.T_map_lidar;

  if (result.converged) ++s_gicp_converged;

  // Quality gate: only feed the result as a factor if alignment is good enough.
  const bool quality_ok =
      result.error <= indoor_gicp_max_factor_error &&
      static_cast<int>(result.num_inliers) >= indoor_gicp_min_factor_inliers;

  if (quality_ok)
  {
    indoor_pos_enu_meas = result.T_map_lidar.translation();
    indoor_rot_enu_meas = Eigen::Quaterniond(result.T_map_lidar.rotation()).normalized();
    indoor_pose_time    = timestamp;
    indoor_pose_valid   = true;
  }
  else
  {
    indoor_pose_valid = false;
  }

  static size_t s_gicp_rejected = 0;
  if (!quality_ok) ++s_gicp_rejected;

  if (s_gicp_total <= 5 || s_gicp_total % 100 == 0 || !quality_ok) {
    const Eigen::Vector3d t = result.T_map_lidar.translation();
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[indoor/gicp] #%zu  %s  iter=%zu inliers=%zu err=%.2f  "
                "T=(%.3f,%.3f,%.3f)  factor=%s  conv=%zu/%zu rej=%zu",
                s_gicp_total,
                result.converged ? "OK" : "maxiter",
                result.iterations, result.num_inliers, result.error,
                t.x(), t.y(), t.z(),
                quality_ok ? "accept" : "REJECT",
                s_gicp_converged, s_gicp_total, s_gicp_rejected);
  }

  return true;
}

static void buildMapCloudSysEnu() {
  if (!s_map_cloud_ready || !s_sys_to_map_valid) return;
  if (s_map_cloud_sys_ready) return;

  const Eigen::Matrix3d R_m2s = s_R_sys_to_map.transpose();
  const Eigen::Vector3d t_m2s = -R_m2s * s_t_sys_to_map;

  pcl::PointCloud<pcl::PointXYZI> raw;
  pcl::fromROSMsg(s_map_cloud_msg, raw);

  pcl::PointCloud<pcl::PointXYZI> transformed;
  transformed.reserve(raw.size());
  for (const auto& p : raw.points) {
    Eigen::Vector3d ps = R_m2s * Eigen::Vector3d(p.x, p.y, p.z) + t_m2s;
    pcl::PointXYZI q;
    q.x = static_cast<float>(ps.x());
    q.y = static_cast<float>(ps.y());
    q.z = static_cast<float>(ps.z());
    q.intensity = p.intensity;
    transformed.push_back(q);
  }

  pcl::toROSMsg(transformed, s_map_cloud_sys_msg);
  s_map_cloud_sys_msg.header.frame_id = "map";
  s_map_cloud_sys_ready = true;
}

void publishIndoorMapCloudOnly(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_map,
    double timestamp_sec) {
  if (!pub_map || !s_map_cloud_ready || s_map_published) return;

  buildMapCloudSysEnu();

  const int32_t  sec     = static_cast<int32_t>(std::floor(timestamp_sec));
  const uint32_t nanosec = static_cast<uint32_t>(std::round((timestamp_sec - sec) * 1e9));

  auto& msg = s_map_cloud_sys_ready ? s_map_cloud_sys_msg : s_map_cloud_msg;
  msg.header.stamp.sec     = sec;
  msg.header.stamp.nanosec = nanosec;
  pub_map->publish(msg);
  s_map_published = true;
  RCLCPP_INFO(rclcpp::get_logger("ligo"),
              "[indoor/gicp] map cloud published (%u pts, frame=%s) -> /indoor/map_cloud",
              msg.width * msg.height,
              s_map_cloud_sys_ready ? "sys_enu" : "map_local_enu");
}

void publishIndoorViz(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_map,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_scan,
    const CloudT::ConstPtr& scan_world,
    const Eigen::Matrix3d& R_local_to_enu,
    const Eigen::Vector3d& t_local_to_enu,
    const Eigen::Isometry3d& T_map_lidar,
    double timestamp_sec) {
  if (!s_gicp_localizer || !s_gicp_localizer->hasMap()) return;

  const int32_t  sec     = static_cast<int32_t>(std::floor(timestamp_sec));
  const uint32_t nanosec = static_cast<uint32_t>(std::round((timestamp_sec - sec) * 1e9));

  // --- map cloud (latched): delegate to dedicated function ---
  publishIndoorMapCloudOnly(pub_map, timestamp_sec);

  // --- GICP-aligned scan in system-global-ENU (matches outdoor frame) ---
  if (pub_scan && scan_world && !scan_world->empty()) {
    const Eigen::Matrix3d R_gicp = T_map_lidar.rotation();
    const Eigen::Vector3d t_gicp = T_map_lidar.translation();
    const Eigen::Matrix3d R_m2s = s_R_sys_to_map.transpose();
    const Eigen::Vector3d t_m2s = -R_m2s * s_t_sys_to_map;

    CloudT scan_aligned;
    scan_aligned.reserve(scan_world->size());
    for (const auto& p : scan_world->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      Eigen::Vector3d p_sys = R_local_to_enu * Eigen::Vector3d(p.x, p.y, p.z) + t_local_to_enu;
      // local → sys-ENU → map-local-ENU → GICP → sys-ENU
      if (s_sys_to_map_valid) {
        Eigen::Vector3d p_map = s_R_sys_to_map * p_sys + s_t_sys_to_map;
        Eigen::Vector3d p_aligned_map = R_gicp * p_map + t_gicp;
        p_sys = R_m2s * p_aligned_map + t_m2s;
      }
      PointT q = p;
      q.x = static_cast<float>(p_sys.x());
      q.y = static_cast<float>(p_sys.y());
      q.z = static_cast<float>(p_sys.z());
      scan_aligned.push_back(q);
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(scan_aligned, msg);
    msg.header.stamp.sec     = sec;
    msg.header.stamp.nanosec = nanosec;
    msg.header.frame_id      = "map";
    pub_scan->publish(msg);
  }
}

#endif  // LIGO_WITH_SMALL_GICP

// ---------------------------------------------------------------------------
// GTSAM factor insertion (compiled only with NMEA)
// ---------------------------------------------------------------------------
void addIndoorFactorToGraph(int frame_num) {
#ifdef LIGO_WITH_NMEA
  if (!(indoor_flag || indoor_flag_dynamic) || !indoor_pose_valid) return;
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

  const bool init_phase = (frame_num < p_nmea->delete_thred);
  const auto& noise = init_phase ? indoorPoseNoiseInit : indoorPoseNoise;
  p_nmea->p_assign->gtSAMgraph.add(ligo::IndoorLocalizationFactor(
      P(0), E(0), A(frame_num), R(frame_num),
      false, values, Eigen::Vector3d::Zero(), p_nmea->Rex_imu_r, noise));

  static size_t s_indoor_factor_cnt = 0;
  ++s_indoor_factor_cnt;
  if (s_indoor_factor_cnt <= 5 || s_indoor_factor_cnt % 50 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[INDOOR FACTOR INPUT] #%zu  frame=%d  pos_enu=(%.2f,%.2f,%.2f)  "
                "quat=(%.3f,%.3f,%.3f,%.3f)  noise=%s",
                s_indoor_factor_cnt, frame_num,
                indoor_pos_enu_meas[0], indoor_pos_enu_meas[1], indoor_pos_enu_meas[2],
                indoor_rot_enu_meas.w(), indoor_rot_enu_meas.x(),
                indoor_rot_enu_meas.y(), indoor_rot_enu_meas.z(),
                init_phase ? "init" : "normal");
  }
#else
  (void)frame_num;
#endif
}

}  // namespace indoor
}  // namespace ligo
