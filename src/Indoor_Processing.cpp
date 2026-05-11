#include "Indoor_Processing.h"
#include "parameters.h"
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "indoor_localization_factor.hpp"

#include "Indoor_SmallGICP_Localizer.h"
#include "Indoor_GridMapRegistry.h"
#include "common_lib.h"
#include <memory>
#include <pcl/io/pcd_io.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace {
  static std::shared_ptr<ligo::indoor::SmallGICPLocalizer> s_gicp_localizer;
  static bool s_map_published = false;
  static bool s_occ_grid_published = false;
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
  // Latch T_map_lidar^{-1} (map-local): shifts reference PCD to where raw LIO places the scan.
  static Eigen::Isometry3d s_refmap_display_T_inv = Eigen::Isometry3d::Identity();
  static bool s_refmap_display_T_inv_valid = false;
  static uint64_t s_gicp_session_nonce = 0;

  /** small_gicp T acts on map-frame scan points: p' = T * p. Compose with body→map Isometry, then express in system ENU. */
  static void computeEnuPoseFromGicpMapTransform(const Eigen::Matrix3d& R_local_to_enu,
                                                 const Eigen::Vector3d& t_local_to_enu,
                                                 const Eigen::Isometry3d& T_gicp,
                                                 Eigen::Vector3d* out_pos,
                                                 Eigen::Quaterniond* out_rot) {
    const Eigen::Vector3d p_local = kf_output.x_.pos;
    const Eigen::Matrix3d R_local = kf_output.x_.rot;
    const Eigen::Matrix3d R_body_enu = R_local_to_enu * R_local;
    const Eigen::Vector3d t_body_enu = R_local_to_enu * p_local + t_local_to_enu;

    Eigen::Isometry3d iso_body_in_map = Eigen::Isometry3d::Identity();
    if (s_sys_to_map_valid) {
      iso_body_in_map.linear() = s_R_sys_to_map * R_body_enu;
      iso_body_in_map.translation() = s_R_sys_to_map * t_body_enu + s_t_sys_to_map;
    } else {
      iso_body_in_map.linear() = R_body_enu;
      iso_body_in_map.translation() = t_body_enu;
    }

    const Eigen::Isometry3d iso_gicp = T_gicp * iso_body_in_map;

    Eigen::Matrix3d R_sys;
    Eigen::Vector3d p_sys;
    if (s_sys_to_map_valid) {
      R_sys = s_R_sys_to_map.transpose() * iso_gicp.rotation();
      p_sys = s_R_sys_to_map.transpose() * (iso_gicp.translation() - s_t_sys_to_map);
    } else {
      R_sys = iso_gicp.rotation();
      p_sys = iso_gicp.translation();
    }
    *out_pos = p_sys;
    *out_rot = Eigen::Quaterniond(R_sys).normalized();
  }

  static void applyRefMapDisplayCorrectionToGrid(nav_msgs::msg::OccupancyGrid& grid) {
    if (!indoor_gicp_align_reference_map_to_lio || !s_refmap_display_T_inv_valid) return;
    const Eigen::Isometry3d& T = s_refmap_display_T_inv;
    Eigen::Vector3d p0(grid.info.origin.position.x, grid.info.origin.position.y,
                       grid.info.origin.position.z);
    Eigen::Quaterniond q0(grid.info.origin.orientation.w, grid.info.origin.orientation.x,
                           grid.info.origin.orientation.y, grid.info.origin.orientation.z);
    Eigen::Matrix3d R0 = q0.toRotationMatrix();
    const Eigen::Vector3d p1 = T * p0;
    const Eigen::Matrix3d R1 = T.rotation() * R0;
    const Eigen::Quaterniond q1(R1);
    grid.info.origin.position.x = p1.x();
    grid.info.origin.position.y = p1.y();
    grid.info.origin.position.z = p1.z();
    grid.info.origin.orientation.w = q1.w();
    grid.info.origin.orientation.x = q1.x();
    grid.info.origin.orientation.y = q1.y();
    grid.info.origin.orientation.z = q1.z();
  }

  static void recomputeSysToMapTransform() {
    if (s_map_transform_valid && s_sys_anchor_valid) {
      // p_map = R_map^T * R_sys * p_sys + R_map^T * (anc_sys - anc_map)
      s_R_sys_to_map = s_map_R_ecef_enu.transpose() * s_sys_R_ecef_enu;
      s_t_sys_to_map = s_map_R_ecef_enu.transpose() * (s_sys_anchor_ecef - s_map_anchor_ecef);
      s_sys_to_map_valid = true;
    }
  }

  static void applySysTransformToOccupancyGrid(nav_msgs::msg::OccupancyGrid& grid) {
    if (!s_sys_to_map_valid) return;
    const Eigen::Matrix3d R_m2s = s_R_sys_to_map.transpose();
    const Eigen::Vector3d t_m2s = -R_m2s * s_t_sys_to_map;
    const Eigen::Vector3d p0(grid.info.origin.position.x, grid.info.origin.position.y,
                             grid.info.origin.position.z);
    const Eigen::Vector3d ps = R_m2s * p0 + t_m2s;
    grid.info.origin.position.x = ps.x();
    grid.info.origin.position.y = ps.y();
    grid.info.origin.position.z = ps.z();
    const Eigen::Quaterniond q(R_m2s);
    grid.info.origin.orientation.w = q.w();
    grid.info.origin.orientation.x = q.x();
    grid.info.origin.orientation.y = q.y();
    grid.info.origin.orientation.z = q.z();
  }

  static void maybePublishIndoorOccGrid(
      const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub,
      double timestamp_sec) {
    if (!pub || s_occ_grid_published) return;
    if (!ligo::indoor::indoorGridMapsLoaded()) {
      s_occ_grid_published = true;
      return;
    }

    std::string map_id;
    if (auto m = ligo::indoor::lookupIndoorMapIdBySourcePcd(s_gicp_grid_resolved_pcd)) map_id = *m;
    if (map_id.empty()) {
      RCLCPP_WARN_ONCE(rclcpp::get_logger("ligo"),
                       "[indoor/gicp] no 2D grid for loaded PCD (no matching *_grid2d source_pcd) — "
                       "skip /indoor/map_2d");
      s_occ_grid_published = true;
      return;
    }

    nav_msgs::msg::OccupancyGrid grid;
    if (!ligo::indoor::buildIndoorOccupancyGridForMapId(map_id, grid)) {
      s_occ_grid_published = true;
      return;
    }

    applyRefMapDisplayCorrectionToGrid(grid);
    applySysTransformToOccupancyGrid(grid);

    const int32_t sec = static_cast<int32_t>(std::floor(timestamp_sec));
    const uint32_t nanosec = static_cast<uint32_t>(std::round((timestamp_sec - sec) * 1e9));
    grid.header.stamp.sec = sec;
    grid.header.stamp.nanosec = nanosec;
    grid.info.map_load_time = grid.header.stamp;

    pub->publish(grid);
    s_occ_grid_published = true;
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[indoor/gicp] 2D occupancy map published -> /indoor/map_2d  %ux%u  res=%.3f  frame=map",
                grid.info.width, grid.info.height, grid.info.resolution);
  }
}

namespace ligo {
namespace indoor {

// ---------------------------------------------------------------------------
// Original placeholder: superseded by runIndoorGICPUpdate() when GICP runs.
// ---------------------------------------------------------------------------
void updateIndoorLocalizationPlaceholder(const Eigen::Vector3d& pos_enu,
                                         const Eigen::Matrix3d& rot_enu,
                                         double ts_sec) {
  if (!indoor_flag) {
    return;
  }
  (void)pos_enu; (void)rot_enu; (void)ts_sec;
}

void addIndoorFactorToGraphStubCommented() {
  // No-op stub kept as design reference.
}

// ---------------------------------------------------------------------------
// GICP-backed localization
// ---------------------------------------------------------------------------

void setIndoorGICPConfigForGridSelection(const SmallGICPConfig& cfg) {
  s_gicp_cfg_grid = cfg;
}

// Called at session start: loads map without resetting indoor_gicp_T_map_lidar.
// Caller (laserMapping reset block) seeds T_map_lidar immediately after this returns.
bool loadIndoorGICPMapForSession(const Eigen::Vector3d& ecef_m) {
  if (!indoorGridMapsLoaded()) {
    RCLCPP_ERROR(rclcpp::get_logger("ligo"),
                 "[indoor/gicp] grid maps not loaded at session start: ecef=(%.3f, %.3f, %.3f)",
                 ecef_m.x(), ecef_m.y(), ecef_m.z());
    return false;
  }

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
      RCLCPP_WARN(rclcpp::get_logger("ligo"),
                  "[indoor/gicp] lookup detail: %s",
                  debugIndoorGridLookup(ecef_m).c_str());
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
  s_refmap_display_T_inv.setIdentity();
  s_refmap_display_T_inv_valid = false;
  RCLCPP_WARN(rclcpp::get_logger("ligo"), "[indoor/gicp] map switched mid-session: map_id=%s  file=%s",
              opt->first.c_str(), pcd.c_str());
  return indoor_gicp_map_loaded;
}

void initIndoorGICP(const std::string& map_pcd_path, const SmallGICPConfig& cfg) {
  s_gicp_localizer = std::make_shared<SmallGICPLocalizer>(cfg);
  s_map_published = false;
  s_occ_grid_published = false;
  s_map_cloud_ready = false;
  s_map_cloud_sys_ready = false;
  s_refmap_display_T_inv.setIdentity();
  s_refmap_display_T_inv_valid = false;
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
  s_occ_grid_published = false;
  s_map_cloud_sys_ready = false;
  s_refmap_display_T_inv.setIdentity();
  s_refmap_display_T_inv_valid = false;
  ++s_gicp_session_nonce;
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
  static size_t s_gicp_rejected = 0;
  static uint64_t s_gicp_sess_seen = 0;
  if (s_gicp_sess_seen != s_gicp_session_nonce) {
    s_gicp_sess_seen   = s_gicp_session_nonce;
    s_gicp_total       = 0;
    s_gicp_converged   = 0;
    s_gicp_rejected    = 0;
  }

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
    // Global indoor constraint uses ONLY GICP-refined ENU pose (T_map_lidar ∘ body pose in map frame).
    computeEnuPoseFromGicpMapTransform(R_local_to_enu, t_local_to_enu, result.T_map_lidar,
                                       &indoor_pos_enu_meas, &indoor_rot_enu_meas);
    indoor_pose_time  = timestamp;
    indoor_pose_valid = true;
  }
  else
  {
    indoor_pose_valid = false;
  }

  if (!quality_ok) ++s_gicp_rejected;

  // One-shot display alignment: PCD is in "map file" frame; LIO scan is in map-local after sys_to_map.
  // GICP finds T with T * scan_map ≈ pcd, so shifting PCD by T^{-1} matches raw LIO in map frame.
  if (indoor_gicp_align_reference_map_to_lio && !s_refmap_display_T_inv_valid) {
    if (result.converged || s_gicp_total >= 25) {
      s_refmap_display_T_inv         = indoor_gicp_T_map_lidar.inverse();
      s_refmap_display_T_inv_valid = true;
      s_map_cloud_sys_ready          = false;
      s_map_published                = false;
      s_occ_grid_published           = false;
      const Eigen::Vector3d te = s_refmap_display_T_inv.translation();
      RCLCPP_INFO(rclcpp::get_logger("ligo"),
                  "[indoor/gicp] reference map display aligned to raw LIO (T_inv trans=%.3f,%.3f,%.3f m) "
                  "conv=%d frame=%zu",
                  te.x(), te.y(), te.z(), result.converged ? 1 : 0, s_gicp_total);
    }
  }

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
  const Eigen::Isometry3d Tpre =
      (indoor_gicp_align_reference_map_to_lio && s_refmap_display_T_inv_valid)
          ? s_refmap_display_T_inv
          : Eigen::Isometry3d::Identity();

  pcl::PointCloud<pcl::PointXYZI> raw;
  pcl::fromROSMsg(s_map_cloud_msg, raw);

  pcl::PointCloud<pcl::PointXYZI> transformed;
  transformed.reserve(raw.size());
  for (const auto& p : raw.points) {
    const Eigen::Vector3d pm = Tpre * Eigen::Vector3d(p.x, p.y, p.z);
    Eigen::Vector3d ps        = R_m2s * pm + t_m2s;
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
    double timestamp_sec,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub_occ_grid,
    bool defer_until_gicp_if_align) {
  if (!pub_map || !s_map_cloud_ready || s_map_published) return;
  if (indoor_gicp_align_reference_map_to_lio && !s_refmap_display_T_inv_valid &&
      defer_until_gicp_if_align) {
    return;
  }

  buildMapCloudSysEnu();

  const int32_t  sec     = static_cast<int32_t>(std::floor(timestamp_sec));
  const uint32_t nanosec = static_cast<uint32_t>(std::round((timestamp_sec - sec) * 1e9));

  auto& msg = s_map_cloud_sys_ready ? s_map_cloud_sys_msg : s_map_cloud_msg;
  msg.header.stamp.sec     = sec;
  msg.header.stamp.nanosec = nanosec;
  pub_map->publish(msg);
  s_map_published = true;
  maybePublishIndoorOccGrid(pub_occ_grid, timestamp_sec);
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
    double timestamp_sec,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub_occ_grid) {
  if (!s_gicp_localizer || !s_gicp_localizer->hasMap()) return;

  const int32_t  sec     = static_cast<int32_t>(std::floor(timestamp_sec));
  const uint32_t nanosec = static_cast<uint32_t>(std::round((timestamp_sec - sec) * 1e9));

  // --- map cloud (latched): defer if align mode until first GICP latch (same frame ok after runIndoorGICPUpdate) ---
  publishIndoorMapCloudOnly(pub_map, timestamp_sec, pub_occ_grid, true);

  // --- GICP-aligned scan in system-global-ENU (same pipeline as /indoor/map_cloud) ---
  if (pub_scan && scan_world && !scan_world->empty()) {
    const Eigen::Matrix3d R_gicp = T_map_lidar.rotation();
    const Eigen::Vector3d t_gicp = T_map_lidar.translation();
    const Eigen::Matrix3d R_m2s = s_R_sys_to_map.transpose();
    const Eigen::Vector3d t_m2s = -R_m2s * s_t_sys_to_map;
    // Must match buildMapCloudSysEnu(): optional Tpre shifts map-local points before sys-ENU,
    // so the aligned scan overlaps /indoor/map_cloud when indoor_gicp_align_reference_map_to_lio is on.
    const Eigen::Isometry3d Tpre_map_display =
        (indoor_gicp_align_reference_map_to_lio && s_refmap_display_T_inv_valid)
            ? s_refmap_display_T_inv
            : Eigen::Isometry3d::Identity();

    CloudT scan_aligned;
    scan_aligned.reserve(scan_world->size());
    for (const auto& p : scan_world->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      Eigen::Vector3d p_sys = R_local_to_enu * Eigen::Vector3d(p.x, p.y, p.z) + t_local_to_enu;
      // local → sys-ENU → map-local → GICP → (same Tpre as reference PCD) → sys-ENU
      if (s_sys_to_map_valid) {
        Eigen::Vector3d p_map = s_R_sys_to_map * p_sys + s_t_sys_to_map;
        Eigen::Vector3d p_aligned_map = R_gicp * p_map + t_gicp;
        const Eigen::Vector3d pm_for_viz = Tpre_map_display * p_aligned_map;
        p_sys = R_m2s * pm_for_viz + t_m2s;
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

// ---------------------------------------------------------------------------
// GTSAM factor insertion (indoor pose vs NMEA graph anchor)
// ---------------------------------------------------------------------------
void addIndoorFactorToGraph(int frame_num) {
  if (!(indoor_flag || indoor_flag_dynamic) || !indoor_pose_valid) return;
  if (!p_nmea) return;
  if (!indoorPoseNoise || !indoorPoseNoiseInit) return;

  const Eigen::Matrix3d rot_meas = indoor_rot_enu_meas.normalized().toRotationMatrix();
  const bool init_phase = (frame_num < p_nmea->delete_thred);
  const auto& noise = init_phase ? indoorPoseNoiseInit : indoorPoseNoise;
  p_nmea->p_assign->gtSAMgraph.add(ligo::IndoorLocalizationFactor(
      P(0), E(0), A(frame_num), R(frame_num),
      p_nmea->Tex_imu_r, p_nmea->anc_local,
      indoor_pos_enu_meas, rot_meas,
      indoor_gicp_factor_sqrt_info_scale,
      p_nmea->Rex_imu_r, noise));

  static size_t s_indoor_factor_cnt = 0;
  ++s_indoor_factor_cnt;
  if (s_indoor_factor_cnt <= 5 || s_indoor_factor_cnt % 50 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[INDOOR FACTOR INPUT (GICP ENU)] #%zu  frame=%d  pos_enu=(%.2f,%.2f,%.2f)  "
                "quat=(%.3f,%.3f,%.3f,%.3f)  sqrt_info=%.3f  noise=%s",
                s_indoor_factor_cnt, frame_num,
                indoor_pos_enu_meas[0], indoor_pos_enu_meas[1], indoor_pos_enu_meas[2],
                indoor_rot_enu_meas.w(), indoor_rot_enu_meas.x(),
                indoor_rot_enu_meas.y(), indoor_rot_enu_meas.z(),
                indoor_gicp_factor_sqrt_info_scale,
                init_phase ? "init" : "normal");
  }
}

}  // namespace indoor
}  // namespace ligo
