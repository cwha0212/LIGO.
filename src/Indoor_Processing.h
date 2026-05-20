#pragma once

#include <optional>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/publisher.hpp>

#include "Indoor_SmallGICP_Localizer.h"

namespace ligo {
namespace indoor {

using PointT = pcl::PointXYZINormal;
using CloudT = pcl::PointCloud<PointT>;

/** Update indoor ENU pose measurement from LIO state (no-op when outdoors). */
void updateIndoorLocalizationPlaceholder(const Eigen::Vector3d& pos_enu,
                                         const Eigen::Matrix3d& rot_enu,
                                         double ts_sec);

/** Stub kept for reference; actual insertion is via addIndoorFactorToGraph(). */
void addIndoorFactorToGraphStubCommented();

/** Config used when loading PCD chosen by indoor grid membership (see ensureIndoorGICPMapFromGridEcef). */
void setIndoorGICPConfigForGridSelection(const SmallGICPConfig& cfg);

/** At indoor session start: load GICP map for the given ECEF position via grid lookup.
 *  Does NOT call resetIndoorGICP() — caller is responsible for seeding T_map_lidar afterward. */
bool loadIndoorGICPMapForSession(const Eigen::Vector3d& ecef_m);

/** During an active session: if the rover has moved to a different grid cell, reload the map. */
bool ensureIndoorGICPMapFromGridEcef(const Eigen::Vector3d& ecef_m);

/** Load the indoor reference map PCD into the GICP localizer. */
void initIndoorGICP(const std::string& map_pcd_path, const SmallGICPConfig& cfg);

/** Reset GICP pose tracking (call on every indoor/outdoor transition). */
void resetIndoorGICP();

/** Returns the absolute path of the currently loaded reference PCD, or empty string. */
std::string getIndoorGicpMapPath();

/** Convert an ECEF position to the currently loaded map's local ENU frame.
 *  Uses the ecef_from_enu transform stored in the map's grid YAML.
 *  Returns nullopt if no map is loaded or the map has no ECEF transform. */
std::optional<Eigen::Vector3d> ecefToLoadedMapEnu(const Eigen::Vector3d& ecef);

/** Provide the system (GNSS) ECEF anchor so that scan points in system-ENU
 *  can be converted to the loaded map's local ENU during GICP. */
void setSystemEcefAnchor(const Eigen::Vector3d& anc_ecef,
                         const Eigen::Matrix3d& R_ecef_enu);

/** Run scan-to-map GICP; on quality pass sets indoor_* to GICP-refined ENU pose only.
 *  scan_world: current scan in LIO local/world frame.
 *  Returns true after successful registration (even if quality gate fails). */
bool runIndoorGICPUpdate(const CloudT::ConstPtr& scan_world,
                         double timestamp,
                         const Eigen::Matrix3d& R_local_to_enu,
                         const Eigen::Vector3d& t_local_to_enu);

/** Publish just the indoor reference map cloud (latched).
 *  Safe to call every frame; no-op after first successful publish. */
void publishIndoorMapCloudOnly(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_map,
    double timestamp_sec,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub_occ_grid = nullptr,
    /** When align_reference_map_to_lio: skip publish until GICP warms up if this is true (same frame can publish after runIndoorGICPUpdate). */
    bool defer_until_gicp_if_align = false);

/** Publish indoor map (latched, first call or after map change) and GICP-aligned scan.
 *  Called every frame regardless of convergence so RViz shows live scan vs map. */
void publishIndoorViz(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_map,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_scan,
    const CloudT::ConstPtr& scan_world,
    const Eigen::Matrix3d& R_local_to_enu,
    const Eigen::Vector3d& t_local_to_enu,
    const Eigen::Isometry3d& T_map_lidar,
    double timestamp_sec,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& pub_occ_grid = nullptr);

/** Add one IndoorLocalizationFactor: global ENU measurement = GICP-refined pose only (quality-gated in runIndoorGICPUpdate). */
void addIndoorFactorToGraph(int frame_num);

}  // namespace indoor
}  // namespace ligo
