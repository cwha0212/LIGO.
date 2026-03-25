#pragma once

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/publisher.hpp>

#ifdef LIGO_WITH_SMALL_GICP
#include "Indoor_SmallGICP_Localizer.h"
#endif

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

#ifdef LIGO_WITH_SMALL_GICP
/** Load the indoor reference map PCD into the GICP localizer. */
void initIndoorGICP(const std::string& map_pcd_path, const SmallGICPConfig& cfg);

/** Reset GICP pose tracking (call on every indoor/outdoor transition). */
void resetIndoorGICP();

/** Run scan-to-map GICP and update indoor_pos_enu_meas / indoor_rot_enu_meas.
 *  scan_world: current scan in LIO local/world frame.
 *  Returns true on GICP convergence. */
bool runIndoorGICPUpdate(const CloudT::ConstPtr& scan_world,
                         double timestamp,
                         const Eigen::Matrix3d& R_local_to_enu,
                         const Eigen::Vector3d& t_local_to_enu);

/** Publish indoor map (one-shot) and aligned current scan to RViz. */
void publishIndoorViz(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_map,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub_scan,
    const CloudT::ConstPtr& scan_world,
    const Eigen::Matrix3d& R_local_to_enu,
    const Eigen::Vector3d& t_local_to_enu,
    double timestamp_sec);
#endif  // LIGO_WITH_SMALL_GICP

/** Add IndoorLocalizationFactor to the GTSAM graph for the given NMEA frame index.
 *  frame_num must correspond to A(frame_num), R(frame_num) already in the graph. */
void addIndoorFactorToGraph(int frame_num);

}  // namespace indoor
}  // namespace ligo
