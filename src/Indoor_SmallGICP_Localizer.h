#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ligo {
namespace indoor {

struct SmallGICPConfig {
  int num_threads = 4;
  int num_neighbors = 20;
  int max_iterations = 30;

  double map_downsampling_resolution = 0.20;
  double scan_downsampling_resolution = 0.20;
  double max_correspondence_distance = 2.0;
};

struct SmallGICPResult {
  bool success = false;
  bool converged = false;

  Eigen::Isometry3d T_map_lidar = Eigen::Isometry3d::Identity();
  size_t iterations = 0;
  size_t num_inliers = 0;
  double error = 0.0;
};

class SmallGICPLocalizer {
public:
  using LidarPoint = pcl::PointXYZINormal;
  using LidarCloud = pcl::PointCloud<LidarPoint>;

  explicit SmallGICPLocalizer(const SmallGICPConfig& config = SmallGICPConfig());

  void setConfig(const SmallGICPConfig& config);
  SmallGICPConfig getConfig() const;

  bool loadMapFromPLY(const std::string& ply_path);
  bool setMap(const LidarCloud::ConstPtr& map_cloud);
  bool hasMap() const;

  SmallGICPResult localize(const LidarCloud::ConstPtr& scan_cloud, const Eigen::Isometry3d& init_T_map_lidar) const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace indoor
}  // namespace ligo

