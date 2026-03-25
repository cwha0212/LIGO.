#include "Indoor_SmallGICP_Localizer.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace ligo {
namespace indoor {

namespace {

small_gicp::PointCloud::Ptr toSmallGICPCloud(const SmallGICPLocalizer::LidarCloud& cloud) {
  auto out = std::make_shared<small_gicp::PointCloud>();
  out->resize(cloud.size());

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& p = cloud.points[i];
    out->point(i) << static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z), 1.0;
  }

  return out;
}

}  // namespace

struct SmallGICPLocalizer::Impl {
  using Tree = small_gicp::KdTree<small_gicp::PointCloud>;

  mutable std::mutex mutex;
  SmallGICPConfig config;
  small_gicp::PointCloud::Ptr map;
  std::shared_ptr<Tree> map_tree;
};

SmallGICPLocalizer::SmallGICPLocalizer(const SmallGICPConfig& config) : impl_(std::make_shared<Impl>()) {
  impl_->config = config;
}

void SmallGICPLocalizer::setConfig(const SmallGICPConfig& config) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->config = config;
}

SmallGICPConfig SmallGICPLocalizer::getConfig() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->config;
}

bool SmallGICPLocalizer::loadMapFromPLY(const std::string& ply_path) {
  auto cloud_xyzi_n = pcl::make_shared<LidarCloud>();
  if (pcl::io::loadPLYFile<LidarPoint>(ply_path, *cloud_xyzi_n) == 0) {
    return setMap(cloud_xyzi_n);
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, cloud_xyz) != 0) {
    std::cerr << "[SmallGICPLocalizer] failed to load map PLY: " << ply_path << std::endl;
    return false;
  }

  auto converted = pcl::make_shared<LidarCloud>();
  converted->reserve(cloud_xyz.size());
  for (const auto& p : cloud_xyz.points) {
    LidarPoint q;
    q.x = p.x;
    q.y = p.y;
    q.z = p.z;
    q.intensity = 0.0f;
    q.normal_x = 0.0f;
    q.normal_y = 0.0f;
    q.normal_z = 0.0f;
    q.curvature = 0.0f;
    converted->push_back(q);
  }

  return setMap(converted);
}

bool SmallGICPLocalizer::setMap(const LidarCloud::ConstPtr& map_cloud) {
  if (!map_cloud || map_cloud->empty()) {
    std::cerr << "[SmallGICPLocalizer] map cloud is empty." << std::endl;
    return false;
  }

  SmallGICPConfig cfg;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    cfg = impl_->config;
  }

  auto map_raw = toSmallGICPCloud(*map_cloud);
  auto map_ds = small_gicp::voxelgrid_sampling_omp(*map_raw, cfg.map_downsampling_resolution, cfg.num_threads);
  auto map_tree = std::make_shared<Impl::Tree>(map_ds, small_gicp::KdTreeBuilderOMP(cfg.num_threads));
  small_gicp::estimate_covariances_omp(*map_ds, *map_tree, cfg.num_neighbors, cfg.num_threads);

  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->map = map_ds;
    impl_->map_tree = map_tree;
  }

  return true;
}

bool SmallGICPLocalizer::hasMap() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->map && impl_->map_tree && !impl_->map->empty();
}

SmallGICPResult SmallGICPLocalizer::localize(const LidarCloud::ConstPtr& scan_cloud, const Eigen::Isometry3d& init_T_map_lidar) const {
  SmallGICPResult out;
  if (!scan_cloud || scan_cloud->empty()) {
    std::cerr << "[SmallGICPLocalizer] scan cloud is empty." << std::endl;
    return out;
  }

  SmallGICPConfig cfg;
  small_gicp::PointCloud::Ptr map_points;
  std::shared_ptr<Impl::Tree> map_tree;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->map || !impl_->map_tree || impl_->map->empty()) {
      std::cerr << "[SmallGICPLocalizer] map is not initialized." << std::endl;
      return out;
    }
    cfg = impl_->config;
    map_points = impl_->map;
    map_tree = impl_->map_tree;
  }

  auto scan_raw = toSmallGICPCloud(*scan_cloud);
  auto scan_ds = small_gicp::voxelgrid_sampling_omp(*scan_raw, cfg.scan_downsampling_resolution, cfg.num_threads);
  auto scan_tree = std::make_shared<Impl::Tree>(scan_ds, small_gicp::KdTreeBuilderOMP(cfg.num_threads));
  small_gicp::estimate_covariances_omp(*scan_ds, *scan_tree, cfg.num_neighbors, cfg.num_threads);

  small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
  registration.reduction.num_threads = cfg.num_threads;
  registration.rejector.max_dist_sq = cfg.max_correspondence_distance * cfg.max_correspondence_distance;
  registration.optimizer.max_iterations = cfg.max_iterations;

  const auto result = registration.align(*map_points, *scan_ds, *map_tree, init_T_map_lidar);

  out.success = true;
  out.converged = result.converged;
  out.T_map_lidar = result.T_target_source;
  out.iterations = result.iterations;
  out.num_inliers = result.num_inliers;
  out.error = result.error;

  return out;
}

}  // namespace indoor
}  // namespace ligo

