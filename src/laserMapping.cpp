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

// #include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <atomic>
#include "li_initialization.h"
#include <ligo/msg/nmea_heading_align_status.hpp>

/** Implemented in li_initialization.cpp (not declared in li_initialization.h to avoid include-order / GTSAM issues). */
void ligo_try_create_nmea_stamp_diag_publisher(std::shared_ptr<rclcpp::Node> node);

#include "Indoor_Processing.h"
#include "Indoor_GridMapRegistry.h"
#include <malloc.h>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <limits>
#include <cstdint>
#include <cstring>
#include <string>
#include <filesystem>
#include <iomanip>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "chi-square.h"
#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

bool init_map = false, flg_first_scan = true;
nav_msgs::msg::Odometry::SharedPtr nmea_cur;
Eigen::Vector3d first_pvt_anc, first_lla_anc;
Eigen::Vector3d first_pvt_used, first_lla_used;

bool  flg_reset = false, flg_exit = false;
bool  flg_reset_indoor_reloc = false;
bool  indoor_reloc_applied_once = false;
Eigen::Vector3d indoor_reloc_pos_enu = Eigen::Vector3d::Zero();
Eigen::Matrix3d indoor_reloc_rot_enu = Eigen::Matrix3d::Identity();
double indoor_reloc_pose_time = 0.0;
// Last GNSS position (ECEF) received with acceptable covariance (outdoor quality).
// Converted to the loaded map's local ENU at indoor session start for the GICP seed.
Eigen::Vector3d last_good_gnss_ecef       = Eigen::Vector3d::Zero();
bool            last_good_gnss_ecef_valid = false;

/** Set by /ligo/indoor_mode true; main loop snaps LIO→ENU anchor then clears. */
static std::atomic<bool> g_pending_indoor_topic_snap{false};

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;
nav_msgs::msg::Path nmea_aligned_path;
nav_msgs::msg::Odometry nmea_aligned_odom;
geometry_msgs::msg::PoseStamped nmea_aligned_pose;

static inline bool nmeaCovarianceIsHigh(const nav_msgs::msg::Odometry::SharedPtr &msg,
                                        double threshold)
{
    return msg->pose.covariance[0] >= threshold ||
           msg->pose.covariance[7] >= threshold ||
           msg->pose.covariance[14] >= threshold;
}

// Matches NMEAProcess::processNMEA gate for collecting alignment window (reject if any diagonal > thr).
static inline bool nmeaCovarianceAcceptableForNmeaInit(const nav_msgs::msg::Odometry::SharedPtr &msg,
                                                        double threshold)
{
    return msg->pose.covariance[0] <= threshold &&
           msg->pose.covariance[7] <= threshold &&
           msg->pose.covariance[14] <= threshold;
}

static std::string ligo_json_escape(const std::string &s)
{
    std::string o;
    o.reserve(s.size() + 8);
    for (unsigned char uc : s)
    {
        const char c = static_cast<char>(uc);
        switch (c)
        {
        case '\\':
            o += "\\\\";
            break;
        case '"':
            o += "\\\"";
            break;
        case '\n':
            o += "\\n";
            break;
        case '\r':
            o += "\\r";
            break;
        default:
            o += c;
        }
    }
    return o;
}

static int nmea_outdoor_good_streak = 0;
static bool nmea_cycle_reopen_pending = false;
static std::atomic<bool> pending_outdoor_realign_ivox_reset{false};

static void nmeaMaybeTriggerOutdoorRealignAfterIndoor(const nav_msgs::msg::Odometry::SharedPtr &nmea_cur)
{
    if (!NMEA_ENABLE)
        return;
    if (!p_nmea->nmea_ready || !indoor_reloc_applied_once)
        return;
    const double thr = p_nmea->p_assign->ppp_std_threshold;
    const int need = p_nmea->wind_size < 1 ? 1 : p_nmea->wind_size;
    if (!nmeaCovarianceAcceptableForNmeaInit(nmea_cur, thr))
    {
        nmea_outdoor_good_streak = 0;
        return;
    }
    nmea_outdoor_good_streak++;
    if (nmea_outdoor_good_streak >= need)
    {
        RCLCPP_WARN(rclcpp::get_logger("ligo"),
                    "NMEA outdoor re-align: graph reset after %d good-covariance samples (ICP retained; IVox reset deferred)",
                    nmea_outdoor_good_streak);
        p_nmea->ResetGraphClearingInitRetainIcp();
        indoor_flag_dynamic = false;
        ligo::indoor::resetIndoorGICP();
        nmea_cycle_reopen_pending = true;
        nmea_outdoor_good_streak = 0;
    }
}

static void nmeaClearCycleIfRealignComplete()
{
    if (!nmea_cycle_reopen_pending || !p_nmea->nmea_ready)
        return;
    indoor_reloc_applied_once = false;
    nmea_cycle_reopen_pending = false;
    nmea_outdoor_good_streak = 0;
    indoor_flag_dynamic = false;
    ligo::indoor::resetIndoorGICP();
    pending_outdoor_realign_ivox_reset.store(true, std::memory_order_release);
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "Outdoor re-align complete: scheduling deferred IVox/traj reset; indoor GICP cleared");
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void MapIncremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);
    
    for (size_t i = 0; i < cur_pts; ++i) {
        /* decide if need add to map */
        PointType &point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }
    ivox_->AddPoints(points_to_add);
}

void publish_init_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
    int size_init_map = init_feats_world->size();

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
                
    pcl::toROSMsg(*init_feats_world, laserCloudmsg);
        
    laserCloudmsg.header.stamp.sec = static_cast<int32_t>(std::floor(lidar_end_time));
    laserCloudmsg.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time - std::floor(lidar_end_time)) * 1e9));
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
static std::vector<Eigen::Vector3d> pcl_wait_ray_origins;
PointCloudXYZI::Ptr pcl_wait_save_tmp_map(new PointCloudXYZI());
static std::vector<Eigen::Vector3d> pcl_wait_tmp_map_ray_origins;
static double g_tmp_map_time_base_sec = std::numeric_limits<double>::quiet_NaN();
static long long g_tmp_map_bucket_idx = -1;

static bool ligo_try_write_binary_pcd(const std::string &file_path, const PointCloudXYZI::Ptr &cloud);
static std::string ligo_replace_pcd_suffix(const std::string &pcd_path, const std::string &suffix);

static std::filesystem::path ligo_make_map_root_dir()
{
    return std::filesystem::path(ROOT_DIR) / "PCD" / pcd_save_map_name / pcd_save_sub_map_name;
}

/** Single sub-map directory: `PCD/{map_name}/{sub_map_name}/{sub_map_name}.pcd` (overwrite on each save). */
static std::string ligo_make_pcd_save_path()
{
    return (ligo_make_map_root_dir() / (pcd_save_sub_map_name + ".pcd")).string();
}

static std::string ligo_make_tmp_map_save_path(long long bucket_idx, double interval_sec)
{
    const long long idx = std::max(0LL, bucket_idx);
    const long long start_sec = std::max(0LL, static_cast<long long>(std::llround(static_cast<double>(idx) * interval_sec)));
    const long long end_sec = std::max(start_sec + 1LL, static_cast<long long>(std::llround(static_cast<double>(idx + 1LL) * interval_sec)));
    char name_buf[128];
    std::snprintf(name_buf, sizeof(name_buf), "%s_%05lld_%06lld.pcd", pcd_save_map_name.c_str(), start_sec, end_sec);
    return (std::filesystem::path(ROOT_DIR) / "tmp_map" / name_buf).string();
}

static PointCloudXYZI::Ptr ligo_convert_enu_cloud_to_ecef(
    const PointCloudXYZI::Ptr &cloud_enu,
    const Eigen::Matrix3d &R_ecef_enu,
    const Eigen::Vector3d &anchor_ecef_m)
{
    if (!cloud_enu || cloud_enu->empty())
    {
        return PointCloudXYZI::Ptr(new PointCloudXYZI());
    }
    PointCloudXYZI::Ptr cloud_ecef(new PointCloudXYZI(static_cast<int>(cloud_enu->size()), 1));
    for (size_t i = 0; i < cloud_enu->size(); ++i)
    {
        const auto &src = cloud_enu->points[i];
        const Eigen::Vector3d p_enu(src.x, src.y, src.z);
        const Eigen::Vector3d p_ecef = anchor_ecef_m + R_ecef_enu * p_enu;
        auto &dst = cloud_ecef->points[i];
        dst.x = static_cast<float>(p_ecef.x());
        dst.y = static_cast<float>(p_ecef.y());
        dst.z = static_cast<float>(p_ecef.z());
        dst.intensity = src.intensity;
    }
    return cloud_ecef;
}

static bool ligo_save_ecef_companion_pcd(
    const std::string &enu_pcd_path,
    const PointCloudXYZI::Ptr &cloud_enu,
    const Eigen::Matrix3d &R_ecef_enu,
    const Eigen::Vector3d &anchor_ecef_m)
{
    const std::string ecef_path = ligo_replace_pcd_suffix(enu_pcd_path, "_ecef.pcd");
    const PointCloudXYZI::Ptr cloud_ecef =
        ligo_convert_enu_cloud_to_ecef(cloud_enu, R_ecef_enu, anchor_ecef_m);
    if (!ligo_try_write_binary_pcd(ecef_path, cloud_ecef))
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "[pcd] failed to save ECEF companion: %s", ecef_path.c_str());
        return false;
    }
    return true;
}

static bool ligo_flush_tmp_map_bucket(const rclcpp::Logger &logger)
{
    if (!pcd_tmp_map_enable || !mapping_mode)
    {
        return false;
    }
    if (!pcl_wait_save_tmp_map || pcl_wait_save_tmp_map->empty() || g_tmp_map_bucket_idx < 0)
    {
        return false;
    }
    const std::string out_path = ligo_make_tmp_map_save_path(g_tmp_map_bucket_idx, pcd_tmp_map_interval_sec);
    if (!ligo_try_write_binary_pcd(out_path, pcl_wait_save_tmp_map))
    {
        RCLCPP_ERROR(logger, "[tmp_map] failed to save bucket file: %s", out_path.c_str());
        return false;
    }
    RCLCPP_INFO(
        logger,
        "[tmp_map] saved split map: bucket=%lld points=%zu path=%s",
        g_tmp_map_bucket_idx,
        pcl_wait_save_tmp_map->size(),
        out_path.c_str());
    pcl_wait_save_tmp_map->clear();
    pcl_wait_tmp_map_ray_origins.clear();
    return true;
}

static void ligo_cleanup_tmp_map_pcd_files(const rclcpp::Logger &logger)
{
    const std::filesystem::path tmp_dir = std::filesystem::path(ROOT_DIR) / "tmp_map";
    try
    {
        if (!std::filesystem::exists(tmp_dir) || !std::filesystem::is_directory(tmp_dir))
        {
            return;
        }
        size_t removed_count = 0;
        for (const auto &entry : std::filesystem::directory_iterator(tmp_dir))
        {
            if (!entry.is_regular_file())
            {
                continue;
            }
            const std::filesystem::path p = entry.path();
            if (p.extension() != ".pcd")
            {
                continue;
            }
            std::error_code ec;
            if (std::filesystem::remove(p, ec) && !ec)
            {
                ++removed_count;
            }
            else if (ec)
            {
                RCLCPP_WARN(
                    logger,
                    "[tmp_map] failed to remove %s: %s",
                    p.string().c_str(),
                    ec.message().c_str());
            }
        }
        RCLCPP_INFO(logger, "[tmp_map] cleanup complete: removed %zu pcd files", removed_count);
    }
    catch (const std::exception &e)
    {
        RCLCPP_WARN(logger, "[tmp_map] cleanup skipped due to error: %s", e.what());
    }
}

static void ligo_tmp_map_on_scan(
    const PointCloudXYZI::Ptr &cloud,
    const Eigen::Vector3d &ray_origin,
    double lidar_stamp_sec,
    const rclcpp::Logger &logger)
{
    if (!pcd_tmp_map_enable || !mapping_mode || !cloud || cloud->empty())
    {
        return;
    }
    if (pcd_tmp_map_interval_sec <= 0.0)
    {
        RCLCPP_WARN(logger, "[tmp_map] invalid interval_sec=%.6f", pcd_tmp_map_interval_sec);
        return;
    }
    if (!std::isfinite(g_tmp_map_time_base_sec))
    {
        g_tmp_map_time_base_sec = lidar_stamp_sec;
    }
    const double elapsed_sec = std::max(0.0, lidar_stamp_sec - g_tmp_map_time_base_sec);
    const long long bucket_idx = static_cast<long long>(std::floor(elapsed_sec / pcd_tmp_map_interval_sec));
    if (g_tmp_map_bucket_idx < 0)
    {
        g_tmp_map_bucket_idx = bucket_idx;
    }
    else if (bucket_idx != g_tmp_map_bucket_idx)
    {
        if (ligo_flush_tmp_map_bucket(logger))
        {
            g_tmp_map_bucket_idx = bucket_idx;
        }
    }

    *pcl_wait_save_tmp_map += *cloud;
    pcl_wait_tmp_map_ray_origins.insert(
        pcl_wait_tmp_map_ray_origins.end(),
        cloud->size(),
        ray_origin);
}

static std::string ligo_replace_pcd_suffix(const std::string &pcd_path, const std::string &suffix)
{
    if (pcd_path.size() >= 4 && pcd_path.substr(pcd_path.size() - 4) == ".pcd")
    {
        return pcd_path.substr(0, pcd_path.size() - 4) + suffix;
    }
    return pcd_path + suffix;
}

static bool ligo_ensure_parent_dir(const std::string &file_path)
{
    try
    {
        const std::filesystem::path p(file_path);
        const std::filesystem::path parent = p.parent_path();
        if (parent.empty())
        {
            return true;
        }
        if (std::filesystem::exists(parent))
        {
            return true;
        }
        return std::filesystem::create_directories(parent);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "[pcd] failed to create parent dir: %s", e.what());
        return false;
    }
}

static bool ligo_try_write_binary_pcd(const std::string &file_path, const PointCloudXYZI::Ptr &cloud)
{
    if (!cloud)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "[pcd] null cloud pointer");
        return false;
    }
    if (!ligo_ensure_parent_dir(file_path))
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "[pcd] parent directory is unavailable: %s", file_path.c_str());
        return false;
    }
    try
    {
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(file_path, *cloud);
        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "[pcd] writeBinary failed: %s (%s)", file_path.c_str(), e.what());
        return false;
    }
}

static void save_grid2d_from_cloud_with_rays(
    const PointCloudXYZI::Ptr &cloud,
    const std::vector<Eigen::Vector3d> &ray_origins,
    const std::string &pcd_path,
    const std::string &frame_id,
    double resolution_m,
    int min_points_per_cell = 3,
    double z_min_m = -1e9,
    double z_max_m = 1e9,
    bool add_enu_to_ecef_metadata = false,
    const Eigen::Matrix3d &R_ecef_enu = Eigen::Matrix3d::Identity(),
    const Eigen::Vector3d &anchor_ecef_m = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d &anchor_lla_deg_m = Eigen::Vector3d::Zero())
{
    if (!cloud || cloud->empty() || resolution_m <= 0.0 || ray_origins.size() != cloud->size())
    {
        return;
    }

    std::unordered_map<uint64_t, uint32_t> cell_counts;
    cell_counts.reserve(cloud->size() / 2 + 1);

    int min_ix = std::numeric_limits<int>::max();
    int min_iy = std::numeric_limits<int>::max();
    int max_ix = std::numeric_limits<int>::min();
    int max_iy = std::numeric_limits<int>::min();

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const auto &pt = cloud->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        {
            continue;
        }
        if (pt.z < z_min_m || pt.z > z_max_m)
        {
            continue;
        }

        const int ix = static_cast<int>(std::floor(static_cast<double>(pt.x) / resolution_m));
        const int iy = static_cast<int>(std::floor(static_cast<double>(pt.y) / resolution_m));
        const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
                             static_cast<uint32_t>(iy);
        cell_counts[key] += 1U;

        min_ix = std::min(min_ix, ix);
        min_iy = std::min(min_iy, iy);
        max_ix = std::max(max_ix, ix);
        max_iy = std::max(max_iy, iy);

        const Eigen::Vector3d &org = ray_origins[i];
        if (std::isfinite(org.x()) && std::isfinite(org.y()))
        {
            const int iox = static_cast<int>(std::floor(org.x() / resolution_m));
            const int ioy = static_cast<int>(std::floor(org.y() / resolution_m));
            min_ix = std::min(min_ix, iox);
            min_iy = std::min(min_iy, ioy);
            max_ix = std::max(max_ix, iox);
            max_iy = std::max(max_iy, ioy);
        }
    }

    if (cell_counts.empty())
    {
        return;
    }

    const std::string pgm_path = ligo_replace_pcd_suffix(pcd_path, "_grid2d.pgm");
    const std::string yaml_path = ligo_replace_pcd_suffix(pcd_path, "_grid2d.yaml");

    const int width = max_ix - min_ix + 1;
    const int height = max_iy - min_iy + 1;
    if (width <= 0 || height <= 0)
    {
        return;
    }
    constexpr uint64_t kMaxPgmPixels = 120000000ULL;
    const uint64_t num_pixels = static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
    if (num_pixels > kMaxPgmPixels)
    {
        RCLCPP_WARN(
            rclcpp::get_logger("ligo"),
            "[map/grid2d] skip pgm export: image too large (%d x %d = %llu pixels)",
            width, height, static_cast<unsigned long long>(num_pixels));
        return;
    }

    // PGM occupancy encoding:
    //   0   : occupied
    //   254 : free
    //   205 : unknown
    std::vector<unsigned char> image(static_cast<size_t>(num_pixels), 205);

    auto mark_cell = [&](int ix, int iy, unsigned char value) {
        const int col = ix - min_ix;
        const int row = max_iy - iy;
        if (row >= 0 && row < height && col >= 0 && col < width)
        {
            const size_t idx = static_cast<size_t>(row) * static_cast<size_t>(width) + static_cast<size_t>(col);
            // Keep occupied strongest.
            if (image[idx] != 0 || value == 0)
            {
                image[idx] = value;
            }
        }
    };

    // Ray-cast free cells from sensor origin to hit point endpoint (endpoint excluded).
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const auto &pt = cloud->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        {
            continue;
        }
        if (pt.z < z_min_m || pt.z > z_max_m)
        {
            continue;
        }
        const Eigen::Vector3d &org = ray_origins[i];
        if (!std::isfinite(org.x()) || !std::isfinite(org.y()))
        {
            continue;
        }

        int x0 = static_cast<int>(std::floor(org.x() / resolution_m));
        int y0 = static_cast<int>(std::floor(org.y() / resolution_m));
        const int x1 = static_cast<int>(std::floor(static_cast<double>(pt.x) / resolution_m));
        const int y1 = static_cast<int>(std::floor(static_cast<double>(pt.y) / resolution_m));

        int dx = std::abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true)
        {
            if (!(x0 == x1 && y0 == y1))
            {
                mark_cell(x0, y0, 254);
            }
            if (x0 == x1 && y0 == y1)
            {
                break;
            }
            const int e2 = 2 * err;
            if (e2 >= dy)
            {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    size_t occupied_cells = 0;
    for (const auto &kv : cell_counts)
    {
        const uint64_t key = kv.first;
        const int ix = static_cast<int32_t>(static_cast<uint32_t>(key >> 32));
        const int iy = static_cast<int32_t>(static_cast<uint32_t>(key & 0xffffffffULL));
        const uint32_t cnt = kv.second;
        const int occupied = (cnt >= static_cast<uint32_t>(std::max(1, min_points_per_cell))) ? 1 : 0;
        if (!occupied)
        {
            continue;
        }
        occupied_cells++;

        mark_cell(ix, iy, 0);
    }

    std::ofstream f_pgm(pgm_path, std::ios::binary);
    if (!f_pgm.is_open())
    {
        RCLCPP_WARN(rclcpp::get_logger("ligo"), "failed to open grid pgm: %s", pgm_path.c_str());
        return;
    }
    f_pgm << "P5\n" << width << " " << height << "\n255\n";
    f_pgm.write(reinterpret_cast<const char *>(image.data()), static_cast<std::streamsize>(image.size()));
    f_pgm.close();

    std::ofstream f_yaml(yaml_path);
    if (!f_yaml.is_open())
    {
        RCLCPP_WARN(rclcpp::get_logger("ligo"), "failed to open grid yaml: %s", yaml_path.c_str());
        return;
    }
    // Keep high precision in metadata for large-coordinate frames.
    f_yaml << std::fixed << std::setprecision(17);
    f_yaml << "image: " << pgm_path << "\n";
    f_yaml << "resolution: " << resolution_m << "\n";
    f_yaml << "origin: ["
           << (static_cast<double>(min_ix) * resolution_m) << ", "
           << (static_cast<double>(min_iy) * resolution_m) << ", 0.0]\n";
    f_yaml << "negate: 0\n";
    f_yaml << "occupied_thresh: 0.65\n";
    f_yaml << "free_thresh: 0.196\n";
    f_yaml << "mode: trinary\n";
    f_yaml << "source_pcd: " << pcd_path << "\n";
    f_yaml << "frame_id: " << frame_id << "\n";
    f_yaml << "grid_type: occupancy_2d_raycast\n";
    f_yaml << "min_points_per_cell: " << std::max(1, min_points_per_cell) << "\n";
    f_yaml << "z_filter_m: [" << z_min_m << ", " << z_max_m << "]\n";
    f_yaml << "width_cells: " << width << "\n";
    f_yaml << "height_cells: " << height << "\n";
    f_yaml << "num_cells_total: " << cell_counts.size() << "\n";
    f_yaml << "num_cells_occupied: " << occupied_cells << "\n";
    if (add_enu_to_ecef_metadata)
    {
        f_yaml << "ecef_from_enu:\n";
        f_yaml << "  equation: p_ecef = anchor_ecef_m + R_ecef_enu * p_enu\n";
        f_yaml << "  anchor_ecef_m: ["
               << anchor_ecef_m.x() << ", "
               << anchor_ecef_m.y() << ", "
               << anchor_ecef_m.z() << "]\n";
        f_yaml << "  anchor_lla_deg_m: ["
               << anchor_lla_deg_m.x() << ", "
               << anchor_lla_deg_m.y() << ", "
               << anchor_lla_deg_m.z() << "]\n";
        f_yaml << "  R_ecef_enu_row_major: ["
               << R_ecef_enu(0, 0) << ", " << R_ecef_enu(0, 1) << ", " << R_ecef_enu(0, 2) << ", "
               << R_ecef_enu(1, 0) << ", " << R_ecef_enu(1, 1) << ", " << R_ecef_enu(1, 2) << ", "
               << R_ecef_enu(2, 0) << ", " << R_ecef_enu(2, 1) << ", " << R_ecef_enu(2, 2) << "]\n";
    }
    f_yaml.close();

    RCLCPP_INFO(
        rclcpp::get_logger("ligo"),
        "[map/grid2d] saved pgm grid: frame=%s cells=%zu occupied=%zu res=%.2f -> %s",
        frame_id.c_str(),
        cell_counts.size(),
        occupied_cells,
        resolution_m,
        pgm_path.c_str());
}

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body); // (points_num); // 
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        bool use_enu_for_pub = false;
        use_enu_for_pub = (NMEA_ENABLE && p_nmea && p_nmea->icp_tf_ready);
        Eigen::Matrix3d R_local_to_enu = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_local_to_enu = Eigen::Vector3d::Zero();
        if (use_enu_for_pub)
        {
            R_local_to_enu = p_nmea->icp_R_local_to_enu;
            t_local_to_enu = p_nmea->icp_t_local_to_enu;
        }
        
        for (int i = 0; i < size; i++)
        {
            const Eigen::Vector3d p_local(
                feats_down_world->points[i].x,
                feats_down_world->points[i].y,
                feats_down_world->points[i].z);
            const Eigen::Vector3d p_pub = use_enu_for_pub ? (R_local_to_enu * p_local + t_local_to_enu) : p_local;
            laserCloudWorld->points[i].x = p_pub.x();
            laserCloudWorld->points[i].y = p_pub.y();
            laserCloudWorld->points[i].z = p_pub.z();
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        
        laserCloudmsg.header.stamp.sec = static_cast<int32_t>(std::floor(lidar_end_time));
        laserCloudmsg.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time - std::floor(lidar_end_time)) * 1e9));
        laserCloudmsg.header.frame_id = use_enu_for_pub ? "map" : "camera_init";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        // publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (mapping_mode)
    {
        if (NMEA_ENABLE && p_nmea)
        {
            static int scan_wait_num = 0;
            if (!p_nmea->icp_tf_ready)
            {
                if (pcl_wait_save->size() > 0)
                    pcl_wait_save->clear();
                if (!pcl_wait_ray_origins.empty())
                    pcl_wait_ray_origins.clear();
                scan_wait_num = 0;
            }
            else
            {
                const int size = static_cast<int>(feats_down_world->points.size());
                if (size <= 0)
                {
                    return;
                }
                PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

                const Eigen::Matrix3d R_local_to_enu = p_nmea->icp_R_local_to_enu;
                const Eigen::Vector3d t_local_to_enu = p_nmea->icp_t_local_to_enu;
                const Eigen::Matrix3d R_ecef_enu = gnss_comm::ecef2rotation(first_gps_ecef);
                const Eigen::Vector3d origin_local = kf_output.x_.pos;
                const Eigen::Vector3d origin_enu = R_local_to_enu * origin_local + t_local_to_enu;
                const Eigen::Vector3d origin_ecef = first_gps_ecef + R_ecef_enu * origin_enu;

                for (int i = 0; i < size; i++)
                {
                    const Eigen::Vector3d p_local(
                        feats_down_world->points[i].x,
                        feats_down_world->points[i].y,
                        feats_down_world->points[i].z);

                    const Eigen::Vector3d p_enu = R_local_to_enu * p_local + t_local_to_enu;

                    laserCloudWorld->points[i].x = p_enu.x();
                    laserCloudWorld->points[i].y = p_enu.y();
                    laserCloudWorld->points[i].z = p_enu.z();
                    laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
                }

                *pcl_wait_save += *laserCloudWorld;
                pcl_wait_ray_origins.insert(pcl_wait_ray_origins.end(), static_cast<size_t>(size), origin_enu);
                {
                    const PointCloudXYZI::Ptr laserCloudEcef =
                        ligo_convert_enu_cloud_to_ecef(laserCloudWorld, R_ecef_enu, first_gps_ecef);
                    ligo_tmp_map_on_scan(laserCloudEcef, origin_ecef, lidar_end_time, rclcpp::get_logger("ligo"));
                }

                scan_wait_num++;
                if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
                {
                    string all_points_dir = ligo_make_pcd_save_path();
                    cout << "current scan saved to " << all_points_dir << " (ENU)" << endl;
                    if (ligo_try_write_binary_pcd(all_points_dir, pcl_wait_save))
                    {
                        save_grid2d_from_cloud_with_rays(
                            pcl_wait_save, pcl_wait_ray_origins, all_points_dir, "enu", pcd_save_grid2d_resolution_m,
                            3, -1e9, 1e9, true, R_ecef_enu, first_gps_ecef, first_gps_lla);
                        ligo_save_ecef_companion_pcd(all_points_dir, pcl_wait_save, R_ecef_enu, first_gps_ecef);
                    }
                    pcl_wait_save->clear();
                    pcl_wait_ray_origins.clear();
                    scan_wait_num = 0;
                }
            }
        }
        else
        {
            int size = points_num;
            PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
            const Eigen::Vector3d origin_local = kf_output.x_.pos;

            for (int i = 0; i < size; i++)
            {
                laserCloudWorld->points[i].x = feats_down_world->points[i].x;
                laserCloudWorld->points[i].y = feats_down_world->points[i].y;
                laserCloudWorld->points[i].z = feats_down_world->points[i].z;
                laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
            }

            *pcl_wait_save += *laserCloudWorld;
            pcl_wait_ray_origins.insert(pcl_wait_ray_origins.end(), static_cast<size_t>(size), origin_local);
            if (pcd_tmp_map_enable)
            {
                static bool warned_tmp_map_non_enu = false;
                if (!warned_tmp_map_non_enu)
                {
                    warned_tmp_map_non_enu = true;
                    RCLCPP_WARN(
                        rclcpp::get_logger("ligo"),
                        "[tmp_map] skipped: ECEF anchor not ready (NMEA/icp_tf_required).");
                }
            }

            static int scan_wait_num = 0;
            scan_wait_num++;
            if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
            {
                string all_points_dir = ligo_make_pcd_save_path();
                cout << "current scan saved to " << all_points_dir << endl;
                ligo_try_write_binary_pcd(all_points_dir, pcl_wait_save);
                pcl_wait_save->clear();
                pcl_wait_ray_origins.clear();
                scan_wait_num = 0;
            }
        }
    }
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp.sec = static_cast<int32_t>(std::floor(lidar_end_time));
    laserCloudmsg.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time - std::floor(lidar_end_time)) * 1e9));
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body->publish(laserCloudmsg);
}

template<typename T>
void set_posestamp(T & out)
{
    {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        Eigen::Quaterniond q(kf_output.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

/** LIO pose를 ENU로 변환. icp_tf_ready일 때만 사용 */
template<typename T>
void set_posestamp_enu(T & out)
{
    if (NMEA_ENABLE && p_nmea && p_nmea->icp_tf_ready)
    {
        const Eigen::Vector3d p_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.pos + p_nmea->icp_t_local_to_enu;
        const Eigen::Matrix3d R_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.rot;
        out.position.x = p_enu.x();
        out.position.y = p_enu.y();
        out.position.z = p_enu.z();
        Eigen::Quaterniond q(R_enu);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
        return;
    }
    set_posestamp(out);
}

/** Same ENU pose as /aft_mapped_to_init when NMEA_ENABLE and icp_tf_ready (see set_posestamp_enu). */
static bool ligo_fused_lio_pose_enu(Eigen::Vector3d &p_enu, Eigen::Matrix3d &R_enu)
{
    if (!NMEA_ENABLE || !p_nmea || !p_nmea->icp_tf_ready)
        return false;
    p_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.pos + p_nmea->icp_t_local_to_enu;
    R_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.rot;
    return true;
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped, tf2_ros::TransformBroadcaster & br)
{
    if (flg_exit || !rclcpp::ok())
    {
        return;
    }
    const bool use_enu = (NMEA_ENABLE && p_nmea && p_nmea->icp_tf_ready);
    odomAftMapped.header.frame_id = use_enu ? "map" : "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp.sec = static_cast<int32_t>(std::floor(time_current));
        odomAftMapped.header.stamp.nanosec = static_cast<uint32_t>(std::round((time_current - std::floor(time_current)) * 1e9));
    }
    else
    {
        odomAftMapped.header.stamp.sec = static_cast<int32_t>(std::floor(lidar_end_time));
        odomAftMapped.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time - std::floor(lidar_end_time)) * 1e9));
    }
    set_posestamp_enu(odomAftMapped.pose.pose);
    
    pubOdomAftMapped->publish(odomAftMapped);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odomAftMapped.header;
    transform.child_frame_id = "aft_mapped";
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation = odomAftMapped.pose.pose.orientation;
    br.sendTransform(transform);
    if (use_enu)
    {
        geometry_msgs::msg::TransformStamped tf_enu_cam;
        tf_enu_cam.header = odomAftMapped.header;
        tf_enu_cam.header.frame_id = "map";
        tf_enu_cam.child_frame_id = "camera_init";
        const Eigen::Matrix3d Rt = p_nmea->icp_R_local_to_enu.transpose();
        const Eigen::Vector3d tt = -Rt * p_nmea->icp_t_local_to_enu;
        tf_enu_cam.transform.translation.x = tt.x();
        tf_enu_cam.transform.translation.y = tt.y();
        tf_enu_cam.transform.translation.z = tt.z();
        Eigen::Quaterniond q(Rt);
        tf_enu_cam.transform.rotation.x = q.coeffs()[0];
        tf_enu_cam.transform.rotation.y = q.coeffs()[1];
        tf_enu_cam.transform.rotation.z = q.coeffs()[2];
        tf_enu_cam.transform.rotation.w = q.coeffs()[3];
        br.sendTransform(tf_enu_cam);
    }
}

static void try_publish_fused_enu_position(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr &pubEnuPosition)
{
    if (flg_exit || !rclcpp::ok())
        return;
    if (!pubEnuPosition)
        return;
    Eigen::Vector3d p_enu;
    if (!compute_ligo_global_topic_enu(p_enu))
        return;
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = enu_position_frame_id;
    const double ts = publish_odometry_without_downsample ? time_current : lidar_end_time;
    msg.header.stamp.sec = static_cast<int32_t>(std::floor(ts));
    msg.header.stamp.nanosec =
        static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));
    msg.point.x = p_enu(0);
    msg.point.y = p_enu(1);
    msg.point.z = p_enu(2);
    pubEnuPosition->publish(msg);
}

static void try_publish_fused_enu_heading_deg(
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pubEnuHeadingDeg)
{
    if (flg_exit || !rclcpp::ok())
        return;
    if (!pubEnuHeadingDeg)
        return;

    Eigen::Vector3d p_enu;
    Eigen::Matrix3d R_enu;
    if (!ligo_fused_lio_pose_enu(p_enu, R_enu))
        return;

    const double yaw_rad = std::atan2(R_enu(1, 0), R_enu(0, 0));
    const double yaw_deg = yaw_rad * 180.0 / std::acos(-1.0);
    double heading_deg = std::fmod(90.0 - yaw_deg, 360.0);
    if (heading_deg < 0.0)
        heading_deg += 360.0;

    std_msgs::msg::Float64 msg;
    msg.data = heading_deg;
    pubEnuHeadingDeg->publish(msg);
}

static void try_publish_fused_global_nav_sat(
    const rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr &pubGlobalFix)
{
    if (flg_exit || !rclcpp::ok())
        return;
    if (!pubGlobalFix)
        return;
    Eigen::Vector3d lla;
    if (!compute_ligo_global_topic_geo(lla))
        return;
    sensor_msgs::msg::NavSatFix msg;
    msg.header.frame_id = "wgs84";
    const double ts = publish_odometry_without_downsample ? time_current : lidar_end_time;
    msg.header.stamp.sec = static_cast<int32_t>(std::floor(ts));
    msg.header.stamp.nanosec =
        static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));
    msg.latitude = lla(0);
    msg.longitude = lla(1);
    msg.altitude = lla(2);
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    pubGlobalFix->publish(msg);
}

static void try_publish_fused_ecef_position(
    const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr &pubEcefPosition)
{
    if (flg_exit || !rclcpp::ok())
        return;
    if (!pubEcefPosition)
        return;
    Eigen::Vector3d p_ecef;
    if (!compute_ligo_global_topic_ecef(p_ecef))
        return;
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = ecef_position_frame_id;
    const double ts = publish_odometry_without_downsample ? time_current : lidar_end_time;
    msg.header.stamp.sec = static_cast<int32_t>(std::floor(ts));
    msg.header.stamp.nanosec =
        static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));
    msg.point.x = p_ecef(0);
    msg.point.y = p_ecef(1);
    msg.point.z = p_ecef(2);
    pubEcefPosition->publish(msg);
}

static void try_publish_nmea_graph_anchor_marker(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
    if (flg_exit || !rclcpp::ok() || !pub || !NMEA_ENABLE || !p_nmea)
        return;
    const double ts = publish_odometry_without_downsample ? time_current : lidar_end_time;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(std::floor(ts));
    stamp.nanosec = static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));

    Eigen::Vector3d p;
    if (!p_nmea->graphAnchorEnu(p))
    {
        visualization_msgs::msg::Marker del;
        del.header.stamp = stamp;
        del.header.frame_id = "map";
        del.ns = "nmea_graph_anchor";
        del.id = 0;
        del.action = visualization_msgs::msg::Marker::DELETE;
        pub->publish(del);
        return;
    }

    visualization_msgs::msg::Marker m;
    m.header.stamp = stamp;
    m.header.frame_id = "map";
    m.ns = "nmea_graph_anchor";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    constexpr double kSphereDiameterM = 4.0;
    m.scale.x = kSphereDiameterM;
    m.scale.y = kSphereDiameterM;
    m.scale.z = kSphereDiameterM;
    m.color.r = 1.0f;
    m.color.g = 0.15f;
    m.color.b = 1.0f;
    m.color.a = 0.82f;
    pub->publish(m);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    if (flg_exit || !rclcpp::ok())
    {
        return;
    }
    const bool use_enu = (NMEA_ENABLE && p_nmea && p_nmea->icp_tf_ready);
    static bool was_enu = false;
    if (use_enu && !was_enu)
    {
        was_enu = true;
        path.poses.clear();  // ENU 전환 시 기존 camera_init 경로 제거
    }
    if (!use_enu) was_enu = false;
    set_posestamp_enu(msg_body_pose.pose);
    msg_body_pose.header.stamp.sec = static_cast<int32_t>(std::floor(lidar_end_time));
    msg_body_pose.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time - std::floor(lidar_end_time)) * 1e9));
    msg_body_pose.header.frame_id = use_enu ? "map" : "camera_init";
    path.header.frame_id = msg_body_pose.header.frame_id;
    static int jjj = 0;
    jjj++;
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}

/** Publishes LIO↔ENU initial heading / local→ENU alignment status (TIME-PAIR or indoor reloc). */
static void publish_heading_align_status(
    const rclcpp::Publisher<ligo::msg::NmeaHeadingAlignStatus>::SharedPtr &pub,
    double lidar_end_time_sec)
{
    if (flg_exit || !rclcpp::ok() || !pub || !NMEA_ENABLE || !p_nmea)
    {
        return;
    }
    ligo::msg::NmeaHeadingAlignStatus msg;
    const double sec_d = std::floor(lidar_end_time_sec);
    msg.header.stamp.sec = static_cast<int32_t>(sec_d);
    msg.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time_sec - sec_d) * 1e9));
    msg.header.frame_id = "map";

    using M = ligo::msg::NmeaHeadingAlignStatus;
    if (!p_nmea->icp_tf_ready)
    {
        if (p_nmea->init_start_set || !p_nmea->init_pos_buf.empty())
        {
            msg.status = M::STATUS_COLLECTING;
        }
        else
        {
            msg.status = M::STATUS_UNALIGNED;
        }
        msg.source = M::SOURCE_NONE;
    }
    else
    {
        msg.status = M::STATUS_LOCKED;
        msg.source = p_nmea->heading_align_source;
    }
    msg.icp_tf_ready = p_nmea->icp_tf_ready;
    msg.yaw_enu_local_rad = p_nmea->yaw_enu_local;
    msg.post_rmse_m = p_nmea->heading_align_post_rmse_m;
    pub->publish(msg);
}

void publish_nmea_aligned(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubNmeaAlignedOdom,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubNmeaAlignedPath)
{
    if (flg_exit || !rclcpp::ok())
    {
        return;
    }
    static int skip_log_count = 0;
    static bool icp_path_was_ready = false;
    if (!NMEA_ENABLE || !p_nmea)
    {
        return;
    }
    if (!nmea_cur)
    {
        if (++skip_log_count % 200 == 0)
        {
            RCLCPP_INFO(
                rclcpp::get_logger("ligo"),
                "[nmea/aligned] no publish yet: nmea_ready=%d nmea_cur=%d nmea_msg_buf=%zu",
                p_nmea->nmea_ready ? 1 : 0,
                nmea_cur ? 1 : 0,
                p_nmea->nmea_msg.size());
        }
        return;
    }
    const Eigen::Vector3d p_enu(
        nmea_cur->pose.pose.position.x,
        nmea_cur->pose.pose.position.y,
        nmea_cur->pose.pose.position.z);
    if (p_nmea->icp_tf_ready && !icp_path_was_ready)
    {
        icp_path_was_ready = true;
        nmea_aligned_path.poses.clear();
    }
    else if (!p_nmea->icp_tf_ready && ++skip_log_count % 200 == 0)
    {
        RCLCPP_INFO(
            rclcpp::get_logger("ligo"),
            "[nmea/aligned] fallback publish(un-aligned): ICP transform not ready yet");
    }

    nmea_aligned_odom.header = nmea_cur->header;
    nmea_aligned_odom.header.frame_id = "map";
    // latency 없음 가정: 보정 안 함
    const double nmea_lat = 0.0;
    if (false)  // nmea_lat always 0
    {
        const double t_raw = rclcpp::Time(nmea_cur->header.stamp).seconds();
        const double t_corrected = t_raw - nmea_lat;
        nmea_aligned_odom.header.stamp.sec = static_cast<int32_t>(std::floor(t_corrected));
        nmea_aligned_odom.header.stamp.nanosec = static_cast<uint32_t>(std::round((t_corrected - std::floor(t_corrected)) * 1e9));
    }
    nmea_aligned_odom.child_frame_id = p_nmea->icp_tf_ready ? "nmea_aligned" : "nmea_unaligned";
    nmea_aligned_odom.pose.pose.position.x = p_enu.x();
    nmea_aligned_odom.pose.pose.position.y = p_enu.y();
    nmea_aligned_odom.pose.pose.position.z = p_enu.z();
    nmea_aligned_odom.pose.pose.orientation.w = 1.0;
    pubNmeaAlignedOdom->publish(nmea_aligned_odom);

    // ICP 적용된 GPS 경로만 시각화 (icp_tf_ready일 때만 path에 누적). ENU 좌표계
    nmea_aligned_path.header = nmea_aligned_odom.header;
    nmea_aligned_path.header.frame_id = "map";
    if (p_nmea->icp_tf_ready)
    {
        nmea_aligned_pose.header = nmea_aligned_odom.header;
        nmea_aligned_pose.pose = nmea_aligned_odom.pose.pose;
        nmea_aligned_path.poses.emplace_back(nmea_aligned_pose);
        if (nmea_aligned_path.poses.size() > 5000)
        {
            nmea_aligned_path.poses.erase(nmea_aligned_path.poses.begin());
        }
    }
    pubNmeaAlignedPath->publish(nmea_aligned_path);
}

void publish_icp_pairs_marker(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pubIcpPairs,
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pubNmeaLioErrorXy,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pubNmea03mDiag)
{
    if (flg_exit || !rclcpp::ok())
    {
        return;
    }
    if (!NMEA_ENABLE || !p_nmea || !p_nmea->icp_tf_ready) return;
    if (p_nmea->icp_pairs_lio.empty() || p_nmea->icp_pairs_nmea_local.empty()) return;
    const size_t n = std::min(p_nmea->icp_pairs_lio.size(), p_nmea->icp_pairs_nmea_local.size());
    if (n == 0) return;

    // ICP 이후 전체 경로 pair 2D RMSE (xy) 퍼블리시
    if (pubNmeaLioErrorXy && p_nmea->n_nmea_fusion_count > 0)
    {
        std_msgs::msg::Float64 err_msg;
        err_msg.data = std::sqrt(p_nmea->sum_nmea_lio_err_sq_xy / p_nmea->n_nmea_fusion_count);
        pubNmeaLioErrorXy->publish(err_msg);
    }

    // 0.3m 시점 LIO-GPS 비교 (latency 진단). icp 완료 시 1회 퍼블리시
    if (pubNmea03mDiag)
    {
        static bool diag_03m_published = false;
        if (!p_nmea->diag_03m_valid) diag_03m_published = false;
        else if (!diag_03m_published)
        {
            std_msgs::msg::Float64MultiArray diag;
            diag.layout.dim.resize(1);
            diag.layout.dim[0].label = "latency_s,t_lio,t_gps,lio_x,lio_y,lio_z,gps_x,gps_y,gps_z,lio_disp,gps_disp_at_t_lio";
            diag.layout.dim[0].size = 11;
            diag.layout.dim[0].stride = 11;
            diag.data = {
                p_nmea->diag_03m_latency_s, p_nmea->diag_03m_t_lio, p_nmea->diag_03m_t_gps,
                p_nmea->diag_03m_lio_pos.x(), p_nmea->diag_03m_lio_pos.y(), p_nmea->diag_03m_lio_pos.z(),
                p_nmea->diag_03m_gps_at_t_lio.x(), p_nmea->diag_03m_gps_at_t_lio.y(), p_nmea->diag_03m_gps_at_t_lio.z(),
                p_nmea->diag_03m_lio_disp, p_nmea->diag_03m_gps_disp_at_t_lio
            };
            pubNmea03mDiag->publish(diag);
            diag_03m_published = true;
        }
    }

    const auto stamp = rclcpp::Time(0);
    const Eigen::Matrix3d &Ricp = p_nmea->icp_R_local_to_enu;
    const Eigen::Vector3d &ticp = p_nmea->icp_t_local_to_enu;

    // 1) Lines connecting LIO <-> GPS pairs (ENU 좌표계)
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = stamp;
    line_marker.ns = "icp_lines";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.04;
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.5;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    line_marker.points.clear();
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector3d pl_enu = Ricp * p_nmea->icp_pairs_lio[i] + ticp;
        const Eigen::Vector3d pn_enu = Ricp * p_nmea->icp_pairs_nmea_local[i] + ticp;
        geometry_msgs::msg::Point pt_lio, pt_nmea;
        pt_lio.x = pl_enu.x(); pt_lio.y = pl_enu.y(); pt_lio.z = pl_enu.z();
        pt_nmea.x = pn_enu.x(); pt_nmea.y = pn_enu.y(); pt_nmea.z = pn_enu.z();
        line_marker.points.push_back(pt_lio);
        line_marker.points.push_back(pt_nmea);
    }
    pubIcpPairs->publish(line_marker);

    // 2) LIO points (green spheres, ENU)
    visualization_msgs::msg::Marker lio_marker;
    lio_marker.header.frame_id = "map";
    lio_marker.header.stamp = stamp;
    lio_marker.ns = "icp_lio";
    lio_marker.id = 0;
    lio_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    lio_marker.action = visualization_msgs::msg::Marker::ADD;
    lio_marker.scale.x = lio_marker.scale.y = lio_marker.scale.z = 0.12;
    lio_marker.color.r = 0.0;
    lio_marker.color.g = 1.0;
    lio_marker.color.b = 0.0;
    lio_marker.color.a = 1.0;
    lio_marker.points.clear();
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector3d pl_enu = Ricp * p_nmea->icp_pairs_lio[i] + ticp;
        geometry_msgs::msg::Point pt;
        pt.x = pl_enu.x(); pt.y = pl_enu.y(); pt.z = pl_enu.z();
        lio_marker.points.push_back(pt);
    }
    pubIcpPairs->publish(lio_marker);

    // 3) GPS points (red spheres, ENU)
    visualization_msgs::msg::Marker gps_marker;
    gps_marker.header.frame_id = "map";
    gps_marker.header.stamp = stamp;
    gps_marker.ns = "icp_gps";
    gps_marker.id = 0;
    gps_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    gps_marker.action = visualization_msgs::msg::Marker::ADD;
    gps_marker.scale.x = gps_marker.scale.y = gps_marker.scale.z = 0.12;
    gps_marker.color.r = 1.0;
    gps_marker.color.g = 0.0;
    gps_marker.color.b = 0.0;
    gps_marker.color.a = 1.0;
    gps_marker.points.clear();
    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector3d pn_enu = Ricp * p_nmea->icp_pairs_nmea_local[i] + ticp;
        geometry_msgs::msg::Point pt;
        pt.x = pn_enu.x(); pt.y = pn_enu.y(); pt.z = pn_enu.z();
        gps_marker.points.push_back(pt);
    }
    pubIcpPairs->publish(gps_marker);

    // 4) GPS path as LINE_STRIP (ENU, 이미 nmea_aligned_path가 ENU)
    visualization_msgs::msg::Marker gps_path_marker;
    gps_path_marker.header.frame_id = "map";
    gps_path_marker.header.stamp = stamp;
    gps_path_marker.ns = "gps_path";
    gps_path_marker.id = 0;
    gps_path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    gps_path_marker.action = visualization_msgs::msg::Marker::ADD;
    gps_path_marker.scale.x = 0.06;
    gps_path_marker.color.r = 0.0;
    gps_path_marker.color.g = 0.5;
    gps_path_marker.color.b = 1.0;
    gps_path_marker.color.a = 1.0;
    gps_path_marker.points.clear();
    for (const auto &ps : nmea_aligned_path.poses)
    {
        geometry_msgs::msg::Point pt;
        pt.x = ps.pose.position.x;
        pt.y = ps.pose.position.y;
        pt.z = ps.pose.position.z;
        gps_path_marker.points.push_back(pt);
    }
    if (!gps_path_marker.points.empty())
        pubIcpPairs->publish(gps_path_marker);
}

void publish_init_pairs_marker_from_gps_move(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
  if (flg_exit || !rclcpp::ok()) return;
  if (!NMEA_ENABLE || !p_nmea) return;
  if (!p_nmea->init_start_set || p_nmea->init_pos_buf.size() < 2 ||
      p_nmea->init_nmea_buf.size() != p_nmea->init_pos_buf.size() ||
      p_nmea->init_lio_time_buf.size() != p_nmea->init_pos_buf.size())
    return;

  const auto &init_pos = p_nmea->init_pos_buf;
  const auto &init_nmea = p_nmea->init_nmea_buf;
  const auto &init_lio_time = p_nmea->init_lio_time_buf;
  const Eigen::Vector3d &start_lio = p_nmea->init_start_lio;
  const Eigen::Vector3d &start_nmea = p_nmea->init_start_nmea;
  const int n_valid = static_cast<int>(init_pos.size());

  // 변위, 지연 추정 (NMEALIAlign과 동일 로직)
  std::vector<double> lio_disp(n_valid), gps_disp(n_valid);
  for (int i = 0; i < n_valid; ++i)
  {
    lio_disp[i] = (init_pos[i] - start_lio).norm();
    Eigen::Vector3d gv(init_nmea[i]->pose.pose.position.x, init_nmea[i]->pose.pose.position.y, init_nmea[i]->pose.pose.position.z);
    gps_disp[i] = (gv - start_nmea).norm();
  }
  int first_lio_03 = -1, first_gps_03 = -1;
  constexpr double THRESH_03 = 0.3;
  for (int i = 0; i < n_valid; ++i)
  {
    if (first_lio_03 < 0 && lio_disp[i] >= THRESH_03) first_lio_03 = i;
    if (first_gps_03 < 0 && gps_disp[i] >= THRESH_03) first_gps_03 = i;
  }
  double latency_est = 0.0;
  if (first_lio_03 >= 0 && first_gps_03 >= 0)
  {
    latency_est = rclcpp::Time(init_nmea[first_gps_03]->header.stamp).seconds() - init_lio_time[first_lio_03];
    latency_est = std::max(0.0, latency_est);  // 추정값 그대로 사용 (상한 제거)
    { static int _n = 0; if (++_n <= 2) RCLCPP_INFO(rclcpp::get_logger("ligo"),
      "[nmea/pair_dbg] first_lio_03=%d first_gps_03=%d L=%.3f | lio_disp[fl03]=%.3f gps_disp[fg03]=%.3f",
      first_lio_03, first_gps_03, latency_est, lio_disp[first_lio_03], gps_disp[first_gps_03]); }
  }

  auto get_lio_at_time = [&](double t_want) -> Eigen::Vector3d {
    if (t_want <= init_lio_time.front()) return init_pos.front();
    if (t_want >= init_lio_time.back()) return init_pos.back();
    for (int j = 0; j + 1 < n_valid; ++j)
    {
      if (init_lio_time[j] <= t_want && t_want <= init_lio_time[j + 1])
      {
        double a = (init_lio_time[j + 1] - init_lio_time[j]) > 1e-9 ? (t_want - init_lio_time[j]) / (init_lio_time[j + 1] - init_lio_time[j]) : 0;
        return (1 - a) * init_pos[j] + a * init_pos[j + 1];
      }
    }
    return init_pos.back();
  };
  auto get_lio_at_time_dbg = [&](double t_want, int src_i, bool is_first) -> Eigen::Vector3d {
    Eigen::Vector3d ret = get_lio_at_time(t_want);
    static int dbg_count = 0;
    if (is_first && (++dbg_count <= 3))
    {
      RCLCPP_INFO(rclcpp::get_logger("ligo"),
        "[nmea/pair_dbg] first_pair: src_i=%d t_want=%.3f L=%.3f stamp_i=%.3f | lio_time[fl03]=%.3f | ret_disp=%.3f",
        src_i, t_want, latency_est, src_i < n_valid ? rclcpp::Time(init_nmea[src_i]->header.stamp).seconds() : -1.0,
        first_lio_03 >= 0 && first_lio_03 < n_valid ? init_lio_time[first_lio_03] : -1.0,
        (ret - start_lio).norm());
    }
    return ret;
  };

  // 0.3m 이후: 보정 시각(T-L)으로 (LIO, GPS) pair. 같은 위치 = 같은 시각대.
  std::vector<Eigen::Vector3d> lio_pts, gps_local_pts;
  for (int i = 0; i < n_valid; ++i)
  {
    if (lio_disp[i] < THRESH_03 || gps_disp[i] < THRESH_03) continue;
    const double t_want = rclcpp::Time(init_nmea[i]->header.stamp).seconds() - latency_est;
    Eigen::Vector3d lio_pt = (latency_est > 0.01)
        ? get_lio_at_time_dbg(t_want, i, lio_pts.empty())
        : init_pos[i];
    Eigen::Vector3d gps_enu(init_nmea[i]->pose.pose.position.x, init_nmea[i]->pose.pose.position.y, init_nmea[i]->pose.pose.position.z);
    Eigen::Vector3d gps_local = (gps_enu - start_nmea) + start_lio;
    lio_pts.push_back(lio_pt);
    gps_local_pts.push_back(gps_local);
  }
  if (lio_pts.empty()) return;

  const size_t n_vis = lio_pts.size();
  const auto stamp = rclcpp::Time(0);

  // 각 pair 변위 로그 (50회마다 또는 pair수 변경 시). lio>>gps면 LIO가 더 이동한 시점에 연결된 것
  {
    static size_t last_n_vis = 0;
    static int log_count = 0;
    if (last_n_vis != n_vis || (++log_count % 50 == 1))
    {
      last_n_vis = n_vis;
      double sum_gap = 0.0;
      for (size_t i = 0; i < n_vis; ++i)
      {
        const double gap = (lio_pts[i] - start_lio).norm() - (gps_local_pts[i] - start_lio).norm();
        sum_gap += std::fabs(gap);
      }
      const double mean_gap = n_vis > 0 ? sum_gap / n_vis : 0.0;
      if (n_vis >= 1)
      {
        const double l0 = (lio_pts[0] - start_lio).norm();
        const double g0 = (gps_local_pts[0] - start_lio).norm();
        RCLCPP_INFO(rclcpp::get_logger("ligo"),
          "[nmea/pair_disp] n=%zu first: lio=%.3f gps=%.3f gap=%.3f | mean_|gap|=%.3f",
          n_vis, l0, g0, l0 - g0, mean_gap);
      }
    }
  }

  // 1) Lines: 0.3m 이후 보정된 pair. LIO(T-L)↔GPS(stamp T) = 같은 시각대 = 같은 위치
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.frame_id = "camera_init";
  line_marker.header.stamp = stamp;
  line_marker.ns = "init_pairs_lines";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.04;
  line_marker.color.r = 1.0;
  line_marker.color.g = 0.5;
  line_marker.color.b = 0.0;
  line_marker.color.a = 1.0;
  line_marker.points.clear();
  for (size_t i = 0; i < n_vis; ++i)
  {
    geometry_msgs::msg::Point pt_lio, pt_gps;
    pt_lio.x = lio_pts[i].x();
    pt_lio.y = lio_pts[i].y();
    pt_lio.z = lio_pts[i].z();
    pt_gps.x = gps_local_pts[i].x();
    pt_gps.y = gps_local_pts[i].y();
    pt_gps.z = gps_local_pts[i].z();
    line_marker.points.push_back(pt_lio);
    line_marker.points.push_back(pt_gps);
  }
  pub->publish(line_marker);

  // 2) LIO points (green)
  visualization_msgs::msg::Marker lio_marker;
  lio_marker.header.frame_id = "camera_init";
  lio_marker.header.stamp = stamp;
  lio_marker.ns = "init_pairs_lio";
  lio_marker.id = 0;
  lio_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  lio_marker.action = visualization_msgs::msg::Marker::ADD;
  lio_marker.scale.x = lio_marker.scale.y = lio_marker.scale.z = 0.1;
  lio_marker.color.r = 0.0;
  lio_marker.color.g = 1.0;
  lio_marker.color.b = 0.0;
  lio_marker.color.a = 1.0;
  lio_marker.points.clear();
  for (size_t i = 0; i < n_vis; ++i)
  {
    geometry_msgs::msg::Point pt;
    pt.x = lio_pts[i].x();
    pt.y = lio_pts[i].y();
    pt.z = lio_pts[i].z();
    lio_marker.points.push_back(pt);
  }
  pub->publish(lio_marker);

  // 3) GPS points (red)
  visualization_msgs::msg::Marker gps_marker;
  gps_marker.header.frame_id = "camera_init";
  gps_marker.header.stamp = stamp;
  gps_marker.ns = "init_pairs_gps";
  gps_marker.id = 0;
  gps_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  gps_marker.action = visualization_msgs::msg::Marker::ADD;
  gps_marker.scale.x = gps_marker.scale.y = gps_marker.scale.z = 0.1;
  gps_marker.color.r = 1.0;
  gps_marker.color.g = 0.0;
  gps_marker.color.b = 0.0;
  gps_marker.color.a = 1.0;
  gps_marker.points.clear();
  for (size_t i = 0; i < n_vis; ++i)
  {
    geometry_msgs::msg::Point pt;
    pt.x = gps_local_pts[i].x();
    pt.y = gps_local_pts[i].y();
    pt.z = gps_local_pts[i].z();
    gps_marker.points.push_back(pt);
  }
  pub->publish(gps_marker);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("laserMapping");
    readParameters(node.get());
    if (mapping_mode)
    {
        RCLCPP_INFO(node->get_logger(),
                    "[pcd] mapping saves overwrite PCD/%s/%s/%s.pcd (+ grid yaml/pgm, ecef pcd)",
                    pcd_save_map_name.c_str(),
                    pcd_save_sub_map_name.c_str(),
                    pcd_save_sub_map_name.c_str());
    }
    RCLCPP_INFO(node->get_logger(), "lidar_type: %d", lidar_type);
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    ivox_last_ = std::make_shared<IVoxType>(ivox_options_);
    
    path.header.stamp.sec = static_cast<int32_t>(std::floor(lidar_end_time));
    path.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time - std::floor(lidar_end_time)) * 1e9));
    path.header.frame_id = "camera_init";  // publish_path에서 ICP 시 enu로 전환
    nmea_aligned_path.header = path.header;

    /*** variables definition for counting ***/
    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    {
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;
    if (NMEA_ENABLE)
    {
        p_nmea->Tex_imu_r << VEC_FROM_ARRAY(extrinT_gnss);
        p_nmea->Rex_imu_r << MAT_FROM_ARRAY(extrinR_gnss);
        p_nmea->nmea_ready = false;
        p_nmea->nolidar = nolidar;
        p_nmea->pre_integration->setnoise();
    }
    // Load grid registry / reference PCD whenever paths are set (also mapping_mode: indoor_flag may be false).
    if (!indoor_grid_map_dir.empty() || !indoor_map_pcd_path.empty())
    {
        ligo::indoor::SmallGICPConfig gicp_cfg;
        gicp_cfg.num_threads                    = 4;
        gicp_cfg.map_downsampling_resolution    = indoor_gicp_map_voxel_m;
        gicp_cfg.scan_downsampling_resolution   = indoor_gicp_scan_voxel_m;
        gicp_cfg.max_correspondence_distance    = indoor_gicp_max_correspondence_m;
        gicp_cfg.max_iterations                 = indoor_gicp_max_iterations_reg;
        ligo::indoor::setIndoorGICPConfigForGridSelection(gicp_cfg);
        if (!indoor_grid_map_dir.empty())
        {
            if (ligo::indoor::loadIndoorGridMapsFromDirectory(indoor_grid_map_dir))
            {
                RCLCPP_INFO(node->get_logger(),
                            "[indoor/gicp] grid_map_dir loaded; reference PCD follows occupancy grid membership (ECEF)");
            }
            else
            {
                RCLCPP_WARN(node->get_logger(),
                            "[indoor/gicp] indoor.grid_map_dir=%s invalid or empty — check * _grid2d.yaml",
                            indoor_grid_map_dir.c_str());
            }
            if (!ligo::indoor::indoorGridMapsLoaded() && !indoor_map_pcd_path.empty())
            {
                ligo::indoor::initIndoorGICP(indoor_map_pcd_path, gicp_cfg);
            }
        }
        else if (!indoor_map_pcd_path.empty())
        {
            ligo::indoor::initIndoorGICP(indoor_map_pcd_path, gicp_cfg);
        }
    }
    // IMU uses h_dyn_share_modified_2; esekfom 2h does not assign slot 2 — always 3h (NMEA slot unused when NMEA_ENABLE is false / no NMEA updates).
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output, h_model_NMEA_output);
    Eigen::Matrix<double, 24, 24> P_init_output; // = MD(24, 24)::Identity() * 0.01;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_output = process_noise_cov_output();
    open_file();

    /*** ROS2 subscribe initialization ***/
    rclcpp::QoS qos_lidar(200000);
    rclcpp::SubscriptionBase::SharedPtr sub_pcl;
    if (p_pre->lidar_type == AVIA)
        sub_pcl = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, qos_lidar, livox_pcl_cbk);
    else
        sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, qos_lidar, standard_pcl_cbk);
    auto sub_imu = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, qos_lidar, imu_cbk);

    rclcpp::SubscriptionBase::SharedPtr sub_nmea_meas;
    #ifdef process_ppp
    PPPfromTXT(LOCAL_FILE_DIR(ppp_fname), ppp_sol, ppp_ecef);
    if (NMEA_ENABLE)
    {
        if (ppp_ecef.size() > 0)
        {
            first_pvt_used = ppp_ecef[0].segment<3>(1);
            first_lla_used = gnss_comm::ecef2geo(first_pvt_used);
            if (!nmea_global_anchor_ready)
            {
                nmea_global_anchor_lla = first_lla_used;
                nmea_global_anchor_ready = true;
                ligo::indoor::setSystemEcefAnchor(
                    gnss_comm::geo2ecef(nmea_global_anchor_lla),
                    gnss_comm::geo2rotation(nmea_global_anchor_lla));
            }
            for (int i = 0; i < ppp_sol.size(); i++)
            {
                nav_msgs::msg::Odometry gps_odom;
                {
                    const double t = ppp_sol[i][0];
                    const int32_t sec = static_cast<int32_t>(std::floor(t));
                    const uint32_t nanosec = static_cast<uint32_t>(std::round((t - std::floor(t)) * 1e9));
                    gps_odom.header.stamp.sec = sec;
                    gps_odom.header.stamp.nanosec = nanosec;
                }
                gps_odom.pose.pose.position.x = ppp_sol[i][1];
                gps_odom.pose.pose.position.y = ppp_sol[i][2];
                gps_odom.pose.pose.position.z = ppp_sol[i][3];
                gps_odom.pose.covariance[0] = ppp_sol[i][4];
                gps_odom.pose.covariance[1] = ppp_sol[i][5];
                gps_odom.pose.covariance[2] = ppp_sol[i][6];
                nmea_meas_buf.push(std::make_shared<nav_msgs::msg::Odometry>(gps_odom));
            }
        }
    }
    #endif
    if (NMEA_ENABLE)
    {
        rclcpp::QoS qos_nmea(10000);
        if (nmea_input_type == "navsatfix")
        {
            sub_nmea_meas = node->create_subscription<sensor_msgs::msg::NavSatFix>(
                nmea_meas_topic, qos_nmea, gpsHandler);
            RCLCPP_INFO(node->get_logger(), "NMEA subscription active (NavSatFix->Odom bridge): %s", nmea_meas_topic.c_str());
        }
        else
        {
            sub_nmea_meas = node->create_subscription<nav_msgs::msg::Odometry>(
                nmea_meas_topic, qos_nmea, nmea_meas_callback);
            RCLCPP_INFO(node->get_logger(), "NMEA subscription active (Odometry): %s", nmea_meas_topic.c_str());
        }
        ligo_try_create_nmea_stamp_diag_publisher(node);
    }

    rclcpp::QoS qos_pub(1000);
    auto pubLaserCloudFullRes = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", qos_pub);
    auto pubLaserCloudFullRes_body = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", qos_pub);
    auto pubLaserCloudEffect = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", qos_pub);
    auto pubLaserCloudMap = node->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", qos_pub);
    rclcpp::QoS qos_latched = rclcpp::QoS(1).transient_local();
    auto pubIndoorMapCloud  = node->create_publisher<sensor_msgs::msg::PointCloud2>("/indoor/map_cloud", qos_latched);
    auto pubIndoorMap2d =
        node->create_publisher<nav_msgs::msg::OccupancyGrid>("/indoor/map_2d", qos_latched);
    auto pubIndoorAlignedScan = node->create_publisher<sensor_msgs::msg::PointCloud2>("/indoor/aligned_scan", qos_pub);
    // Subscriber for dynamic indoor/outdoor mode toggle
    auto subIndoorFlag = node->create_subscription<std_msgs::msg::Bool>(
        "/ligo/indoor_mode", 10,
        [](const std_msgs::msg::Bool::SharedPtr msg) {
            const bool was_indoor = indoor_flag_dynamic;
            if (!msg->data)
            {
                g_pending_indoor_topic_snap = false;
                indoor_flag_dynamic = false;
                if (was_indoor)
                {
                    RCLCPP_INFO(rclcpp::get_logger("ligo"), "[indoor] dynamic indoor mode OFF");
                    ligo::indoor::resetIndoorGICP();
                }
                return;
            }
            // Enter: do not set indoor_flag_dynamic here — reloc reset applies anchor then sets it true.
            if (!was_indoor)
            {
                RCLCPP_INFO(rclcpp::get_logger("ligo"),
                            "[indoor] dynamic indoor mode ON (pending fused LIO→ENU snap on next lidar frame)");
                g_pending_indoor_topic_snap = true;
            }
        });
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", qos_pub);
    auto pubPath = node->create_publisher<nav_msgs::msg::Path>("/path", qos_pub);
    // indoor/outdoor 전환 시에만 JSON publish (MQTT 브리지는 /ligo/mode 구독 → MQTT nav1/mode 등으로 별도 토픽)
    auto pubLigoMode = node->create_publisher<std_msgs::msg::String>(
        "/ligo/mode", rclcpp::QoS(10).transient_local());
    auto pubNmeaAlignedOdom = node->create_publisher<nav_msgs::msg::Odometry>("/nmea_aligned_to_init", qos_pub);
    auto pubNmeaAlignedPath = node->create_publisher<nav_msgs::msg::Path>("/nmea_aligned_path", qos_pub);
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubNmeaLioErrorXy;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubNmea03mDiag;
    rclcpp::Publisher<ligo::msg::NmeaHeadingAlignStatus>::SharedPtr pubHeadingAlignStatus;
    if (NMEA_ENABLE)
    {
        pubNmeaLioErrorXy = node->create_publisher<std_msgs::msg::Float64>("/ligo/nmea_lio_error_xy", qos_pub);
        pubNmea03mDiag = node->create_publisher<std_msgs::msg::Float64MultiArray>("/ligo/nmea_03m_diag", qos_pub);
        pubHeadingAlignStatus =
            node->create_publisher<ligo::msg::NmeaHeadingAlignStatus>("/ligo/nmea_heading_align_status", qos_pub);
        RCLCPP_INFO(node->get_logger(),
                    "NMEA heading align status: topic=/ligo/nmea_heading_align_status "
                    "(STATUS: 0=UNALIGNED 1=COLLECTING 2=LOCKED; SOURCE: 0=NONE 1=TIME_PAIR 2=INDOOR_RELOC)");
    }
    auto pubIcpPairs = node->create_publisher<visualization_msgs::msg::Marker>("/icp_pairs_marker", qos_pub);
    auto pubInitPairsFromGpsMove = node->create_publisher<visualization_msgs::msg::Marker>(
        "/init_pairs_from_gps_move_marker", qos_pub);
    auto plane_pub = node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", qos_pub);
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubEnuPosition;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubEnuHeadingDeg;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubEcefPosition;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGlobalNavSat;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubNmeaGraphAnchorMarker;
    if (NMEA_ENABLE)
    {
        pubEnuPosition =
            node->create_publisher<geometry_msgs::msg::PointStamped>(enu_position_topic, qos_pub);
        RCLCPP_INFO(node->get_logger(), "ENU position: topic=%s frame_id=%s", enu_position_topic.c_str(),
                    enu_position_frame_id.c_str());
        pubEnuHeadingDeg =
            node->create_publisher<std_msgs::msg::Float64>(enu_heading_topic, qos_pub);
        RCLCPP_INFO(node->get_logger(),
                    "ENU heading: topic=%s (deg, clockwise from north in ENU)", enu_heading_topic.c_str());
        pubGlobalNavSat =
            node->create_publisher<sensor_msgs::msg::NavSatFix>(global_position_topic, qos_pub);
        RCLCPP_INFO(node->get_logger(), "Global WGS84 position (NavSatFix): topic=%s (anchor auto-detected at NMEA init)",
                    global_position_topic.c_str());
        pubEcefPosition =
            node->create_publisher<geometry_msgs::msg::PointStamped>(ecef_position_topic, qos_pub);
        RCLCPP_INFO(node->get_logger(), "ECEF position: topic=%s frame_id=%s", ecef_position_topic.c_str(),
                    ecef_position_frame_id.c_str());
        pubNmeaGraphAnchorMarker = node->create_publisher<visualization_msgs::msg::Marker>(
            "/ligo/nmea_graph_anchor_marker", qos_pub);
        RCLCPP_INFO(node->get_logger(),
                    "NMEA graph anchor E(0) RViz: topic=/ligo/nmea_graph_anchor_marker frame_id=map "
                    "(Fixed Frame=map, magenta sphere ~4m)");
    }

    rclcpp::on_shutdown([]() {
        flg_exit = true;
        ligo_reset_nmea_stamp_diag_publisher();
    });
    tf2_ros::TransformBroadcaster tf_br(node);
    rclcpp::Rate loop_rate(500);
    while (rclcpp::ok() && !flg_exit)
    {
        rclcpp::spin_some(node);
        if(sync_packages(Measures, p_nmea->nmea_msg)) 
        {
            if (pending_outdoor_realign_ivox_reset.exchange(false, std::memory_order_acq_rel))
            {
                RCLCPP_WARN(rclcpp::get_logger("ligo"),
                            "Outdoor re-align: applying deferred IVox + trajectory map reset (LIO/KF/IMU unchanged)");
                if (NMEA_ENABLE && traj_manager)
                    traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                ivox_ = std::make_shared<IVoxType>(ivox_options_);
                ivox_last_ = std::make_shared<IVoxType>(ivox_options_);
                traj_manager.reset(new curvefitter::TrajectoryManager<4>());
                init_map = false;
            }
            if (g_pending_indoor_topic_snap.load(std::memory_order_acquire) && !flg_reset && !mapping_mode)
            {
                if (ligo_fused_lio_pose_enu(indoor_reloc_pos_enu, indoor_reloc_rot_enu))
                {
                    indoor_reloc_pose_time = time_predict_last_const;
                    indoor_reloc_applied_once = true;
                    flg_reset_indoor_reloc = true;
                    flg_reset = true;
                    g_pending_indoor_topic_snap.store(false, std::memory_order_release);
                    RCLCPP_WARN(
                        node->get_logger(),
                        "[indoor] /ligo/indoor_mode: anchor = fused LIO ENU (same as /aft_mapped_to_init) "
                        "pos=(%.2f,%.2f,%.2f)",
                        indoor_reloc_pos_enu.x(), indoor_reloc_pos_enu.y(), indoor_reloc_pos_enu.z());
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                                         "[indoor] /ligo/indoor_mode pending: wait for NMEA icp_tf_ready "
                                         "(cannot snap to /aft_mapped_to_init ENU yet)");
                }
            }
            if (flg_reset)
            {
                if (flg_reset_indoor_reloc)
                {
                    RCLCPP_WARN(node->get_logger(), "reset by indoor relocalization");
                }
                else
                {
                    RCLCPP_WARN(node->get_logger(), "reset when rosbag play back");
                }
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                {
                    state_out = state_output();
                    if (flg_reset_indoor_reloc)
                    {
                        RCLCPP_WARN(node->get_logger(), "indoor mode on: GICP anchor applied");
                        // Keep ENU anchor in graph init, restart local state at origin with GICP orientation.
                        state_out.pos = Eigen::Vector3d::Zero();
                        state_out.rot = indoor_reloc_rot_enu;
                        state_out.vel = Eigen::Vector3d::Zero();
                        state_out.gravity << VEC_FROM_ARRAY(gravity);
                        state_out.acc = -state_out.rot.transpose() * state_out.gravity;
                        kf_output.x_ = state_out;
                        p_imu->imu_need_init_ = false;
                        p_imu->after_imu_init_ = true;
                        p_nmea->SetInitFromLocalization(indoor_reloc_pos_enu, indoor_reloc_rot_enu, kf_output.x_, indoor_reloc_pose_time);
                        indoor_flag_dynamic = true;
                        // ECEF for grid→PCD: last good fix, else anchor + indoor ENU (same as before).
                        Eigen::Vector3d reloc_ecef = Eigen::Vector3d::Zero();
                        bool reloc_ecef_ok = false;
                        if (last_good_gnss_ecef_valid)
                        {
                            reloc_ecef = last_good_gnss_ecef;
                            reloc_ecef_ok = true;
                        }
                        else if (nmea_global_anchor_ready)
                        {
                            const Eigen::Vector3d anc_ecef = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                            const Eigen::Matrix3d R_ecef_enu = gnss_comm::geo2rotation(nmea_global_anchor_lla);
                            reloc_ecef = anc_ecef + R_ecef_enu * indoor_reloc_pos_enu;
                            reloc_ecef_ok = true;
                        }
                        ligo::indoor::SmallGICPConfig session_gicp_cfg;
                        session_gicp_cfg.num_threads                  = 4;
                        session_gicp_cfg.map_downsampling_resolution  = indoor_gicp_map_voxel_m;
                        session_gicp_cfg.scan_downsampling_resolution = indoor_gicp_scan_voxel_m;
                        session_gicp_cfg.max_correspondence_distance  = indoor_gicp_max_correspondence_m;
                        session_gicp_cfg.max_iterations               = indoor_gicp_max_iterations_reg;
                        ligo::indoor::setIndoorGICPConfigForGridSelection(session_gicp_cfg);
                        RCLCPP_WARN(node->get_logger(),
                                    "[indoor/gicp] session-start debug: grid_loaded=%d grid_count=%zu "
                                    "reloc_ecef_ok=%d last_good_ecef=%d anchor_ready=%d "
                                    "grid_map_dir='%s' map_pcd_path='%s'",
                                    static_cast<int>(ligo::indoor::indoorGridMapsLoaded()),
                                    ligo::indoor::indoorGridMapCount(),
                                    static_cast<int>(reloc_ecef_ok),
                                    static_cast<int>(last_good_gnss_ecef_valid),
                                    static_cast<int>(nmea_global_anchor_ready),
                                    indoor_grid_map_dir.c_str(),
                                    indoor_map_pcd_path.c_str());
                        if (reloc_ecef_ok) {
                            RCLCPP_WARN(node->get_logger(),
                                        "[indoor/gicp] session-start ECEF=(%.3f, %.3f, %.3f)",
                                        reloc_ecef.x(), reloc_ecef.y(), reloc_ecef.z());
                        }
                        if (ligo::indoor::indoorGridMapsLoaded())
                        {
                            if (reloc_ecef_ok)
                            {
                                ligo::indoor::loadIndoorGICPMapForSession(reloc_ecef);
                            }
                            else if (ligo::indoor::indoorGridMapCount() == 1u)
                            {
                                ligo::indoor::loadIndoorGICPMapForSession(Eigen::Vector3d::Zero());
                            }
                            else if (!indoor_map_pcd_path.empty())
                            {
                                ligo::indoor::loadIndoorGICPMapForSession(Eigen::Vector3d::Zero());
                            }
                            else
                            {
                                RCLCPP_WARN(node->get_logger(),
                                            "[indoor/gicp] multiple grid maps but no ECEF yet — set "
                                            "indoor.map_pcd_path or wait for GNSS anchor / last good fix");
                            }
                        }
                        else if (!indoor_map_pcd_path.empty())
                        {
                            ligo::indoor::initIndoorGICP(indoor_map_pcd_path, session_gicp_cfg);
                        }
                        else
                        {
                            RCLCPP_WARN(node->get_logger(),
                                        "[indoor/gicp] no grids in memory and indoor.map_pcd_path empty — "
                                        "check indoor.grid_map_dir and *_grid2d.yaml / PCD paths at startup");
                        }
                        if (!indoor_gicp_map_loaded)
                        {
                            RCLCPP_ERROR(node->get_logger(),
                                         "[indoor/gicp] Map NOT loaded at session start! "
                                         "Set indoor.grid_map_dir or indoor.map_pcd_path in config.");
                        }
                        // Seed GICP pose: scan is already converted to map-local-ENU
                        // inside runIndoorGICPUpdate, so T_map_lidar ≈ Identity.
                        indoor_gicp_T_map_lidar = Eigen::Isometry3d::Identity();
                        const char* seed_src = "identity";
                        RCLCPP_WARN(node->get_logger(),
                                    "[indoor/gicp] session started — map_loaded=%d  "
                                    "seed_pos=(%.2f,%.2f,%.2f)  seed_src=%s  reference_pcd=%s",
                                    static_cast<int>(indoor_gicp_map_loaded),
                                    indoor_gicp_T_map_lidar.translation().x(),
                                    indoor_gicp_T_map_lidar.translation().y(),
                                    indoor_gicp_T_map_lidar.translation().z(),
                                    seed_src,
                                    ligo::indoor::getIndoorGicpMapPath().c_str());
                        if (nmea_global_anchor_ready && ligo::indoor::indoorGridMapsLoaded())
                        {
                            if (auto gtf = ligo::indoor::getFirstGridMapTransform())
                            {
                                const Eigen::Vector3d lla_grid =
                                    gnss_comm::ecef2geo(gtf->anchor_ecef_m);
                                const Eigen::Vector3d d = lla_grid - nmea_global_anchor_lla;
                                const double lat_rad = lla_grid.x() * M_PI / 180.0;
                                const double horiz_m =
                                    std::hypot(d.x() * 111319.9, d.y() * 111319.9 * std::max(0.1, std::cos(lat_rad)));
                                if (std::abs(d.x()) > 1e-6 || std::abs(d.y()) > 1e-6 || std::abs(d.z()) > 2.0)
                                {
                                    RCLCPP_WARN(node->get_logger(),
                                                "[indoor/align] grid yaml anchor LLA != this session "
                                                "nmea_global_anchor (d_lat=%.2e deg d_lon=%.2e d_alt=%.2f m, ~%.1f m horiz). "
                                                "Cross-session drift: set nmea.use_fixed_anchor true and "
                                                "nmea.fixed_anchor_lla_deg = anchor_lla_deg_m from *_grid2d.yaml",
                                                d.x(), d.y(), d.z(), horiz_m);
                                }
                                else
                                {
                                    RCLCPP_INFO(node->get_logger(),
                                                "[indoor/align] grid vs session anchor LLA consistent (within tol); "
                                                "remaining map vs live offset is mostly per-run NMEA–LIO ICP (icp_R/t) difference");
                                }
                            }
                        }
                    }
                    kf_output.change_P(P_init_output);
                }
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                flg_reset_indoor_reloc = false;
                init_map = false;
                
                {
                    ivox_.reset(new IVoxType(ivox_options_));
                    ivox_last_.reset(new IVoxType(ivox_options_));
                    traj_manager.reset(new curvefitter::TrajectoryManager<4>());
                }
            }

            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1)
                {
                    first_imu_time = rclcpp::Time(imu_next.header.stamp).seconds();
                    // printf("first imu time: %f acceleration: %f%f%f\n", first_imu_time, imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z);
                }
                time_current = 0.0;
                if(imu_en)
                {
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc *= -1; 

                    if (!nolidar && !imu_deque.empty())
                    {
                        while (Measures.lidar_beg_time > rclcpp::Time(imu_next.header.stamp).seconds()) // if it is needed for the new map?
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty())
                            {
                                break;
                            }
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_deque.pop();
                        }
                    }
                }
                else
                {
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc *= -1; 
                    p_imu->imu_need_init_ = false;
                    // p_imu->after_imu_init_ = true;
                }  
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
                // Legacy GNSS init path removed (NMEA-only).
            }

            double t0, t5;
            t0 = omp_get_wtime();
            
            /*** downsample the feature points in a scan ***/
            p_imu->Process(Measures, feats_undistort);
            if(space_down_sample)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            else
            {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            if (!nolidar)
            {
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }
            else
            {
                time_seq.clear();
            }
         
            if (!p_imu->after_imu_init_)
            {
                if (!p_imu->imu_need_init_)
                { 
                    V3D tmp_gravity;
                    if (init_with_imu && imu_en)
                    {
                        tmp_gravity = - p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    }
                    else
                    {   tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                        p_imu->after_imu_init_ = true;
                    }
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    // Legacy GNSS yaw init removed.
                    kf_output.x_.rot = rot_init;
                    // kf_output.x_.rot; //.normalize();
                    kf_output.x_.acc = - rot_init.transpose() * kf_output.x_.gravity;
                }
                else{
                goto after_sync_packages;}
            }
            /*** initialize the map ***/
            if(!init_map && !nolidar && !lose_lid)
            {
                feats_down_world->resize(feats_undistort->size());
                for(int i = 0; i < feats_undistort->size(); i++)
                {
                    {
                        pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
                    }
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) 
                {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if(init_feats_world->size() < init_map_size) 
                {init_map = false;}
                else
                {   
                    ivox_->AddPoints(init_feats_world->points);
                    // 
                    publish_init_map(pubLaserCloudMap); //(pubLaserCloudFullRes);
                    
                    init_feats_world.reset(new PointCloudXYZI());
                    init_map = true;
                    if (NMEA_ENABLE) traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                }
                goto after_sync_packages;
            }

            /*** ICP and Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);
            // t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);
            pimu_list.reserve(feats_down_size);
            // pbody_ext_list.reserve(feats_down_size);
                          
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    pimu_list[i] = point_this;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i]=point_crossmat;
            }
            {     
                effct_feat_num = 0;
                /**** point by point update ****/
                if (time_seq.size() > 0)
                {
                    if (NMEA_ENABLE)
                    {
                        p_nmea->p_assign->process_feat_num += time_seq.size();
                        p_nmea->nolidar_cur = false;
                    }
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (time_current < time_predict_last_const)
                    {
                        continue;
                    }

                    if (is_first_frame)
                    {
                        if(imu_en && !imu_deque.empty())
                        {
                            while (time_current > rclcpp::Time(imu_next.header.stamp).seconds())
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            if (imu_deque.empty()) break;
                        }
                        if (NMEA_ENABLE)
                        {
                            p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;
                        }
                        is_first_frame = false;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if(imu_en && !imu_deque.empty())
                    {
                        bool last_imu = rclcpp::Time(imu_next.header.stamp).seconds() == rclcpp::Time(imu_deque.front()->header.stamp).seconds();
                        while (rclcpp::Time(imu_next.header.stamp).seconds() < time_predict_last_const && !imu_deque.empty())
                        {
                            if (!last_imu)
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                break;
                            }
                            else
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            if (imu_deque.empty()) break;
                        }
                        bool imu_comes = time_current >= rclcpp::Time(imu_next.header.stamp).seconds();
                        while (imu_comes) 
                        {
                            if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                                const double nmea_lat = 0.0;  // latency 없음 가정
                                while (rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat < time_predict_last_const)
                                {
                                    p_nmea->nmea_msg.pop();
                                    if (!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                if (p_nmea->nmea_msg.empty()) break;
                                while ((rclcpp::Time(imu_next.header.stamp).seconds() >= rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat) && (rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat >= time_predict_last_const))
                                {
                                    double dt = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat - time_predict_last_const;
                                    double dt_cov = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat - time_update_last;

                                    nmeaMaybeTriggerOutdoorRealignAfterIndoor(nmea_cur);

                                    if (!p_nmea->nmea_ready)
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        kf_output.predict(dt, Q_output, input_in, true, false);
                                        time_predict_last_const = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat;
                                        time_update_last = time_predict_last_const;
                                        state_out = kf_output.x_;
                                        p_nmea->processNMEA(nmea_cur, state_out);
                                        nmeaClearCycleIfRealignComplete();
                                    }
                                    else
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        kf_output.predict(dt, Q_output, input_in, true, false);
                                        time_predict_last_const = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat;
                                        time_update_last = time_predict_last_const;
                                        p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                        p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                        double err_sq_xy_pre = 0.0;
                                        if (p_nmea->icp_tf_ready)
                                        {
                                            const Eigen::Vector3d p_gps_enu(nmea_cur->pose.pose.position.x, nmea_cur->pose.pose.position.y, nmea_cur->pose.pose.position.z);
                                            const Eigen::Vector3d p_lio_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.pos + p_nmea->icp_t_local_to_enu;
                                            const double dx = p_lio_enu.x() - p_gps_enu.x(), dy = p_lio_enu.y() - p_gps_enu.y();
                                            err_sq_xy_pre = dx * dx + dy * dy;
                                        }
                                        try
                                        {
                                            update_nmea = p_nmea->Evaluate(kf_output.x_);
                                        }
                                        catch (const std::exception &e)
                                        {
                                            RCLCPP_ERROR(
                                                rclcpp::get_logger("ligo"),
                                                "[nmea/eval] exception: %s. Skip this NMEA frame and continue.",
                                                e.what());
                                            const size_t pending_factors = p_nmea->p_assign->gtSAMgraph.size();
                                            const size_t pending_values = p_nmea->p_assign->initialEstimate.size();
                                            p_nmea->p_assign->gtSAMgraph.resize(0);
                                            p_nmea->p_assign->initialEstimate.clear();
                                            RCLCPP_WARN(
                                                rclcpp::get_logger("ligo"),
                                                "[nmea/eval] soft-recovery: dropped pending graph/values "
                                                "(factors=%zu, values=%zu) to avoid duplicate-key reinsert.",
                                                pending_factors, pending_values);
                                            update_nmea = false;
                                            p_nmea->nmea_ready = true;
                                        }
                                        if (!p_nmea->nmea_ready)
                                        {
                                            flg_reset = true;
                                            p_nmea->nmea_msg.pop();
                                            if (!p_nmea->nmea_msg.empty())
                                            {
                                                nmea_cur = p_nmea->nmea_msg.front();
                                            }
                                            break;
                                        }

                                        const bool cov_high_cfg = nmeaCovarianceIsHigh(nmea_cur, p_nmea->p_assign->ppp_std_threshold);
                                        const bool cov_high_temp = nmeaCovarianceIsHigh(nmea_cur, nmea_indoor_high_cov_threshold);
                                        // Track the last GNSS position (ECEF) with good outdoor quality.
                                        if (!cov_high_cfg && nmea_global_anchor_ready)
                                        {
                                            const Eigen::Vector3d p_enu_cur(
                                                nmea_cur->pose.pose.position.x,
                                                nmea_cur->pose.pose.position.y,
                                                nmea_cur->pose.pose.position.z);
                                            const Eigen::Vector3d anc = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                                            const Eigen::Matrix3d R   = gnss_comm::geo2rotation(nmea_global_anchor_lla);
                                            last_good_gnss_ecef       = anc + R * p_enu_cur;
                                            last_good_gnss_ecef_valid = true;
                                        }
                                        const bool trigger_normal = !mapping_mode && indoor_flag && indoor_pose_valid && !indoor_reloc_applied_once && cov_high_cfg;
                                        const bool trigger_temp = !mapping_mode && nmea_force_indoor_on_high_cov && !indoor_reloc_applied_once && cov_high_temp;
                                        if (trigger_normal || trigger_temp)
                                        {
                                            if (trigger_normal)
                                            {
                                                indoor_reloc_pos_enu = indoor_pos_enu_meas;
                                                indoor_reloc_rot_enu = indoor_rot_enu_meas.normalized().toRotationMatrix();
                                                indoor_reloc_pose_time = indoor_pose_time;
                                            }
                                            else
                                            {
                                                // Prefer fused LIO in ENU (same as /aft_mapped_to_init); then GNSS; then local.
                                                if (!ligo_fused_lio_pose_enu(indoor_reloc_pos_enu, indoor_reloc_rot_enu))
                                                {
                                                    if (last_good_gnss_ecef_valid && nmea_global_anchor_ready)
                                                    {
                                                        const Eigen::Vector3d anc = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                                                        const Eigen::Matrix3d R   = gnss_comm::geo2rotation(nmea_global_anchor_lla);
                                                        indoor_reloc_pos_enu = R.transpose() * (last_good_gnss_ecef - anc);
                                                        if (p_nmea->icp_tf_ready)
                                                            indoor_reloc_rot_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.rot;
                                                        else
                                                            indoor_reloc_rot_enu = kf_output.x_.rot;
                                                    }
                                                    else
                                                    {
                                                        indoor_reloc_pos_enu = kf_output.x_.pos;
                                                        indoor_reloc_rot_enu = kf_output.x_.rot;
                                                    }
                                                }
                                                indoor_reloc_pose_time = time_predict_last_const;
                                            }
                                            indoor_reloc_applied_once = true;
                                            flg_reset_indoor_reloc = true;
                                            flg_reset = true;
                                            break;
                                        }

                                        if (update_nmea)
                                        {
                                            if (p_nmea->icp_tf_ready) { p_nmea->sum_nmea_lio_err_sq_xy += err_sq_xy_pre; p_nmea->n_nmea_fusion_count++; }
                                            kf_output.update_iterated_dyn_share_NMEA();
                                            if (!runtime_pos_log) cout_state_to_file_nmea();
                                            // Add indoor pose factor alongside the NMEA factor just inserted.
                                            if (!mapping_mode && (indoor_flag || indoor_flag_dynamic) && indoor_pose_valid)
                                                ligo::indoor::addIndoorFactorToGraph(p_nmea->frame_num - 1);
                                        }
                                    }
                                    p_nmea->nmea_msg.pop();
                                    if (!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                            }
                            if (flg_reset)
                            {
                                break;
                            }
                            angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            double dt = rclcpp::Time(imu_next.header.stamp).seconds() - time_predict_last_const;
                            time_predict_last_const = rclcpp::Time(imu_next.header.stamp).seconds(); 
                            double dt_cov = rclcpp::Time(imu_next.header.stamp).seconds() - time_update_last; 

                            if (dt_cov > 0.0)
                            {
                                time_update_last = rclcpp::Time(imu_next.header.stamp).seconds();

                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            }
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            kf_output.update_iterated_dyn_share_IMU();
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_comes = time_current >= rclcpp::Time(imu_next.header.stamp).seconds();
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }
                    if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                    {
                        nmea_cur = p_nmea->nmea_msg.front();
                        const double nmea_lat2 = 0.0;  // latency 없음 가정
                        while (rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat2 < time_predict_last_const)
                        {
                            p_nmea->nmea_msg.pop();
                            if (!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (p_nmea->nmea_msg.empty()) break;
                        while (time_current >= rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat2 && rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat2 >= time_predict_last_const)
                        {
                            double dt = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat2 - time_predict_last_const;
                            double dt_cov = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat2 - time_update_last;

                            nmeaMaybeTriggerOutdoorRealignAfterIndoor(nmea_cur);

                            if (!p_nmea->nmea_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat2;
                                time_update_last = time_predict_last_const;
                                state_out = kf_output.x_;
                                p_nmea->processNMEA(nmea_cur, state_out);
                                nmeaClearCycleIfRealignComplete();
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);

                                time_predict_last_const = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat2;
                                time_update_last = time_predict_last_const;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                double err_sq_xy_pre2 = 0.0;
                                if (p_nmea->icp_tf_ready)
                                {
                                    const Eigen::Vector3d p_gps_enu(nmea_cur->pose.pose.position.x, nmea_cur->pose.pose.position.y, nmea_cur->pose.pose.position.z);
                                    const Eigen::Vector3d p_lio_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.pos + p_nmea->icp_t_local_to_enu;
                                    const double dx = p_lio_enu.x() - p_gps_enu.x(), dy = p_lio_enu.y() - p_gps_enu.y();
                                    err_sq_xy_pre2 = dx * dx + dy * dy;
                                }
                                try
                                {
                                    update_nmea = p_nmea->Evaluate(kf_output.x_);
                                }
                                catch (const std::exception &e)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("ligo"),
                                        "[nmea/eval] exception: %s. Skip this NMEA frame and continue.",
                                        e.what());
                                    const size_t pending_factors = p_nmea->p_assign->gtSAMgraph.size();
                                    const size_t pending_values = p_nmea->p_assign->initialEstimate.size();
                                    p_nmea->p_assign->gtSAMgraph.resize(0);
                                    p_nmea->p_assign->initialEstimate.clear();
                                    RCLCPP_WARN(
                                        rclcpp::get_logger("ligo"),
                                        "[nmea/eval] soft-recovery: dropped pending graph/values "
                                        "(factors=%zu, values=%zu) to avoid duplicate-key reinsert.",
                                        pending_factors, pending_values);
                                    update_nmea = false;
                                    p_nmea->nmea_ready = true;
                                }
                                if (!p_nmea->nmea_ready)
                                {
                                    flg_reset = true;
                                    p_nmea->nmea_msg.pop();
                                    if (!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    break;
                                }

                                const bool cov_high_cfg = nmeaCovarianceIsHigh(nmea_cur, p_nmea->p_assign->ppp_std_threshold);
                                const bool cov_high_temp = nmeaCovarianceIsHigh(nmea_cur, nmea_indoor_high_cov_threshold);
                                if (!cov_high_cfg && nmea_global_anchor_ready)
                                {
                                    const Eigen::Vector3d p_enu_cur(
                                        nmea_cur->pose.pose.position.x,
                                        nmea_cur->pose.pose.position.y,
                                        nmea_cur->pose.pose.position.z);
                                    const Eigen::Vector3d anc = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                                    const Eigen::Matrix3d R   = gnss_comm::geo2rotation(nmea_global_anchor_lla);
                                    last_good_gnss_ecef       = anc + R * p_enu_cur;
                                    last_good_gnss_ecef_valid = true;
                                }
                                const bool trigger_normal = !mapping_mode && indoor_flag && indoor_pose_valid && !indoor_reloc_applied_once && cov_high_cfg;
                                const bool trigger_temp = !mapping_mode && nmea_force_indoor_on_high_cov && !indoor_reloc_applied_once && cov_high_temp;
                                if (trigger_normal || trigger_temp)
                                {
                                    if (trigger_normal)
                                    {
                                        indoor_reloc_pos_enu = indoor_pos_enu_meas;
                                        indoor_reloc_rot_enu = indoor_rot_enu_meas.normalized().toRotationMatrix();
                                        indoor_reloc_pose_time = indoor_pose_time;
                                    }
                                    else
                                    {
                                        if (!ligo_fused_lio_pose_enu(indoor_reloc_pos_enu, indoor_reloc_rot_enu))
                                        {
                                            if (last_good_gnss_ecef_valid && nmea_global_anchor_ready)
                                            {
                                                const Eigen::Vector3d anc = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                                                const Eigen::Matrix3d R   = gnss_comm::geo2rotation(nmea_global_anchor_lla);
                                                indoor_reloc_pos_enu = R.transpose() * (last_good_gnss_ecef - anc);
                                                if (p_nmea->icp_tf_ready)
                                                    indoor_reloc_rot_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.rot;
                                                else
                                                    indoor_reloc_rot_enu = kf_output.x_.rot;
                                            }
                                            else
                                            {
                                                indoor_reloc_pos_enu = kf_output.x_.pos;
                                                indoor_reloc_rot_enu = kf_output.x_.rot;
                                            }
                                        }
                                        indoor_reloc_pose_time = time_predict_last_const;
                                    }
                                    indoor_reloc_applied_once = true;
                                    flg_reset_indoor_reloc = true;
                                    flg_reset = true;
                                    break;
                                }

                                if (update_nmea)
                                {
                                    if (p_nmea->icp_tf_ready) { p_nmea->sum_nmea_lio_err_sq_xy += err_sq_xy_pre2; p_nmea->n_nmea_fusion_count++; }
                                    kf_output.update_iterated_dyn_share_NMEA();
                                    if (!runtime_pos_log) cout_state_to_file_nmea();
                                    if (!mapping_mode && (indoor_flag || indoor_flag_dynamic) && indoor_pose_valid)
                                        ligo::indoor::addIndoorFactorToGraph(p_nmea->frame_num - 1);
                                }
                            }
                            p_nmea->nmea_msg.pop();
                            if (!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }
                    double dt = time_current - time_predict_last_const;
                    // double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;   
                        }
                    }
                    // if (dt > 0.0)
                    {
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    time_predict_last_const = time_current;
                    if (feats_down_size < 1)
                    {
                        RCLCPP_WARN(node->get_logger(), "No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) 
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    }
                    // else
                    // {
                    //     idx = idx+time_seq[k];
                    //     continue;
                    // }
                    
                    // solve_start = omp_get_wtime();
                        
                    if (publish_odometry_without_downsample && !flg_exit && rclcpp::ok())
                    {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, tf_br);
                        try_publish_fused_enu_position(pubEnuPosition);
                        try_publish_fused_enu_heading_deg(pubEnuHeadingDeg);
                        try_publish_fused_global_nav_sat(pubGlobalNavSat);
                        try_publish_fused_ecef_position(pubEcefPosition);
                        if (runtime_pos_log)
                        {
                            euler_cur = SO3ToEuler(kf_output.x_.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << kf_output.x_.pos.transpose() << " " << euler_cur.transpose() << " " << kf_output.x_.vel.transpose() \
                            <<" "<<kf_output.x_.omg.transpose()<<" "<<kf_output.x_.acc.transpose()<<" "<<kf_output.x_.gravity.transpose()<<" "<<kf_output.x_.bg.transpose()<<" "<<kf_output.x_.ba.transpose() << endl;
                        }
                    }
                    std::vector<Eigen::Vector3d> lidarpoints;
                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    if (NMEA_ENABLE)
                        lidarpoints.push_back(pimu_list[idx+j+1]); // (Eigen::Vector3d(point_body_j.x, point_body_j.y, point_body_j.z));
                    }
                    if (NMEA_ENABLE)
                    {
                        if (pose_graph_key_pose.empty()){
                            traj_manager->AddGraphPose(Eigen::Quaterniond(kf_output.x_.rot).normalized(), kf_output.x_.pos, lidarpoints, time_current, pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                        }else
                        {
                            if (time_current > pose_graph_key_pose.back().timestamp && lidarpoints.size() > 0)
                                traj_manager->AddGraphPose(Eigen::Quaterniond(kf_output.x_.rot).normalized(), kf_output.x_.pos, lidarpoints, time_current, pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                        }
                    }
                    idx += time_seq[k];
                }
                }
                else
                {
                    if (NMEA_ENABLE)  p_nmea->nolidar_cur = true;
                    if (!imu_deque.empty())
                    { 
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                    while (rclcpp::Time(imu_next.header.stamp).seconds() > time_current && ((rclcpp::Time(imu_next.header.stamp).seconds() < imu_first_time + lidar_time_inte && nolidar) || (rclcpp::Time(imu_next.header.stamp).seconds() < Measures.lidar_beg_time + lidar_time_inte && !nolidar)))
                    { // >= ?
                        if (is_first_frame)
                        {
                            if (!nolidar && NMEA_ENABLE)
                            {p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;}

                            if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                                const double nmea_lat_sync = 0.0;  // latency 없음 가정
                                double front_nmea_ts = rclcpp::Time(nmea_cur->header.stamp).seconds(); // take time
                                time_current = front_nmea_ts - time_diff_nmea_local - nmea_lat_sync;
                                while (rclcpp::Time(imu_next.header.stamp).seconds() < time_current) // 0.05
                                {
                                    RCLCPP_WARN(node->get_logger(), "throw IMU, only should happen at the beginning 2510");
                                    imu_deque.pop_front();
                                    if (imu_deque.empty()) break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front()); // could be used to initialize
                                }
                                if (imu_deque.empty()) break;
                            }
                            else
                            {
                                if (nolidar)
                                {
                                    while (rclcpp::Time(imu_next.header.stamp).seconds() < imu_first_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front()); // could be used to initialize
                                    }
                                    // if (imu_deque.empty()) break;
                                }
                                else
                                {
                                    while (rclcpp::Time(imu_next.header.stamp).seconds() < Measures.lidar_beg_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front());
                                    }
                                }
                                break;
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            if (nolidar) kf_output.x_.omg = angvel_avr;
                                            
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            time_current = rclcpp::Time(imu_next.header.stamp).seconds();

                            time_update_last = time_current;
                            time_predict_last_const = time_current;
                            acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                            if (NMEA_ENABLE)
                            {
                            p_nmea->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            p_nmea->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr); 
                            }
                            {
                                is_first_frame = false;
                            }
                        }
                        time_current = rclcpp::Time(imu_next.header.stamp).seconds();

                        if (!is_first_frame)
                        {
                        if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
                        {
                            nmea_cur = p_nmea->nmea_msg.front();
                            const double nmea_lat3 = 0.0;  // latency 없음 가정
                            while (rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3 < time_predict_last_const)
                            {
                                p_nmea->nmea_msg.pop();
                                if(!p_nmea->nmea_msg.empty())
                                {
                                    nmea_cur = p_nmea->nmea_msg.front();
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (p_nmea->nmea_msg.empty()) break;
                        while ((time_current > rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3) && (rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3 >= time_predict_last_const))
                        {
                            double dt = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3 - time_predict_last_const;
                            double dt_cov = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3 - time_update_last;

                            nmeaMaybeTriggerOutdoorRealignAfterIndoor(nmea_cur);

                            if (!p_nmea->nmea_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local;
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                if (p_nmea->nmea_ready)
                                {
                                    if (nolidar)
                                    {
                                        Eigen::Matrix3d R_enu_local;
                                        R_enu_local = p_nmea->Rot_nmea_init; 
                                        kf_output.x_.pos = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(0);
                                        kf_output.x_.rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_nmea->frame_num-1)).matrix();
                                        kf_output.x_.vel = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(3);
                                        kf_output.x_.ba = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.bg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.omg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.gravity = R_enu_local * kf_output.x_.gravity; // * R_enu_local_ 
                                        kf_output.x_.acc = kf_output.x_.rot.transpose() * (-kf_output.x_.gravity); // R_ecef_enu * state.vel_end;.conjugate().normalized()
                                        
                                        kf_output.P_ = MD(24,24)::Identity() * INIT_COV;
                                    }
                                }
                                nmeaClearCycleIfRealignComplete();
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                p_nmea->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); //acc_avr_norm, angvel_avr); 
                                time_predict_last_const = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                if (!nolidar)
                                {
                                    p_nmea->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                }
                                // ICP 이후 LIO-GPS 2D 오차 (ENU): Evaluate 직전에 계산
                                double err_sq_xy_pre3 = 0.0;
                                if (p_nmea->icp_tf_ready)
                                {
                                    const Eigen::Vector3d p_gps_enu(nmea_cur->pose.pose.position.x, nmea_cur->pose.pose.position.y, nmea_cur->pose.pose.position.z);
                                    const Eigen::Vector3d p_lio_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.pos + p_nmea->icp_t_local_to_enu;
                                    const double dx = p_lio_enu.x() - p_gps_enu.x(), dy = p_lio_enu.y() - p_gps_enu.y();
                                    err_sq_xy_pre3 = dx * dx + dy * dy;
                                }
                                try
                                {
                                    update_nmea = p_nmea->Evaluate(kf_output.x_);
                                }
                                catch (const std::exception &e)
                                {
                                    RCLCPP_ERROR(
                                        rclcpp::get_logger("ligo"),
                                        "[nmea/eval] exception: %s. Skip this NMEA frame and continue.",
                                        e.what());
                                    const size_t pending_factors = p_nmea->p_assign->gtSAMgraph.size();
                                    const size_t pending_values = p_nmea->p_assign->initialEstimate.size();
                                    p_nmea->p_assign->gtSAMgraph.resize(0);
                                    p_nmea->p_assign->initialEstimate.clear();
                                    RCLCPP_WARN(
                                        rclcpp::get_logger("ligo"),
                                        "[nmea/eval] soft-recovery: dropped pending graph/values "
                                        "(factors=%zu, values=%zu) to avoid duplicate-key reinsert.",
                                        pending_factors, pending_values);
                                    update_nmea = false;
                                    p_nmea->nmea_ready = true;
                                }
                                if (!p_nmea->nmea_ready)
                                {
                                    flg_reset = true;
                                    p_nmea->nmea_msg.pop();
                                    if(!p_nmea->nmea_msg.empty())
                                    {
                                        nmea_cur = p_nmea->nmea_msg.front();
                                    }
                                    break;
                                }
                                const bool cov_high_cfg = nmeaCovarianceIsHigh(nmea_cur, p_nmea->p_assign->ppp_std_threshold);
                                const bool cov_high_temp = nmeaCovarianceIsHigh(nmea_cur, nmea_indoor_high_cov_threshold);
                                if (!cov_high_cfg && nmea_global_anchor_ready)
                                {
                                    const Eigen::Vector3d p_enu_cur(
                                        nmea_cur->pose.pose.position.x,
                                        nmea_cur->pose.pose.position.y,
                                        nmea_cur->pose.pose.position.z);
                                    const Eigen::Vector3d anc = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                                    const Eigen::Matrix3d R   = gnss_comm::geo2rotation(nmea_global_anchor_lla);
                                    last_good_gnss_ecef       = anc + R * p_enu_cur;
                                    last_good_gnss_ecef_valid = true;
                                }
                                const bool trigger_normal = !mapping_mode && indoor_flag && indoor_pose_valid && !indoor_reloc_applied_once && cov_high_cfg;
                                const bool trigger_temp = !mapping_mode && nmea_force_indoor_on_high_cov && !indoor_reloc_applied_once && cov_high_temp;
                                if (trigger_normal || trigger_temp)
                                {
                                    if (trigger_normal)
                                    {
                                        indoor_reloc_pos_enu = indoor_pos_enu_meas;
                                        indoor_reloc_rot_enu = indoor_rot_enu_meas.normalized().toRotationMatrix();
                                        indoor_reloc_pose_time = indoor_pose_time;
                                    }
                                    else
                                    {
                                        if (!ligo_fused_lio_pose_enu(indoor_reloc_pos_enu, indoor_reloc_rot_enu))
                                        {
                                            if (last_good_gnss_ecef_valid && nmea_global_anchor_ready)
                                            {
                                                const Eigen::Vector3d anc = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                                                const Eigen::Matrix3d R   = gnss_comm::geo2rotation(nmea_global_anchor_lla);
                                                indoor_reloc_pos_enu = R.transpose() * (last_good_gnss_ecef - anc);
                                                if (p_nmea->icp_tf_ready)
                                                    indoor_reloc_rot_enu = p_nmea->icp_R_local_to_enu * kf_output.x_.rot;
                                                else
                                                    indoor_reloc_rot_enu = kf_output.x_.rot;
                                            }
                                            else
                                            {
                                                indoor_reloc_pos_enu = kf_output.x_.pos;
                                                indoor_reloc_rot_enu = kf_output.x_.rot;
                                            }
                                        }
                                        indoor_reloc_pose_time = time_predict_last_const;
                                    }
                                    indoor_reloc_applied_once = true;
                                    flg_reset_indoor_reloc = true;
                                    flg_reset = true;
                                    break;
                                }
                                if (update_nmea)
                                {
                                    if (p_nmea->icp_tf_ready) { p_nmea->sum_nmea_lio_err_sq_xy += err_sq_xy_pre3; p_nmea->n_nmea_fusion_count++; }
                                    if (!nolidar)
                                    {
                                        kf_output.update_iterated_dyn_share_NMEA();
                                        // reset_cov_output(kf_output.P_);
                                    }
                                    if (!runtime_pos_log) cout_state_to_file_nmea();
                                    if (!mapping_mode && (indoor_flag || indoor_flag_dynamic) && indoor_pose_valid)
                                        ligo::indoor::addIndoorFactorToGraph(p_nmea->frame_num - 1);
                                }
                            }
                            p_nmea->nmea_msg.pop();
                            if(!p_nmea->nmea_msg.empty())
                            {
                                nmea_cur = p_nmea->nmea_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        }
                        if (flg_reset)
                        {
                            break;
                        }
                        double dt = time_current - time_predict_last_const;
                        {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0)
                            {
                                // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                            // kf_output.predict(dt, Q_output, input_in, true, false);
                            if (NMEA_ENABLE)   p_nmea->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); // acc_avr_norm, angvel_avr); // 
                        }

                        time_predict_last_const = time_current;

                        angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        if (nolidar) kf_output.x_.omg = angvel_avr;
                        acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z; 
                        acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                        kf_output.update_iterated_dyn_share_IMU();
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    else
                    {
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    }
                    }
                }
            }
            
            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample && !flg_exit && rclcpp::ok())
            {
                publish_odometry(pubOdomAftMapped, tf_br);
                try_publish_fused_enu_position(pubEnuPosition);
                try_publish_fused_enu_heading_deg(pubEnuHeadingDeg);
                try_publish_fused_global_nav_sat(pubGlobalNavSat);
                try_publish_fused_ecef_position(pubEcefPosition);
            }

            /*** add the feature points to map ***/
            if(feats_down_size > 4)
            {
                MapIncremental();
            }

            // Grid-based map selection: only in an active indoor session (indoor_flag_dynamic)
            if (!mapping_mode && indoor_flag_dynamic && ligo::indoor::indoorGridMapsLoaded() &&
                NMEA_ENABLE)
            {
                Eigen::Vector3d p_ecef;
                if (compute_ligo_global_topic_ecef(p_ecef))
                {
                    ligo::indoor::ensureIndoorGICPMapFromGridEcef(p_ecef);
                }
            }
            // Indoor GICP first when possible so reference map can latch T^{-1} before first /indoor/map_cloud.
            if (!mapping_mode && indoor_flag_dynamic &&
                indoor_gicp_map_loaded && p_nmea && p_nmea->icp_tf_ready &&
                feats_down_world && !feats_down_world->empty())
            {
                ligo::indoor::runIndoorGICPUpdate(
                    feats_down_world,
                    lidar_end_time,
                    p_nmea->icp_R_local_to_enu,
                    p_nmea->icp_t_local_to_enu);

                // Publish viz every attempt (success or failure) so RViz shows live scan vs map
                ligo::indoor::publishIndoorViz(
                    pubIndoorMapCloud, pubIndoorAlignedScan,
                    feats_down_world,
                    p_nmea->icp_R_local_to_enu,
                    p_nmea->icp_t_local_to_enu,
                    indoor_gicp_T_map_lidar,
                    lidar_end_time,
                    pubIndoorMap2d);

            }
            else
            if (!mapping_mode && indoor_flag_dynamic && indoor_gicp_map_loaded)
            {
                // No GICP this frame (no icp_tf / no feats): publish map without align deferral
                const bool defer_align = false;
                ligo::indoor::publishIndoorMapCloudOnly(pubIndoorMapCloud, lidar_end_time, pubIndoorMap2d,
                                                        defer_align);
            }

            t5 = omp_get_wtime();
            if (log_lidar_frame_time_ms) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "[lidar/timing] frame_wall_ms=%.2f lidar_end_t=%.6f feats_down=%d",
                    (t5 - t0) * 1000.0, lidar_end_time, feats_down_size);
            }
            /******* Publish points *******/
            if (!flg_exit && rclcpp::ok())
            {
                publish_nmea_aligned(pubNmeaAlignedOdom, pubNmeaAlignedPath);
                publish_icp_pairs_marker(pubIcpPairs, pubNmeaLioErrorXy, pubNmea03mDiag);
                publish_heading_align_status(pubHeadingAlignStatus, lidar_end_time);
                try_publish_nmea_graph_anchor_marker(pubNmeaGraphAnchorMarker);
                publish_init_pairs_marker_from_gps_move(pubInitPairsFromGpsMove);
                if (path_en)                         publish_path(pubPath);
                if (scan_pub_en || mapping_mode)      publish_frame_world(pubLaserCloudFullRes);
                if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
            }
            
            /*** Debug variables Logging ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                if (!publish_odometry_without_downsample)
                {
                    {
                        {
                            Eigen::Vector3d pos_r = kf_output.x_.rot * p_nmea->Tex_imu_r + kf_output.x_.pos;
                            time_frame.push_back(lidar_end_time); //(time_predict_last_const);
                            est_poses.push_back(pos_r);
                        }
                        euler_cur = SO3ToEuler(kf_output.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << kf_output.x_.pos.transpose() << " " << euler_cur.transpose() << " " << kf_output.x_.vel.transpose() \
                        <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose() << endl;
                    }
                }
            }
        }
after_sync_packages:
        {
            const bool cur_dyn = indoor_flag_dynamic;
            static bool mode_edge_init = false;
            static bool mode_prev_dyn = false;
            if (!mode_edge_init)
            {
                mode_edge_init = true;
                mode_prev_dyn = cur_dyn;
            }
            else if (cur_dyn != mode_prev_dyn)
            {
                std_msgs::msg::String sm;
                if (cur_dyn)
                {
                    // 파일명만 전달. indoor.map_pcd_path 미설정 시 그리드/GICP가 실제 로드한 PCD 경로 사용.
                    std::string pcd_name;
                    if (!indoor_map_pcd_path.empty())
                    {
                        pcd_name = std::filesystem::path(indoor_map_pcd_path).filename().string();
                    }
                    if (pcd_name.empty())
                    {
                        const std::string resolved = ligo::indoor::getIndoorGicpMapPath();
                        if (!resolved.empty())
                            pcd_name = std::filesystem::path(resolved).filename().string();
                    }
                    sm.data = std::string("{\"mode\":\"indoor\",\"pcd_name\":\"") + ligo_json_escape(pcd_name) + "\"}";
                }
                else
                {
                    sm.data = "{\"mode\":\"outdoor\"}";
                }
                pubLigoMode->publish(sm);
                RCLCPP_WARN(node->get_logger(), "[ligo/mode] %s", cur_dyn ? "indoor" : "outdoor");
                mode_prev_dyn = cur_dyn;
            }
        }
        loop_rate.sleep();
    }
    // Tear down ROS entities before context shutdown to avoid late publish during destruction.
    sub_pcl.reset();
    sub_imu.reset();
    sub_nmea_meas.reset();
    pubLaserCloudFullRes.reset();
    pubLaserCloudFullRes_body.reset();
    pubLaserCloudEffect.reset();
    pubLaserCloudMap.reset();
    pubOdomAftMapped.reset();
    pubPath.reset();
    pubLigoMode.reset();
    pubNmeaAlignedOdom.reset();
    pubNmeaAlignedPath.reset();
    pubNmeaLioErrorXy.reset();
    pubNmea03mDiag.reset();
    pubHeadingAlignStatus.reset();
    pubIcpPairs.reset();
    pubInitPairsFromGpsMove.reset();
    plane_pub.reset();
    pubEnuPosition.reset();
    pubEnuHeadingDeg.reset();
    pubEcefPosition.reset();
    pubGlobalNavSat.reset();
    pubIndoorMapCloud.reset();
    pubIndoorMap2d.reset();
    pubIndoorAlignedScan.reset();
    ligo_reset_nmea_stamp_diag_publisher();

    fout_out.close();
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && mapping_mode)
    {
        string all_points_dir = ligo_make_pcd_save_path();
        const bool saved_in_enu = (NMEA_ENABLE && p_nmea && p_nmea->icp_tf_ready);
        const Eigen::Matrix3d R_ecef_enu = saved_in_enu ? gnss_comm::ecef2rotation(first_gps_ecef) : Eigen::Matrix3d::Identity();
        const Eigen::Vector3d anchor_ecef_m = saved_in_enu ? first_gps_ecef : Eigen::Vector3d::Zero();
        const Eigen::Vector3d anchor_lla_deg_m = saved_in_enu ? first_gps_lla : Eigen::Vector3d::Zero();
        cout << "current scan saved to " << all_points_dir << (saved_in_enu ? " (ENU)" : "") << endl;
        if (ligo_try_write_binary_pcd(all_points_dir, pcl_wait_save))
        {
            if (saved_in_enu)
            {
                save_grid2d_from_cloud_with_rays(
                    pcl_wait_save, pcl_wait_ray_origins, all_points_dir, "enu", pcd_save_grid2d_resolution_m,
                    3, -1e9, 1e9, true, R_ecef_enu, anchor_ecef_m, anchor_lla_deg_m);
                ligo_save_ecef_companion_pcd(all_points_dir, pcl_wait_save, R_ecef_enu, anchor_ecef_m);
            }
        }
    }
    if (mapping_mode && pcd_tmp_map_enable && pcl_wait_save_tmp_map->size() > 0)
    {
        ligo_flush_tmp_map_bucket(node->get_logger());
    }
    if (mapping_mode)
    {
        ligo_cleanup_tmp_map_pcd_files(node->get_logger());
    }
    {
        Eigen::Vector3d ref_ecef = first_pvt_used;
        Eigen::Vector3d ref_lla = first_lla_used;
        if (NMEA_ENABLE && nmea_global_anchor_ready)
        {
#ifdef process_ppp
            const bool ppp_anchor = !ppp_ecef.empty();
#else
            const bool ppp_anchor = false;
#endif
            if (!ppp_anchor)
            {
                ref_ecef = gnss_comm::geo2ecef(nmea_global_anchor_lla);
                ref_lla = nmea_global_anchor_lla;
            }
        }
        first_pvt_anc = ref_ecef;
        first_lla_anc = ref_lla;
        Eigen::Matrix3d enu_rot = gnss_comm::ecef2rotation(ref_ecef);
        for (int i = 0; i < time_frame.size(); i++)
        {
            if (NMEA_ENABLE)
            {
                Eigen::Vector3d ecef_r = enu_rot * est_poses[i] + ref_ecef;
                Eigen::Vector3d pos_enu = gnss_comm::ecef2enu(first_lla_anc, ecef_r - first_pvt_anc);
#ifdef process_ppp
                if (!ppp_ecef.empty())
                {
                    fout_global << setw(20) << time_frame[i] - ppp_ecef[0][0] + 18.0 << " " << pos_enu.transpose() << endl;
                }
                else
#endif
                {
                    fout_global << setw(20) << time_frame[i] - time_frame[0] << " " << pos_enu.transpose() << endl;
                }
            }
            else
            {
                fout_global << setw(20) << time_frame[i] - time_frame[0] << " " << est_poses[i].transpose() << endl;
            }
        }
    if (fout_global.is_open())
        fout_global.close();
    }

    #ifdef process_ppp
    for (int i = 0; i < ppp_ecef.size(); i++)
    {
        Eigen::Vector3d pos_enu = gnss_comm::ecef2enu(first_lla_used, ppp_ecef[i].segment<3>(1) - first_pvt_used);
        fout_ppp << setw(20) << ppp_ecef[i][0] - ppp_ecef[0][0] << " " << pos_enu.transpose() << endl;
    }
    fout_ppp.close();
    #endif
    
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
