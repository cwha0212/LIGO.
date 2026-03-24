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
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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
#include "li_initialization.h"

/** Implemented in li_initialization.cpp (not declared in li_initialization.h to avoid include-order / GTSAM issues). */
void ligo_try_create_nmea_stamp_diag_publisher(std::shared_ptr<rclcpp::Node> node);

#include "Indoor_Processing.h"
#include <malloc.h>
#include <fstream>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "chi-square.h"
#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0; 

bool init_map = false, flg_first_scan = true;
std::vector<ObsPtr> gnss_cur;
nav_msgs::msg::Odometry::SharedPtr nmea_cur;
Eigen::Vector3d first_pvt_anc, first_lla_anc;
Eigen::Vector3d first_pvt_used, first_lla_used;

bool  flg_reset = false, flg_exit = false;
bool  flg_reset_indoor_reloc = false;
bool  indoor_reloc_applied_once = false;
Eigen::Vector3d indoor_reloc_pos_enu = Eigen::Vector3d::Zero();
Eigen::Matrix3d indoor_reloc_rot_enu = Eigen::Matrix3d::Identity();
double indoor_reloc_pose_time = 0.0;
constexpr bool kTempForceIndoorByNmeaCov = true;
constexpr double kTempIndoorCovThreshold = 50.0;

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

void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_WARN(rclcpp::get_logger("ligo"), "catch sig %d", sig);
    sig_buffer.notify_all();
}

static inline bool nmeaCovarianceIsHigh(const nav_msgs::msg::Odometry::SharedPtr &msg,
                                        double threshold)
{
    return msg->pose.covariance[0] >= threshold ||
           msg->pose.covariance[7] >= threshold ||
           msg->pose.covariance[14] >= threshold;
}

#ifndef LIGO_WITHOUT_GNSS
// Matches NMEAProcess::processNMEA gate for collecting alignment window (reject if any diagonal > thr).
static inline bool nmeaCovarianceAcceptableForNmeaInit(const nav_msgs::msg::Odometry::SharedPtr &msg,
                                                        double threshold)
{
    return msg->pose.covariance[0] <= threshold &&
           msg->pose.covariance[7] <= threshold &&
           msg->pose.covariance[14] <= threshold;
}

static int nmea_outdoor_good_streak = 0;
static bool nmea_cycle_reopen_pending = false;

// #region agent log
/** NDJSON debug ingest: session a3a668 — hypotheses H1–H4 for NMEA outdoor re-align path. */
static void nmeaOutdoorDebugLog(const char *hypothesisId, const char *location, const char *message,
                                long long cov0_u, long long cov7_u, long long cov14_u,
                                int streak, int need, int pending, int nmea_ready_i, int indoor_once_i)
{
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    std::ofstream lf("/home/chang/projects/NAVICOM/GPS_LIO_ws/src/LIGO./.cursor/debug-a3a668.log", std::ios::app);
    if (!lf)
        return;
    lf << "{\"sessionId\":\"a3a668\",\"runId\":\"pre-verify\",\"hypothesisId\":\"" << hypothesisId
       << "\",\"location\":\"" << location << "\",\"message\":\"" << message
       << "\",\"data\":{\"cov0_e6\":" << cov0_u << ",\"cov7_e6\":" << cov7_u << ",\"cov14_e6\":" << cov14_u
       << ",\"streak\":" << streak << ",\"need\":" << need << ",\"pending\":" << pending
       << ",\"nmea_ready\":" << nmea_ready_i << ",\"indoor_reloc_once\":" << indoor_once_i << "}"
       << ",\"timestamp\":" << ms << "}\n";
}
// #endregion

static void nmeaMaybeTriggerOutdoorRealignAfterIndoor(const nav_msgs::msg::Odometry::SharedPtr &nmea_cur)
{
    if (!NMEA_ENABLE)
        return;
    // H4: path never entered because preconditions false (sample occasionally to avoid spam)
    static int nmea_dbg_skip = 0;
    if (!p_nmea->nmea_ready || !indoor_reloc_applied_once)
    {
        if ((++nmea_dbg_skip % 200) == 0)
        {
            // #region agent log
            nmeaOutdoorDebugLog("H4", "laserMapping.cpp:nmeaMaybeTriggerOutdoorRealignAfterIndoor", "precondition_false",
                                (long long)std::llround(nmea_cur->pose.covariance[0] * 1e6),
                                (long long)std::llround(nmea_cur->pose.covariance[7] * 1e6),
                                (long long)std::llround(nmea_cur->pose.covariance[14] * 1e6),
                                nmea_outdoor_good_streak, p_nmea ? (p_nmea->wind_size < 1 ? 1 : p_nmea->wind_size) : -1,
                                nmea_cycle_reopen_pending ? 1 : 0, p_nmea && p_nmea->nmea_ready ? 1 : 0,
                                indoor_reloc_applied_once ? 1 : 0);
            // #endregion
        }
        return;
    }
    const double thr = p_nmea->p_assign->ppp_std_threshold;
    const long long c0 = (long long)std::llround(nmea_cur->pose.covariance[0] * 1e6);
    const long long c7 = (long long)std::llround(nmea_cur->pose.covariance[7] * 1e6);
    const long long c14 = (long long)std::llround(nmea_cur->pose.covariance[14] * 1e6);
    const int need = p_nmea->wind_size < 1 ? 1 : p_nmea->wind_size;
    if (!nmeaCovarianceAcceptableForNmeaInit(nmea_cur, thr))
    {
        const int was = nmea_outdoor_good_streak;
        nmea_outdoor_good_streak = 0;
        if (was > 0)
        {
            // #region agent log
            // H3: good-cov streak broken by a bad sample while still in indoor session
            nmeaOutdoorDebugLog("H3", "laserMapping.cpp:nmeaMaybeTriggerOutdoorRealignAfterIndoor", "streak_broken_by_cov",
                                c0, c7, c14, was, need, nmea_cycle_reopen_pending ? 1 : 0, 1, 1);
            // #endregion
        }
        return;
    }
    nmea_outdoor_good_streak++;
    // #region agent log
    // H1/H4: progress toward outdoor Reset (init-quality covariance streak)
    nmeaOutdoorDebugLog("H4", "laserMapping.cpp:nmeaMaybeTriggerOutdoorRealignAfterIndoor", "good_cov_streak_tick",
                        c0, c7, c14, nmea_outdoor_good_streak, need, nmea_cycle_reopen_pending ? 1 : 0, 1, 1);
    // #endregion
    if (nmea_outdoor_good_streak >= need)
    {
        RCLCPP_WARN(rclcpp::get_logger("ligo"),
                    "NMEA outdoor re-align: Reset() after %d consecutive init-quality covariance samples (wind_size=%d)",
                    nmea_outdoor_good_streak, p_nmea->wind_size);
        // #region agent log
        // H2: NMEA graph Reset() fired for outdoor re-align
        nmeaOutdoorDebugLog("H2", "laserMapping.cpp:nmeaMaybeTriggerOutdoorRealignAfterIndoor", "reset_called_before",
                            c0, c7, c14, nmea_outdoor_good_streak, need, 0, 1, 1);
        // #endregion
        p_nmea->Reset();
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
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "NMEA outdoor re-align finished; indoor_reloc_applied_once cleared (indoor can trigger again)");
    // #region agent log
    // H2: realign finished — indoor cycle can trigger again
    nmeaOutdoorDebugLog("H2", "laserMapping.cpp:nmeaClearCycleIfRealignComplete", "cycle_cleared_indoor_rearmed",
                        0, 0, 0, 0, p_nmea ? (p_nmea->wind_size < 1 ? 1 : p_nmea->wind_size) : -1, 0, 1, 0);
    // #endregion
}
#endif

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
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body); // (points_num); // 
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        
        for (int i = 0; i < size; i++)
        {
            // if (i % 3 == 0)
            {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x; // updatedmap[i / 3](0); // 
            laserCloudWorld->points[i].y = feats_down_world->points[i].y; // updatedmap[i / 3](1); // 
            laserCloudWorld->points[i].z = feats_down_world->points[i].z; // updatedmap[i / 3](2); // 
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; // updatedmap[i / 3](2); //feats_down_world->points[i].z; // 
            }
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        
        laserCloudmsg.header.stamp.sec = static_cast<int32_t>(std::floor(lidar_end_time));
        laserCloudmsg.header.stamp.nanosec = static_cast<uint32_t>(std::round((lidar_end_time - std::floor(lidar_end_time)) * 1e9));
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        // publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = points_num; // feats_down_world->points.size();
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x; // updatedmap[i](0); //
            laserCloudWorld->points[i].y = feats_down_world->points[i].y; // updatedmap[i](1); //
            laserCloudWorld->points[i].z = feats_down_world->points[i].z; // updatedmap[i](2); //
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // updatedmap[i](2); //
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
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
    }
    else
    {
        set_posestamp(out);
    }
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped, tf2_ros::TransformBroadcaster & br)
{
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
#ifndef LIGO_WITHOUT_GNSS
    if (!pubEnuPosition)
        return;
    Eigen::Vector3d p_enu;
    if (!compute_fused_imu_position_enu(p_enu))
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
#else
    (void)pubEnuPosition;
#endif
}

static void try_publish_fused_global_nav_sat(
    const rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr &pubGlobalFix)
{
#ifndef LIGO_WITHOUT_GNSS
    if (!pubGlobalFix)
        return;
    Eigen::Vector3d lla;
    if (!compute_fused_imu_position_geo(lla))
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
#else
    (void)pubGlobalFix;
#endif
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
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

void publish_nmea_aligned(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubNmeaAlignedOdom,
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubNmeaAlignedPath)
{
#ifndef LIGO_WITHOUT_GNSS
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
#endif
}

void publish_icp_pairs_marker(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pubIcpPairs,
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &pubNmeaLioErrorXy,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pubNmea03mDiag)
{
#ifndef LIGO_WITHOUT_GNSS
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
#endif
}

void publish_init_pairs_marker_from_gps_move(
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
{
#ifndef LIGO_WITHOUT_GNSS
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
#endif
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("laserMapping");
    readParameters(node.get());
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
#ifndef LIGO_WITHOUT_GNSS
    if (GNSS_ENABLE)
    {
        std::copy(default_gnss_iono_params.begin(), default_gnss_iono_params.end(), 
            std::back_inserter(p_gnss->p_assign->latest_gnss_iono_params));
        p_gnss->Tex_imu_r << VEC_FROM_ARRAY(extrinT_gnss);
        p_gnss->gnss_ready = false; // gnss_quick_init; // edit
        p_gnss->nolidar = nolidar; // edit
        p_gnss->pre_integration->setnoise();

        if (p_gnss->p_assign->ephem_from_rinex)
        {
            p_gnss->p_assign->Ephemfromrinex(LOCAL_FILE_DIR(ephem_fname));
        }
    }
    else if (NMEA_ENABLE)
    {
        p_nmea->Tex_imu_r << VEC_FROM_ARRAY(extrinT_gnss);
        p_nmea->Rex_imu_r << MAT_FROM_ARRAY(extrinR_gnss);
        p_nmea->nmea_ready = false; // gnss_quick_init; // edit
        p_nmea->nolidar = nolidar; // edit
        p_nmea->pre_integration->setnoise();
    }
#endif
    if (NMEA_ENABLE)
    {
        kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output, h_model_NMEA_output);
    }
    else
    {
        kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output, h_model_GNSS_output);
    }
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

#ifndef LIGO_WITHOUT_GNSS
    rclcpp::SubscriptionBase::SharedPtr sub_ephem, sub_glo_ephem, sub_gnss_meas, sub_gnss_iono_params;
    rclcpp::SubscriptionBase::SharedPtr sub_gnss_time_pluse_info, sub_local_trigger_info;
    rclcpp::SubscriptionBase::SharedPtr sub_rtk_pvt_info, sub_rtk_lla_info;
    rclcpp::SubscriptionBase::SharedPtr sub_nmea_meas;
    if (GNSS_ENABLE)
    {
        rclcpp::QoS qos_gnss(10000);
        sub_ephem = node->create_subscription<gnss_comm::msg::GnssEphemMsg>(gnss_ephem_topic, qos_gnss, gnss_ephem_callback);
        sub_glo_ephem = node->create_subscription<gnss_comm::msg::GnssGloEphemMsg>(gnss_glo_ephem_topic, qos_gnss, gnss_glo_ephem_callback);
        #ifdef LIGO_WITH_URBANNAV_MSG
        if (p_gnss->p_assign->obs_from_rinex)
        {
            sub_gnss_meas = node->create_subscription<nlosExclusion::GNSS_Raw_Array>("/gnss_preprocessor_node/GNSSPsrCarRov1", 200, gnss_meas_callback_urbannav);
            sub_rtk_pvt_info = node->create_subscription<gnss_comm::msg::GnssPVTSolnMsg>("/gnss_preprocessor_node/ECEFSolutionRTK", 500, rtklibOdomHandler);
        }
        else
        #endif
        {
            sub_gnss_meas = node->create_subscription<gnss_comm::msg::GnssMeasMsg>(gnss_meas_topic, qos_gnss, gnss_meas_callback);
            sub_rtk_lla_info = node->create_subscription<sensor_msgs::msg::NavSatFix>(rtk_lla_topic, 1000, rtk_lla_callback);
        }

        if (p_gnss->p_assign->pvt_is_gt)
        {
            sub_rtk_pvt_info = node->create_subscription<gnss_comm::msg::GnssPVTSolnMsg>(rtk_pvt_topic, 1000, rtk_pvt_callback);
        }
        else
        {
            std::vector<Eigen::Vector4d> gt_holder;
            if (gt_file_type == LIVOX)
            {
                GtfromTXT_LIVOX(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            else if (gt_file_type == URBAN)
            {
                GtfromTXT_URBAN(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            else if (gt_file_type == M2DGR)
            {
                GtfromTXT_M2DGR(LOCAL_FILE_DIR(gt_fname), gt_holder);
            }
            std::cout << "check gt size:" << gt_holder.size() << std::endl;
            if (gt_file_type == M2DGR)
            {
                for (size_t i = 0; i < gt_holder.size(); i++)
                {
                    inputpvt_ecef(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                            p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
                }
            }
            else
            {
                for (size_t i = 0; i < gt_holder.size(); i++)
                {
                    inputpvt_lla(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                            p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
                }
            }
        }
        sub_gnss_iono_params = node->create_subscription<gnss_comm::msg::StampedFloat64Array>(gnss_iono_params_topic, qos_gnss, gnss_iono_params_callback);

        if (gnss_local_online_sync)
        {
            sub_gnss_time_pluse_info = node->create_subscription<gnss_comm::msg::GnssTimePulseInfoMsg>(gnss_tp_info_topic, 100, gnss_tp_info_callback);
            sub_local_trigger_info = node->create_subscription<ligo::msg::LocalSensorExternalTrigger>(local_trigger_info_topic, 100, local_trigger_info_callback);
        }
        else
        {
            time_diff_gnss_local = gnss_local_time_diff; // 18.0
            p_gnss->inputGNSSTimeDiff(time_diff_gnss_local);
            time_diff_valid = true;
        }
    }
    else
    {
        if (!NMEA_ENABLE)
        {
            sub_rtk_pvt_info = node->create_subscription<gnss_comm::msg::GnssPVTSolnMsg>(rtk_pvt_topic, 100, rtk_pvt_callback);
        }
    }
#endif
    #ifdef process_ppp
    if (NMEA_ENABLE)
    {
        std::vector<Eigen::Vector4d> gt_holder;
        // GtfromTXT_DJI(string("/home/joannahe/NewDisk/self-collected/gt_deg2.txt"), gt_urbannav_holder);
        // std::vector<Eigen::Vector4d>().swap(gt_urbannav_holder);
        // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_TST_GT_raw.txt"), gt_urbannav_holder);
        // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_whampoa_raw.txt"), gt_urbannav_holder);
        // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_mongkok_GT_part_raw.txt"), gt_urbannav_holder);
        // GtfromTXT(string("/home/joannahe/NewDisk/gnss-lio/urbannav/gt/UrbanNav_tunnel_GT_raw.txt"), gt_urbannav_holder);
        // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/tree3x.txt"), gt_urbannav_holder);
        // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/switch2_rawcut.txt"), gt_urbannav_holder);
        if (gt_file_type == LIVOX)
        {
            GtfromTXT_LIVOX(LOCAL_FILE_DIR(gt_fname), gt_holder);
        }
        else if (gt_file_type == URBAN)
        {
            GtfromTXT_URBAN(LOCAL_FILE_DIR(gt_fname), gt_holder);
        }
        else if (gt_file_type == M2DGR)
        {
            GtfromTXT_M2DGR(LOCAL_FILE_DIR(gt_fname), gt_holder);
        }
        // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/parking2.txt"), gt_urbannav_holder);
        // GtfromTXT_M2DGR(string("/home/joannahe/NewDisk/m2dgr/M2DGR-plus/gt/bridge2.txt"), gt_urbannav_holder);
        if (gt_file_type == M2DGR)
        {
            for (size_t i = 0; i < gt_holder.size(); i++)
            {
                inputpvt_ecef(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                        p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
            }
        }
        else
        {
            for (size_t i = 0; i < gt_holder.size(); i++)
            {
                inputpvt_lla(gt_holder[i][0], gt_holder[i][1], gt_holder[i][2], gt_holder[i][3], p_gnss->first_lla_pvt, p_gnss->first_xyz_ecef_pvt, p_gnss->pvt_time, 
                        p_gnss->pvt_holder, p_gnss->diff_holder, p_gnss->float_holder); // 
            }
        }
    }
    
    PPPfromTXT(LOCAL_FILE_DIR(ppp_fname), ppp_sol, ppp_ecef);
    if (NMEA_ENABLE)
    {

        if (GNSS_ENABLE)
        {
            first_pvt_anc = p_gnss->first_xyz_ecef_pvt;
            first_lla_anc = p_gnss->first_lla_pvt;
        }
        if (ppp_ecef.size() > 0)
        {
            if (GNSS_ENABLE && p_gnss->p_assign->pvt_is_gt)
            {
                first_pvt_anc << VEC_FROM_ARRAY(ppp_anc);
                first_lla_anc = ecef2geo(first_pvt_anc);
            }
            first_pvt_used = ppp_ecef[0].segment<3>(1);
            first_lla_used = ecef2geo(first_pvt_used);
            if (!nmea_global_anchor_ready)
            {
                nmea_global_anchor_lla = first_lla_used;
                nmea_global_anchor_ready = true;
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
    auto pubOdomAftMapped = node->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", qos_pub);
    auto pubPath = node->create_publisher<nav_msgs::msg::Path>("/path", qos_pub);
    auto pubNmeaAlignedOdom = node->create_publisher<nav_msgs::msg::Odometry>("/nmea_aligned_to_init", qos_pub);
    auto pubNmeaAlignedPath = node->create_publisher<nav_msgs::msg::Path>("/nmea_aligned_path", qos_pub);
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubNmeaLioErrorXy;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubNmea03mDiag;
    if (NMEA_ENABLE)
    {
        pubNmeaLioErrorXy = node->create_publisher<std_msgs::msg::Float64>("/ligo/nmea_lio_error_xy", qos_pub);
        pubNmea03mDiag = node->create_publisher<std_msgs::msg::Float64MultiArray>("/ligo/nmea_03m_diag", qos_pub);
    }
    auto pubIcpPairs = node->create_publisher<visualization_msgs::msg::Marker>("/icp_pairs_marker", qos_pub);
    auto pubInitPairsFromGpsMove = node->create_publisher<visualization_msgs::msg::Marker>(
        "/init_pairs_from_gps_move_marker", qos_pub);
    auto plane_pub = node->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", qos_pub);
#ifndef LIGO_WITHOUT_GNSS
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubEnuPosition;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGlobalNavSat;
    if (NMEA_ENABLE)
    {
        pubEnuPosition =
            node->create_publisher<geometry_msgs::msg::PointStamped>(enu_position_topic, qos_pub);
        RCLCPP_INFO(node->get_logger(), "ENU position: topic=%s frame_id=%s", enu_position_topic.c_str(),
                    enu_position_frame_id.c_str());
        pubGlobalNavSat =
            node->create_publisher<sensor_msgs::msg::NavSatFix>(global_position_topic, qos_pub);
        RCLCPP_INFO(node->get_logger(), "Global WGS84 position (NavSatFix): topic=%s (anchor auto-detected at NMEA init)",
                    global_position_topic.c_str());
    }
#else
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubEnuPosition;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pubGlobalNavSat;
#endif

    signal(SIGINT, SigHandle);
    tf2_ros::TransformBroadcaster tf_br(node);
    rclcpp::Rate loop_rate(500);
    bool status = rclcpp::ok();
    while (status)
    {
        if (flg_exit) break;
        rclcpp::spin_some(node);
        if(sync_packages(Measures, p_gnss->gnss_msg, p_nmea->nmea_msg)) 
        {
            ligo::indoor::updateIndoorLocalizationPlaceholder(state_out.pos, state_out.rot, time_current);
            // TODO(indoor): enable real graph insertion after localization module is implemented.
            // ligo::indoor::addIndoorFactorToGraphStubCommented();
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
                        RCLCPP_WARN(node->get_logger(), "indoor mode on");
                        // Keep ENU anchor in graph init, but restart local state at origin.
                        state_out.pos = Eigen::Vector3d::Zero();
                        state_out.rot = indoor_reloc_rot_enu;
                        state_out.vel = Eigen::Vector3d::Zero();
                        state_out.gravity << VEC_FROM_ARRAY(gravity);
                        state_out.acc = -state_out.rot.transpose() * state_out.gravity;
                        kf_output.x_ = state_out;
                        p_imu->imu_need_init_ = false;
                        p_imu->after_imu_init_ = true;
                        p_nmea->SetInitFromLocalization(indoor_reloc_pos_enu, indoor_reloc_rot_enu, kf_output.x_, indoor_reloc_pose_time);
                    }
                    kf_output.change_P(P_init_output);
                }
                is_first_gnss = true;
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                flg_reset_indoor_reloc = false;
                init_map = false;
                
                {
                    ivox_.reset(new IVoxType(ivox_options_));
                    ivox_last_.reset(new IVoxType(ivox_options_)); // = std::make_shared<IVoxType>(*ivox_);
                    traj_manager.reset(new curvefitter::TrajectoryManager<4>());
                    // while (!empty_voxels.empty())
                    // {
                        // std::unordered_set<Eigen::Matrix<int, 3, 1>, faster_lio::hash_vec<3>>().swap(empty_voxels[0]);
                        // empty_voxels.pop_front();
                    // }
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
                // if (GNSS_ENABLE)
                // {   
                //     // p_gnss->gnss_ready = true;
                //     // p_gnss->gtSAMgraphMade = true;
                //     set_gnss_offline_init(false);
                // }         
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
                    // p_gnss->Rot_gnss_init = rot_init;  
                    kf_output.x_.rot = rot_init;
                    // kf_output.x_.rot; //.normalize();
                    kf_output.x_.acc = - rot_init.transpose() * kf_output.x_.gravity;
                }
                else{
                continue;}
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
                    if (GNSS_ENABLE || NMEA_ENABLE) traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                }
                continue;
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
                if (time_seq.size() > 0) // || (!GNSS_ENABLE && !NMEA_ENABLE) )
                {
#ifndef LIGO_WITHOUT_GNSS
                    if (GNSS_ENABLE)  
                    {p_gnss->p_assign->process_feat_num += time_seq.size();
                    p_gnss->nolidar_cur = false;}
                    if (NMEA_ENABLE)  
                    {p_nmea->p_assign->process_feat_num += time_seq.size();
                    p_nmea->nolidar_cur = false;}
#endif
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
                        if (GNSS_ENABLE)
                        {
                            // std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
#ifndef LIGO_WITHOUT_GNSS
                            p_gnss->p_assign->process_feat_num = 0;
                            p_gnss->norm_vec_num = 0;
#endif
                            // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                            // p_gnss->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            // p_gnss->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr);
                        }
                        if (NMEA_ENABLE)
                        {
#ifndef LIGO_WITHOUT_GNSS
                            p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;
#endif
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
#ifndef LIGO_WITHOUT_GNSS
                            if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                            {   
                                gnss_cur = p_gnss->gnss_msg.front();
                                // printf("%f, %f, %f\n", time2sec(gnss_cur[0]->time), time_diff_gnss_local, time_predict_last_const);
                                while (time2sec(gnss_cur[0]->time) - time_diff_gnss_local < time_predict_last_const)
                                {
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                                if (p_gnss->gnss_msg.empty()) break;
                                while ((rclcpp::Time(imu_next.header.stamp).seconds() >= time2sec(gnss_cur[0]->time) - time_diff_gnss_local) && (time2sec(gnss_cur[0]->time) - time_diff_gnss_local >= time_predict_last_const))
                                {
                                    double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                                    double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;

                                    if (p_gnss->gnss_ready)
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        kf_output.predict(dt, Q_output, input_in, true, false);
                                        // p_gnss->pre_integration->push_back(dt, kf_output.x_.acc + kf_output.x_.ba, kf_output.x_.omg + kf_output.x_.bg); // acc_avr, angvel_avr); 
                                        // p_gnss->processIMUOutput(dt, kf_output.x_.acc, kf_output.x_.omg);
                                        time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        time_update_last = time_predict_last_const;
                                        p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                        p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                        // p_gnss->sqrt_lidar *= 0.002;
                                        update_gnss = p_gnss->Evaluate(kf_output.x_);
                                        if (!p_gnss->gnss_ready)
                                        {
                                            flg_reset = true;
                                            p_gnss->gnss_msg.pop();
                                            if(!p_gnss->gnss_msg.empty())
                                            {
                                                gnss_cur = p_gnss->gnss_msg.front();
                                            }
                                            break; // ?
                                        }

                                        if (update_gnss)
                                        {
                                            state_output out_state = kf_output.x_;
                                            kf_output.update_iterated_dyn_share_GNSS();
                                            Eigen::Vector3d pos_enu;
                                            if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                            // sensor_msgs::NavSatFix gnss_lla_msg;
                                            // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                            // gnss_lla_msg.header.frame_id = "camera_init";
                                            // gnss_lla_msg.latitude = pos_enu(0);
                                            // gnss_lla_msg.longitude = pos_enu(1);
                                            // gnss_lla_msg.altitude = pos_enu(2);
                                            // pub_gnss_lla.publish(gnss_lla_msg);
                                            if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                            {                                                
                                                curvefitter::PoseData pose_data;
                                                pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                                map_time = pose_data.timestamp;
                                                pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                                pose_data.position = kf_output.x_.pos;
                                                if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                                {
                                                    pose_time_vector.push_back(pose_data.timestamp);
                                                    pose_graph_key_pose.emplace_back(pose_data);
                                                }
                                                else
                                                // else if (map_time == pose_time_vector.back())
                                                {
                                                    pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                                    pose_graph_key_pose.back() = pose_data;
                                                }
                                                // curvefitter::Trajectory<4> traj(0.1);
                                                // std::shared_ptr<curvefitter::Trajectory<4> > Traj_ptr = std::make_shared<curvefitter::Trajectory<4> >(traj);  
                                                traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                                traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                                updatedmap.resize(points_num);
                                                updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                                ivox_last_->AddPoints(updatedmap);
                                                ivox_->grids_map_ = ivox_last_->grids_map_;
                                                // for (auto &t : ivox_last_->grids_map_)
                                                // {
                                                    // ivox_->grids_map_[t.first] = (t.second);
                                                // }
                                                // ivox_ = std::make_shared<IVoxType>(*ivox_last_);
                                            }
                                            else
                                            {
                                                ivox_last_->grids_map_ = ivox_->grids_map_;
                                            }
                                            // reset_cov_output(kf_output.P_);
                                            traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                        }
                                    }
                                    else
                                    {
                                        if (dt_cov > 0.0)
                                        {
                                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        }
                                        
                                        kf_output.predict(dt, Q_output, input_in, true, false);

                                        time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        time_update_last = time_predict_last_const;
                                        state_out = kf_output.x_;
                                        // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                        // state_out.rot.normalize();
                                        // state_out.pos = state_out.pos;
                                        // state_out.vel = state_out.vel;
                                        p_gnss->processGNSS(gnss_cur, state_out);
                                        if (p_gnss->gnss_ready)
                                        {
                                            // printf("time gnss ready: %f \n", time_predict_last_const);
                                            Eigen::Vector3d pos_enu;
                                            if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                            // sensor_msgs::NavSatFix gnss_lla_msg;
                                            // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                            // gnss_lla_msg.header.frame_id = "camera_init";
                                            // gnss_lla_msg.latitude = pos_enu(0);
                                            // gnss_lla_msg.longitude = pos_enu(1);
                                            // gnss_lla_msg.altitude = pos_enu(2);
                                            // pub_gnss_lla.publish(gnss_lla_msg);
                                        }
                                    }
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    else
                                    {
                                        break;
                                    }
                                }
                            }
#endif
#ifndef LIGO_WITHOUT_GNSS
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
                                        const bool cov_high_temp = nmeaCovarianceIsHigh(nmea_cur, kTempIndoorCovThreshold);
                                        const bool trigger_normal = indoor_flag && indoor_pose_valid && !indoor_reloc_applied_once && cov_high_cfg;
                                        const bool trigger_temp = kTempForceIndoorByNmeaCov && !indoor_reloc_applied_once && cov_high_temp;
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
                                                indoor_reloc_pos_enu = kf_output.x_.pos;
                                                indoor_reloc_rot_enu = kf_output.x_.rot;
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
#endif
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
#ifndef LIGO_WITHOUT_GNSS
                    if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                    {
                        gnss_cur = p_gnss->gnss_msg.front();
                        // printf("%f, %f, %f\n", time2sec(gnss_cur[0]->time), time_diff_gnss_local, time_predict_last_const);
                        while ( time2sec(gnss_cur[0]->time) - time_diff_gnss_local < time_predict_last_const)
                        {
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        if (p_gnss->gnss_msg.empty()) break;
                        while (time_current >= time2sec(gnss_cur[0]->time) - time_diff_gnss_local && time2sec(gnss_cur[0]->time) - time_diff_gnss_local >= time_predict_last_const)
                        {
                            double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                            double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;
                            // cout << "check gnss ready:" << p_gnss->gnss_ready << endl;
                            if (p_gnss->gnss_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);

                                // p_gnss->pre_integration->push_back(dt, kf_output.x_.acc + kf_output.x_.ba, kf_output.x_.omg + kf_output.x_.bg); // acc_avr, angvel_avr); 
                                // p_gnss->processIMUOutput(dt, kf_output.x_.acc, kf_output.x_.omg);

                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                time_update_last = time_predict_last_const;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                // p_gnss->sqrt_lidar *= 0.002;
                                update_gnss = p_gnss->Evaluate(kf_output.x_);
                                if (!p_gnss->gnss_ready)
                                {
                                    flg_reset = true;
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    break; // ?
                                }

                                if (update_gnss)
                                {
                                    state_output out_state = kf_output.x_;
                                    kf_output.update_iterated_dyn_share_GNSS();
                                    // reset_cov_output(kf_output.P_);
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                    if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                    {                                         
                                        curvefitter::PoseData pose_data;
                                        pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                        map_time = pose_data.timestamp;
                                        // pose_time_vector.push_back(pose_data.timestamp);
                                        pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                        pose_data.position = kf_output.x_.pos;
                                        if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                        {
                                            pose_time_vector.push_back(pose_data.timestamp);
                                            pose_graph_key_pose.emplace_back(pose_data);
                                        }
                                        else
                                        // else if (map_time == pose_time_vector.back())
                                        {
                                            pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                            pose_graph_key_pose.back() = pose_data;
                                        }
                                        // pose_graph_key_pose.emplace_back(pose_data);
                                        traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                        traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                        updatedmap.resize(points_num);
                                        updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                        ivox_last_->AddPoints(updatedmap);
                                        ivox_->grids_map_ = ivox_last_->grids_map_;
                                    }
                                    else
                                    {
                                        ivox_last_->grids_map_ = ivox_->grids_map_;
                                    }
                                    traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                time_update_last = time_predict_last_const;
                                state_out = kf_output.x_;
                                // state_out.rot = state_out.rot; //.normalized().toRotationMatrix();
                                // state_out.rot.normalize();
                                // state_out.pos = state_out.pos;
                                // state_out.vel = state_out.vel;
                                p_gnss->processGNSS(gnss_cur, state_out);
                                if (p_gnss->gnss_ready)
                                {
                                    // printf("time gnss ready: %f \n", time_predict_last_const);
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                }
                            }
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
#endif
#ifndef LIGO_WITHOUT_GNSS
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
                                const bool cov_high_temp = nmeaCovarianceIsHigh(nmea_cur, kTempIndoorCovThreshold);
                                const bool trigger_normal = indoor_flag && indoor_pose_valid && !indoor_reloc_applied_once && cov_high_cfg;
                                const bool trigger_temp = kTempForceIndoorByNmeaCov && !indoor_reloc_applied_once && cov_high_temp;
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
                                        indoor_reloc_pos_enu = kf_output.x_.pos;
                                        indoor_reloc_rot_enu = kf_output.x_.rot;
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
#endif
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
                        
                    if (publish_odometry_without_downsample)
                    {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped, tf_br);
                        try_publish_fused_enu_position(pubEnuPosition);
                        try_publish_fused_global_nav_sat(pubGlobalNavSat);
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
                    if (GNSS_ENABLE || NMEA_ENABLE)
                        lidarpoints.push_back(pimu_list[idx+j+1]); // (Eigen::Vector3d(point_body_j.x, point_body_j.y, point_body_j.z));
                    }
                    if (GNSS_ENABLE || NMEA_ENABLE)
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
#ifndef LIGO_WITHOUT_GNSS
                    if (GNSS_ENABLE)  p_gnss->nolidar_cur = true;
                    if (NMEA_ENABLE)  p_nmea->nolidar_cur = true;
#endif
                    if (!imu_deque.empty())
                    { 
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                    while (rclcpp::Time(imu_next.header.stamp).seconds() > time_current && ((rclcpp::Time(imu_next.header.stamp).seconds() < imu_first_time + lidar_time_inte && nolidar) || (rclcpp::Time(imu_next.header.stamp).seconds() < Measures.lidar_beg_time + lidar_time_inte && !nolidar)))
                    { // >= ?
                        if (is_first_frame)
                        {
#ifndef LIGO_WITHOUT_GNSS
                            if (!nolidar && GNSS_ENABLE) //std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            {p_gnss->p_assign->process_feat_num = 0;
                            p_gnss->norm_vec_num = 0;}
                            if (!nolidar && NMEA_ENABLE) //std::vector<Eigen::Vector3d>().swap(p_gnss->norm_vec_holder);
                            {p_nmea->p_assign->process_feat_num = 0;
                            p_nmea->norm_vec_num = 0;}
#endif

                            if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                                double front_gnss_ts = time2sec(gnss_cur[0]->time); // take time
                                time_current = front_gnss_ts - time_diff_gnss_local;
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
                            else if (!p_nmea->nmea_msg.empty() && NMEA_ENABLE)
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
#ifndef LIGO_WITHOUT_GNSS
                            if (GNSS_ENABLE)
                            {
                            p_gnss->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            p_gnss->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr); 
                            }
                            if (NMEA_ENABLE)
                            {
                            p_nmea->pre_integration->repropagate(kf_output.x_.ba, kf_output.x_.bg);
                            p_nmea->pre_integration->setacc0gyr0(acc_avr_norm, angvel_avr); 
                            }
#endif
                            {
                                is_first_frame = false;
                            }
                        }
                        time_current = rclcpp::Time(imu_next.header.stamp).seconds();

                        if (!is_first_frame)
                        {
#ifndef LIGO_WITHOUT_GNSS
                        if (!p_gnss->gnss_msg.empty() && GNSS_ENABLE)
                        {
                            gnss_cur = p_gnss->gnss_msg.front();
                            while (time2sec(gnss_cur[0]->time) - time_diff_gnss_local <= time_predict_last_const)
                            {
                                p_gnss->gnss_msg.pop();
                                if(!p_gnss->gnss_msg.empty())
                                {
                                    gnss_cur = p_gnss->gnss_msg.front();
                                }
                                else
                                {
                                    break;
                                }
                            }
                            if (p_gnss->gnss_msg.empty()) break;
                        while ((time_current > time2sec(gnss_cur[0]->time) - time_diff_gnss_local) && (time2sec(gnss_cur[0]->time) - time_diff_gnss_local > time_predict_last_const))
                        {
                            double dt = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_predict_last_const;
                            double dt_cov = time2sec(gnss_cur[0]->time) - time_diff_gnss_local - time_update_last;

                            if (p_gnss->gnss_ready)
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                p_gnss->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); //acc_avr_norm, angvel_avr); 
                                // change to state_const.omg and state_const.acc? 
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                if (!nolidar)
                                {
                                    p_gnss->sqrt_lidar = Eigen::LLT<Eigen::Matrix<double, 24, 24>>(kf_output.P_.inverse()).matrixL().transpose();
                                }
                                update_gnss = p_gnss->Evaluate(kf_output.x_); 
                                if (!p_gnss->gnss_ready)
                                {
                                    flg_reset = true;
                                    p_gnss->gnss_msg.pop();
                                    if(!p_gnss->gnss_msg.empty())
                                    {
                                        gnss_cur = p_gnss->gnss_msg.front();
                                    }
                                    break; // ?
                                }
                                if (update_gnss)
                                {
                                    if (!nolidar)
                                    {
                                        state_output out_state = kf_output.x_;
                                        kf_output.update_iterated_dyn_share_GNSS();
                                        // reset_cov_output(kf_output.P_);
                                        if ((out_state.pos - kf_output.x_.pos).norm() > 0.1 && pose_graph_key_pose.size() > 4)
                                        {                                    
                                            curvefitter::PoseData pose_data;
                                            pose_data.timestamp = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                            map_time = pose_data.timestamp;
                                            // pose_time_vector.push_back(pose_data.timestamp);
                                            pose_data.orientation = Sophus::SO3d(Eigen::Quaterniond(kf_output.x_.rot).normalized().toRotationMatrix());
                                            pose_data.position = kf_output.x_.pos;
                                            if (map_time > pose_graph_key_pose.back().timestamp) // + 1e-9)
                                            {
                                                pose_time_vector.push_back(pose_data.timestamp);
                                                pose_graph_key_pose.emplace_back(pose_data);
                                            }
                                            // else if (map_time == pose_time_vector.back())
                                            else
                                            {
                                                pose_data.timestamp = pose_graph_key_pose.back().timestamp;
                                                pose_graph_key_pose.back() = pose_data;
                                            }
                                            // pose_graph_key_pose.emplace_back(pose_data);
                                            // curvefitter::Trajectory<4> traj(0.1);
                                            // std::shared_ptr<curvefitter::Trajectory<4> > Traj_ptr = std::make_shared<curvefitter::Trajectory<4> >(traj);  
                                            traj_manager->SetTrajectory(std::make_shared<curvefitter::Trajectory<4> >(0.025));
                                            traj_manager->FitCurve(pose_graph_key_pose[0].orientation.unit_quaternion(), pose_graph_key_pose[0].position, pose_time_vector[0], pose_time_vector.back(), pose_graph_key_pose);
                                            updatedmap.resize(points_num);
                                            updatedmap = traj_manager->GetUpdatedMapPoints(pose_time_vector, LiDAR_points);
                                            ivox_last_->AddPoints(updatedmap);
                                            ivox_->grids_map_ = ivox_last_->grids_map_;
                                            // for (auto &t : ivox_last_->grids_map_)
                                            // {
                                                // (ivox_->grids_map_[t.first]) = (t.second);
                                            // }
                                            // ivox_ = std::make_shared<IVoxType>(*ivox_last_);
                                        }
                                        else
                                        {
                                            ivox_last_->grids_map_ = ivox_->grids_map_;
                                        }
                                        traj_manager->ResetTrajectory(pose_graph_key_pose, pose_time_vector, LiDAR_points, points_num);
                                    }
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    // kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                }
                                // kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = time2sec(gnss_cur[0]->time) - time_diff_gnss_local;
                                p_gnss->processGNSS(gnss_cur, kf_output.x_);
                                if (p_gnss->gnss_ready)
                                {
                                    Eigen::Vector3d pos_enu;
                                    if (!runtime_pos_log) cout_state_to_file(pos_enu);
                                    // printf("time gnss ready: %f \n", time_predict_last_const);
                                    // sensor_msgs::NavSatFix gnss_lla_msg;
                                    // gnss_lla_msg.header.stamp = ros::Time().fromSec(time_current);
                                    // gnss_lla_msg.header.frame_id = "camera_init";
                                    // gnss_lla_msg.latitude = pos_enu(0);
                                    // gnss_lla_msg.longitude = pos_enu(1);
                                    // gnss_lla_msg.altitude = pos_enu(2);
                                    // pub_gnss_lla.publish(gnss_lla_msg);
                                    if (nolidar)
                                    {
                                        // Eigen::Matrix3d R_enu_local_;
                                        // R_enu_local_ = Eigen::AngleAxisd(p_gnss->yaw_enu_local, Eigen::Vector3d::UnitZ());
                                        kf_output.x_.pos = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_gnss->frame_num-1)).segment<3>(0); // p_gnss->anc_ecef - p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end * p_gnss->Tex_imu_r;
                                        kf_output.x_.rot = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_gnss->frame_num-1)).matrix(); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end;
                                        // kf_output.x_.rot.normalize();
                                        kf_output.x_.vel = p_gnss->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_gnss->frame_num-1)).segment<3>(3); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.vel_end; // Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.ba = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.bg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.omg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.gravity = p_gnss->R_ecef_enu * kf_output.x_.gravity; // * R_enu_local_ 
                                        kf_output.x_.acc = kf_output.x_.rot.transpose() * (-kf_output.x_.gravity); // R_ecef_enu * state.vel_end;.conjugate().normalized()
                                        
                                        kf_output.P_ = MD(24,24)::Identity() * INIT_COV;
                                    }
                                }
                            }
                            p_gnss->gnss_msg.pop();
                            if(!p_gnss->gnss_msg.empty())
                            {
                                gnss_cur = p_gnss->gnss_msg.front();
                            }
                            else
                            {
                                break;
                            }
                        }
                        }
#endif
#ifndef LIGO_WITHOUT_GNSS
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
                                        R_enu_local = Eigen::AngleAxisd(p_nmea->yaw_enu_local, Eigen::Vector3d::UnitZ()); 
                                        kf_output.x_.pos = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(0); // p_gnss->anc_ecef - p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end * p_gnss->Tex_imu_r;
                                        kf_output.x_.rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_nmea->frame_num-1)).matrix(); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end;
                                        kf_output.x_.vel = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(3); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.vel_end; // Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
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
                                    break; // ?
                                update_nmea = p_nmea->Evaluate(kf_output.x_); 
                                const bool cov_high_cfg = nmeaCovarianceIsHigh(nmea_cur, p_nmea->p_assign->ppp_std_threshold);
                                const bool cov_high_temp = nmeaCovarianceIsHigh(nmea_cur, kTempIndoorCovThreshold);
                                const bool trigger_normal = indoor_flag && indoor_pose_valid && !indoor_reloc_applied_once && cov_high_cfg;
                                const bool trigger_temp = kTempForceIndoorByNmeaCov && !indoor_reloc_applied_once && cov_high_temp;
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
                                        indoor_reloc_pos_enu = kf_output.x_.pos;
                                        indoor_reloc_rot_enu = kf_output.x_.rot;
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
                                }
                            }
                            else
                            {
                                if (dt_cov > 0.0)
                                {
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    time_update_last = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3;
                                }
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = rclcpp::Time(nmea_cur->header.stamp).seconds() - time_diff_nmea_local - nmea_lat3;
                                p_nmea->processNMEA(nmea_cur, kf_output.x_);
                                if (p_nmea->nmea_ready)
                                {
                                    if (nolidar && p_nmea->frame_num > 0 &&
                                        p_nmea->p_assign->isamCurrentEstimate.exists(F(p_nmea->frame_num-1)) &&
                                        p_nmea->p_assign->isamCurrentEstimate.exists(R(p_nmea->frame_num-1)))
                                    {
                                        Eigen::Matrix3d R_enu_local;
                                        R_enu_local = Eigen::AngleAxisd(p_nmea->yaw_enu_local, Eigen::Vector3d::UnitZ()); 
                                        kf_output.x_.pos = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(0); // p_gnss->anc_ecef - p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end * p_gnss->Tex_imu_r;
                                        kf_output.x_.rot = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(p_nmea->frame_num-1)).matrix(); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.rot_end;
                                        kf_output.x_.vel = p_nmea->p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(p_nmea->frame_num-1)).segment<3>(3); // p_gnss->R_ecef_enu * R_enu_local_ * state_const.vel_end; // Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.ba = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.bg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.omg = Eigen::Vector3d::Zero(); // R_ecef_enu * state.vel_end;
                                        kf_output.x_.gravity = R_enu_local * kf_output.x_.gravity; // * R_enu_local_ 
                                        kf_output.x_.acc = kf_output.x_.rot.transpose() * (-kf_output.x_.gravity); // R_ecef_enu * state.vel_end;.conjugate().normalized()
                                        
                                        kf_output.P_ = MD(24,24)::Identity() * INIT_COV;
                                    }
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
#endif
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
#ifndef LIGO_WITHOUT_GNSS
                            if (GNSS_ENABLE)   p_gnss->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); // acc_avr_norm, angvel_avr); // 
                            if (NMEA_ENABLE)   p_nmea->pre_integration->push_back(dt, acc_avr_norm, angvel_avr); // acc_avr_norm, angvel_avr); // 
#endif
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
            if (!publish_odometry_without_downsample)
            {
                publish_odometry(pubOdomAftMapped, tf_br);
                try_publish_fused_enu_position(pubEnuPosition);
                try_publish_fused_global_nav_sat(pubGlobalNavSat);
            }

            /*** add the feature points to map ***/
            if(feats_down_size > 4)
            {
                MapIncremental();
            }

            t5 = omp_get_wtime();
            /******* Publish points *******/
            publish_nmea_aligned(pubNmeaAlignedOdom, pubNmeaAlignedPath);
            publish_icp_pairs_marker(pubIcpPairs, pubNmeaLioErrorXy, pubNmea03mDiag);
            publish_init_pairs_marker_from_gps_move(pubInitPairsFromGpsMove);
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
            
            /*** Debug variables Logging ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                s_plot[time_log_counter] = t5 - t0;
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter ++;
                if (!publish_odometry_without_downsample)
                {
                    {
                        {
#ifndef LIGO_WITHOUT_GNSS
                            Eigen::Matrix3d R_enu_local_;
                            Eigen::Vector3d pos_r = kf_output.x_.rot * p_gnss->Tex_imu_r + kf_output.x_.pos; // .normalized()
#else
                            Eigen::Vector3d pos_r = kf_output.x_.pos;
#endif
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
        }
        status = rclcpp::ok();
        loop_rate.sleep();
    }
    fout_out.close();
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    // if (GNSS_ENABLE || NMEA_ENABLE)
    {
#ifndef LIGO_WITHOUT_GNSS
        Eigen::Matrix3d enu_rot = ecef2rotation(first_pvt_used);
        for (int i = 0; i < time_frame.size(); i++)
        {
            // Eigen::Vector3d euler_ext = SO3ToEuler(local_rots[i]);
            if (NMEA_ENABLE)
            {
                Eigen::Vector3d ecef_r = enu_rot * est_poses[i] + first_pvt_used;
                Eigen::Vector3d pos_enu = ecef2enu(first_lla_anc, ecef_r - first_pvt_anc);
                fout_global << setw(20) << time_frame[i] - ppp_ecef[0][0] + 18.0 << " " << pos_enu.transpose() << endl; //"\n"; // p_gnss->pvt_time[0] + 18.0
            }
            else
            {
                fout_global << setw(20) << time_frame[i] - time_frame[0] << " " << est_poses[i].transpose() << endl; // << " " << local_poses[i].transpose() << " " << euler_ext.transpose() << endl; //"\n"; // p_gnss->pvt_time[0] + 18.0
                // fout_global << setw(20) << time_frame[i] - ppp_ecef[0][0] + 18.0 << " " << est_poses[i].transpose() << endl; //"\n"; // p_gnss->pvt_time[0] + 18.0
            }
            // printf("time: %f, pos: %f %f %f\n", ppp_ecef[0][0] + 18.0, est_poses[i](0), est_poses[i](1), est_poses[i](2));
            // Eigen::Vector3d euler_ext = SO3ToEuler(local_rots[i]);
        }
#else
        // When LIGO_WITHOUT_GNSS, fout_global is not opened in open_file(); skip
#endif
#ifndef LIGO_WITHOUT_GNSS
        fout_global.close();
#endif
    }

#ifndef LIGO_WITHOUT_GNSS
    for (int i = 0; i < p_gnss->pvt_time.size(); i++)
    {
        fout_rtk << setw(20) << p_gnss->pvt_time[i] - p_gnss->pvt_time[0] << " " << p_gnss->pvt_holder[i].transpose() << " " << p_gnss->diff_holder[i] << " " << p_gnss->float_holder[i] << endl; // "\n";
    }
    fout_rtk.close();
#endif
    #ifdef process_ppp
    for (int i = 0; i < ppp_ecef.size(); i++)
    {
        Eigen::Vector3d pos_enu = ecef2enu(p_gnss->first_lla_pvt, ppp_ecef[i].segment<3>(1) - p_gnss->first_xyz_ecef_pvt);
        fout_ppp << setw(20) << ppp_ecef[i][0] - ppp_ecef[0][0] << " " << pos_enu.transpose() << endl;
    }
    fout_ppp.close();
    #endif
    
    return 0;
}
