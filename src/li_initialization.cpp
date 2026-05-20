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

#include "li_initialization.h"
#include "parameters.h"
#include <rclcpp/rclcpp.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <fstream>
#include "Indoor_Processing.h"

static rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr g_nmea_stamp_diag_pub;

void ligo_try_create_nmea_stamp_diag_publisher(std::shared_ptr<rclcpp::Node> node)
{
    if (!node)
        return;
    if (!NMEA_ENABLE || !nmea_publish_stamp_diag || nmea_input_type != std::string("navsatfix"))
        return;
    g_nmea_stamp_diag_pub =
        node->create_publisher<std_msgs::msg::Float64MultiArray>("/ligo/nmea_stamp_diag", rclcpp::QoS(100));
    RCLCPP_INFO(node->get_logger(),
                "NMEA stamp diag: publishing on /ligo/nmea_stamp_diag "
                "(raw, stamp_in_buf, last_lidar, offset_sec, offset_inited)");
}

void ligo_reset_nmea_stamp_diag_publisher()
{
    g_nmea_stamp_diag_pub.reset();
}

bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; //, mean_acc_norm = 9.81;
double imu_first_time = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
bool timediff_set_flg = false;
V3D gravity_lio = V3D::Zero();
mutex mtx_buffer;
sensor_msgs::msg::Imu imu_last, imu_next;
// sensor_msgs::Imu::ConstPtr imu_last_ptr;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());

bool first_gps = false;
Eigen::Vector3d first_gps_lla;
Eigen::Vector3d first_gps_ecef;

void ligo_apply_fixed_nmea_anchor_if_configured()
{
    if (!NMEA_ENABLE || !nmea_use_fixed_anchor)
        return;

    Eigen::Vector3d geo = Eigen::Vector3d::Zero();
    const bool have_lla = (nmea_fixed_anchor_lla_deg.size() == 3);
    if (have_lla)
    {
        geo << nmea_fixed_anchor_lla_deg[0], nmea_fixed_anchor_lla_deg[1], nmea_fixed_anchor_lla_deg[2];
    }
    else if (ppp_anc.size() == 3)
    {
        const Eigen::Vector3d ecef(ppp_anc[0], ppp_anc[1], ppp_anc[2]);
        geo = gnss_comm::ecef2geo(ecef);
        RCLCPP_WARN(rclcpp::get_logger("ligo"),
                    "[nmea] fixed anchor from nmea.ppp_anc (ECEF→LLA). If this is not YOUR site, NMEA–LIO ICP can fail and "
                    "RViz (Fixed Frame=map) stays empty until icp_tf_ready — prefer nmea.fixed_anchor_lla_deg from map *_grid2d.yaml");
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("ligo"),
                    "[nmea] use_fixed_anchor true: set nmea.fixed_anchor_lla_deg [lat,lon,alt] or nmea.ppp_anc ECEF [x,y,z] "
                    "(3 values each)");
        return;
    }

    first_gps_lla = geo;
    first_gps_ecef = gnss_comm::geo2ecef(geo);
    nmea_global_anchor_lla = first_gps_lla;
    nmea_global_anchor_ready = true;
    first_gps = true;
    ligo::indoor::setSystemEcefAnchor(gnss_comm::geo2ecef(first_gps_lla), gnss_comm::geo2rotation(first_gps_lla));
    RCLCPP_INFO(rclcpp::get_logger("ligo"),
                "[nmea] fixed ENU anchor: LLA=(%.9f, %.9f, %.3f) deg,m  ECEF=(%.3f, %.3f, %.3f) m — NavSatFix will not re-seat anchor",
                first_gps_lla.x(), first_gps_lla.y(), first_gps_lla.z(), first_gps_ecef.x(), first_gps_ecef.y(),
                first_gps_ecef.z());
}

condition_variable sig_buffer;
int loop_count = 0;
int scan_count_point = 0;
int frame_ct = 0, wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false;
std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
std::deque<double>               time_buffer;
std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
std::queue<nav_msgs::msg::Odometry::SharedPtr> nmea_meas_buf;

void nmea_meas_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &meas_msg)
{
    nav_msgs::msg::Odometry::SharedPtr nmea_meas = std::make_shared<nav_msgs::msg::Odometry>(*meas_msg);
    last_nmea_time = rclcpp::Time(nmea_meas->header.stamp).seconds();
    nmea_meas_buf.push(std::move(nmea_meas)); // ?
}

void gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gpsMsg)
{
    if (gpsMsg->status.status != 0)
    {
        return;
    }

    // NMEA 전역 입력: 고도는 사용하지 않음(0 m 가정). LIO 추정 z는 별도 경로에서 유지.
    nmea_last_raw_lla << gpsMsg->latitude, gpsMsg->longitude, 0.0;
    nmea_last_raw_lla_valid = true;

    // Align NavSatFix timestamp base to LiDAR/IMU ROS(bag) time if needed.
    // Some datasets publish GNSS with Unix epoch stamps while LiDAR uses bag time.
    static bool nmea_stamp_offset_inited = false;
    static double nmea_stamp_offset_sec = 0.0;
    const double gps_ts_raw = rclcpp::Time(gpsMsg->header.stamp).seconds();
    bool offset_just_inited = false;
    if (!nmea_stamp_offset_inited && last_timestamp_lidar > 1.0)
    {
        const double diff = gps_ts_raw - last_timestamp_lidar;
        if (std::fabs(diff) > 1000.0)  // clearly different time bases
        {
            nmea_stamp_offset_sec = diff;
            nmea_stamp_offset_inited = true;
            offset_just_inited = true;
        }
    }

    Eigen::Vector3d trans_local_;
    if (!first_gps) {
        first_gps = true;
        Eigen::Vector3d geo;
        geo << gpsMsg->latitude, gpsMsg->longitude, 0.0;
        first_gps_lla = geo;
        first_gps_ecef = gnss_comm::geo2ecef(geo);
        nmea_global_anchor_lla = first_gps_lla;
        nmea_global_anchor_ready = true;
        ligo::indoor::setSystemEcefAnchor(
            gnss_comm::geo2ecef(first_gps_lla),
            gnss_comm::geo2rotation(first_gps_lla));
    }
    Eigen::Vector3d cur_ecef =
        gnss_comm::geo2ecef(Eigen::Vector3d(gpsMsg->latitude, gpsMsg->longitude, 0.0));
    trans_local_ = gnss_comm::ecef2enu(first_gps_lla, cur_ecef - first_gps_ecef);
    trans_local_.z() = 0.0;

    nav_msgs::msg::Odometry gps_odom;
    if (nmea_stamp_offset_inited)
    {
        const double gps_ts_aligned = gps_ts_raw - nmea_stamp_offset_sec;
        const int32_t sec = static_cast<int32_t>(std::floor(gps_ts_aligned));
        const uint32_t nanosec = static_cast<uint32_t>(std::round((gps_ts_aligned - std::floor(gps_ts_aligned)) * 1e9));
        gps_odom.header.stamp.sec = sec;
        gps_odom.header.stamp.nanosec = nanosec;
    }
    else
    {
        gps_odom.header.stamp = gpsMsg->header.stamp;
    }
    gps_odom.header.frame_id = "map";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    if (gpsMsg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
    {
        gps_odom.pose.covariance[0] = gpsMsg->position_covariance[0];  // xx
        gps_odom.pose.covariance[7] = gpsMsg->position_covariance[4];  // yy
        gps_odom.pose.covariance[14] = gpsMsg->position_covariance[8]; // zz
    }
    // gps_odom->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    // pubGpsOdom.publish(gps_odom);
    // gpsQueue.push_back(gps_odom);
    if (offset_just_inited)
    {
        // Clear any previously buffered NMEA messages with incompatible time base
        // (e.g., first NavSatFix arriving before LiDAR time is available).
        while (!nmea_meas_buf.empty()) nmea_meas_buf.pop();
    }
    nmea_meas_buf.push(std::make_shared<nav_msgs::msg::Odometry>(gps_odom));
    const double stamp_in_buf_sec = rclcpp::Time(gps_odom.header.stamp).seconds();
    // /ligo/nmea_stamp_diag: LIGO 내부에서 nmea_meas_buf에 넣는 stamp가 LiDAR 시간축과 맞는지 외부에서 검증용
    if (!rclcpp::ok())
    {
        return;
    }
    if (g_nmea_stamp_diag_pub)
    {
        std_msgs::msg::Float64MultiArray diag;
        diag.layout.dim.resize(1);
        diag.layout.dim[0].label = "raw,stamp_in_buf,last_lidar,offset_sec,offset_inited";
        diag.layout.dim[0].size = 5;
        diag.layout.dim[0].stride = 5;
        diag.data.resize(5);
        diag.data[0] = gps_ts_raw;
        diag.data[1] = stamp_in_buf_sec;
        diag.data[2] = last_timestamp_lidar;
        diag.data[3] = nmea_stamp_offset_sec;
        diag.data[4] = nmea_stamp_offset_inited ? 1.0 : 0.0;
        g_nmea_stamp_diag_pub->publish(diag);
    }
    // stamp 보정 확인: 50번마다 raw/aligned/lidar 출력
    {
      static int n = 0;
      if (++n <= 3 || n % 50 == 0)
      {
        RCLCPP_INFO(
            rclcpp::get_logger("ligo"),
            "[nmea/stamp] raw=%.3f aligned=%.3f lidar=%.3f offset_ok=%d diff=%.6f",
            gps_ts_raw, stamp_in_buf_sec, last_timestamp_lidar, nmea_stamp_offset_inited ? 1 : 0,
            last_timestamp_lidar > 0 ? stamp_in_buf_sec - last_timestamp_lidar : 0.0);
      }
    }
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    scan_count ++;
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "lidar loop back, clear buffer");
        return;
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
    p_pre->process(msg, ptr);
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            // cout << "ptr div num:" << ptr_div->size() << endl;
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    { 
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
        }
    }
    }
    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    scan_count ++;
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "lidar loop back, clear buffer");

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
        // lidar_buffer.shrink_to_fit();
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();    

    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
    p_pre->process(msg, ptr); 
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
        }
    }
    }
    // s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in) 
{
    sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

    double t = rclcpp::Time(msg->header.stamp).seconds() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar;
    msg->header.stamp.sec = static_cast<int32_t>(std::floor(t));
    msg->header.stamp.nanosec = static_cast<uint32_t>(std::round((t - std::floor(t)) * 1e9));

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    if (timestamp < last_timestamp_imu)
    {
        RCLCPP_ERROR(rclcpp::get_logger("ligo"), "imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        // cout << "check time:" << timestamp << ";" << last_timestamp_imu << endl;
        // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);
        
        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }

    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas, queue<nav_msgs::msg::Odometry::SharedPtr> &nmea_msg)
{
    static uint64_t sync_call_count = 0;
    ++sync_call_count;

    if (sync_call_count % 500 == 0)
    {
        RCLCPP_DEBUG(
            rclcpp::get_logger("ligo"),
            "sync diag: lidar_buf=%zu imu_buf=%zu nmea_buf=%zu time_diff_valid=%d dt_nmea=%.6f last_imu=%.6f",
            lidar_buffer.size(), imu_deque.size(), nmea_meas_buf.size(),
            time_diff_valid ? 1 : 0, time_diff_nmea_local, last_timestamp_imu);
    }

    if (!imu_en)
    {
        if (!lidar_buffer.empty())
        {
            if (!lidar_pushed)
            {
                meas.lidar = lidar_buffer.front();
                meas.lidar_beg_time = time_buffer.front();
                lose_lid = false;
                if(meas.lidar->points.size() < 1) 
                {
                    cout << "lose lidar" << std::endl;
                    // return false;
                    lose_lid = true;
                }
                else
                {
                    double end_time = meas.lidar->points.back().curvature;
                    for (auto pt: meas.lidar->points)
                    {
                        if (pt.curvature > end_time)
                        {
                            end_time = pt.curvature;
                        }
                    }
                    lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                    meas.lidar_last_time = lidar_end_time;
                }
                lidar_pushed = true;
            }
            
            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty()) // or can wait for a short time?
                {
                    double front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds();
                    while (front_nmea_ts < meas.lidar_beg_time + time_diff_nmea_local) // 0.05
                    {
                        RCLCPP_WARN(rclcpp::get_logger("ligo"), "throw nmea, only should happen at the beginning 542");
                        nmea_meas_buf.pop();
                        if (nmea_meas_buf.empty()) break;
                        front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds();
                    }
                    if (!nmea_meas_buf.empty())
                    {
                    while ((!lose_lid && (front_nmea_ts <= lidar_end_time + time_diff_nmea_local)) || (lose_lid && (front_nmea_ts <= meas.lidar_beg_time + time_diff_nmea_local + lidar_time_inte) ))
                    {
                        nmea_msg.push(nmea_meas_buf.front());
                        nmea_meas_buf.pop();
                        if (nmea_meas_buf.empty()) break;
                        front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds();
                    }
                    if (!nmea_msg.empty())
                    {
                        time_buffer.pop_front();
                        lidar_buffer.pop_front();
                        lidar_pushed = false;
                        return true;
                    }
                    }
                }
            }
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            lidar_pushed = false;
            if (!lose_lid)
            {
                return true;
            }
            else
            {
                return false;
            }
        }        
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        lose_lid = false;
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if(meas.lidar->points.size() < 1) 
        {
            cout << "lose lidar" << endl;
            lose_lid = true;
            // lidar_buffer.pop_front();
            // time_buffer.pop_front();
            // return false;
        }
        else
        {
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            // cout << "check time lidar:" << end_time << endl;
            meas.lidar_last_time = lidar_end_time;
        }
        lidar_pushed = true;
    }

    if (!lose_lid && (last_timestamp_imu < lidar_end_time + 2))
    {
        // lidar_pushed = false;
        return false;
    }
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte + 2)
    {
        // lidar_pushed = false;
        return false;
    }

    if (!lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            const size_t imu_buf_before = imu_deque.size();
            double imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
            imu_next = *(imu_deque.front());
            meas.imu.shrink_to_fit();
            while (imu_time < lidar_end_time)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds(); // can be changed
                imu_next = *(imu_deque.front());
            }
            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty())
                {
                    const size_t nmea_buf_before_trim = nmea_meas_buf.size();
                    double front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds(); 
                    while (front_nmea_ts < lidar_end_time + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds(); // take time
                    }
                    const size_t nmea_trimmed = nmea_buf_before_trim - nmea_meas_buf.size();
                    if (nmea_trimmed > 0)
                    {
                        RCLCPP_DEBUG(
                            rclcpp::get_logger("ligo"),
                            "[sync/lidar] NMEA pre-trim: trimmed=%zu remain=%zu end=%.6f dt=%.6f",
                            nmea_trimmed, nmea_meas_buf.size(), lidar_end_time, time_diff_nmea_local);
                    }
                }
            }
            RCLCPP_DEBUG(
                rclcpp::get_logger("ligo"),
                "[sync/lidar] IMU cut: beg=%.6f end=%.6f pushed=%zu imu_buf %zu->%zu",
                meas.lidar_beg_time, lidar_end_time, meas.imu.size(), imu_buf_before, imu_deque.size());
        }
        imu_pushed = true;
    }
    
    if (lose_lid && !imu_pushed)
    { 
        /*** push imu data, and pop from imu buffer ***/
        if (p_imu->imu_need_init_)
        {
            const size_t imu_buf_before = imu_deque.size();
            double imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
            meas.imu.shrink_to_fit();

            imu_next = *(imu_deque.front());
            while (imu_time < meas.lidar_beg_time + lidar_time_inte)
            {
                meas.imu.emplace_back(imu_deque.front());
                imu_last = imu_next;
                imu_deque.pop_front();
                if(imu_deque.empty()) break;
                imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds(); // can be changed
                imu_next = *(imu_deque.front());
            }

            if (NMEA_ENABLE)
            {
                if (!nmea_meas_buf.empty())
                {
                    const size_t nmea_buf_before_trim = nmea_meas_buf.size();
                    double front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds(); // take time
                    while (front_nmea_ts < meas.lidar_beg_time + lidar_time_inte + time_diff_nmea_local)
                    {
                        nmea_meas_buf.pop();
                        if(nmea_meas_buf.empty()) break;
                        front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds(); // take time
                    }
                    const size_t nmea_trimmed = nmea_buf_before_trim - nmea_meas_buf.size();
                    if (nmea_trimmed > 0)
                    {
                        RCLCPP_DEBUG(
                            rclcpp::get_logger("ligo"),
                            "[sync/lidar-lose] NMEA pre-trim: trimmed=%zu remain=%zu end=%.6f dt=%.6f",
                            nmea_trimmed, nmea_meas_buf.size(), meas.lidar_beg_time + lidar_time_inte, time_diff_nmea_local);
                    }
                }
            }
            RCLCPP_DEBUG(
                rclcpp::get_logger("ligo"),
                "[sync/lidar-lose] IMU cut: beg=%.6f end=%.6f pushed=%zu imu_buf %zu->%zu",
                meas.lidar_beg_time, meas.lidar_beg_time + lidar_time_inte, meas.imu.size(), imu_buf_before, imu_deque.size());
        }

        imu_pushed = true;
    }

    if (NMEA_ENABLE)
    {
        if (!nmea_meas_buf.empty()) // or can wait for a short time?
        {
            const size_t nmea_buf_before_fill = nmea_meas_buf.size();
            const size_t nmea_msg_before_fill = nmea_msg.size();
            double front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds(); // take time
            while ((!lose_lid && (front_nmea_ts < lidar_end_time + time_diff_nmea_local)) || (lose_lid && (front_nmea_ts < meas.lidar_beg_time + time_diff_nmea_local + lidar_time_inte) )) 
            {
                nmea_msg.push(nmea_meas_buf.front());
                nmea_meas_buf.pop();
                if (nmea_meas_buf.empty()) break;
                front_nmea_ts = rclcpp::Time(nmea_meas_buf.front()->header.stamp).seconds();
            }
            const size_t nmea_pushed = nmea_msg.size() - nmea_msg_before_fill;
            const size_t nmea_popped = nmea_buf_before_fill - nmea_meas_buf.size();
            if (nmea_pushed > 0 || nmea_popped > 0)
            {
                RCLCPP_DEBUG(
                    rclcpp::get_logger("ligo"),
                    "[sync/lidar] NMEA fill: pushed=%zu popped=%zu remain_buf=%zu lose_lid=%d",
                    nmea_pushed, nmea_popped, nmea_meas_buf.size(), lose_lid ? 1 : 0);
            }
            if (!nmea_msg.empty())
            {
                const size_t lidar_buf_before_pop = lidar_buffer.size();
                const size_t time_buf_before_pop = time_buffer.size();
                time_buffer.pop_front();
                lidar_buffer.pop_front();
                RCLCPP_DEBUG(
                    rclcpp::get_logger("ligo"),
                    "[sync/lidar] frame consume(with nmea): lidar_buf %zu->%zu time_buf %zu->%zu",
                    lidar_buf_before_pop, lidar_buffer.size(), time_buf_before_pop, time_buffer.size());
                lidar_pushed = false;
                imu_pushed = false;
                return true;
            }
        }
    }

    const size_t lidar_buf_before_pop = lidar_buffer.size();
    const size_t time_buf_before_pop = time_buffer.size();
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    RCLCPP_DEBUG(
        rclcpp::get_logger("ligo"),
        "[sync/lidar] frame consume(final): lidar_buf %zu->%zu time_buf %zu->%zu nmea_buf=%zu",
        lidar_buf_before_pop, lidar_buffer.size(), time_buf_before_pop, time_buffer.size(),
        nmea_meas_buf.size());
    lidar_pushed = false;
    imu_pushed = false;
    return true;
}

