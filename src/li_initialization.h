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

#pragma once

#include <common_lib.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ligo/msg/local_sensor_external_trigger.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "Estimator.h"

extern bool data_accum_finished, data_accum_start, online_calib_finish, refine_print;
extern int frame_num_init;
extern double time_lag_IMU_wtr_lidar, move_start_time, online_calib_starts_time; //, mean_acc_norm = 9.81;

extern double timediff_imu_wrt_lidar;
extern bool timediff_set_flg;
extern V3D gravity_lio;
extern mutex mtx_buffer;
extern condition_variable sig_buffer;
extern int loop_count;
extern int scan_count_point;
extern int frame_ct, wait_num;
extern std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
extern std::deque<double>               time_buffer;
extern std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
extern std::queue<nav_msgs::msg::Odometry::SharedPtr> nmea_meas_buf;
extern std::mutex m_time;
extern bool lidar_pushed, imu_pushed;
extern double imu_first_time;
extern bool lose_lid;
extern sensor_msgs::msg::Imu imu_last, imu_next;
extern PointCloudXYZI::Ptr  ptr_con;
extern bool first_gps;
extern Eigen::Vector3d first_gps_lla;
extern Eigen::Vector3d first_gps_ecef;
// extern sensor_msgs::Imu::ConstPtr imu_last_ptr;

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in);
bool sync_packages(MeasureGroup &meas, queue<nav_msgs::msg::Odometry::SharedPtr> &nmea_msg);
void ligo_reset_nmea_stamp_diag_publisher();
/** After readParameters: if nmea.use_fixed_anchor, sets first_gps_* and nmea_global_anchor_* from yaml. */
void ligo_apply_fixed_nmea_anchor_if_configured();
void gpsHandler(const sensor_msgs::msg::NavSatFix::ConstSharedPtr & gpsMsg);