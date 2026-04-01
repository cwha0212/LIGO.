/*
 * Stub for building LIGO without gnss_comm (LiDAR+IMU only).
 * Used when LIGO_WITH_NMEA is not defined.
 */
#pragma once

#include "common_lib.h"
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <memory>
#include <vector>

class NMEAProcess {
 public:
  std::queue<nav_msgs::msg::Odometry::SharedPtr> nmea_msg;
  int norm_vec_num = 0;
  state_output state_const_;
  Eigen::Vector3d Tex_imu_r{Eigen::Vector3d::Zero()};
};
