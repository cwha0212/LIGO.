/*
 * Stub for building LIGO without gnss_comm (LiDAR+IMU only).
 * Define LIGO_WITHOUT_GNSS when gnss_comm is not available.
 */
#pragma once

#include "common_lib.h"
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <memory>
#include <vector>

namespace gnss_comm {
struct gtime_t {
  int week = 0;
  double tow = 0;
};
inline double time2sec(const gtime_t &t) {
  return t.week * 7 * 24 * 3600 + t.tow;
}
struct Obs {
  gtime_t time;
};
using ObsPtr = std::shared_ptr<Obs>;
}
using gnss_comm::ObsPtr;
using gnss_comm::time2sec;

class GNSSProcess {
 public:
  std::queue<std::vector<gnss_comm::ObsPtr>> gnss_msg;
  int norm_vec_num = 0;
  state_output state_const_;
  Eigen::Vector3d Tex_imu_r{Eigen::Vector3d::Zero()};
};

class NMEAProcess {
 public:
  std::queue<nav_msgs::msg::Odometry::SharedPtr> nmea_msg;
  int norm_vec_num = 0;
  state_output state_const_;
};
