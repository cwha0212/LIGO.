/*
 * BSD 3-Clause License
 *
 * Indoor absolute pose factor in ENU — same variable layout and geometry chain as NMEAFactor,
 * but residual is 6D (position + orientation) only; no velocity measurement.
 */

#ifndef INDOOR_LOCALIZATION_FACTOR_H_
#define INDOOR_LOCALIZATION_FACTOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gnss_comm/gnss_ros.hpp>

using namespace gnss_comm;

namespace ligo {

/**
 * @brief 6-DoF absolute pose in ENU (indoor localization / ICP global pose), aligned with NMEAFactor geometry.
 *
 * Keys (same as NMEAFactor): `P` (ext Rot3), `E` (anchor Vector3), `A` (pos+vel Vector6), `R` (body Rot3).
 *
 * `values_[17]` layout (same packing as NMEAFactor):
 *   [0:2] Tex_imu_r, [3:5] anc_local, [6:8] position measurement in ENU (m),
 *   [9:11] unused by this factor (may be 0), [12:15] quaternion (w,x,y,z) of absolute orientation in ENU,
 *   [16] relative_sqrt_info (scalar weight on residuals).
 *
 * Predicted ENU pose matches NMEAFactor:
 *   P_enu = R_enu_local * (R * Tex + p_body - anc) + ref_enu
 *   R_enu = R_enu_local * R * Rex_imu_r
 *
 * Residual (6): [(P_enu - pos_meas); Log(R_meas^T R_enu)] * sqrt_info
 *
 * Noise model dimension must be 6 (e.g. tx,ty,tz, rx,ry,rz variances).
 */
class IndoorLocalizationFactor
    : public gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3> {
 public:
  /** @param hat_omg_T_ Same slot as NMEAFactor (unused here; no velocity residual). */
  IndoorLocalizationFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                           bool invalid_lidar_, double values_[17],
                           const Eigen::Vector3d& hat_omg_T_, Eigen::Matrix3d Rex_imu_r_,
                           const gtsam::SharedNoiseModel& model)
      : invalid_lidar(invalid_lidar_),
        Rex_imu_r(Rex_imu_r_),
        gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>(model, j1, j2,
                                                                                            j3, j4) {
    (void)hat_omg_T_;
    Tex_imu_r << values_[0], values_[1], values_[2];
    anc_local << values_[3], values_[4], values_[5];
    pos_meas << values_[6], values_[7], values_[8];
    rot_meas = Eigen::Quaterniond(values_[12], values_[13], values_[14], values_[15]).normalized().toRotationMatrix();
    relative_sqrt_info = values_[16];
  }

  virtual ~IndoorLocalizationFactor() {}

  gtsam::Vector evaluateError(
      const gtsam::Rot3& rot_ext, const gtsam::Vector3& pos_ext, const gtsam::Vector6& pos_vel,
      const gtsam::Rot3& rot,
      boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none) const override {
    const Eigen::Vector3d ref_enu = pos_ext;
    const Eigen::Vector3d local_pos = rot * Tex_imu_r + pos_vel.segment<3>(0) - anc_local;
    const Eigen::Matrix3d R_enu_local = rot_ext.matrix();
    const Eigen::Vector3d P_enu = R_enu_local * local_pos + ref_enu;
    const Eigen::Matrix3d R_enu = R_enu_local * rot.matrix() * Rex_imu_r;

    const Eigen::Matrix3d res_R = rot_meas.transpose() * R_enu;
    Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));

    if (H1) {
      (*H1) = gtsam::Matrix::Zero(6, 3);
      Eigen::Matrix3d d_pos;
      d_pos << 0.0, local_pos[2], -local_pos[1], -local_pos[2], 0.0, local_pos[0], local_pos[1], -local_pos[0],
          0.0;
      (*H1).block<3, 3>(0, 0) = R_enu_local * d_pos * relative_sqrt_info;
      (*H1).block<3, 3>(3, 0) =
          Jacob_right_inv<double>(res_r) * Rex_imu_r.transpose() * rot.matrix().transpose() * relative_sqrt_info;
    }
    if (H2) {
      (*H2) = gtsam::Matrix::Zero(6, 3);
      (*H2).block<3, 3>(0, 0) = gtsam::Matrix::Identity(3, 3) * relative_sqrt_info;
    }
    if (H3) {
      (*H3) = gtsam::Matrix::Zero(6, 6);
      (*H3).block<3, 3>(0, 0) = R_enu_local * relative_sqrt_info;
    }
    if (H4) {
      (*H4) = gtsam::Matrix::Zero(6, 3);
      (*H4).block<3, 3>(3, 0) = Jacob_right_inv<double>(res_r) * Rex_imu_r.transpose() * relative_sqrt_info;
    }

    gtsam::Vector residual(6);
    residual.segment<3>(0) = (P_enu - pos_meas) * relative_sqrt_info;
    residual.segment<3>(3) = res_r * relative_sqrt_info;
    return residual;
  }

 private:
  Eigen::Vector3d Tex_imu_r, anc_local, pos_meas;
  Eigen::Matrix3d rot_meas, Rex_imu_r;
  double relative_sqrt_info;
  bool invalid_lidar;
};

}  // namespace ligo

#endif  // INDOOR_LOCALIZATION_FACTOR_H_
