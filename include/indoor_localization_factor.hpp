/*
 * BSD 3-Clause License
 *
 * Indoor absolute pose factor in system-ENU.
 *
 * Variable layout matches NMEAFactor / NmeaLioGravRelFactor:
 *   - P (Rot3): R_enu_local (extrinsic local→ENU rotation)
 *   - E (Vector3): ENU anchor translation (ref_enu)
 *   - A (Vector6): body pos (0:2) + body vel (3:5) in local frame
 *   - R (Rot3): body rotation in local frame
 *
 * Input format is typed (same style as NmeaLioGravRelFactor / NMEAFactor):
 *   Tex_imu_r, anc_local, pos_meas (ENU), rot_meas (ENU, Eigen::Matrix3d),
 *   relative_sqrt_info, Rex_imu_r.
 *
 * Predicted ENU pose (identical chain to NMEAFactor):
 *   P_enu = R_enu_local * (R * Tex_imu_r + p_body - anc_local) + ref_enu
 *   R_enu = R_enu_local * R * Rex_imu_r
 *
 * Residual (6D): [(P_enu - pos_meas); Log(rot_meas^T * R_enu)] * sqrt_info
 *
 * Notes:
 *   - Velocity measurement is unused (no GICP velocity).
 *   - Noise model must be 6-dim.
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

namespace ligo {

class IndoorLocalizationFactor
    : public gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3> {
 public:
  IndoorLocalizationFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4,
                           const Eigen::Vector3d& Tex_imu_r_,
                           const Eigen::Vector3d& anc_local_,
                           const Eigen::Vector3d& pos_meas_,
                           const Eigen::Matrix3d& rot_meas_,
                           double relative_sqrt_info_,
                           const Eigen::Matrix3d& Rex_imu_r_,
                           const gtsam::SharedNoiseModel& model)
      : Tex_imu_r(Tex_imu_r_),
        anc_local(anc_local_),
        pos_meas(pos_meas_),
        rot_meas(rot_meas_),
        Rex_imu_r(Rex_imu_r_),
        relative_sqrt_info(relative_sqrt_info_),
        gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>(model, j1, j2,
                                                                                            j3, j4) {}

  virtual ~IndoorLocalizationFactor() {}

  gtsam::Vector evaluateError(
      const gtsam::Rot3& rot_ext, const gtsam::Vector3& pos_ext, const gtsam::Vector6& pos_vel,
      const gtsam::Rot3& rot,
      gtsam::OptionalMatrixType H1 = OptionalNone, gtsam::OptionalMatrixType H2 = OptionalNone,
      gtsam::OptionalMatrixType H3 = OptionalNone, gtsam::OptionalMatrixType H4 = OptionalNone) const override {
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
      d_pos << 0.0, local_pos[2], -local_pos[1],
               -local_pos[2], 0.0, local_pos[0],
               local_pos[1], -local_pos[0], 0.0;
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
};

}  // namespace ligo

#endif  // INDOOR_LOCALIZATION_FACTOR_H_
