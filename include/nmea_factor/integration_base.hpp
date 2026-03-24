/*
 * BSD 3-Clause License
 */

#pragma once

#include <ceres/ceres.h>
#include <so3_math.h>

using namespace Eigen;

class IntegrationBase {
 public:
  IntegrationBase() = delete;
  IntegrationBase(const Eigen::Vector3d _acc_0, const Eigen::Vector3d _gyr_0,
                  const Eigen::Vector3d _linearized_ba, const Eigen::Vector3d _linearized_bg)
      : acc_0{_acc_0},
        gyr_0{_gyr_0},
        linearized_ba{_linearized_ba},
        linearized_bg{_linearized_bg},
        jacobian{Eigen::Matrix<double, 15, 15>::Identity()},
        covariance{Eigen::Matrix<double, 15, 15>::Zero()},
        sum_dt{0.0},
        delta_p{Eigen::Vector3d::Zero()},
        delta_q{Eigen::Matrix3d::Identity()},
        delta_v{Eigen::Vector3d::Zero()} {
    noise = Eigen::Matrix<double, 18, 18>::Zero();
  }

  void push_back(double dt, const Eigen::Vector3d acc, const Eigen::Vector3d gyr) { propagate(dt, acc, gyr); }

  void repropagate(const Eigen::Vector3d &_linearized_ba, const Eigen::Vector3d &_linearized_bg) {
    sum_dt = 0.0;
    delta_p.setZero();
    delta_q.setIdentity();
    delta_v.setZero();
    linearized_ba = _linearized_ba;
    linearized_bg = _linearized_bg;
    jacobian.setIdentity();
    covariance.setZero();
  }

  void setacc0gyr0(Eigen::Vector3d &acc0_, Eigen::Vector3d &gyr0_) {
    acc_0 = acc0_;
    gyr_0 = gyr0_;
  }

  void setnoise() {
    noise = Eigen::Matrix<double, 18, 18>::Zero();
    noise.block<3, 3>(0, 0) = (acc_n * acc_n) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(3, 3) = (gyr_n * gyr_n) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(6, 6) = (acc_n * acc_n) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(9, 9) = (gyr_n * gyr_n) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(12, 12) = (acc_w * acc_w) * Eigen::Matrix3d::Identity();
    noise.block<3, 3>(15, 15) = (gyr_w * gyr_w) * Eigen::Matrix3d::Identity();
  }

  void midPointIntegration(double _dt, const Eigen::Vector3d &_acc_0, const Eigen::Vector3d &_gyr_0,
                           const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1,
                           const Eigen::Vector3d &delta_p, const Eigen::Matrix3d &delta_q,
                           const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba,
                           const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
                           Eigen::Matrix3d &result_delta_q, Eigen::Vector3d &result_delta_v,
                           Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg,
                           bool update_jacobian) {
    Vector3d un_acc_0 = delta_q * (_acc_0 - linearized_ba);
    Vector3d un_gyr = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
    result_delta_q = delta_q * Exp(un_gyr, _dt);
    Vector3d un_acc_1 = result_delta_q * (_acc_1 - linearized_ba);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * _dt + 0.5 * un_acc * _dt * _dt;
    result_delta_v = delta_v + un_acc * _dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian) {
      Vector3d w_x = 0.5 * (_gyr_0 + _gyr_1) - linearized_bg;
      Vector3d a_0_x = _acc_0 - linearized_ba;
      Vector3d a_1_x = _acc_1 - linearized_ba;
      Matrix3d R_w_x, R_a_0_x, R_a_1_x;

      R_w_x << 0, -w_x(2), w_x(1), w_x(2), 0, -w_x(0), -w_x(1), w_x(0), 0;
      R_a_0_x << 0, -a_0_x(2), a_0_x(1), a_0_x(2), 0, -a_0_x(0), -a_0_x(1), a_0_x(0), 0;
      R_a_1_x << 0, -a_1_x(2), a_1_x(1), a_1_x(2), 0, -a_1_x(0), -a_1_x(1), a_1_x(0), 0;

      MatrixXd F = MatrixXd::Zero(15, 15);
      F.block<3, 3>(0, 0) = Matrix3d::Identity();
      F.block<3, 3>(0, 3) =
          -0.25 * delta_q * R_a_0_x * _dt * _dt +
          -0.25 * result_delta_q * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt * _dt;
      F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * _dt;
      F.block<3, 3>(0, 9) = -0.25 * (delta_q + result_delta_q) * _dt * _dt;
      F.block<3, 3>(0, 12) = -0.25 * result_delta_q * R_a_1_x * _dt * _dt * -_dt;
      F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * _dt;
      F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3, 3) * _dt;
      F.block<3, 3>(6, 3) =
          -0.5 * delta_q * R_a_0_x * _dt +
          -0.5 * result_delta_q * R_a_1_x * (Matrix3d::Identity() - R_w_x * _dt) * _dt;
      F.block<3, 3>(6, 6) = Matrix3d::Identity();
      F.block<3, 3>(6, 9) = -0.5 * (delta_q + result_delta_q) * _dt;
      F.block<3, 3>(6, 12) = -0.5 * result_delta_q * R_a_1_x * _dt * -_dt;
      F.block<3, 3>(9, 9) = Matrix3d::Identity();
      F.block<3, 3>(12, 12) = Matrix3d::Identity();

      MatrixXd V = MatrixXd::Zero(15, 18);
      V.block<3, 3>(0, 0) = 0.25 * delta_q * _dt * _dt;
      V.block<3, 3>(0, 3) = 0.25 * -result_delta_q * R_a_1_x * _dt * _dt * 0.5 * _dt;
      V.block<3, 3>(0, 6) = 0.25 * result_delta_q * _dt * _dt;
      V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
      V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
      V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * _dt;
      V.block<3, 3>(6, 0) = 0.5 * delta_q * _dt;
      V.block<3, 3>(6, 3) = 0.5 * -result_delta_q * R_a_1_x * _dt * 0.5 * _dt;
      V.block<3, 3>(6, 6) = 0.5 * result_delta_q * _dt;
      V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
      V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * _dt;
      V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * _dt;

      jacobian = F * jacobian;
      covariance = F * covariance * F.transpose() + V * noise * V.transpose();
    }
  }

  void propagate(double _dt, const Eigen::Vector3d &_acc_1, const Eigen::Vector3d &_gyr_1) {
    dt = _dt;
    acc_1 = _acc_1;
    gyr_1 = _gyr_1;
    Vector3d result_delta_p;
    Matrix3d result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    midPointIntegration(_dt, acc_0, gyr_0, _acc_1, _gyr_1, delta_p, delta_q, delta_v, linearized_ba,
                        linearized_bg, result_delta_p, result_delta_q, result_delta_v,
                        result_linearized_ba, result_linearized_bg, 1);

    delta_p = result_delta_p;
    delta_q = result_delta_q;
    delta_v = result_delta_v;
    linearized_ba = result_linearized_ba;
    linearized_bg = result_linearized_bg;
    sum_dt += dt;
    acc_0 = acc_1;
    gyr_0 = gyr_1;
  }

  double dt;
  Eigen::Vector3d acc_0, gyr_0;
  Eigen::Vector3d acc_1, gyr_1;
  Eigen::Vector3d linearized_ba, linearized_bg;

  Eigen::Matrix<double, 15, 15> jacobian, covariance;
  Eigen::Matrix<double, 15, 15> step_jacobian;
  Eigen::Matrix<double, 15, 18> step_V;
  Eigen::Matrix<double, 18, 18> noise;

  double sum_dt, acc_n, gyr_n, acc_w, gyr_w;
  Eigen::Vector3d delta_p;
  Eigen::Matrix3d delta_q;
  Eigen::Vector3d delta_v;
};
