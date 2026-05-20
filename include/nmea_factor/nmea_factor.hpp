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

#ifndef NMEA_FACTOR_H_
#define NMEA_FACTOR_H_

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>

namespace ligo {

/**
 * NMEA absolute measurement factor in system-ENU.
 *
 * Input format is typed (same style as NmeaLioGravRelFactor / IndoorLocalizationFactor):
 *   Tex_imu_r, anc_local, pos_meas (ENU), vel_meas (ENU), rot_meas (ENU, Eigen::Matrix3d),
 *   relative_sqrt_info (YAML: nmea.factor_sqrt_info_scale, stored in NMEAProcess::nmea_weight), hat_omg_T, Rex_imu_r.
 *
 * Predicted ENU pose (identical chain to IndoorLocalizationFactor):
 *   P_enu = R_enu_local * (R * Tex_imu_r + p_body - anc_local) + ref_enu
 *   V_enu = R^T * v_body + hat_omg_T
 *   R_enu = R_enu_local * R * Rex_imu_r
 *
 * Residual:
 *   - position_only_=true  (e.g. NavSatFix bridge):  r = (P_enu - pos_meas) * sqrt_info ∈ R^3
 *   - position_only_=false (Odometry input)         :  r = [(P_enu-pos_meas); (V_enu-vel_meas); Log(rot_meas^T R_enu)] * sqrt_info ∈ R^9
 *
 * Notes:
 *   - In NavSatFix (default) path the input rot_meas / vel_meas are unused because the
 *     NavSatFix→Odometry bridge does not supply twist/orientation.
 */
class NMEAFactor : public gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>
{
    public:
        NMEAFactor(gtsam::Key j1, gtsam::Key j2, gtsam::Key j3, gtsam::Key j4, bool invalid_lidar_,
        const Eigen::Vector3d& Tex_imu_r_,
        const Eigen::Vector3d& anc_local_,
        const Eigen::Vector3d& pos_meas_,
        const Eigen::Vector3d& vel_meas_,
        const Eigen::Matrix3d& rot_meas_,
        double relative_sqrt_info_,
        const Eigen::Vector3d& hat_omg_T_,
        const Eigen::Matrix3d& Rex_imu_r_,
        const gtsam::SharedNoiseModel& model,
        bool position_only = false) :
        Tex_imu_r(Tex_imu_r_), anc_local(anc_local_), pos_meas(pos_meas_), vel_meas(vel_meas_),
        hat_omg_T(hat_omg_T_), rot_meas(rot_meas_), Rex_imu_r(Rex_imu_r_),
        relative_sqrt_info(relative_sqrt_info_), invalid_lidar(invalid_lidar_), position_only_(position_only),
        gtsam::NoiseModelFactor4<gtsam::Rot3, gtsam::Vector3, gtsam::Vector6, gtsam::Rot3>(model, j1, j2, j3, j4) {}

        virtual ~NMEAFactor() {}

        gtsam::Vector evaluateError(const gtsam::Rot3 &rot_ext, const gtsam::Vector3 &pos_ext, const gtsam::Vector6 &pos_vel, const gtsam::Rot3 &rot,
            gtsam::OptionalMatrixType H1 = OptionalNone, gtsam::OptionalMatrixType H2 = OptionalNone, 
            gtsam::OptionalMatrixType H3 = OptionalNone, gtsam::OptionalMatrixType H4 = OptionalNone) const override
        {
            Eigen::Vector3d ref_enu = pos_ext;

            const Eigen::Vector3d local_pos = rot * Tex_imu_r + pos_vel.segment<3>(0) - anc_local;
            const Eigen::Vector3d local_vel = rot.matrix().transpose()*pos_vel.segment<3>(3);

            Eigen::Matrix3d R_enu_local = rot_ext.matrix(); // R_ecef_enu_cur * R_enu_local;

            Eigen::Vector3d P_enu = R_enu_local * local_pos + ref_enu;

            if (position_only_)
            {
                gtsam::Vector residual(3);
                residual = (P_enu - pos_meas) * relative_sqrt_info;
                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(3, 3);
                    Eigen::Matrix3d d_pos;
                    d_pos << 0.0, local_pos[2], -local_pos[1],
                             -local_pos[2], 0.0, local_pos[0],
                             local_pos[1], -local_pos[0], 0.0;
                    (*H1).block<3, 3>(0, 0) = R_enu_local * d_pos * relative_sqrt_info;
                }
                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(3, 3);
                    (*H2).block<3, 3>(0, 0) = gtsam::Matrix::Identity(3, 3) * relative_sqrt_info;
                }
                if (H3)
                {
                    (*H3) = gtsam::Matrix::Zero(3, 6);
                    (*H3).block<3, 3>(0, 0) = R_enu_local * relative_sqrt_info;
                }
                if (H4)
                {
                    (*H4) = gtsam::Matrix::Zero(3, 3);
                    // Position-only mode still depends on rot via local_pos = rot * Tex_imu_r + pos - anc.
                    // Provide Jacobian wrt rot to avoid underconstraining R(frame) in NavSatFix path.
                    Eigen::Matrix3d d_pos_rot;
                    d_pos_rot << 0.0, -Tex_imu_r[2], Tex_imu_r[1],
                                 Tex_imu_r[2], 0.0, -Tex_imu_r[0],
                                 -Tex_imu_r[1], Tex_imu_r[0], 0.0;
                    (*H4).block<3, 3>(0, 0) = -R_enu_local * rot.matrix() * d_pos_rot * relative_sqrt_info;
                }
                return residual;
            }

            Eigen::Vector3d V_enu = local_vel + hat_omg_T;

            Eigen::Matrix3d R_enu = R_enu_local * rot.matrix() * Rex_imu_r;


            gtsam::Vector9 residual;
            
            {    
                residual.segment<3>(0) = (P_enu - pos_meas) * relative_sqrt_info;

                residual.segment<3>(3) = (V_enu - vel_meas) * relative_sqrt_info;

                Eigen::Matrix3d res_R = rot_meas.transpose() * R_enu;
                Eigen::Vector3d res_r = gtsam::Rot3::Logmap(gtsam::Rot3(res_R));

                residual.segment<3>(6) = res_r * relative_sqrt_info;

                if (H1)
                {
                    (*H1) = gtsam::Matrix::Zero(9,3);
                    Eigen::Matrix3d d_pos;
                        d_pos << 0.0, local_pos[2], -local_pos[1], 
                                    -local_pos[2], 0.0, local_pos[0], 
                                    local_pos[1], -local_pos[0], 0.0;
                    (*H1).block<3,3>(0,0) = R_enu_local * d_pos * relative_sqrt_info;
                    // (*H1).block<3,3>(3,0) = -d_vel * relative_sqrt_info;
                    (*H1).block<3,3>(6,0) = Jacob_right_inv<double>(res_r) * Rex_imu_r.transpose() * rot.matrix().transpose() * relative_sqrt_info;
                }

                if (H2)
                {
                    (*H2) = gtsam::Matrix::Zero(9,3);
                    (*H2).block<3, 3>(0, 0) = gtsam::Matrix::Identity(3, 3) * relative_sqrt_info;
                }

                if (H3)
                {
                    (*H3) = gtsam::Matrix::Zero(9,6);
                    (*H3).block<3, 3>(0, 0) = R_enu_local * relative_sqrt_info;
                    (*H3).block<3, 3>(3, 3) = rot.matrix().transpose() * relative_sqrt_info;
                }

                if (H4)
                {
                    Eigen::Matrix3d d_vel;
                    d_vel << 0.0, -local_vel[2], local_vel[1], 
                                local_vel[2], 0.0, -local_vel[0], 
                                -local_vel[1], local_vel[0], 0.0;
                    (*H4) = gtsam::Matrix::Zero(9, 3);
                    (*H4).block<3, 3>(3, 0) = d_vel * relative_sqrt_info;
                    (*H4).block<3, 3>(6, 0) = Jacob_right_inv<double>(res_r) * Rex_imu_r.transpose() * relative_sqrt_info;
                }
                return residual;
            }
        }
    private:
        Eigen::Vector3d Tex_imu_r, anc_local, pos_meas, vel_meas, hat_omg_T;
        Eigen::Matrix3d rot_meas, Rex_imu_r;
        double relative_sqrt_info;
        bool invalid_lidar;
        bool position_only_;
};
}

#endif