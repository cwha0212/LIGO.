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
#include <vector>
#include <eigen3/Eigen/Dense>
#include <gtsam/nonlinear/Marginals.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <common_lib.h>
#include <numeric>
#include <opencv2/core/eigen.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <nmea_factor/nmea_lio_gravity_rel_factor.hpp>
#include <nmea_factor/nmea_factor.hpp>

using gtsam::symbol_shorthand::R; // body/local rotation Rot3
using gtsam::symbol_shorthand::P; // extrinsic rotation Rot3 (local->ENU)
using gtsam::symbol_shorthand::E; // extrinsic translation Vector3 (ENU anchor)
using gtsam::symbol_shorthand::A; // body pos+vel Vector6
using gtsam::symbol_shorthand::O; // omg, acc, bg, ba Vector12
using gtsam::symbol_shorthand::G; // gravity Vector3
// using gtsam::symbol_shorthand::G; // ext_R
// using gtsam::symbol_shorthand::Y; // local enu (yaw)
// using gtsam::symbol_shorthand::A; // anchor point (anc) total = 18 dimensions

class NMEAAssignment
{
    public:
        NMEAAssignment();
        NMEAAssignment(const NMEAAssignment&) = delete;
        NMEAAssignment& operator=(const NMEAAssignment&) = delete;
        ~NMEAAssignment() {};

        // use GTSAM
        gtsam::NonlinearFactorGraph gtSAMgraph; // store factors //
        gtsam::Values initialEstimate; // store initial values of the node //

        gtsam::Values isamCurrentEstimate; // 
        gtsam::ISAM2 isam;

        double prior_noise = 0.01;
        double marg_noise = 0.01;
        double odo_noise = 0.01;
        double grav_noise = 0.01;
        double pos_noise = 0.01;
        double pos_noise_z = 0.01;  // z-axis position noise (often larger for GNSS/PPP vertical)
        double vel_noise = 0.01;
        double rot_noise = 0.01;

        bool outlier_rej = false;
        double outlier_thres = 0.1;
        double outlier_thres_init = 0.1;
        int process_feat_num = 0;
        double ppp_std_threshold = 30.0;  // gate vs pose.covariance[0],[7] only (horiz.); not [14]

        gtsam::noiseModel::Base::shared_ptr margrotNoise;
        gtsam::noiseModel::Base::shared_ptr margposNoise;
        // gtsam::noiseModel::Diagonal::shared_ptr priorvelNoise;
        gtsam::noiseModel::Base::shared_ptr margNoise;

        gtsam::noiseModel::Base::shared_ptr priorrotNoise;
        gtsam::noiseModel::Base::shared_ptr priorposNoise;
        gtsam::noiseModel::Base::shared_ptr priorextrotNoise;
        gtsam::noiseModel::Base::shared_ptr priorNoise;
        gtsam::noiseModel::Base::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Base::shared_ptr priorGravNoise;
        gtsam::noiseModel::Base::shared_ptr priorextposNoise;
        gtsam::noiseModel::Base::shared_ptr odomNoise;
        gtsam::noiseModel::Base::shared_ptr odomaNoise;
        gtsam::noiseModel::Base::shared_ptr relatNoise;
        // gtsam::noiseModel::Diagonal::shared_ptr odomNoiseIMU;
        gtsam::noiseModel::Base::shared_ptr odomNoiseIMU;
        gtsam::noiseModel::Base::shared_ptr robustnmeaNoise;
        gtsam::noiseModel::Base::shared_ptr robustnmeaNoise_init;
        void initNoises(void); 

        int marg_thred = 1;
        int change_ext = 1;
        std::deque<std::vector<size_t>> factor_id_frame; // 

        void delete_variables(size_t frame_delete, int frame_num, size_t &id_accumulate, gtsam::FactorIndices delete_factor);
};