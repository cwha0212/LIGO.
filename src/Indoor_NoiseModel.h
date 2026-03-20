#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace ligo {
namespace indoor {

void initIndoorPoseNoises(double pos_noise, double pos_noise_z, double rot_noise,
                          bool outlier_rej, double outlier_thres, double outlier_thres_init,
                          gtsam::noiseModel::Base::shared_ptr& indoor_pose_noise,
                          gtsam::noiseModel::Base::shared_ptr& indoor_pose_noise_init);

}  // namespace indoor
}  // namespace ligo

