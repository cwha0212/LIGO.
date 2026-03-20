#pragma once

#include <gtsam/linear/NoiseModel.h>

namespace ligo {
namespace indoor {

void initIndoorPoseNoises(double pos_noise, double pos_noise_z, double rot_noise,
                          bool outlier_rej, double outlier_thres, double outlier_thres_init,
                          gtsam::noiseModel::Base::shared_ptr& robustindoorNoise,
                          gtsam::noiseModel::Base::shared_ptr& robustindoorNoise_init);

}  // namespace indoor
}  // namespace ligo

