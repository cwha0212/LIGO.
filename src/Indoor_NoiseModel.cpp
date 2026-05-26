#include "Indoor_NoiseModel.h"

namespace ligo {
namespace indoor {

void initIndoorPoseNoises(double pos_noise, double pos_noise_z, double rot_noise,
                          bool outlier_rej, double outlier_thres, double outlier_thres_init,
                          gtsam::noiseModel::Base::shared_ptr& robustindoorNoise,
                          gtsam::noiseModel::Base::shared_ptr& robustindoorNoise_init) {
  // Keep naming/style aligned with NMEAAssignment::initNoises()
  gtsam::Vector robustindoorNoiseVector6(6);
  robustindoorNoiseVector6 << pos_noise, pos_noise, pos_noise_z, rot_noise, rot_noise, rot_noise;

  if (outlier_rej)
  {
    robustindoorNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres),
        gtsam::noiseModel::Diagonal::Variances(robustindoorNoiseVector6));
    robustindoorNoise_init = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres_init),
        gtsam::noiseModel::Diagonal::Variances(robustindoorNoiseVector6));
  } else {
    robustindoorNoise = gtsam::noiseModel::Diagonal::Variances(robustindoorNoiseVector6);
    robustindoorNoise_init = gtsam::noiseModel::Diagonal::Variances(robustindoorNoiseVector6);
  }
}

}  // namespace indoor
}  // namespace ligo

