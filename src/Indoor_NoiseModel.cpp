#include "Indoor_NoiseModel.h"

namespace ligo {
namespace indoor {

void initIndoorPoseNoises(double pos_noise, double pos_noise_z, double rot_noise,
                          bool outlier_rej, double outlier_thres, double outlier_thres_init,
                          gtsam::noiseModel::Base::shared_ptr& indoor_pose_noise,
                          gtsam::noiseModel::Base::shared_ptr& indoor_pose_noise_init) {
  gtsam::Vector noise_vec(6);
  noise_vec << pos_noise, pos_noise, pos_noise_z, rot_noise, rot_noise, rot_noise;

  if (outlier_rej) {
    indoor_pose_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres),
        gtsam::noiseModel::Diagonal::Variances(noise_vec));
    indoor_pose_noise_init = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres_init),
        gtsam::noiseModel::Diagonal::Variances(noise_vec));
  } else {
    indoor_pose_noise = gtsam::noiseModel::Diagonal::Variances(noise_vec);
    indoor_pose_noise_init = gtsam::noiseModel::Diagonal::Variances(noise_vec);
  }
}

}  // namespace indoor
}  // namespace ligo

