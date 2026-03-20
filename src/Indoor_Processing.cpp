#include "Indoor_Processing.h"
#include "parameters.h"

namespace ligo {
namespace indoor {

void updateIndoorLocalizationPlaceholder(const Eigen::Vector3d& pos_enu,
                                         const Eigen::Matrix3d& rot_enu,
                                         double ts_sec) {
  if (!indoor_flag) {
    return;
  }

  indoor_pos_enu_meas = pos_enu;
  indoor_rot_enu_meas = Eigen::Quaterniond(rot_enu).normalized();
  indoor_pose_time = ts_sec;
  indoor_pose_valid = true;
}

void addIndoorFactorToGraphStubCommented() {
  // TODO(indoor): actual factor add point when localization module is ready.
  // This is intentionally no-op now; keep it as a guide.
  //
  if (indoor_flag && indoor_pose_valid) {
    double values[17] = {0.0};
    values[0] = p_nmea->Tex_imu_r[0]; values[1] = p_nmea->Tex_imu_r[1]; values[2] = p_nmea->Tex_imu_r[2];
    values[3] = p_nmea->anc_local[0]; values[4] = p_nmea->anc_local[1]; values[5] = p_nmea->anc_local[2];
    values[6] = indoor_pos_enu_meas[0]; values[7] = indoor_pos_enu_meas[1]; values[8] = indoor_pos_enu_meas[2];
    values[12] = indoor_rot_enu_meas.w(); values[13] = indoor_rot_enu_meas.x();
    values[14] = indoor_rot_enu_meas.y(); values[15] = indoor_rot_enu_meas.z();
    values[16] = 1.0; // or configurable weight
  
    const auto& indoor_noise = (p_nmea->frame_num < p_nmea->delete_thred) ? indoorPoseNoiseInit : indoorPoseNoise;
    p_nmea->p_assign->gtSAMgraph.add(ligo::IndoorLocalizationFactor(
        P(0), E(0), A(p_nmea->frame_num), R(p_nmea->frame_num),
        false, values, Eigen::Vector3d::Zero(), p_nmea->Rex_imu_r, indoor_noise));
  }
}

}  // namespace indoor
}  // namespace ligo

