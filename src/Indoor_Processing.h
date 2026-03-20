#pragma once

#include <Eigen/Core>

namespace ligo {
namespace indoor {

// Placeholder hook until real indoor localization module is integrated.
// The input pose is ENU absolute pose from localization.
void updateIndoorLocalizationPlaceholder(const Eigen::Vector3d& pos_enu,
                                         const Eigen::Matrix3d& rot_enu,
                                         double ts_sec);

// Stub for future graph integration. Intentionally no-op for now.
void addIndoorFactorToGraphStubCommented();

}  // namespace indoor
}  // namespace ligo

