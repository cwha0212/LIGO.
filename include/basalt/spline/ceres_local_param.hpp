/**
 * Ceres 2.x removed LocalParameterization; trajectory SO(3) knots use
 * ceres::EigenQuaternionManifold (see Curvefitter/trajectory_estimator.hpp).
 * This header remains for includes that expect the path to exist.
 */
#pragma once

#include <ceres/manifold.h>
