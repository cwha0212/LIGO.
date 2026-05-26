/* GTSAM 4.1 uses boost::optional<Matrix&> for factor Jacobians; newer GTSAM
 * exposes OptionalMatrixType / OptionalNone. Inject aliases for older GTSAM. */
#pragma once

#include <boost/optional.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/config.h>

#if GTSAM_VERSION_NUMERIC < 40300
namespace gtsam {
using OptionalMatrixType = boost::optional<Matrix&>;
}
#ifndef OptionalNone
#define OptionalNone boost::none
#endif
#endif
