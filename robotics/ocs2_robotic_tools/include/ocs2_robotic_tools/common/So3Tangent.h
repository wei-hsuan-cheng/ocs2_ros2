#pragma once

#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

namespace ocs2 {

/** SO(3) right-Jacobian inverse with finite generated derivatives at zero.
 * Uses the standard inverse-dexp kernel and near-zero series documented by GTSAM:
 * https://github.com/borglab/gtsam/blob/develop/gtsam/geometry/SO3.cpp
 * Guard the inactive expression before sqrt/division, not only its final result.
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 3> so3RightJacobianInverse(const Eigen::Matrix<Scalar, 3, 1>& phi) {
  using std::sin;
  using std::sqrt;
  const Scalar theta2 = phi.squaredNorm();
  const Scalar threshold(1e-6);
  const Scalar safeTheta2 = CppAD::CondExpLt(theta2, threshold, Scalar(1.0), theta2);
  const Scalar theta = sqrt(safeTheta2);
  const Scalar A = sin(theta) / theta;
  const Scalar halfSine = sin(theta / Scalar(2.0));
  const Scalar B = Scalar(2.0) * halfSine * halfSine / safeTheta2;
  const Scalar regular = (Scalar(1.0) - A / (Scalar(2.0) * B)) / safeTheta2;
  const Scalar series = Scalar(1.0 / 12.0) + theta2 / Scalar(720.0);
  const Scalar D = CppAD::CondExpLt(theta2, threshold, series, regular);
  const Eigen::Matrix<Scalar, 3, 3> skew = skewSymmetricMatrix(phi);
  return Eigen::Matrix<Scalar, 3, 3>::Identity() + Scalar(0.5) * skew + D * skew * skew;
}

}  // namespace ocs2
