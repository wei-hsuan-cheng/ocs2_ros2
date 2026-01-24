/******************************************************************************
Added by wei-hsuan-cheng (Jan. 2026)
******************************************************************************/

#include "ocs2_mobile_manipulator/constraint/JointTrackingConstraint.h"

#include <Eigen/Dense>

#include "ocs2_mobile_manipulator/AccessHelperFunctions.h"

namespace ocs2::mobile_manipulator {

size_t JointTrackingConstraint::computeBasePoseDim(ManipulatorModelType type) {
  switch (type) {
    case ManipulatorModelType::DefaultManipulator:
      return 0;
    case ManipulatorModelType::WheelBasedMobileManipulator:
      // q = [x, y, yaw, q_arm]
      return 3;
    case ManipulatorModelType::FloatingArmManipulator:
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
      // q = [x, y, z, zyx, q_arm]
      return 6;
    default:
      throw std::invalid_argument("[JointTrackingConstraint] Unsupported manipulator model type.");
  }
}

JointTrackingConstraint::JointTrackingConstraint(const ManipulatorModelInfo& info, vector_t desired)
    : StateConstraint(ConstraintOrder::Linear),
      info_(info),
      basePoseDim_(computeBasePoseDim(info.manipulatorModelType)),
      desired_(std::move(desired)) {
  const auto expectedDim = static_cast<long>(basePoseDim_ + info_.armDim);
  if (desired_.rows() != expectedDim || desired_.cols() != 1) {
    throw std::runtime_error(
        "[JointTrackingConstraint] desired vector has wrong dimension. expected " +
        std::to_string(expectedDim) + "x1, got " + std::to_string(desired_.rows()) + "x" +
        std::to_string(desired_.cols()));
  }
}

size_t JointTrackingConstraint::getNumConstraints(scalar_t /*time*/) const {
  return constraintDim();
}

vector_t JointTrackingConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                           const PreComputation& /*preComputation*/) const {
  if (state.rows() != static_cast<long>(info_.stateDim) || state.cols() != 1) {
    throw std::runtime_error("[JointTrackingConstraint] state dimension mismatch.");
  }

  vector_t c(constraintDim());
  c.setZero();

  // Base part (if any)
  if (basePoseDim_ > 0) {
    c.head(static_cast<long>(basePoseDim_)) =
        state.head(static_cast<long>(basePoseDim_)) - desired_.head(static_cast<long>(basePoseDim_));
  }

  // Arm joint part (always the last info.armDim entries)
  const auto q_arm = getArmJointAngles(state, info_);
  c.tail(static_cast<long>(info_.armDim)) =
      q_arm - desired_.tail(static_cast<long>(info_.armDim));

  return c;
}

VectorFunctionLinearApproximation JointTrackingConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComputation) const {
  auto approx = VectorFunctionLinearApproximation(static_cast<int>(constraintDim()),
                                                  static_cast<int>(info_.stateDim), 0);

  // f
  approx.f = getValue(time, state, preComputation);

  // dfdx is just a selection matrix
  approx.dfdx.setZero();

  // Base block
  if (basePoseDim_ > 0) {
    approx.dfdx.block(0, 0, static_cast<int>(basePoseDim_), static_cast<int>(basePoseDim_))
        .setIdentity();
  }

  // Arm joint block (last armDim columns)
  const int armDim = static_cast<int>(info_.armDim);
  const int armStartCol = static_cast<int>(info_.stateDim - info_.armDim);
  approx.dfdx.block(static_cast<int>(basePoseDim_), armStartCol, armDim, armDim).setIdentity();

  // dfdu is zero (StateConstraint)
  return approx;
}

}  // namespace ocs2::mobile_manipulator
