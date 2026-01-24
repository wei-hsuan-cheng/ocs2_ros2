/******************************************************************************
Added by wei-hsuan-cheng (Jan. 2026)
******************************************************************************/

#pragma once

#include <stdexcept>
#include <string>

#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2::mobile_manipulator {

/**
 * JointTrackingConstraint
 *
 * Tracks a desired configuration in "position space" (Pinocchio nq-space):
 * - DefaultManipulator: tracks arm joints only       q_arm
 * - WheelBasedMobileManipulator: tracks planar base  [x, y, yaw] and arm joints
 * - FloatingArmManipulator / FullyActuatedFloatingArmManipulator:
 *     tracks floating base [x, y, z, zyx] and arm joints
 *
 * The constraint value is simply:
 *   c(q) = S*q - q_ref
 * where S selects the tracked coordinates.
 *
 * NOTE:
 *  - For wheel-based: base coordinates are (x, y, yaw) in the first 3 entries of state.
 *  - For floating-base: base coordinates are (x, y, z, zyx) in the first 6 entries of state.
 *  - Arm joints are always the last info.armDim entries of state (see AccessHelperFunctionsImpl).
 */
class JointTrackingConstraint final : public StateConstraint {
public:
  JointTrackingConstraint(const ManipulatorModelInfo& info, vector_t desired);
  ~JointTrackingConstraint() override = default;

  JointTrackingConstraint* clone() const override { return new JointTrackingConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const override;
  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;

  /** Returns base pose dimension tracked by this constraint: 0 / 3 / 6. */
  size_t basePoseDim() const { return basePoseDim_; }

  /** Returns total constraint dimension: basePoseDim + armDim */
  size_t constraintDim() const { return basePoseDim_ + info_.armDim; }

  /** Desired vector layout: [base_pose(optional), q_arm] */
  const vector_t& desired() const { return desired_; }

private:
  JointTrackingConstraint(const JointTrackingConstraint& other) = default;

  static size_t computeBasePoseDim(ManipulatorModelType type);

  ManipulatorModelInfo info_;
  size_t basePoseDim_{0};
  vector_t desired_;
};

}  // namespace ocs2::mobile_manipulator
