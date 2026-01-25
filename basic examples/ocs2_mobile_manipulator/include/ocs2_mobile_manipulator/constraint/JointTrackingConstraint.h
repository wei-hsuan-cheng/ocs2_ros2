#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2::mobile_manipulator {

/**
 * JointTrackingConstraint (ARM ONLY)
 *
 * Constraint vector:
 *   c(x) = q_arm(x) - q_arm_ref
 *
 * where q_arm is extracted from the full state vector by:
 *   baseStateDim = stateDim - armDim
 *   q_arm = state.segment(baseStateDim, armDim)
 */
class JointTrackingConstraint final : public StateConstraint {
 public:
  JointTrackingConstraint(const ManipulatorModelInfo& modelInfo, vector_t qArmRef);

  ~JointTrackingConstraint() override = default;

  JointTrackingConstraint* clone() const override { return new JointTrackingConstraint(*this); }

  size_t getNumConstraints(scalar_t /*time*/) const override { return armDim_; }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state, const PreComputation& preComp) const override;

 private:
  size_t stateDim_{0};
  size_t armDim_{0};
  size_t baseStateDim_{0};

  vector_t qArmRef_;
};

}  // namespace ocs2::mobile_manipulator
