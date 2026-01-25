#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2::mobile_manipulator {

/**
 * BaseTrackingConstraint (BASE ONLY)
 *
 * Constraint vector:
 *   c(x) = base_pose(x) - base_pose_ref
 *
 * Assumption:
 *   base pose is stored at the beginning of the state vector:
 *     base_pose = state.head(basePoseDim)
 *
 * basePoseDim depends on ManipulatorModelType:
 *  - DefaultManipulator: 0
 *  - WheelBasedMobileManipulator: 3  -> [x, y, yaw]
 *  - FloatingArmManipulator / FullyActuatedFloatingArmManipulator: 6 -> [x, y, z, zyx]
 */
class BaseTrackingConstraint final : public StateConstraint {
 public:
  BaseTrackingConstraint(const ManipulatorModelInfo& modelInfo, vector_t baseRef);

  ~BaseTrackingConstraint() override = default;

  BaseTrackingConstraint* clone() const override { return new BaseTrackingConstraint(*this); }

  size_t getNumConstraints(scalar_t /*time*/) const override { return basePoseDim_; }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(
      scalar_t time, const vector_t& state, const PreComputation& preComp) const override;

  size_t getBasePoseDim() const { return basePoseDim_; }

 private:
  size_t stateDim_{0};
  size_t basePoseDim_{0};

  vector_t baseRef_;
};

}  // namespace ocs2::mobile_manipulator
