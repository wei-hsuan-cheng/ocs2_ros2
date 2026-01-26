#include "ocs2_mobile_manipulator/constraint/BaseTrackingConstraint.h"

#include "ocs2_mobile_manipulator/reference/MobileManipulatorReferenceManager.h"

#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <stdexcept>

namespace ocs2::mobile_manipulator {

static size_t computeBasePoseDim(ManipulatorModelType type) {
  switch (type) {
    case ManipulatorModelType::WheelBasedMobileManipulator:
      return 3;
    case ManipulatorModelType::FloatingArmManipulator:
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator:
      return 6;
    case ManipulatorModelType::DefaultManipulator:
    default:
      return 0;
  }
}

BaseTrackingConstraint::BaseTrackingConstraint(const ManipulatorModelInfo& modelInfo, vector_t baseRef)
    : StateConstraint(ConstraintOrder::Linear),
      stateDim_(modelInfo.stateDim),
      basePoseDim_(computeBasePoseDim(modelInfo.manipulatorModelType)),
      baseRef_(std::move(baseRef)) {

  if (basePoseDim_ == 0) {
    throw std::runtime_error(
        "[BaseTrackingConstraint] basePoseDim == 0 (DefaultManipulator). "
        "Do not activate baseTracking for this model.");
  }

  if (baseRef_.size() != static_cast<long>(basePoseDim_)) {
    throw std::runtime_error("[BaseTrackingConstraint] baseRef size mismatch with basePoseDim.");
  }

  if (stateDim_ < basePoseDim_) {
    throw std::runtime_error("[BaseTrackingConstraint] stateDim < basePoseDim. Check state definition.");
  }
}

BaseTrackingConstraint::BaseTrackingConstraint(const ManipulatorModelInfo& modelInfo, vector_t baseRef,
                                               const ocs2::ReferenceManager& referenceManager, bool active)
    : BaseTrackingConstraint(modelInfo, std::move(baseRef)) {
  referenceManagerPtr_ = &referenceManager;
  active_ = active;
}

scalar_t BaseTrackingConstraint::getActivationScale(scalar_t time) const {
  if (!active_) {
    return 0.0;
  }
  if (referenceManagerPtr_ == nullptr) {
    return 1.0;
  }
  const auto* mmRefManager = dynamic_cast<const MobileManipulatorReferenceManager*>(referenceManagerPtr_);
  if (mmRefManager == nullptr) {
    return 1.0;
  }
  return mmRefManager->getBlendingWeights(time).alphaBase;
}

vector_t BaseTrackingConstraint::getValue(scalar_t time, const vector_t& state,
                                         const PreComputation& /*preComp*/) const {
  if (state.size() != static_cast<long>(stateDim_)) {
    throw std::runtime_error("[BaseTrackingConstraint] state size mismatch with stateDim.");
  }
  const scalar_t activationScale = getActivationScale(time);
  if (activationScale == 0.0) {
    return vector_t::Zero(static_cast<long>(basePoseDim_));
  }
  const vector_t basePose = state.head(static_cast<long>(basePoseDim_));
  return activationScale * (basePose - baseRef_);
}

VectorFunctionLinearApproximation BaseTrackingConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComp) const {

  VectorFunctionLinearApproximation out;
  const scalar_t activationScale = getActivationScale(time);
  if (activationScale == 0.0) {
    out.f = vector_t::Zero(static_cast<long>(basePoseDim_));
    out.dfdx.setZero(static_cast<long>(basePoseDim_), static_cast<long>(stateDim_));
    return out;
  }
  out.f = getValue(time, state, preComp);

  out.dfdx.setZero(static_cast<long>(basePoseDim_), static_cast<long>(stateDim_));
  // c = basePose - baseRef  => dc/d(basePose) = I
  out.dfdx.block(0, 0, static_cast<long>(basePoseDim_), static_cast<long>(basePoseDim_)).setIdentity();
  out.dfdx *= activationScale;

  return out;
}

}  // namespace ocs2::mobile_manipulator
