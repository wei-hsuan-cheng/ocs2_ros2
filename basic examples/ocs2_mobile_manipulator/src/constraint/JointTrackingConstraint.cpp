#include "ocs2_mobile_manipulator/constraint/JointTrackingConstraint.h"

#include "ocs2_mobile_manipulator/reference/MobileManipulatorReferenceManager.h"

#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <stdexcept>

namespace ocs2::mobile_manipulator {

JointTrackingConstraint::JointTrackingConstraint(const ManipulatorModelInfo& modelInfo, vector_t qArmRef)
    : StateConstraint(ConstraintOrder::Linear),
      stateDim_(modelInfo.stateDim),
      armDim_(modelInfo.armDim),
      baseStateDim_(modelInfo.stateDim - modelInfo.armDim),
      qArmRef_(std::move(qArmRef)) {
  if (qArmRef_.size() != static_cast<long>(armDim_)) {
    throw std::runtime_error("[JointTrackingConstraint] qArmRef size mismatch with armDim.");
  }
  if (baseStateDim_ + armDim_ != stateDim_) {
    throw std::runtime_error("[JointTrackingConstraint] stateDim != baseStateDim + armDim. Check state definition.");
  }
}

JointTrackingConstraint::JointTrackingConstraint(const ManipulatorModelInfo& modelInfo, vector_t qArmRef,
                                                 const ocs2::ReferenceManager& referenceManager, bool active)
    : JointTrackingConstraint(modelInfo, std::move(qArmRef)) {
  referenceManagerPtr_ = &referenceManager;
  active_ = active;
}

scalar_t JointTrackingConstraint::getActivationScale(scalar_t time) const {
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
  return mmRefManager->getBlendingWeights(time).alphaJoint;
}

vector_t JointTrackingConstraint::getValue(scalar_t time, const vector_t& state,
                                          const PreComputation& /*preComp*/) const {
  if (state.size() != static_cast<long>(stateDim_)) {
    throw std::runtime_error("[JointTrackingConstraint] state size mismatch with stateDim.");
  }
  const scalar_t activationScale = getActivationScale(time);
  if (activationScale == 0.0) {
    return vector_t::Zero(static_cast<long>(armDim_));
  }
  const vector_t qArm = state.segment(static_cast<long>(baseStateDim_), static_cast<long>(armDim_));
  return activationScale * (qArm - qArmRef_);
}

VectorFunctionLinearApproximation JointTrackingConstraint::getLinearApproximation(
    scalar_t time, const vector_t& state, const PreComputation& preComp) const {
  VectorFunctionLinearApproximation out;
  const scalar_t activationScale = getActivationScale(time);
  if (activationScale == 0.0) {
    out.f = vector_t::Zero(static_cast<long>(armDim_));
    out.dfdx.setZero(static_cast<long>(armDim_), static_cast<long>(stateDim_));
    return out;
  }
  out.f = getValue(time, state, preComp);

  out.dfdx.setZero(static_cast<long>(armDim_), static_cast<long>(stateDim_));
  // c = q_arm - q_ref  => dc/dq_arm = I
  out.dfdx.block(0, static_cast<long>(baseStateDim_), static_cast<long>(armDim_), static_cast<long>(armDim_))
      .setIdentity();
  out.dfdx *= activationScale;

  return out;
}

}  // namespace ocs2::mobile_manipulator
