#include "ocs2_mobile_manipulator/reference/MobileManipulatorReferenceManager.h"

#include <cmath>

namespace ocs2::mobile_manipulator {

BlendingWeights MobileManipulatorReferenceManager::normalizeWeights(const BlendingWeights& w) const {
  if (!normalize_) {
    return w;
  }
  const scalar_t s = w.alphaEe + w.alphaBase + w.alphaJoint;
  if (s <= eps_) {
    // fallback: keep EE only if sum is degenerate
    return BlendingWeights{1.0, 0.0, 0.0};
  }
  return BlendingWeights{w.alphaEe / s, w.alphaBase / s, w.alphaJoint / s};
}

BlendingWeights MobileManipulatorReferenceManager::getBlendingWeights(scalar_t time) const {
  const auto& ms = this->getModeSchedule();
  const size_t mode = ms.modeAtTime(time);

  auto it = modeWeights_.find(mode);
  const BlendingWeights w = (it != modeWeights_.end()) ? it->second : default_;
  return normalizeWeights(w);
}

}  // namespace ocs2::mobile_manipulator
