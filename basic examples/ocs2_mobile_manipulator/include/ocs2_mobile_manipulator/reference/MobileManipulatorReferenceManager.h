#pragma once

#include <unordered_map>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2::mobile_manipulator {

struct BlendingWeights {
  scalar_t alphaEe{1.0};
  scalar_t alphaBase{0.0};
  scalar_t alphaJoint{0.0};
};

class MobileManipulatorReferenceManager final : public ocs2::ReferenceManager {
 public:
  MobileManipulatorReferenceManager() = default;

  MobileManipulatorReferenceManager(BlendingWeights defaultWeights,
                                    std::unordered_map<size_t, BlendingWeights> modeWeights,
                                    bool normalize = true, scalar_t eps = 1e-9)
      : default_(defaultWeights),
        modeWeights_(std::move(modeWeights)),
        normalize_(normalize),
        eps_(eps) {}

  void setDefaultBlendingWeights(const BlendingWeights& w) { default_ = w; }
  void setModeBlendingWeights(size_t mode, const BlendingWeights& w) { modeWeights_[mode] = w; }
  void clearModeBlendingWeights() { modeWeights_.clear(); }

  void setNormalizeBlending(bool enable) { normalize_ = enable; }
  void setBlendingEpsilon(scalar_t eps) { eps_ = eps; }

  BlendingWeights getBlendingWeights(scalar_t time) const;

 private:
  BlendingWeights normalizeWeights(const BlendingWeights& w) const;

  BlendingWeights default_{};
  std::unordered_map<size_t, BlendingWeights> modeWeights_;
  bool normalize_{true};
  scalar_t eps_{1e-9};
};

}  // namespace ocs2::mobile_manipulator
