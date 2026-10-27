#include <targeter/RootMotion.h>
#include "MathOps.h"

namespace targeter {

namespace {
using detail::Quat;
using detail::Vec3;
using detail::QMul;
using detail::QInverse;
using detail::QNormalize;

// Conjugate (inverse for unit quaternion).
[[nodiscard]] Quat Inverse(Quat q) noexcept { return detail::QInverse(q); }
} // namespace

void GroundRoot(Clip& clip, const Skeleton& skeleton, std::string_view rootBone) {
    int ri = clip.BoneIndex(rootBone.empty() ? "Root" : rootBone);
    if (ri < 0) return;

    for (int f = 0; f < clip.numFrames; ++f) {
        Transform& rt = clip.At(f, ri);
        // Zero lateral translation and ground vertical
        rt.t = {0.f, 0.f, rt.t[2]};
    }

    // Remove vertical drift: subtract first-frame Z offset
    if (clip.numFrames > 0) {
        float baseZ = clip.At(0, ri).t[2];
        for (int f = 0; f < clip.numFrames; ++f)
            clip.At(f, ri).t[2] -= baseZ;
    }
}

} // namespace targeter
