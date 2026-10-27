#include <targeter/Skeleton.h>
#include "MathOps.h"

#include <algorithm>
#include <cassert>
#include <stdexcept>

namespace targeter {

namespace {
using detail::Vec3;
using detail::Quat;
using detail::QMul;
using detail::QRotate;
using detail::QNormalize;

// Multiply two quaternions and two positions — Havok QsTransform semantics:
// scale(s), rotate(r), translate(t).  Scale ignored in composition helpers.
[[nodiscard]] Vec3 Mul(const std::array<float,3>& v, float s) noexcept {
    return {v[0]*s, v[1]*s, v[2]*s};
}
} // namespace

int Skeleton::BoneIndex(std::string_view bone) const {
    for (int i = 0; i < static_cast<int>(bones.size()); ++i)
        if (bones[i].name == bone) return i;
    return -1;
}

std::vector<Transform> Skeleton::RestLocal() const {
    std::vector<Transform> out;
    out.reserve(bones.size());
    for (const auto& b : bones) out.push_back(b.rest);
    return out;
}

Transform Compose(const Transform& parent, const Transform& child) {
    // QsTransform: result.s = parent.s * child.s (uniform scale only here)
    // result.r = parent.r * child.r
    // result.t = parent.t + parent.r * (parent.s * child.t)
    detail::Quat pr{parent.r[0], parent.r[1], parent.r[2], parent.r[3]};
    detail::Quat cr{child.r[0],  child.r[1],  child.r[2],  child.r[3]};
    detail::Quat rr = detail::QMul(pr, cr);
    rr = detail::QNormalize(rr);

    detail::Vec3 ct{child.t[0], child.t[1], child.t[2]};
    float avgParentScale = (parent.s[0] + parent.s[1] + parent.s[2]) / 3.f;
    detail::Vec3 scaledCt = detail::Scale(ct, avgParentScale);
    detail::Vec3 rotated  = detail::QRotate(pr, scaledCt);

    Transform out;
    out.t = {parent.t[0] + rotated[0],
             parent.t[1] + rotated[1],
             parent.t[2] + rotated[2]};
    out.r = {rr[0], rr[1], rr[2], rr[3]};
    out.s = {parent.s[0] * child.s[0],
             parent.s[1] * child.s[1],
             parent.s[2] * child.s[2]};
    return out;
}

std::vector<Transform> ComputeModelSpace(const Skeleton& skeleton,
                                          std::span<const Transform> local) {
    assert(local.size() == skeleton.bones.size());
    std::vector<Transform> out(skeleton.bones.size());
    for (std::size_t i = 0; i < skeleton.bones.size(); ++i) {
        int parent = skeleton.bones[i].parent;
        if (parent < 0) {
            out[i] = local[i];
        } else {
            assert(parent < static_cast<int>(i)); // parent-before-child invariant
            out[i] = Compose(out[parent], local[i]);
        }
    }
    return out;
}

} // namespace targeter
