#pragma once
// Skeleton — a bone hierarchy with a local rest pose, plus forward kinematics.
// Parents must precede their children in the `bones` array; ComputeModelSpace
// and RetargetClip both rely on this.

#include <targeter/Clip.h>   // Transform

#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace targeter {

struct SkeletonBone {
    std::string name;
    int         parent = -1;   // index into Skeleton::bones; -1 = root
    Transform   rest;          // local (parent-space) rest transform
    bool operator==(const SkeletonBone&) const = default;
};

struct Skeleton {
    std::string               name;
    std::vector<SkeletonBone> bones;

    /// @return the bone index, or -1 if not found.
    [[nodiscard]] int BoneIndex(std::string_view bone) const;
    /// @return the rest transforms as a flat vector, index-parallel to `bones`.
    [[nodiscard]] std::vector<Transform> RestLocal() const;
    bool operator==(const Skeleton&) const = default;
};

/// parent * child (Havok QsTransform convention: scale, then rotate, then translate).
[[nodiscard]] Transform Compose(const Transform& parent, const Transform& child);

/// Model-space transforms from local ones (index-parallel to `skeleton.bones`).
/// Requires parents to precede children.
[[nodiscard]] std::vector<Transform> ComputeModelSpace(const Skeleton& skeleton,
                                                        std::span<const Transform> local);

} // namespace targeter
