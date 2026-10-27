#pragma once
// BoneMap — describes how to retarget from a source skeleton onto a target.
// All bone names are exact byte matches against Skeleton::bones[i].name.

#include <map>
#include <string>
#include <string_view>

namespace targeter {

/// A retargeting map from a source rig to a named target rig.
/// `bones` maps each target bone name to the source bone that drives it.
/// `aim` optionally maps a target bone to the child used to infer its aim direction.
struct BoneMap {
    std::string name;
    std::string target;           // target skeleton name
    std::string sourceRoot;       // source root (world-space reference bone)
    std::string sourceHips;       // source hips (vertical reference)
    std::string sourceLeftHip;    // source left hip — used to establish +X facing
    std::string sourceRightHip;   // source right hip — used to establish +X facing

    std::string targetRoot = "Root";
    std::string targetHips = "Hips";

    /// target bone name → source bone name
    std::map<std::string, std::string> bones;
    /// target bone name → aim-child bone name (for aim-driven retargeting)
    std::map<std::string, std::string> aim;

    /// @return the source bone driving `targetBone`, or empty string if unmapped.
    [[nodiscard]] std::string SourceOf(std::string_view targetBone) const;

    bool operator==(const BoneMap&) const = default;
};

} // namespace targeter
