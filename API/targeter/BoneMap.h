#pragma once
// BoneMap — describes how to retarget from a source skeleton onto a target.
// All bone names are exact byte matches against Skeleton::bones[i].name.

#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>

namespace targeter {

/// A twist bone's driver: the bone whose twist it takes a share of.
struct TwistSource {
    /// The bone twisting: the twist bone's parent (an upper arm's own twist, shared back down to its twist bones) or a
    /// child of that parent (a hand's pronation, shared up the forearm's twist bones).
    std::string driver;
    float       fraction = 0.5f;   ///< the share of the driver's twist; negative turns against it
    bool operator==(const TwistSource&) const = default;
};

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
    /// The target's feet (each the end of a two-bone leg: thigh, calf, foot), for foot locking. Empty = no locking.
    std::string targetLeftFoot, targetRightFoot;

    /// target bone name → source bone name
    std::map<std::string, std::string> bones;
    /// target bone name → aim-child bone name (for aim-driven retargeting). Ignored on the root and hips.
    std::map<std::string, std::string> aim;
    /// The retarget pose: target bone name → local rotation (x, y, z, w) on top of the rest alignment, applied parent-first
    /// so it carries down the chain (UE's "edit retarget pose"). The offset right-multiplies the bone's aligned rest, so on
    /// every frame the bone's world rotation becomes W * offset. TurnPoseOffset edits one in model space.
    std::map<std::string, std::array<float, 4>> pose;
    /// Unmapped target twist bones → the bone whose twist they share. Without one a twist bone holds its rest.
    std::map<std::string, TwistSource> twist;
    /// Unmapped target bones that bend with their chain: each takes its share, by length, of the turn between the mapped
    /// bone above it and the one mapped bone below it (a source with fewer spine bones than the target).
    std::set<std::string> between;
    /// The local axis every source bone points along (mocap layouts aim theirs). With it a bone whose aim child has no
    /// source is still aligned, by that axis; without it such a bone keeps its rest.
    std::optional<std::array<float, 3>> sourceAxis;

    /// @return the source bone driving `targetBone`, or empty string if unmapped.
    [[nodiscard]] std::string SourceOf(std::string_view targetBone) const;

    bool operator==(const BoneMap&) const = default;
};

} // namespace targeter
