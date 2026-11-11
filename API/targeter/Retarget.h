#pragma once
// RetargetClip — rest-aligned world-delta retargeting.
// Algorithm: align source rest to target rest, then for each frame compute the
// world-space rotation delta from source rest and apply it to the target bone.
// The target root carries only the heading (yaw) and the ground travel; the hips carry the height.

#include <targeter/BoneMap.h>
#include <targeter/Clip.h>
#include <targeter/FootPlants.h>
#include <targeter/Skeleton.h>

#include <array>
#include <expected>
#include <string>
#include <vector>

namespace targeter {

struct RetargetOptions {
    /// Target size / source size; 0 = the hips' height over the ground at rest, target over source.
    float scale = 0.f;
    /// Without a source root the heading comes from the hips, which sway a few degrees each stride: this is the sigma
    /// (seconds) of the Gaussian that smooths it. 0 = unsmoothed.
    float rootYawSmoothSeconds = 0.3f;
    /// Pin each planted foot where it lands, through two-bone leg IK (needs the map's target feet).
    bool lockFeet = false;
    /// Plant detection on the source, in target units (scaled onto the source).
    PlantThresholds plants;
    /// Frames over which a lock blends in before a plant and out after it.
    int lockBlendFrames = 4;
};

struct RetargetReport {
    int mappedBones   = 0;
    int unmappedBones = 0;   // target bones with no source mapping (left at rest)
    std::vector<std::string> missingSource;     // map entries naming bones the source lacks
    std::vector<std::string> unmappedTargets;   // target bones no map entry drives, by name
    std::vector<std::string> ignoredAims;       // aims the retarget can't use (on the root or hips, or naming no bone)
    std::vector<std::string> ignoredTwists;     // twist entries whose driver isn't the twist bone's parent or sibling

    // How source positions were placed into the target's frame, for drawing the source beside the result.
    float                scale = 1.f;              // target hip height / source hip height
    std::array<float, 4> facing{0.f, 0.f, 0.f, 1.f};   // x, y, z, w
    float                sourceGround = 0.f, targetGround = 0.f;
    std::array<float, 2> startOffset{0.f, 0.f};    // subtracted from XY to make the clip start-relative

    // Foot locking.
    int   lockedFrames = 0;    ///< frames on which some foot was pinned
    float lockMiss     = 0.f;  ///< the farthest a pinned foot stayed from its plant (the leg couldn't reach)
};

/// A source model-space position in the retargeted clip's frame (facing, scale, ground, start).
[[nodiscard]] std::array<float, 3> PlaceSourcePoint(const RetargetReport& report, const std::array<float, 3>& sourceModel);

/// Retarget `clip` (recorded on `source`) onto `target` using `map`.
/// Returns a new Clip keyed to `target`'s bones on success.
/// Unmapped target bones are left at their rest pose for all frames, but for `map.twist` and `map.between`.
[[nodiscard]] std::expected<Clip, std::string> RetargetClip(
    const Skeleton&        source,
    const Clip&            clip,
    const Skeleton&        target,
    const BoneMap&         map,
    const RetargetOptions& options,
    RetargetReport*        report = nullptr);

/// RetargetClip with the default options.
[[nodiscard]] std::expected<Clip, std::string> RetargetClip(
    const Skeleton& source,
    const Clip&     clip,
    const Skeleton& target,
    const BoneMap&  map,
    RetargetReport* report = nullptr);

/// The retarget pose offset (BoneMap::pose) that turns a bone, shown at model rotation `boneWorld` with `offset`, by
/// `turn` in model space: the bone then shows at turn * boneWorld. All quaternions x, y, z, w.
[[nodiscard]] std::array<float, 4> TurnPoseOffset(const std::array<float, 4>& offset, const std::array<float, 4>& boneWorld,
                                                  const std::array<float, 4>& turn);

} // namespace targeter
