#pragma once
// RetargetClip — rest-aligned world-delta retargeting.
// Algorithm: align source rest to target rest, then for each frame compute the
// world-space rotation delta from source rest and apply it to the target bone.
// Yaw of the root is extracted and applied to the target root to preserve facing.

#include <targeter/BoneMap.h>
#include <targeter/Clip.h>
#include <targeter/Skeleton.h>

#include <array>
#include <expected>
#include <string>
#include <vector>

namespace targeter {

struct RetargetReport {
    int mappedBones   = 0;
    int unmappedBones = 0;   // target bones with no source mapping (left at rest)
    std::vector<std::string> missingSource;   // map entries naming bones the source lacks

    // How source positions were placed into the target's frame, for drawing the source beside the result.
    float                scale = 1.f;              // target hip height / source hip height
    std::array<float, 4> facing{0.f, 0.f, 0.f, 1.f};   // x, y, z, w
    float                sourceGround = 0.f, targetGround = 0.f;
    std::array<float, 2> startOffset{0.f, 0.f};    // subtracted from XY to make the clip start-relative
};

/// A source model-space position in the retargeted clip's frame (facing, scale, ground, start).
[[nodiscard]] std::array<float, 3> PlaceSourcePoint(const RetargetReport& report, const std::array<float, 3>& sourceModel);

/// Retarget `clip` (recorded on `source`) onto `target` using `map`.
/// Returns a new Clip keyed to `target`'s bones on success.
/// Unmapped target bones are left at their rest pose for all frames.
[[nodiscard]] std::expected<Clip, std::string> RetargetClip(
    const Skeleton& source,
    const Clip&     clip,
    const Skeleton& target,
    const BoneMap&  map,
    RetargetReport* report = nullptr);

} // namespace targeter
