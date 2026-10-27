#pragma once
// RetargetClip — rest-aligned world-delta retargeting.
// Algorithm: align source rest to target rest, then for each frame compute the
// world-space rotation delta from source rest and apply it to the target bone.
// Yaw of the root is extracted and applied to the target root to preserve facing.

#include <targeter/BoneMap.h>
#include <targeter/Clip.h>
#include <targeter/Skeleton.h>

#include <expected>
#include <string>

namespace targeter {

struct RetargetReport {
    int mappedBones   = 0;
    int unmappedBones = 0;   // target bones with no source mapping (left at rest)
};

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
