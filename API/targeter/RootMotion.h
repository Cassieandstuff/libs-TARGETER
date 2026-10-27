#pragma once
// RootMotion — operations on the root bone's world-space trajectory.

#include <targeter/Clip.h>
#include <targeter/Skeleton.h>

#include <string_view>

namespace targeter {

/// Remove vertical drift and lateral translation from the root bone so the
/// character stays grounded at the origin for the full clip.  Modifies `clip`
/// in place.  `rootBone` defaults to "Root" if empty.
void GroundRoot(Clip& clip, const Skeleton& skeleton, std::string_view rootBone = "Root");

} // namespace targeter
