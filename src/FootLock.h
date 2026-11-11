#pragma once
// FootLock — pin planted feet on a retargeted clip with analytic two-bone leg IK. Private to the library: RetargetClip
// runs it when RetargetOptions::lockFeet is set. After Dance Card Creator's FootLock (src/anim/src/FootLock.cpp), whose
// leg solve comes from SCT's IkSolver (sct-pipeline anim/IkSolver.h, SolveTwoBoneIk), reimplemented over this library's
// math.

#include <targeter/Clip.h>
#include <targeter/Skeleton.h>

#include <array>
#include <vector>

namespace targeter::detail {

struct LockResult {
    int   lockedFrames = 0;   ///< frames on which some foot was pinned
    float miss         = 0.f; ///< the farthest a pinned foot stayed from its goal
};

/// @brief Pin `clip`'s feet (target bone indices; each foot's parent is its calf, the calf's its thigh) to where each plant
///        of `plants` ([left, right], [first, last) frames) begins, blending in and out over `blend` frames. `clip`'s tracks
///        are index-parallel to `skeleton`. Only the thigh, calf and foot tracks change; the foot keeps its model rotation.
LockResult LockFeet(Clip& clip, const Skeleton& skeleton, const std::array<int, 2>& feet,
                    const std::array<std::vector<FrameRange>, 2>& plants, int blend);

} // namespace targeter::detail
