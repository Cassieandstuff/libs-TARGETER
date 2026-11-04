#pragma once
// FootPlants — when each foot stands on the ground in a clip, and where. Ported from Dance Card Creator's autotag: a foot
// is planted while it is low and slow, slow growing with the root's speed because at a run the ankle keeps rolling through
// a contact.

#include <targeter/Clip.h>
#include <targeter/Skeleton.h>

#include <array>
#include <string>
#include <vector>

namespace targeter {

/// @brief The bones measured, byte-exact. A clip's track for a bone is matched by name; a bone with none holds its rest.
struct PlantBones {
    std::string root, leftFoot, rightFoot;
};

/// @brief Game units, seconds and frames.
struct PlantThresholds {
    double plantHeight     = 3.0;    ///< a foot at most this far above its lowest point in the clip
    double plantSpeed      = 25.0;   ///< and moving at most this fast (units/s)
    double plantSpeedScale = 0.25;   ///< or this fraction of the root's speed, whichever is more
    int    minFrames       = 2;      ///< shorter runs are dropped: a sprint contact is about 3 frames
    int    gapFrames       = 2;      ///< gaps up to this long inside a run are filled, and at the clip's edges while the foot stays low
    int    rootSmoothHalf  = 3;      ///< the root's speed: a moving average over this many frames each side
    int    footSmoothHalf  = 1;      ///< the foot's speed: the same, kept short so short plants survive
};

struct FootPlants {
    bool hasFeet = false;   ///< false when the skeleton lacks a foot: everything else is empty
    /// @brief [left, right]: each plant as [first, last) frames.
    std::array<std::vector<FrameRange>, 2> ranges;
    /// @brief [left, right]: the foot's model-space position per frame, root travel included.
    std::array<std::vector<std::array<float, 3>>, 2> foot;
    /// @brief The root's model-space position per frame.
    std::vector<std::array<float, 3>> root;
};

/// @brief Measure the feet of `clip` on `skeleton` and detect their plants.
[[nodiscard]] FootPlants DetectFootPlants(const Clip& clip, const Skeleton& skeleton, const PlantBones& bones, const PlantThresholds& thresholds = {});

/// @brief The foot's mean ground position over `range`: where a plant stands.
[[nodiscard]] std::array<float, 2> PlantCenter(const std::vector<std::array<float, 3>>& foot, FrameRange range);

} // namespace targeter
