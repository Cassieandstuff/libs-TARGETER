#pragma once
// AutoTag — CB-compatible motion analysis and tag seeding.
// Ports Community Behaviors' `hky-tool autotag` thresholds so tags authored
// here match what CB seeds from compiled clips.

#include <targeter/Clip.h>
#include <targeter/Skeleton.h>

#include <array>
#include <string_view>

namespace targeter {

/// The 14 tag names that AutoTag can write (CB canonical ordering).
inline constexpr std::array<std::string_view, 14> kAutoTagNames = {
    "IsBlocking",   "IsAttacking",  "IsEquipping",  "IsUnequipping",
    "IsMounted",    "IsTurnLeft",   "IsTurnRight",  "IsForward",
    "IsBackward",   "IsLeft",       "IsRight",      "IsStrafing",
    "IsCrouching",  "IsJumping",
};

/// Per-metric scalar measurements extracted from a single clip.
struct ClipMeasure {
    float maxRootSpeed     = 0.f;
    float avgRootSpeed     = 0.f;
    float maxRootYawDelta  = 0.f;   // radians/frame
    float maxSpineYaw      = 0.f;   // radians from rest
    float avgHeadHeight    = 0.f;
    float minHeadHeight    = 0.f;
    float maxHandHeight    = 0.f;
    float maxFootLift      = 0.f;
    bool  hasHorizontalAim = false;
};

/// Velocity/height thresholds that drive AutoTag decisions.
struct AutoTagThresholds {
    float walkSpeed       = 80.f;   // Skyrim units/s
    float runSpeed        = 200.f;
    float strafeAngle     = 0.35f;  // radians — side vs. forward bias
    float yawThreshold    = 0.08f;  // radians/frame — counts as a turn
    float crouchHeight    = 90.f;   // head height below which = crouch
    float jumpLift        = 15.f;   // foot lift above which = jump
};

/// Bone names used during measurement.
struct AutoTagBones {
    std::string_view root  = "Root";
    std::string_view hips  = "Hips";
    std::string_view spine = "Spine1";
    std::string_view head  = "Head";
    std::string_view handL = "NPC L Hand [LHnd]";
    std::string_view handR = "NPC R Hand [RHnd]";
    std::string_view footL = "NPC L Foot [Lft ]";
    std::string_view footR = "NPC R Foot [Rft ]";
};

/// Compute per-frame measurements for a clip.
[[nodiscard]] ClipMeasure MeasureClip(const Clip&     clip,
                                       const Skeleton& skeleton,
                                       const AutoTagBones& bones = {});

/// Seed `clip.tags` from measurements.  Existing tags are not removed.
void AutoTag(Clip&               clip,
             const Skeleton&     skeleton,
             const AutoTagBones& bones      = {},
             const AutoTagThresholds& thresh = {});

} // namespace targeter
