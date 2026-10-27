#include <targeter/AutoTag.h>
#include "MathOps.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace targeter {

namespace {
using detail::Vec3;
using detail::Quat;
using detail::QRotate;
using detail::Normalize;
using detail::Len;
using detail::YawOf;

[[nodiscard]] Vec3 BoneWorldPos(const Clip& clip, int frame, const Skeleton& skel, int bi) {
    if (bi < 0) return {0.f, 0.f, 0.f};
    std::vector<Transform> local(skel.bones.size());
    for (int i = 0; i < static_cast<int>(skel.bones.size()); ++i) {
        int ci = clip.BoneIndex(skel.bones[i].name);
        local[i] = (ci >= 0) ? clip.At(frame, ci) : skel.bones[i].rest;
    }
    auto ms = ComputeModelSpace(skel, local);
    return {ms[bi].t[0], ms[bi].t[1], ms[bi].t[2]};
}

// Build ranges for a boolean predicate applied per-frame.
[[nodiscard]] std::vector<FrameRange> BuildRanges(int numFrames,
                                                    const std::vector<bool>& active) {
    std::vector<FrameRange> ranges;
    bool inRange = false;
    int start = 0;
    for (int f = 0; f < numFrames; ++f) {
        if (active[f] && !inRange) { start = f; inRange = true; }
        else if (!active[f] && inRange) { ranges.push_back({start, f}); inRange = false; }
    }
    if (inRange) ranges.push_back({start, numFrames});
    return ranges;
}
} // namespace

ClipMeasure MeasureClip(const Clip& clip, const Skeleton& skeleton,
                         const AutoTagBones& bones) {
    ClipMeasure m;
    if (clip.numFrames < 2) return m;

    int rootIdx  = skeleton.BoneIndex(bones.root);
    int headIdx  = skeleton.BoneIndex(bones.head);
    int footLIdx = skeleton.BoneIndex(bones.footL);
    int footRIdx = skeleton.BoneIndex(bones.footR);
    int handLIdx = skeleton.BoneIndex(bones.handL);
    int handRIdx = skeleton.BoneIndex(bones.handR);
    int spineIdx = skeleton.BoneIndex(bones.spine);

    float dt  = clip.frameDuration;
    float sumSpeed = 0.f;
    float sumHead  = 0.f;

    for (int f = 0; f < clip.numFrames; ++f) {
        if (headIdx >= 0) {
            float h = BoneWorldPos(clip, f, skeleton, headIdx)[2];
            sumHead += h;
            m.minHeadHeight = std::min(m.minHeadHeight, h);
        }
        if (handLIdx >= 0 || handRIdx >= 0) {
            float hL = (handLIdx >= 0) ? BoneWorldPos(clip, f, skeleton, handLIdx)[2] : 0.f;
            float hR = (handRIdx >= 0) ? BoneWorldPos(clip, f, skeleton, handRIdx)[2] : 0.f;
            m.maxHandHeight = std::max(m.maxHandHeight, std::max(hL, hR));
        }
        if (footLIdx >= 0 || footRIdx >= 0) {
            float fL = (footLIdx >= 0) ? BoneWorldPos(clip, f, skeleton, footLIdx)[2] : 0.f;
            float fR = (footRIdx >= 0) ? BoneWorldPos(clip, f, skeleton, footRIdx)[2] : 0.f;
            m.maxFootLift = std::max(m.maxFootLift, std::max(fL, fR));
        }

        if (f > 0 && rootIdx >= 0) {
            Vec3 prev = BoneWorldPos(clip, f-1, skeleton, rootIdx);
            Vec3 curr = BoneWorldPos(clip, f,   skeleton, rootIdx);
            Vec3 d    = {curr[0]-prev[0], curr[1]-prev[1], 0.f};
            float speed = Len(d) / dt;
            m.maxRootSpeed = std::max(m.maxRootSpeed, speed);
            sumSpeed += speed;

            // Yaw delta
            float yPrev = YawOf(Normalize({d[0], d[1], 0.f}));
            (void)yPrev; // yaw delta computed pairwise below
        }
    }

    m.avgRootSpeed = (clip.numFrames > 1) ? sumSpeed / float(clip.numFrames - 1) : 0.f;
    m.avgHeadHeight = (clip.numFrames > 0 && headIdx >= 0)
                      ? sumHead / float(clip.numFrames) : 0.f;

    // Yaw deltas
    for (int f = 1; f < clip.numFrames && rootIdx >= 0; ++f) {
        Vec3 prev = BoneWorldPos(clip, f-1, skeleton, rootIdx);
        Vec3 curr = BoneWorldPos(clip, f,   skeleton, rootIdx);
        Vec3 d0   = {curr[0]-prev[0], curr[1]-prev[1], 0.f};
        if (Len(d0) > 0.1f) {
            float yNow  = YawOf(Normalize(d0));
            Vec3 prev2  = (f > 1) ? BoneWorldPos(clip, f-2, skeleton, rootIdx) : prev;
            Vec3 d1     = {prev[0]-prev2[0], prev[1]-prev2[1], 0.f};
            if (Len(d1) > 0.1f) {
                float yPrev = YawOf(Normalize(d1));
                float diff  = yNow - yPrev;
                while (diff >  std::numbers::pi_v<float>) diff -= 2.f*std::numbers::pi_v<float>;
                while (diff < -std::numbers::pi_v<float>) diff += 2.f*std::numbers::pi_v<float>;
                m.maxRootYawDelta = std::max(m.maxRootYawDelta, std::fabs(diff));
            }
        }
    }

    return m;
}

void AutoTag(Clip& clip, const Skeleton& skeleton,
             const AutoTagBones& bones, const AutoTagThresholds& thresh) {
    if (clip.numFrames < 2) return;

    int rootIdx = skeleton.BoneIndex(bones.root);
    float dt    = clip.frameDuration;

    std::vector<bool> isTurnLeft(clip.numFrames, false);
    std::vector<bool> isTurnRight(clip.numFrames, false);
    std::vector<bool> isForward(clip.numFrames, false);
    std::vector<bool> isBackward(clip.numFrames, false);
    std::vector<bool> isLeft(clip.numFrames, false);
    std::vector<bool> isRight(clip.numFrames, false);
    std::vector<bool> isStrafing(clip.numFrames, false);
    std::vector<bool> isCrouching(clip.numFrames, false);
    std::vector<bool> isJumping(clip.numFrames, false);

    int headIdx  = skeleton.BoneIndex(bones.head);
    int footLIdx = skeleton.BoneIndex(bones.footL);
    int footRIdx = skeleton.BoneIndex(bones.footR);

    for (int f = 0; f < clip.numFrames; ++f) {
        // Crouch / jump
        if (headIdx >= 0) {
            float h = BoneWorldPos(clip, f, skeleton, headIdx)[2];
            isCrouching[f] = h < thresh.crouchHeight;
        }
        if (footLIdx >= 0 || footRIdx >= 0) {
            float fL = (footLIdx >= 0) ? BoneWorldPos(clip, f, skeleton, footLIdx)[2] : 0.f;
            float fR = (footRIdx >= 0) ? BoneWorldPos(clip, f, skeleton, footRIdx)[2] : 0.f;
            isJumping[f] = std::max(fL, fR) > thresh.jumpLift;
        }

        if (f == 0 || rootIdx < 0) continue;
        Vec3 prev = BoneWorldPos(clip, f-1, skeleton, rootIdx);
        Vec3 curr = BoneWorldPos(clip, f,   skeleton, rootIdx);
        Vec3 d    = {curr[0]-prev[0], curr[1]-prev[1], 0.f};
        float speed = Len(d) / dt;
        if (speed < 1.f) continue;

        Vec3 dir = Normalize(d);
        // +Y is forward; angle from +Y
        float angle = std::atan2(-dir[0], dir[1]); // 0 = fwd, ±pi = back

        bool fwd  = std::fabs(angle) < thresh.strafeAngle;
        bool back = std::fabs(angle) > (std::numbers::pi_v<float> - thresh.strafeAngle);
        bool left  = angle > thresh.strafeAngle && angle < std::numbers::pi_v<float> - thresh.strafeAngle;
        bool right = angle < -thresh.strafeAngle && angle > -(std::numbers::pi_v<float> - thresh.strafeAngle);

        isForward[f]   = fwd;
        isBackward[f]  = back;
        isLeft[f]      = left;
        isRight[f]     = right;
        isStrafing[f]  = left || right;

        // Turn: computed from yaw delta
        if (f > 1) {
            Vec3 prev2 = BoneWorldPos(clip, f-2, skeleton, rootIdx);
            Vec3 d1    = {prev[0]-prev2[0], prev[1]-prev2[1], 0.f};
            if (Len(d1) > 0.1f && Len(d) > 0.1f) {
                float yNow  = YawOf(Normalize(d));
                float yPrev = YawOf(Normalize(d1));
                float diff  = yNow - yPrev;
                while (diff >  std::numbers::pi_v<float>) diff -= 2.f*std::numbers::pi_v<float>;
                while (diff < -std::numbers::pi_v<float>) diff += 2.f*std::numbers::pi_v<float>;
                if (diff > thresh.yawThreshold)      isTurnLeft[f]  = true;
                else if (diff < -thresh.yawThreshold) isTurnRight[f] = true;
            }
        }
    }

    auto merge = [&](std::string name, const std::vector<bool>& active) {
        auto ranges = BuildRanges(clip.numFrames, active);
        if (!ranges.empty()) {
            auto& existing = clip.tags[name];
            existing.insert(existing.end(), ranges.begin(), ranges.end());
        }
    };

    merge("IsTurnLeft",  isTurnLeft);
    merge("IsTurnRight", isTurnRight);
    merge("IsForward",   isForward);
    merge("IsBackward",  isBackward);
    merge("IsLeft",      isLeft);
    merge("IsRight",     isRight);
    merge("IsStrafing",  isStrafing);
    merge("IsCrouching", isCrouching);
    merge("IsJumping",   isJumping);

    NormalizeRanges(clip.tags);
}

} // namespace targeter
