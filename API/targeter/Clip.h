#pragma once
// Clip — uniformly sampled skeletal animation: every bone keyed every frame,
// transforms parent-local (Havok QsTransform layout: translation, rotation xyzw, scale).
// Axis convention: Z up, character faces +Y, Skyrim game units. Root motion lives
// on the root bone's track; nothing here knows about animationdata.

#include <targeter/Transform.h>

#include <array>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace targeter {

/// A frame interval [first, last) — end-exclusive.
using FrameRange = std::pair<int, int>;
/// Tag name -> its frame intervals. Names are compared byte-exact (CB matches them that way).
using RangeMap = std::map<std::string, std::vector<FrameRange>>;

struct Annotation {
    float       time = 0.f;
    std::string text;
    bool operator==(const Annotation&) const = default;
};

/// One hkaAnnotationTrack, kept verbatim (AMR `animmotion`, `FootLeft`, `HitFrame`, …).
struct AnnotationTrack {
    std::string             name;
    std::vector<Annotation> annotations;
    bool operator==(const AnnotationTrack&) const = default;
};

/// A per-frame scalar curve (one value per clip frame).
struct FloatTrack {
    std::string        name;
    std::vector<float> values;
    bool operator==(const FloatTrack&) const = default;
};

/// Uniformly sampled skeletal animation. `poses` is frame-major:
/// `poses[frame * bones.size() + bone]` is the local transform for that bone and frame.
struct Clip {
    std::string              name;
    std::string              skeleton;         // root bone name (CB's `skeleton:` field)
    float                    frameDuration = 1.f / 30.f;
    int                      numFrames     = 0;
    std::vector<std::string> bones;            // track order, index-parallel to `poses`
    std::vector<Transform>   poses;            // frame-major: [frame * bones.size() + bone]
    std::vector<FloatTrack>      floatTracks;
    std::vector<AnnotationTrack> annotationTracks;
    RangeMap                 tags;

    [[nodiscard]] float Duration() const {
        return numFrames > 1 ? float(numFrames - 1) * frameDuration : 0.f;
    }
    [[nodiscard]] Transform& At(int frame, int bone) {
        return poses[std::size_t(frame) * bones.size() + std::size_t(bone)];
    }
    [[nodiscard]] const Transform& At(int frame, int bone) const {
        return poses[std::size_t(frame) * bones.size() + std::size_t(bone)];
    }
    /// @return the bone's track index, or -1.
    [[nodiscard]] int BoneIndex(std::string_view bone) const;
    /// Size `poses` for `numFrames` x `bones.size()`, filling with identity.
    void Resize(int frames);

    bool operator==(const Clip&) const = default;
};

/// Sort each tag's ranges and merge overlapping/adjacent ones; drops empty ranges.
void NormalizeRanges(RangeMap& tags);

} // namespace targeter
