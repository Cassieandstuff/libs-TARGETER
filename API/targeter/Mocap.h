#pragma once
// Mocap — optical-marker capture (positions only, no joint rotations) onto a target skeleton.
// A MocapLayout turns a marker cloud into a posed SOURCE skeleton: virtual markers (weighted sums of markers, e.g.
// joint centres) are synthesised, then each source bone is oriented by aiming it from its origin marker at its aim
// marker and rolling it toward a roll marker or the plane of a hinge marker. A MocapMap names the target bone each
// source bone drives; RetargetMocap applies each source bone's world rotation since the rest frame onto that target
// bone's rest, keeps the target's own bone lengths, and puts the travel and heading on the target's root.
// Approach after Scene Editor's FreeMoCap path (sct-pipeline Mocap / MocapRetargeter), reimplemented over this
// library's data model. Marker and bone names are matched byte-exact.

#include <targeter/Clip.h>
#include <targeter/Skeleton.h>

#include <array>
#include <expected>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace targeter {

/// Marker trajectories. `points` is frame-major: `points[frame * markers.size() + marker]`, NaN x = not seen.
struct MarkerTake {
    std::string                       name;
    float                             frameDuration = 1.f / 240.f;
    int                               numFrames     = 0;
    std::vector<std::string>          markers;
    std::vector<std::array<float, 3>> points;

    [[nodiscard]] const std::array<float, 3>& At(int frame, int marker) const {
        return points[std::size_t(frame) * markers.size() + std::size_t(marker)];
    }
    /// @return the marker's index, or -1.
    [[nodiscard]] int MarkerIndex(std::string_view marker) const;
    bool operator==(const MarkerTake&) const = default;
};

/// A marker synthesised as the weighted sum of others (markers or earlier virtuals), by name.
struct VirtualMarker {
    std::string                                name;
    std::vector<std::pair<std::string, float>> components;   ///< weights should sum to 1
    bool operator==(const VirtualMarker&) const = default;
};

/// One source bone: placed at `origin`, aimed at `aim`, its twist from `hinge` (the normal of the plane through
/// origin, aim and hinge) or else from `roll` (leaned toward that marker); neither = free twist.
struct MocapBone {
    std::string name;
    std::string parent;   ///< "" = a root; parents precede their children
    std::string origin, aim, roll, hinge;
    bool operator==(const MocapBone&) const = default;
};

/// A capture's source skeleton, defined over its marker names.
struct MocapLayout {
    std::string                name;
    std::vector<VirtualMarker> virtuals;
    std::vector<MocapBone>     bones;
    float                      unitScale = 1.f;                     ///< capture units -> game units
    std::array<float, 4>       captureRotation{0.f, 0.f, 0.f, 1.f}; ///< capture axes -> Z up (xyzw)

    /// @return every marker the layout reads that it does not synthesise.
    [[nodiscard]] std::vector<std::string> RealMarkers() const;
    bool operator==(const MocapLayout&) const = default;
};

/// One source bone driving one target bone. `targetAim` is the target bone's aim child; with `alignRest`, the
/// target's rest is turned so the bone points the way the source bone pointed in the rest frame.
struct MocapPair {
    std::string source, target, targetAim;
    bool        alignRest = false;
    bool operator==(const MocapPair&) const = default;
};

/// Source bones onto a target skeleton's bones.
struct MocapMap {
    std::string            name;
    std::vector<MocapPair> pairs;
    std::string            root;           ///< target bone that carries the travel and heading (the skeleton's root)
    std::string            anchor;         ///< target bone that carries the body's height
    std::string            anchorSource;   ///< source bone whose origin travels and whose aim gives the heading
    bool operator==(const MocapMap&) const = default;
};

struct MocapRetargetOptions {
    float frameDuration = 1.f / 30.f;
    float start         = 0.f;   ///< seconds into the take
    float end           = 0.f;   ///< seconds into the take; 0 = its end
    float scale         = 0.f;   ///< target size / source size; 0 = the anchor's rest height ratio
    float maxGap        = 0.25f; ///< seconds of missing marker filled by interpolation
    /// The take whose frame is the rest (the source pose the target's rest stands for); null = the take itself.
    const MarkerTake* restTake  = nullptr;
    int               restFrame = -1;   ///< frame of the rest take; -1 = the frame closest to the target's rest
    std::string       markerPrefix;    ///< prepended to the layout's marker names (a capture's subject id)
};

struct MocapReport {
    int                      mappedBones = 0;
    int                      restFrame   = -1;
    float                    scale       = 1.f;
    std::vector<std::string> missing;   ///< layout markers, source bones or target bones not found
};

/// The prefix (e.g. a subject id "ID2_") before the layout's marker names in `take`; "" when they match bare.
[[nodiscard]] std::string MarkerPrefix(const MarkerTake& take, const MocapLayout& layout);

/// `take` resampled to `frameDuration` (box-filtered over each output frame), gaps up to `maxGap` seconds filled.
[[nodiscard]] MarkerTake ResampleTake(const MarkerTake& take, float frameDuration, float maxGap);

/// The layout's source pose at one frame: model-space transforms, index-parallel to `layout.bones`. A bone whose
/// markers are missing is reported in `valid` (when given) and keeps its parent's orientation.
[[nodiscard]] std::vector<Transform> SourcePose(const MocapLayout& layout, const MarkerTake& take, int frame,
                                                std::string_view markerPrefix = {},
                                                std::vector<char>* valid = nullptr);

/// Retarget `take` onto `target` through `layout` and `map`: a Clip on every target bone.
[[nodiscard]] std::expected<Clip, std::string> RetargetMocap(const MocapLayout&          layout,
                                                             const MocapMap&             map,
                                                             const MarkerTake&           take,
                                                             const Skeleton&             target,
                                                             const MocapRetargetOptions& options = {},
                                                             MocapReport*                report  = nullptr);

} // namespace targeter
