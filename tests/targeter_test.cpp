#include <targeter/Targeter.h>
#include "MathOps.h"

#include <algorithm>
#include <cassert>
#include <initializer_list>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>

// ── helpers ───────────────────────────────────────────────────────────────────

static bool Near(float a, float b, float eps = 1e-4f) {
    return std::fabs(a - b) < eps;
}

static bool NearVec3(const std::array<float,3>& a, const std::array<float,3>& b, float eps = 1e-4f) {
    return Near(a[0], b[0], eps) && Near(a[1], b[1], eps) && Near(a[2], b[2], eps);
}

static bool NearQuat(const std::array<float,4>& a, const std::array<float,4>& b, float eps = 1e-4f) {
    // Quaternions are equivalent under sign flip
    bool pos = Near(a[0], b[0], eps) && Near(a[1], b[1], eps) &&
               Near(a[2], b[2], eps) && Near(a[3], b[3], eps);
    bool neg = Near(a[0],-b[0], eps) && Near(a[1],-b[1], eps) &&
               Near(a[2],-b[2], eps) && Near(a[3],-b[3], eps);
    return pos || neg;
}

static void Pass(const char* name) { std::printf("[PASS] %s\n", name); }
static int  g_failures = 0;
static void Fail(const char* name) { std::printf("[FAIL] %s\n", name); ++g_failures; }

#define CHECK(cond, name) do { if (!(cond)) Fail(name); else Pass(name); } while(0)

// ── Transform default ─────────────────────────────────────────────────────────

static void TestTransformDefault() {
    targeter::Transform t;
    CHECK(NearVec3(t.t, {0,0,0}), "Transform::t default");
    CHECK(NearQuat(t.r, {0,0,0,1}), "Transform::r default");
    CHECK(NearVec3(t.s, {1,1,1}), "Transform::s default");
}

// ── Compose identity ──────────────────────────────────────────────────────────

static void TestComposeIdentity() {
    targeter::Transform identity;
    targeter::Transform a;
    a.t = {1.f, 2.f, 3.f};

    // identity * a = a
    auto result = targeter::Compose(identity, a);
    CHECK(NearVec3(result.t, a.t, 1e-4f), "Compose(identity, a).t == a.t");
    CHECK(NearQuat(result.r, a.r, 1e-4f), "Compose(identity, a).r == a.r");

    // a * identity = a
    auto result2 = targeter::Compose(a, identity);
    CHECK(NearVec3(result2.t, a.t, 1e-4f), "Compose(a, identity).t == a.t");
}

// ── ComputeModelSpace: simple two-bone chain ─────────────────────────────────

static void TestComputeModelSpace() {
    targeter::Skeleton skel;
    skel.name = "test";

    targeter::SkeletonBone root;
    root.name   = "Root";
    root.parent = -1;
    root.rest.t = {0.f, 0.f, 0.f};
    skel.bones.push_back(root);

    targeter::SkeletonBone child;
    child.name   = "Child";
    child.parent = 0;
    child.rest.t = {0.f, 1.f, 0.f};
    skel.bones.push_back(child);

    auto local = skel.RestLocal();
    auto ms    = targeter::ComputeModelSpace(skel, local);

    CHECK(ms.size() == 2, "ComputeModelSpace size");
    CHECK(NearVec3(ms[0].t, {0,0,0}, 1e-4f), "Root model-space t == {0,0,0}");
    CHECK(NearVec3(ms[1].t, {0,1,0}, 1e-4f), "Child model-space t == {0,1,0}");
}

// ── Clip resize and BoneIndex ─────────────────────────────────────────────────

static void TestClip() {
    targeter::Clip clip;
    clip.bones = {"Root", "Hips", "Spine"};
    clip.Resize(10);

    CHECK(clip.numFrames == 10, "Clip::numFrames after Resize");
    CHECK(clip.poses.size() == 30, "Clip::poses.size() == 30");

    CHECK(clip.BoneIndex("Root")  == 0, "BoneIndex Root");
    CHECK(clip.BoneIndex("Hips")  == 1, "BoneIndex Hips");
    CHECK(clip.BoneIndex("Spine") == 2, "BoneIndex Spine");
    CHECK(clip.BoneIndex("None")  == -1, "BoneIndex missing == -1");
}

// ── NormalizeRanges ───────────────────────────────────────────────────────────

static void TestNormalizeRanges() {
    targeter::RangeMap tags;
    tags["IsForward"] = {{0,5},{3,8},{10,12}};
    targeter::NormalizeRanges(tags);
    auto& r = tags["IsForward"];
    CHECK(r.size() == 2, "NormalizeRanges merges overlapping: count == 2");
    CHECK(r[0].first == 0 && r[0].second == 8, "NormalizeRanges merged range [0,8)");
    CHECK(r[1].first == 10 && r[1].second == 12, "NormalizeRanges isolated range [10,12)");
}

// ── BoneMap::SourceOf ─────────────────────────────────────────────────────────

static void TestBoneMap() {
    targeter::BoneMap map;
    map.bones["Hips"] = "pelvis";
    map.bones["Spine1"] = "spine1";

    CHECK(map.SourceOf("Hips")   == "pelvis", "BoneMap SourceOf Hips");
    CHECK(map.SourceOf("Spine1") == "spine1", "BoneMap SourceOf Spine1");
    CHECK(map.SourceOf("Head")   == "",        "BoneMap SourceOf unmapped == empty");
}

// ── KimodoSkeletons ───────────────────────────────────────────────────────────

static void TestKimodoSkeletons() {
    using namespace targeter::kimskel;

    CHECK(smplx22_spec.joints() == 22, "smplx22 joint count");
    CHECK(soma30_spec.joints()  == 30, "soma30 joint count");
    CHECK(g1skel34_spec.joints() == 34, "g1skel34 joint count");

    CHECK(find_skeleton("smplx22")  == &smplx22_spec,  "find_skeleton smplx22");
    CHECK(find_skeleton("soma30")   == &soma30_spec,    "find_skeleton soma30");
    CHECK(find_skeleton("g1skel34") == &g1skel34_spec,  "find_skeleton g1skel34");
    CHECK(find_skeleton("unknown")  == nullptr,         "find_skeleton unknown == null");

    CHECK(find_by_joints(22) == &smplx22_spec,  "find_by_joints 22");
    CHECK(find_by_joints(30) == &soma30_spec,   "find_by_joints 30");
    CHECK(find_by_joints(34) == &g1skel34_spec, "find_by_joints 34");
    CHECK(find_by_joints(99) == nullptr,        "find_by_joints unknown == null");

    // Parents array size matches joint count
    CHECK(smplx22_spec.parents.size()  == 22, "smplx22 parents.size");
    CHECK(soma30_spec.parents.size()   == 30, "soma30 parents.size");
    CHECK(g1skel34_spec.parents.size() == 34, "g1skel34 parents.size");

    // All specs: root has parent -1
    CHECK(smplx22_spec.parents[0]  == -1, "smplx22 root parent == -1");
    CHECK(soma30_spec.parents[0]   == -1, "soma30 root parent == -1");
    CHECK(g1skel34_spec.parents[0] == -1, "g1skel34 root parent == -1");
}

// ── RetargetClip trivial (same skeleton, identity map) ────────────────────────

static targeter::Skeleton MakeSimpleSkeleton(const std::string& name) {
    targeter::Skeleton skel;
    skel.name = name;
    {
        targeter::SkeletonBone root;
        root.name = "Root"; root.parent = -1;
        skel.bones.push_back(root);
    }
    {
        targeter::SkeletonBone hips;
        hips.name = "Hips"; hips.parent = 0;
        hips.rest.t = {0.f, 0.f, 1.f};
        skel.bones.push_back(hips);
    }
    return skel;
}

static void TestRetargetIdentity() {
    auto skel = MakeSimpleSkeleton("src");
    auto tgt  = MakeSimpleSkeleton("tgt");

    targeter::BoneMap map;
    map.sourceRoot      = "Root";
    map.sourceHips      = "Hips";
    map.sourceLeftHip   = "Hips";  // degenerate but valid for this test
    map.sourceRightHip  = "Hips";
    map.targetRoot      = "Root";
    map.targetHips      = "Hips";
    map.bones["Hips"]   = "Hips";

    targeter::Clip clip;
    clip.name = "test";
    clip.bones = {"Root", "Hips"};
    clip.numFrames = 3;
    clip.Resize(3);
    // All frames at rest

    targeter::RetargetReport report;
    auto result = targeter::RetargetClip(skel, clip, tgt, map, &report);
    CHECK(result.has_value(), "RetargetClip returns success");
    CHECK(result->numFrames == 3, "RetargetClip preserves frame count");
    CHECK(report.mappedBones >= 1, "RetargetClip mapped at least 1 bone");
}

// ── Mocap ─────────────────────────────────────────────────────────────────────

static void TestResampleTake() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    targeter::MarkerTake take;
    take.markers       = {"m"};
    take.frameDuration = 1.f / 240.f;
    take.numFrames     = 241;
    for (int f = 0; f < take.numFrames; ++f) take.points.push_back({float(f), 0.f, 0.f});
    for (int f = 100; f < 120; ++f) take.points[std::size_t(f)] = {nan, nan, nan};
    auto out = targeter::ResampleTake(take, 1.f / 30.f, 0.25f);
    CHECK(out.numFrames == 31, "ResampleTake frame count");
    bool line = true;
    for (int k = 1; k < 30; ++k) line = line && Near(out.At(k, 0)[0], float(k * 8), 1e-3f);
    CHECK(line, "ResampleTake follows the line through a filled gap");
}

static void TestMarkerPrefix() {
    targeter::MocapLayout layout;
    layout.bones = {{"b", "", "a", "c", ""}};
    targeter::MarkerTake take;
    take.markers = {"ID2_a", "ID2_c", "ID2_other"};
    CHECK(targeter::MarkerPrefix(take, layout) == "ID2_", "MarkerPrefix finds the subject prefix");
}

static void TestRetargetMocapSwing() {
    // Target: a body with one hanging leg. Source: markers on the body and the leg; the leg swings forward.
    targeter::Skeleton tgt;
    tgt.bones = {{"Root", -1, {}}, {"Body", 0, {}}, {"Leg", 1, {}}, {"Foot", 2, {}}};
    tgt.bones[1].rest.t = {0.f, 0.f, 100.f};
    tgt.bones[3].rest.t = {0.f, 0.f, -50.f};

    targeter::MocapLayout layout;
    layout.bones = {{"body", "", "back", "front", "right"}, {"leg", "body", "hip", "knee", "kneeSide"}};
    targeter::MocapMap map;
    map.root = "Root";
    map.anchor = "Body";
    map.anchorSource = "body";
    map.pairs = {{"body", "Body"}, {"leg", "Leg", "Foot"}};

    targeter::MarkerTake take;
    take.name          = "swing";
    take.markers       = {"back", "front", "right", "hip", "knee", "kneeSide"};
    take.frameDuration = 1.f / 30.f;
    take.numFrames     = 3;
    const float angles[] = {0.f, 0.5f, 1.f};
    for (float a : angles) {
        // Swing about +X: the knee goes forward (+Y) and up.
        const std::array<float, 3> knee{0.f, 40.f * std::sin(a), 100.f - 40.f * std::cos(a)};
        take.points.push_back({0.f, -10.f, 100.f});
        take.points.push_back({0.f, 10.f, 100.f});
        take.points.push_back({10.f, 0.f, 100.f});
        take.points.push_back({0.f, 0.f, 100.f});
        take.points.push_back(knee);
        take.points.push_back({10.f, knee[1], knee[2]});
    }
    targeter::MocapRetargetOptions opt;
    opt.restFrame = 0;
    opt.scale     = 1.f;
    targeter::MocapReport report;
    auto clip = targeter::RetargetMocap(layout, map, take, tgt, opt, &report);
    CHECK(clip.has_value(), "RetargetMocap returns success");
    CHECK(clip->numFrames == 3 && report.mappedBones == 2, "RetargetMocap keys every frame, maps both bones");
    bool follows = true;
    for (int f = 0; f < 3; ++f) {
        std::vector<targeter::Transform> local(clip->poses.begin() + f * 4, clip->poses.begin() + f * 4 + 4);
        const auto model = targeter::ComputeModelSpace(tgt, local);
        const std::array<float, 3> want{0.f, 50.f * std::sin(angles[f]), 100.f - 50.f * std::cos(angles[f])};
        follows = follows && NearVec3(model[3].t, want, 1e-2f);
    }
    CHECK(follows, "RetargetMocap swings the target leg as the markers do, at the target's length");
}

// ── main ──────────────────────────────────────────────────────────────────────

using namespace targeter;

/// @brief Two legged rig: root -> hips -> thigh L/R -> foot L/R, standing on z = 0, its left at -X.
static Skeleton LegsRig() {
    Skeleton s;
    s.name = "legs";
    const auto bone = [&](const char* name, int parent, std::array<float, 3> t) {
        SkeletonBone b;
        b.name   = name;
        b.parent = parent;
        b.rest.t = t;
        s.bones.push_back(b);
    };
    bone("Root", -1, {0, 0, 0});
    bone("Hips", 0, {0, 0, 100});
    bone("LThigh", 1, {-10, 0, 0});
    bone("LFoot", 2, {0, 0, -100});
    bone("RThigh", 1, {10, 0, 0});
    bone("RFoot", 4, {0, 0, -100});
    return s;
}

static BoneMap LegsMap() {
    BoneMap m;
    m.name = "legs";
    m.sourceRoot = "Root", m.sourceHips = "Hips", m.sourceLeftHip = "LThigh", m.sourceRightHip = "RThigh";
    m.targetRoot = "Root", m.targetHips = "Hips";
    for (const char* b : {"Root", "Hips", "LThigh", "LFoot", "RThigh", "RFoot"}) m.bones[b] = b;
    return m;
}

static Clip RestClip(const Skeleton& s, int frames) {
    Clip c;
    c.name = "rest";
    for (const auto& b : s.bones) c.bones.push_back(b.name);
    c.Resize(frames);
    for (int f = 0; f < frames; ++f)
        for (std::size_t b = 0; b < s.bones.size(); ++b) c.At(f, int(b)) = s.bones[b].rest;
    return c;
}

static void TestRetargetPoseOffset() {
    const Skeleton s = LegsRig();
    const Clip     c = RestClip(s, 2);
    BoneMap        m = LegsMap();
    RetargetReport plain;
    const auto     base = RetargetClip(s, c, s, m, &plain);
    CHECK(base.has_value(), "retarget without a pose");
    CHECK(std::abs(plain.scale - 1.f) < 1e-4f, "same rig scales by one");

    // Turning the left thigh 90 deg about X in its own frame swings the foot below it, and only that foot.
    const float k = std::sqrt(0.5f);
    m.pose["LThigh"] = {k, 0.f, 0.f, k};
    const auto posed = RetargetClip(s, c, s, m);
    CHECK(posed.has_value(), "retarget with a pose");
    std::vector<Transform> local(s.bones.size());
    for (std::size_t b = 0; b < s.bones.size(); ++b) local[b] = posed->At(1, int(b));
    const auto model = ComputeModelSpace(s, local);
    CHECK(std::abs(model[3].t[2] - 100.f) < 1e-2f && std::abs(std::abs(model[3].t[1]) - 100.f) < 1e-2f, "the offset swings the child");
    CHECK(std::abs(model[5].t[2]) < 1e-2f, "the other leg stays");
}

static void TestHipsDontAim() {
    // The source's spine rises straight off its hips; the target's starts well behind them, as Skyrim's does.
    const Skeleton source = [] {
        Skeleton s = LegsRig();
        SkeletonBone b;
        b.name = "Spine", b.parent = 1, b.rest.t = {0, 0, 10};
        s.bones.push_back(b);
        b.name = "Chest", b.parent = 6, b.rest.t = {0, 0, 20};
        s.bones.push_back(b);
        return s;
    }();
    Skeleton target = source;
    target.bones[6].rest.t = {0, -8, 6};
    BoneMap m = LegsMap();
    m.bones["Spine"] = "Spine";
    m.bones["Chest"] = "Chest";
    const auto out = RetargetClip(source, RestClip(source, 1), target, m);
    CHECK(out.has_value(), "retarget a spine that starts behind the hips");
    const auto& hips = out->At(0, target.BoneIndex("Hips"));
    CHECK(NearQuat(hips.r, {0, 0, 0, 1}), "the hips keep their rest rotation");
    std::vector<Transform> local(target.bones.size());
    for (std::size_t b = 0; b < target.bones.size(); ++b) local[b] = out->At(0, int(b));
    const auto model = ComputeModelSpace(target, local);
    CHECK(std::abs(model[6].t[1] + 8.f) < 1e-3f && std::abs(model[6].t[2] - 106.f) < 1e-3f, "the spine stays where the rig puts it");
}

static void TestPlaceSourcePoint() {
    const Skeleton s = LegsRig();
    RetargetReport r;
    CHECK(RetargetClip(s, RestClip(s, 1), s, LegsMap(), &r).has_value(), "retarget for placement");
    const auto p = PlaceSourcePoint(r, {3.f, 4.f, 50.f});
    CHECK(std::abs(p[0] - 3.f) < 1e-3f && std::abs(p[1] - 4.f) < 1e-3f && std::abs(p[2] - 50.f) < 1e-3f, "same rig places a point on itself");
}

static void TestFootPlants() {
    Skeleton s = LegsRig();
    Clip     c = RestClip(s, 30);
    // The left foot stands for frames 0-9, lifts and moves over 10-19, stands again 20-29; the right never leaves the floor.
    const int lfoot = c.BoneIndex("LFoot");
    for (int f = 0; f < 30; ++f) {
        const float lift = f >= 10 && f < 20 ? 20.f : 0.f;
        const float step = f < 10 ? 0.f : f < 20 ? float(f - 10) * 5.f : 50.f;
        c.At(f, lfoot).t = {0.f, step, -100.f + lift};
    }
    const FootPlants p = DetectFootPlants(c, s, {"Root", "LFoot", "RFoot"});
    CHECK(p.hasFeet, "a rig with feet");
    CHECK(p.ranges[0].size() == 2, "the left foot plants twice");
    CHECK(p.ranges[0].front().first == 0 && p.ranges[0].back().second == 30, "plants reach the clip's edges");
    CHECK(p.ranges[1].size() == 1 && p.ranges[1][0] == FrameRange(0, 30), "the right foot stands throughout");
    const auto center = PlantCenter(p.foot[0], p.ranges[0].back());
    CHECK(std::abs(center[1] - 50.f) < 1e-3f, "a plant stands where its foot does");
    CHECK(!DetectFootPlants(c, s, {"Root", "Nope", "RFoot"}).hasFeet, "no foot, no plants");
}

// ── Retarget failure cases (docs/bugs.md R1–R9) ──────────────────────────────

namespace d = targeter::detail;

static constexpr float kDeg = std::numbers::pi_v<float> / 180.f;

static d::Quat Rot(float degrees, d::Vec3 axis) { return d::QAngleAxis(degrees * kDeg, d::Normalize(axis)); }

static void AddBone(Skeleton& s, const char* name, const char* parent, std::array<float, 3> t) {
    SkeletonBone b;
    b.name   = name;
    b.parent = parent ? s.BoneIndex(parent) : -1;
    b.rest.t = t;
    s.bones.push_back(b);
}

/// @brief The map driving every target bone from the source bone of the same name.
static BoneMap SameNames(const Skeleton& target, const Skeleton& source) {
    BoneMap m = LegsMap();
    m.bones.clear();
    for (const auto& b : target.bones)
        if (source.BoneIndex(b.name) >= 0) m.bones[b.name] = b.name;
    return m;
}

static std::vector<Transform> ModelAt(const Skeleton& s, const Clip& c, int frame) {
    std::vector<Transform> local(s.bones.size());
    for (std::size_t b = 0; b < s.bones.size(); ++b) local[b] = c.At(frame, c.BoneIndex(s.bones[b].name));
    return ComputeModelSpace(s, local);
}

static d::Quat Q(const std::array<float, 4>& q) { return {q[0], q[1], q[2], q[3]}; }
static d::Vec3 V(const std::array<float, 3>& v) { return {v[0], v[1], v[2]}; }

/// @brief The angle (degrees) of the rotation `q`.
static float AngleOf(const std::array<float, 4>& q) {
    return 2.f * std::acos(std::min(1.f, std::fabs(q[3]))) / kDeg;
}

/// @brief The yaw (degrees) a rotation gives the +Y axis.
static float YawDegrees(const std::array<float, 4>& q) { return d::YawOf(d::QRotate(Q(q), {0.f, 1.f, 0.f})) / kDeg; }

// R1: Skyrim's twist bones held their rest while the hand pronated and the upper arm rolled: the skin candy-wrapped.
static void TestTwistBonesShareTheTwist() {
    Skeleton s;
    AddBone(s, "Root", nullptr, {0, 0, 0});
    AddBone(s, "Hips", "Root", {0, 0, 100});
    AddBone(s, "Upper", "Hips", {-10, 0, 40});
    AddBone(s, "Fore", "Upper", {-20, 0, 0});
    AddBone(s, "Hand", "Fore", {-20, 0, 0});
    Skeleton t = s;
    AddBone(t, "UTwist", "Upper", {-5, 0, 0});
    AddBone(t, "FTwist", "Fore", {-10, 0, 0});
    BoneMap m = SameNames(t, s);
    m.twist["FTwist"] = {"Hand", 0.5f};
    m.twist["UTwist"] = {"Upper", -0.5f};
    Clip c = RestClip(s, 1);
    c.At(0, c.BoneIndex("Hand")).r  = Rot(80.f, {1, 0, 0});
    c.At(0, c.BoneIndex("Upper")).r = Rot(60.f, {1, 0, 0});
    RetargetReport r;
    const auto out = RetargetClip(s, c, t, m, RetargetOptions{}, &r);
    CHECK(out.has_value(), "R1 retarget with twist bones");
    if (!out) return;
    const auto& ft = out->At(0, out->BoneIndex("FTwist")).r;
    const auto& ut = out->At(0, out->BoneIndex("UTwist")).r;
    CHECK(NearQuat(ft, Rot(40.f, {1, 0, 0}), 1e-3f), "R1 a forearm twist bone takes half the hand's pronation");
    CHECK(NearQuat(ut, Rot(-30.f, {1, 0, 0}), 1e-3f), "R1 an upper-arm twist bone turns back half the arm's roll");

    // With no fraction, each takes its share by where it sits: FTwist halfway down the forearm, UTwist a quarter of the
    // way down the upper arm (so three quarters of the roll undone).
    BoneMap byPlace = m;
    byPlace.twist["FTwist"] = {"Hand", std::nullopt};
    byPlace.twist["UTwist"] = {"Upper", std::nullopt};
    const auto placed = RetargetClip(s, c, t, byPlace, RetargetOptions{});
    CHECK(placed.has_value(), "R1 retarget with twist shares by place");
    if (placed) {
        CHECK(NearQuat(placed->At(0, placed->BoneIndex("FTwist")).r, Rot(40.f, {1, 0, 0}), 1e-3f), "R1 halfway down the forearm, half the pronation");
        CHECK(NearQuat(placed->At(0, placed->BoneIndex("UTwist")).r, Rot(-45.f, {1, 0, 0}), 1e-3f), "R1 near the shoulder, most of the roll undone");
    }

    BoneMap bad = m;
    bad.twist["FTwist"] = {"Hips", 0.5f};
    RetargetReport rb;
    (void)RetargetClip(s, c, t, bad, RetargetOptions{}, &rb);
    CHECK(rb.ignoredTwists.size() == 1 && rb.ignoredTwists[0] == "FTwist", "R1 a twist driven from elsewhere is reported");
}

/// @brief Two legs whose thigh and calf split their length `thigh`:`calf`, hips at 100.
static Skeleton KneeRig(float thigh, float calf) {
    Skeleton s;
    AddBone(s, "Root", nullptr, {0, 0, 0});
    AddBone(s, "Hips", "Root", {0, 0, 100});
    AddBone(s, "LThigh", "Hips", {-10, 0, 0});
    AddBone(s, "LCalf", "LThigh", {0, 0, -thigh});
    AddBone(s, "LFoot", "LCalf", {0, 0, -calf});
    AddBone(s, "RThigh", "Hips", {10, 0, 0});
    AddBone(s, "RCalf", "RThigh", {0, 0, -thigh});
    AddBone(s, "RFoot", "RCalf", {0, 0, -calf});
    return s;
}

// R2: one hip-height scale, but legs split differently: the planted feet slid while the hips travelled.
static void TestFootLockPinsPlants() {
    const Skeleton s = KneeRig(50.f, 50.f);
    const Skeleton t = KneeRig(30.f, 70.f);
    // Both feet stay on the floor under the hips' start while the hips slide 15 forward at a constant height of 90.
    Clip c = RestClip(s, 20);
    c.frameDuration = 1.f / 30.f;
    for (int f = 0; f < 20; ++f) {
        const float y = 15.f * float(f) / 19.f;
        c.At(f, c.BoneIndex("Hips")).t = {0.f, y, 90.f};
        // Leg plane YZ: hip at the origin, foot at (-y, -90); the knee forward.
        const float dy = -y, dz = -90.f, dist = std::sqrt(dy * dy + dz * dz);
        const float toward = std::atan2(dy, -dz);
        const float hip    = std::acos(std::clamp(dist / 100.f, -1.f, 1.f));   // equal halves of 50
        for (const char* side : {"L", "R"}) {
            c.At(f, c.BoneIndex(std::string(side) + "Thigh")).r = d::QAngleAxis(toward + hip, {1.f, 0.f, 0.f});
            c.At(f, c.BoneIndex(std::string(side) + "Calf")).r  = d::QAngleAxis(-2.f * hip, {1.f, 0.f, 0.f});
        }
    }
    BoneMap m = SameNames(t, s);
    m.targetLeftFoot = "LFoot", m.targetRightFoot = "RFoot";
    auto drift = [&](const Clip& out) {
        float most = 0.f;
        const auto start = ModelAt(t, out, 0)[std::size_t(t.BoneIndex("LFoot"))].t;
        for (int f = 0; f < out.numFrames; ++f) {
            const auto p = ModelAt(t, out, f)[std::size_t(t.BoneIndex("LFoot"))].t;
            most = std::max(most, d::Len(d::Sub(V(p), V(start))));
        }
        return most;
    };
    const auto loose = RetargetClip(s, c, t, m, RetargetOptions{});
    CHECK(loose.has_value() && drift(*loose) > 2.f, "R2 unlocked, the target's planted foot slides");
    RetargetOptions lock;
    lock.lockFeet = true;
    RetargetReport r;
    const auto locked = RetargetClip(s, c, t, m, lock, &r);
    CHECK(locked.has_value(), "R2 retarget with foot lock");
    if (!locked) return;
    CHECK(drift(*locked) < 0.5f, "R2 locked, the planted foot stays where it landed");
    CHECK(r.lockedFrames == 20, "R2 every frame is locked");
    const auto hips0 = ModelAt(t, *locked, 19)[1].t;
    const auto hips1 = ModelAt(t, *loose, 19)[1].t;
    CHECK(NearVec3(hips0, hips1, 1e-3f), "R2 the lock leaves the hips alone");
}

// R3: the target root took the source root's pitch and roll, and without a source root it took the hips' heading, so
// the pelvis' sway each stride turned the root (and the motion data) back and forth.
static void TestRootIsYawOnlyAndSteady() {
    const Skeleton s = LegsRig();
    Clip c = RestClip(s, 2);
    c.At(1, 0).r = d::QMul(Rot(40.f, {0, 0, 1}), Rot(30.f, {1, 0, 0}));
    const auto out = RetargetClip(s, c, s, LegsMap(), RetargetOptions{});
    CHECK(out.has_value(), "R3 retarget a pitched source root");
    if (!out) return;
    const auto& root = out->At(1, 0).r;
    CHECK(NearVec3(d::QRotate(Q(root), {0.f, 0.f, 1.f}), {0.f, 0.f, 1.f}, 1e-3f), "R3 the root stays upright");
    CHECK(std::fabs(YawDegrees(root) - 40.f) < 0.1f, "R3 the root keeps the source root's heading");

    // No source root: the hips sway 10 deg each way once a second, for four seconds.
    BoneMap m = LegsMap();
    m.sourceRoot = "";
    Clip sway    = RestClip(s, 121);
    sway.frameDuration = 1.f / 30.f;
    for (int f = 0; f < 121; ++f)
        sway.At(f, 1).r = Rot(10.f * std::sin(2.f * std::numbers::pi_v<float> * float(f) / 30.f), {0, 0, 1});
    const auto steady = RetargetClip(s, sway, s, m, RetargetOptions{});
    CHECK(steady.has_value(), "R3 retarget a swaying source without a root");
    if (!steady) return;
    float rootMost = 0.f, hipsErr = 0.f;
    for (int f = 30; f < 91; ++f) {
        rootMost = std::max(rootMost, std::fabs(YawDegrees(steady->At(f, 0).r)));
        const auto hips = ModelAt(s, *steady, f)[1].r;
        hipsErr = std::max(hipsErr, std::fabs(YawDegrees(hips) - 10.f * std::sin(2.f * std::numbers::pi_v<float> * float(f) / 30.f)));
    }
    CHECK(rootMost < 2.5f, "R3 the root's heading ignores the hips' sway");
    CHECK(hipsErr < 0.1f, "R3 the hips still sway as the source's do");
}

// R4: the ground was the lowest joint of all, so a helper bone below the feet (Skyrim's x_NPC, at -2) sank the root
// and grew the scale.
static void TestGroundIgnoresHelpers() {
    const Skeleton s = LegsRig();
    Skeleton       t = LegsRig();
    AddBone(t, "Helper", "Root", {0, 0, -2});
    RetargetReport r;
    const auto out = RetargetClip(s, RestClip(s, 1), t, LegsMap(), RetargetOptions{}, &r);
    CHECK(out.has_value(), "R4 retarget onto a rig with a helper below the floor");
    if (!out) return;
    CHECK(std::fabs(r.scale - 1.f) < 1e-4f, "R4 the helper doesn't change the scale");
    const auto model = ModelAt(t, *out, 0);
    CHECK(std::fabs(model[0].t[2]) < 1e-3f, "R4 the root stays on the floor");
    CHECK(std::fabs(model[1].t[2] - 100.f) < 1e-3f, "R4 the hips stay at their height");
}

/// @brief Hips, a spine of `spine` bones `step` apart, and a neck, on LegsRig.
static Skeleton SpineRig(const std::vector<std::string>& spine, float step) {
    Skeleton s = LegsRig();
    std::string parent = "Hips";
    for (const auto& b : spine) {
        AddBone(s, b.c_str(), parent.c_str(), {0, 0, step});
        parent = b;
    }
    AddBone(s, "Neck", parent.c_str(), {0, 0, step});
    return s;
}

// R5: a source with fewer spine bones left the target's extra one at rest, so the whole bend kinked at one joint.
static void TestBetweenBonesShareTheBend() {
    const Skeleton s = SpineRig({"Spine0", "Spine2"}, 15.f);
    const Skeleton t = SpineRig({"Spine0", "Spine1", "Spine2"}, 10.f);
    BoneMap m = SameNames(t, s);
    m.between.insert("Spine1");
    Clip c = RestClip(s, 1);
    c.At(0, c.BoneIndex("Spine2")).r = Rot(40.f, {1, 0, 0});
    const auto out = RetargetClip(s, c, t, m, RetargetOptions{});
    CHECK(out.has_value(), "R5 retarget a coarse spine");
    if (!out) return;
    CHECK(std::fabs(AngleOf(out->At(0, out->BoneIndex("Spine1")).r) - 20.f) < 0.5f, "R5 the unmapped spine bone takes its share");
    CHECK(std::fabs(AngleOf(out->At(0, out->BoneIndex("Spine2")).r) - 20.f) < 0.5f, "R5 the next bone takes the rest");
    CHECK(std::fabs(AngleOf(ModelAt(t, *out, 0)[std::size_t(t.BoneIndex("Spine2"))].r) - 40.f) < 0.5f, "R5 the chest still turns as the source's");
}

// R6: the default aim was the first mapped child by index: the chest aimed at a clavicle (SCT ED-10). And an aim on the
// hips was dropped without a word.
static void TestDefaultAimFollowsTheChain() {
    auto rig = [](float tilt) {
        Skeleton s = LegsRig();
        AddBone(s, "Spine", "Hips", {0, 0, 10});
        AddBone(s, "Chest", "Spine", {0, 0, 20});
        AddBone(s, "LClav", "Chest", {-10, 0, 15});
        AddBone(s, "Neck", "Chest", {0, 20.f * std::sin(tilt * kDeg), 20.f * std::cos(tilt * kDeg)});
        AddBone(s, "Head", "Neck", {0, 0, 10});
        return s;
    };
    const Skeleton s = rig(20.f), t = rig(0.f);
    BoneMap m = SameNames(t, s);
    m.aim["Hips"] = "Spine";
    RetargetReport r;
    const auto out = RetargetClip(s, RestClip(s, 1), t, m, RetargetOptions{}, &r);
    CHECK(out.has_value(), "R6 retarget a chest with a clavicle listed first");
    if (!out) return;
    const auto model = ModelAt(t, *out, 0);
    const auto neck  = d::Sub(V(model[std::size_t(t.BoneIndex("Neck"))].t), V(model[std::size_t(t.BoneIndex("Chest"))].t));
    CHECK(std::fabs(neck[1] - 20.f * std::sin(20.f * kDeg)) < 0.05f, "R6 the chest aims at the neck, the way the chain runs");
    CHECK(r.ignoredAims.size() == 1 && r.ignoredAims[0] == "Hips", "R6 an aim on the hips is reported");
}

// R7: the rest alignment only swung a bone at its aim child, so a hand whose palm the two rigs roll differently kept
// that roll ("blocky", DCC 3.3c).
static void TestRestRollFollowsTheOtherChildren() {
    auto rig = [](d::Vec3 index) {
        Skeleton s = LegsRig();
        AddBone(s, "Hand", "Hips", {-20, 0, 40});
        AddBone(s, "Middle", "Hand", {-10, 0, 0});
        AddBone(s, "Index", "Hand", index);
        return s;
    };
    const Skeleton s = rig({-9, 3, 0}), t = rig({-9, 0, 3});
    BoneMap m = SameNames(t, s);
    m.aim["Hand"] = "Middle";
    const auto out = RetargetClip(s, RestClip(s, 1), t, m, RetargetOptions{});
    CHECK(out.has_value(), "R7 retarget a hand rolled 90 deg");
    if (!out) return;
    const auto model = ModelAt(t, *out, 0);
    const auto index = d::Sub(V(model[std::size_t(t.BoneIndex("Index"))].t), V(model[std::size_t(t.BoneIndex("Hand"))].t));
    CHECK(NearVec3(index, {-9.f, 3.f, 0.f}, 1e-2f), "R7 the palm turns the way the source's faces");
}

// R8: a heading taken through atan2 jumped from +180 to -180 deg, and its quaternion changed sign between two frames.
static void TestTracksStayInOneHemisphere() {
    const Skeleton s = LegsRig();
    BoneMap m = LegsMap();
    m.sourceRoot = "";
    Clip c = RestClip(s, 61);
    for (int f = 0; f < 61; ++f) c.At(f, 1).r = Rot(6.f * float(f), {0, 0, 1});
    const auto out = RetargetClip(s, c, s, m, RetargetOptions{});
    CHECK(out.has_value(), "R8 retarget a full turn");
    if (!out) return;
    bool same = true;
    for (int f = 1; f < out->numFrames; ++f)
        for (std::size_t b = 0; b < out->bones.size(); ++b)
            same = same && d::QDot(Q(out->At(f - 1, int(b)).r), Q(out->At(f, int(b)).r)) >= 0.f;
    CHECK(same, "R8 no track changes sign between frames");
    CHECK(std::fabs(YawDegrees(out->At(30, 0).r) - 180.f) < 3.f || std::fabs(YawDegrees(out->At(30, 0).r) + 180.f) < 3.f,
          "R8 the root still turns with the hips");
}

// R9: RetargetMocap had its own transfer; PFERD's pairs never set alignRest, so a target whose rest leg points elsewhere
// than the rest frame's kept its own rest.
static void TestMocapAlignsRest() {
    Skeleton tgt;
    tgt.bones = {{"Root", -1, {}}, {"Body", 0, {}}, {"Leg", 1, {}}, {"Foot", 2, {}}};
    tgt.bones[1].rest.t = {0.f, 0.f, 100.f};
    tgt.bones[3].rest.t = {0.f, 50.f * std::sin(30.f * kDeg), -50.f * std::cos(30.f * kDeg)};

    MocapLayout layout;
    layout.bones = {{"body", "", "back", "front", "right"}, {"leg", "body", "hip", "knee", "kneeSide"}};
    MocapMap map;
    map.root = "Root", map.anchor = "Body", map.anchorSource = "body";
    map.pairs = {{"body", "Body"}, {"leg", "Leg", "Foot"}};
    MarkerTake take;
    take.name    = "hang";
    take.markers = {"back", "front", "right", "hip", "knee", "kneeSide"};
    take.frameDuration = 1.f / 30.f, take.numFrames = 2;
    for (int f = 0; f < 2; ++f)
        for (std::array<float, 3> p : std::initializer_list<std::array<float, 3>>{
                 {0, -10, 100}, {0, 10, 100}, {10, 0, 100}, {0, 0, 100}, {0, 0, 60}, {10, 0, 60}})
            take.points.push_back(p);
    MocapRetargetOptions opt;
    opt.restFrame = 0, opt.scale = 1.f;
    const auto clip = RetargetMocap(layout, map, take, tgt, opt);
    CHECK(clip.has_value(), "R9 retarget a hanging leg");
    if (!clip) return;
    std::vector<Transform> local(clip->poses.begin(), clip->poses.begin() + 4);
    const auto model = ComputeModelSpace(tgt, local);
    CHECK(NearVec3(model[3].t, {0.f, 0.f, 50.f}, 1e-2f), "R9 the target leg hangs as the markers' does");
}

static void TestTurnPoseOffset() {
    const Skeleton s = LegsRig();
    BoneMap        m = LegsMap();
    Clip           c = RestClip(s, 1);
    c.At(0, 2).r = Rot(25.f, {1, 0, 0});
    const auto base = RetargetClip(s, c, s, m);
    CHECK(base.has_value(), "TurnPoseOffset base retarget");
    if (!base) return;
    const auto w    = ModelAt(s, *base, 0)[2].r;
    const auto turn = Rot(30.f, {0, 1, 0});
    m.pose["LThigh"] = TurnPoseOffset({0, 0, 0, 1}, w, turn);
    const auto posed = RetargetClip(s, c, s, m);
    CHECK(posed.has_value(), "TurnPoseOffset posed retarget");
    if (!posed) return;
    const auto d2 = d::QMul(turn, Q(w));
    CHECK(NearQuat(ModelAt(s, *posed, 0)[2].r, {d2[0], d2[1], d2[2], d2[3]}, 1e-3f), "TurnPoseOffset turns the bone in model space");
}

int main() {
    TestTwistBonesShareTheTwist();
    TestFootLockPinsPlants();
    TestRootIsYawOnlyAndSteady();
    TestGroundIgnoresHelpers();
    TestBetweenBonesShareTheBend();
    TestDefaultAimFollowsTheChain();
    TestRestRollFollowsTheOtherChildren();
    TestTracksStayInOneHemisphere();
    TestMocapAlignsRest();
    TestTurnPoseOffset();
    TestRetargetPoseOffset();
    TestPlaceSourcePoint();
    TestHipsDontAim();
    TestFootPlants();
    TestTransformDefault();
    TestComposeIdentity();
    TestComputeModelSpace();
    TestClip();
    TestNormalizeRanges();
    TestBoneMap();
    TestKimodoSkeletons();
    TestRetargetIdentity();
    TestResampleTake();
    TestMarkerPrefix();
    TestRetargetMocapSwing();
    if (g_failures) {
        std::printf("%d check(s) failed.\n", g_failures);
        return 1;
    }
    std::printf("All tests passed.\n");
    return 0;
}
