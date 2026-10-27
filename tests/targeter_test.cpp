#include <targeter/Targeter.h>

#include <cassert>
#include <cmath>
#include <cstdio>
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
static void Fail(const char* name) { std::printf("[FAIL] %s\n", name); std::exit(1); }

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

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
    TestTransformDefault();
    TestComposeIdentity();
    TestComputeModelSpace();
    TestClip();
    TestNormalizeRanges();
    TestBoneMap();
    TestKimodoSkeletons();
    TestRetargetIdentity();
    std::printf("All tests passed.\n");
    return 0;
}
