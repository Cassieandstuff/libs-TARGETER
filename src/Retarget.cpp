#include <targeter/Retarget.h>
#include "FootLock.h"
#include "MathOps.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace targeter {

namespace {
using namespace detail;

constexpr float kPi = 3.14159265358979f;

struct World {
    std::vector<Vec3> p;
    std::vector<Quat> q;
};

Quat Q(const std::array<float, 4>& q) { return {q[0], q[1], q[2], q[3]}; }
Vec3 V(const std::array<float, 3>& v) { return {v[0], v[1], v[2]}; }

template <class LocalFn>
World Fk(const Skeleton& skel, LocalFn&& local) {
    const std::size_t n = skel.bones.size();
    World w;
    w.p.resize(n);
    w.q.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Transform& t = local(i);
        const int        p = skel.bones[i].parent;
        if (p < 0) {
            w.q[i] = QNormalize(Q(t.r));
            w.p[i] = V(t.t);
        } else {
            w.q[i] = QNormalize(QMul(w.q[std::size_t(p)], Q(t.r)));
            w.p[i] = Add(w.p[std::size_t(p)], QRotate(w.q[std::size_t(p)], V(t.t)));
        }
    }
    return w;
}

/// @brief The heading (radians about +Z) a world rotation gives. Forward and left both vote, so a body bent over (forward
///        pointing down) or rolled on its side (left pointing down) still has one.
float HeadingOf(Quat q) {
    const Vec3 fwd      = QRotate(q, {0.f, 1.f, 0.f});
    const Vec3 left     = QRotate(q, {-1.f, 0.f, 0.f});
    const Vec3 fromLeft = Cross(left, {0.f, 0.f, 1.f});
    const Vec3 sum{fwd[0] + fromLeft[0], fwd[1] + fromLeft[1], 0.f};
    return LenSq(sum) > 1e-10f ? YawOf(sum) : 0.f;
}

/// @brief `yaw` made continuous across the +-pi seam.
void Unwrap(std::vector<float>& yaw) {
    for (std::size_t f = 1; f < yaw.size(); ++f) {
        float step = yaw[f] - yaw[f - 1];
        while (step > kPi) step -= 2.f * kPi;
        while (step < -kPi) step += 2.f * kPi;
        yaw[f] = yaw[f - 1] + step;
    }
}

/// @brief A Gaussian of `sigma` frames over `v`, the window cut at the clip's ends and renormalised.
std::vector<float> Gaussian(const std::vector<float>& v, float sigma) {
    if (sigma <= 0.f || v.size() < 2) return v;
    const int          half = std::max(1, int(std::ceil(3.f * sigma)));
    std::vector<float> kernel(std::size_t(2 * half + 1));
    for (int k = -half; k <= half; ++k) kernel[std::size_t(k + half)] = std::exp(-0.5f * float(k * k) / (sigma * sigma));
    std::vector<float> out(v.size());
    const int          n = int(v.size());
    for (int i = 0; i < n; ++i) {
        double sum = 0, weight = 0;
        for (int k = -half; k <= half; ++k) {
            const int j = i + k;
            if (j < 0 || j >= n) continue;
            sum += double(kernel[std::size_t(k + half)]) * double(v[std::size_t(j)]);
            weight += double(kernel[std::size_t(k + half)]);
        }
        out[std::size_t(i)] = float(sum / weight);
    }
    return out;
}

/// @brief The rest offset from bone `b` to `child`, in `b`'s frame: the child's own translation when it is a direct child,
///        else measured through the rest pose (an aim may skip unmapped bones, Neck1 -> Head past Neck2).
Vec3 RestOffset(const Skeleton& s, const World& rest, int b, int child) {
    if (s.bones[std::size_t(child)].parent == b) return V(s.bones[std::size_t(child)].rest.t);
    return QRotate(QInverse(rest.q[std::size_t(b)]), Sub(rest.p[std::size_t(child)], rest.p[std::size_t(b)]));
}

/// @brief An unmapped bone that bends with its chain: the mapped bone above it, the one mapped bone below, and its share.
struct Between {
    int   above = -1, below = -1;
    float share = 0.f;
};

/// @brief An unmapped twist bone, and how it reads its driver's twist.
struct Twist {
    int   bone = -1, driver = -1;
    bool  own  = false;   ///< the driver is the twist bone's parent (else its sibling)
    Vec3  axis{1.f, 0.f, 0.f};
    float fraction = 0.f;
};

} // namespace

std::expected<Clip, std::string> RetargetClip(
    const Skeleton&        source,
    const Clip&            clip,
    const Skeleton&        target,
    const BoneMap&         map,
    const RetargetOptions& options,
    RetargetReport*        report)
{
    const std::size_t nT = target.bones.size();
    const std::size_t nS = source.bones.size();
    RetargetReport    rep;

    // ── Bone correspondence ───────────────────────────────────────────────────
    std::vector<int> srcOf(nT, -1);
    for (std::size_t b = 0; b < nT; ++b) {
        const std::string s = map.SourceOf(target.bones[b].name);
        if (!s.empty()) {
            srcOf[b] = source.BoneIndex(s);
            if (srcOf[b] < 0) rep.missingSource.push_back(s);
        }
        if (srcOf[b] < 0) rep.unmappedTargets.push_back(target.bones[b].name);
    }

    auto srcIdx = [&](const std::string& name, const char* fallbackTarget) -> int {
        int i = name.empty() ? -1 : source.BoneIndex(name);
        if (i < 0) {
            const int t = target.BoneIndex(fallbackTarget);
            if (t >= 0) i = srcOf[std::size_t(t)];
        }
        return i;
    };
    const int sHips  = srcIdx(map.sourceHips, map.targetHips.c_str());
    const int sLeft  = srcIdx(map.sourceLeftHip, "LeftLeg");
    const int sRight = srcIdx(map.sourceRightHip, "RightLeg");
    const int sRoot  = map.sourceRoot.empty() ? -1 : source.BoneIndex(map.sourceRoot);
    const int tRoot  = target.BoneIndex(map.targetRoot);
    const int tHips  = target.BoneIndex(map.targetHips);

    if (sHips < 0) return std::unexpected("the map's source hips bone isn't in the source skeleton");
    if (tHips < 0 || tRoot < 0) return std::unexpected("the target skeleton lacks the map's root or hips bone");

    // Aims the retarget can't use: the root and hips keep their rest rotation, and an aim must name two bones.
    for (const auto& [bone, child] : map.aim)
        if (bone == map.targetRoot || bone == map.targetHips || target.BoneIndex(bone) < 0 || target.BoneIndex(child) < 0)
            rep.ignoredAims.push_back(bone);

    // ── Source rest: facing, ground, scale ───────────────────────────────────
    const World srcRest = Fk(source, [&](std::size_t i) -> const Transform& { return source.bones[i].rest; });
    const World tgtRest = Fk(target, [&](std::size_t i) -> const Transform& { return target.bones[i].rest; });

    // Facing = rotation that maps the source lateral axis (LeftHip − RightHip) → −X. A turn about Z only, so heights
    // read the same before and after it.
    Quat facing = {0.f, 0.f, 0.f, 1.f};
    if (sLeft >= 0 && sRight >= 0) {
        Vec3 lateral = {srcRest.p[std::size_t(sLeft)][0] - srcRest.p[std::size_t(sRight)][0],
                        srcRest.p[std::size_t(sLeft)][1] - srcRest.p[std::size_t(sRight)][1], 0.f};
        if (LenSq(lateral) > 1e-8f)
            facing = QFromTo(Normalize(lateral), {-1.f, 0.f, 0.f});
    }

    // The ground is the lowest bone the map drives, the same bone on both sides: a helper below the feet (Skyrim's x_NPC
    // at -2) or an IK target at the origin would otherwise set it. A map that drives nothing below the hips falls back to
    // the lowest bone of each skeleton.
    float srcGround = 0.f, tgtGround = 0.f;
    {
        int lowest = -1;
        for (std::size_t b = 0; b < nT; ++b)
            if (srcOf[b] >= 0 && (lowest < 0 || tgtRest.p[b][2] < tgtRest.p[std::size_t(lowest)][2])) lowest = int(b);
        const bool below = lowest >= 0 && tgtRest.p[std::size_t(tHips)][2] - tgtRest.p[std::size_t(lowest)][2] > 1e-4f &&
                           srcRest.p[std::size_t(sHips)][2] - srcRest.p[std::size_t(srcOf[std::size_t(lowest)])][2] > 1e-4f;
        if (below) {
            tgtGround = tgtRest.p[std::size_t(lowest)][2];
            srcGround = srcRest.p[std::size_t(srcOf[std::size_t(lowest)])][2];
        } else {
            srcGround = srcRest.p[0][2];
            for (const auto& p : srcRest.p) srcGround = std::min(srcGround, p[2]);
            tgtGround = tgtRest.p[0][2];
            for (const auto& p : tgtRest.p) tgtGround = std::min(tgtGround, p[2]);
        }
    }

    const float srcHipH = srcRest.p[std::size_t(sHips)][2] - srcGround;
    const float tgtHipH = tgtRest.p[std::size_t(tHips)][2] - tgtGround;
    if (options.scale <= 0.f && srcHipH <= 1e-4f) return std::unexpected("the source hips are not above its feet at rest");
    const float scale = options.scale > 0.f ? options.scale : tgtHipH / srcHipH;

    // Source rest in facing-corrected world space.
    std::vector<Quat> srcRestQ(nS);
    std::vector<Vec3> srcRestP(nS);
    for (std::size_t j = 0; j < nS; ++j) {
        srcRestQ[j] = QNormalize(QMul(facing, srcRest.q[j]));
        srcRestP[j] = QRotate(facing, {srcRest.p[j][0], srcRest.p[j][1], srcRest.p[j][2] - srcGround});
    }

    // placeP: facing-corrected, scaled, grounded source world position → target world position.
    auto placeP = [&](Vec3 src) -> Vec3 {
        const Vec3 faced = QRotate(facing, {src[0], src[1], src[2] - srcGround});
        return {scale * faced[0], scale * faced[1], scale * faced[2] + tgtGround};
    };

    // ── Rest alignment ───────────────────────────────────────────────────────
    // alignedWorld[b] = target rest world orientation, turned so the direction toward the aim child matches the source
    // bone's direction in facing-corrected space, then rolled about it so its other mapped children match too.
    std::vector<Quat> alignedWorld(nT), alignedLocal(nT);
    std::vector<int>  aimOf(nT, -1);

    for (std::size_t b = 0; b < nT; ++b) {
        const int p     = target.bones[b].parent;
        alignedLocal[b] = QNormalize(Q(target.bones[b].rest.r));
        alignedWorld[b] = p < 0 ? alignedLocal[b] : QNormalize(QMul(alignedWorld[std::size_t(p)], alignedLocal[b]));

        if (srcOf[b] < 0) continue;
        // The root and hips keep their rest rotation: what lies between them and their children is where the rig puts its
        // joints, not a limb's direction. Skyrim's spine starts 54 deg back from COM against a Mannequin's nearly straight
        // up, and aiming across that turned COM, and the pelvis under it, 50 deg forward.
        if (int(b) == tRoot || int(b) == tHips) continue;

        // Aim child: from map.aim, or the mapped child that carries on the way the chain runs into this bone (a chest's
        // neck, not a clavicle listed before it; a hand's middle finger, not its thumb).
        int aimChild = -1;
        if (const auto it = map.aim.find(target.bones[b].name); it != map.aim.end()) aimChild = target.BoneIndex(it->second);
        if (aimChild < 0) {
            const Vec3 in    = p >= 0 ? Sub(tgtRest.p[b], tgtRest.p[std::size_t(p)]) : Vec3{0.f, 0.f, 0.f};
            const bool hasIn = LenSq(in) > 1e-8f;
            float      best  = -2.f;
            for (std::size_t c = b + 1; c < nT; ++c) {
                if (target.bones[c].parent != int(b) || srcOf[c] < 0) continue;
                const Vec3  out   = Sub(tgtRest.p[c], tgtRest.p[b]);
                const float score = hasIn && LenSq(out) > 1e-8f ? Dot(Normalize(in), Normalize(out)) : -1.f;
                if (score > best) {
                    best     = score;
                    aimChild = int(c);
                }
            }
        }
        if (aimChild < 0) continue;
        const int sAim = srcOf[std::size_t(aimChild)];
        if (sAim < 0 && !map.sourceAxis) continue;
        aimOf[b] = aimChild;

        // Target bone vector toward the aim child, and the source bone's: toward its child, or along the axis the source
        // aims its bones by when the child has no source.
        const Vec3 dT = QRotate(alignedWorld[b], RestOffset(target, tgtRest, int(b), aimChild));
        const Vec3 dS = sAim >= 0
            ? Sub(srcRestP[std::size_t(sAim)], srcRestP[std::size_t(srcOf[b])])
            : QRotate(srcRestQ[std::size_t(srcOf[b])], {(*map.sourceAxis)[0], (*map.sourceAxis)[1], (*map.sourceAxis)[2]});
        if (LenSq(dT) < 1e-8f || LenSq(dS) < 1e-8f) continue;
        const Vec3 axis = Normalize(dS);
        alignedWorld[b] = QNormalize(QMul(QFromTo(Normalize(dT), axis), alignedWorld[b]));

        // Roll: the turn about the aim that best lines up the other mapped children (a palm's fingers, a chest's
        // clavicles), each weighted by how far it sits off the axis on both rigs.
        float sinSum = 0.f, cosSum = 0.f;
        for (std::size_t c = b + 1; c < nT; ++c) {
            if (target.bones[c].parent != int(b) || int(c) == aimChild || srcOf[c] < 0) continue;
            const Vec3 vT = QRotate(alignedWorld[b], V(target.bones[c].rest.t));
            const Vec3 vS = Sub(srcRestP[std::size_t(srcOf[c])], srcRestP[std::size_t(srcOf[b])]);
            const Vec3 pT = Sub(vT, Scale(axis, Dot(vT, axis)));
            const Vec3 pS = Sub(vS, Scale(axis, Dot(vS, axis)));
            if (LenSq(pT) < 0.01f * LenSq(vT) || LenSq(pS) < 0.01f * LenSq(vS)) continue;
            cosSum += Dot(pT, pS);
            sinSum += Dot(axis, Cross(pT, pS));
        }
        if (sinSum != 0.f || cosSum != 0.f)
            alignedWorld[b] = QNormalize(QMul(QAngleAxis(std::atan2(sinSum, cosSum), axis), alignedWorld[b]));

        alignedLocal[b] = p < 0 ? alignedWorld[b] : QNormalize(QMul(QInverse(alignedWorld[std::size_t(p)]), alignedWorld[b]));
    }

    // Retarget pose: hand-set local offsets on the aligned rest, carried down the chain.
    if (!map.pose.empty()) {
        for (std::size_t b = 0; b < nT; ++b) {
            if (const auto it = map.pose.find(target.bones[b].name); it != map.pose.end())
                alignedLocal[b] = QNormalize(QMul(alignedLocal[b], Q(it->second)));
            const int p     = target.bones[b].parent;
            alignedWorld[b] = p < 0 ? alignedLocal[b] : QNormalize(QMul(alignedWorld[std::size_t(p)], alignedLocal[b]));
        }
    }

    // ── Bones between mapped ones, and twist bones ───────────────────────────
    std::vector<Between> between(nT);
    for (const std::string& name : map.between) {
        const int u = target.BoneIndex(name);
        if (u < 0 || srcOf[std::size_t(u)] >= 0) continue;
        int above = target.bones[std::size_t(u)].parent;
        while (above >= 0 && srcOf[std::size_t(above)] < 0) above = target.bones[std::size_t(above)].parent;
        // The mapped bones nearest below u: exactly one, or there is no single chain to bend with.
        std::vector<int> below, open{u};
        while (!open.empty()) {
            const int at = open.back();
            open.pop_back();
            for (std::size_t c = std::size_t(at) + 1; c < nT; ++c)
                if (target.bones[c].parent == at) (srcOf[c] >= 0 ? below : open).push_back(int(c));
        }
        if (above < 0 || below.size() != 1) continue;
        auto lengthTo = [&](int from) {
            float len = 0.f;
            for (int x = from; x >= 0 && x != above; x = target.bones[std::size_t(x)].parent) len += Len(V(target.bones[std::size_t(x)].rest.t));
            return len;
        };
        const float whole = lengthTo(below[0]);
        if (whole <= 1e-6f) continue;
        between[std::size_t(u)] = {above, below[0], lengthTo(u) / whole};
    }

    std::vector<Twist> twists;
    for (const auto& [name, from] : map.twist) {
        const int bone   = target.BoneIndex(name);
        const int driver = target.BoneIndex(from.driver);
        const int parent = bone >= 0 ? target.bones[std::size_t(bone)].parent : -1;
        Twist     t{bone, driver, driver >= 0 && driver == parent, {1.f, 0.f, 0.f}, from.fraction};
        bool      ok = bone >= 0 && driver >= 0 && parent >= 0 && srcOf[std::size_t(bone)] < 0;
        if (ok && t.own) {
            // The parent's own long axis: toward its aim child, else its first child that isn't the twist bone.
            int along = aimOf[std::size_t(parent)];
            for (std::size_t c = std::size_t(parent) + 1; along < 0 && c < nT; ++c)
                if (target.bones[c].parent == parent && int(c) != bone) along = int(c);
            ok = along >= 0 && LenSq(RestOffset(target, tgtRest, parent, along)) > 1e-8f;
            if (ok) t.axis = Normalize(RestOffset(target, tgtRest, parent, along));
        } else if (ok) {
            // A sibling driver (a hand under the forearm) twists about the line from their parent to it.
            ok = target.bones[std::size_t(driver)].parent == parent && LenSq(V(target.bones[std::size_t(driver)].rest.t)) > 1e-8f;
            if (ok) t.axis = Normalize(V(target.bones[std::size_t(driver)].rest.t));
        }
        if (ok) twists.push_back(t);
        else rep.ignoredTwists.push_back(name);
    }

    // ── Output clip ──────────────────────────────────────────────────────────
    Clip out;
    out.name             = clip.name;
    out.skeleton         = target.name;
    out.frameDuration    = clip.frameDuration;
    out.numFrames        = clip.numFrames;
    out.annotationTracks = clip.annotationTracks;
    out.tags             = clip.tags;
    out.bones.reserve(nT);
    for (const auto& b : target.bones) out.bones.push_back(b.name);
    out.Resize(clip.numFrames);

    for (std::size_t b = 0; b < nT; ++b) {
        if (srcOf[b] >= 0) ++rep.mappedBones;
        else ++rep.unmappedBones;
    }

    // ── Source per frame, and the root's heading ─────────────────────────────
    std::vector<int> clipTrack(nS);
    for (std::size_t i = 0; i < nS; ++i) clipTrack[i] = clip.BoneIndex(source.bones[i].name);
    std::vector<World> frames(std::size_t(std::max(clip.numFrames, 0)));
    for (int f = 0; f < clip.numFrames; ++f)
        frames[std::size_t(f)] = Fk(source, [&](std::size_t i) -> const Transform& {
            return clipTrack[i] >= 0 ? clip.At(f, clipTrack[i]) : source.bones[i].rest;
        });

    // A source bone's world turn since rest, in facing-corrected space.
    auto delta = [&](const World& src, int j) -> Quat {
        return QNormalize(QMul(QMul(facing, src.q[std::size_t(j)]), QInverse(srcRestQ[std::size_t(j)])));
    };

    // The root carries the heading only. An authored source root's heading is kept as it is; one read off the hips is
    // smoothed, since the pelvis sways a few degrees each stride and the root would turn with it.
    const int          headingBone = sRoot >= 0 ? sRoot : sHips;
    std::vector<float> heading(frames.size());
    for (std::size_t f = 0; f < frames.size(); ++f) heading[f] = HeadingOf(delta(frames[f], headingBone));
    Unwrap(heading);
    if (sRoot < 0 && clip.frameDuration > 0.f) heading = Gaussian(heading, options.rootYawSmoothSeconds / clip.frameDuration);

    // Without a source root, the root stands under the hips at its rest offset from them, turned with the heading.
    Vec3 hipsFromRoot = Sub(tgtRest.p[std::size_t(tHips)], tgtRest.p[std::size_t(tRoot)]);
    hipsFromRoot[2]   = 0.f;
    const float rootZ = tgtRest.p[std::size_t(tRoot)][2];

    // ── Per-frame ────────────────────────────────────────────────────────────
    std::vector<Quat> wq(nT);
    std::vector<Vec3> wp(nT);
    for (int f = 0; f < clip.numFrames; ++f) {
        const World& src     = frames[std::size_t(f)];
        const Vec3   hipsPos = placeP(src.p[std::size_t(sHips)]);
        const Quat   turn    = QAngleAxis(heading[std::size_t(f)], {0.f, 0.f, 1.f});

        for (std::size_t b = 0; b < nT; ++b) {
            const int  p     = target.bones[b].parent;
            const int  si    = srcOf[b];
            const Vec3 restT = V(target.bones[b].rest.t);
            Vec3       pos;

            if (int(b) == tRoot) {
                wq[b] = QNormalize(QMul(turn, alignedWorld[b]));
                if (sRoot >= 0) {
                    const Vec3 placed = placeP(src.p[std::size_t(sRoot)]);
                    pos               = {placed[0], placed[1], rootZ};
                } else {
                    const Vec3 offset = QRotate(turn, hipsFromRoot);
                    pos               = {hipsPos[0] - offset[0], hipsPos[1] - offset[1], rootZ};
                }
            } else {
                if (si >= 0) {
                    wq[b] = QNormalize(QMul(delta(src, si), alignedWorld[b]));
                } else if (const Between& bw = between[b]; bw.above >= 0) {
                    const Quat above = QMul(wq[std::size_t(bw.above)], QInverse(alignedWorld[std::size_t(bw.above)]));
                    const Quat below = delta(src, srcOf[std::size_t(bw.below)]);
                    wq[b]            = QNormalize(QMul(QSlerp(above, below, bw.share), alignedWorld[b]));
                } else {
                    wq[b] = p < 0 ? alignedLocal[b] : QNormalize(QMul(wq[std::size_t(p)], alignedLocal[b]));
                }
                pos = int(b) == tHips ? hipsPos : p < 0 ? restT : Add(wp[std::size_t(p)], QRotate(wq[std::size_t(p)], restT));
            }
            wp[b] = pos;

            Transform& dest = out.At(f, int(b));
            if (p < 0) {
                dest.r = {wq[b][0], wq[b][1], wq[b][2], wq[b][3]};
                dest.t = {pos[0], pos[1], pos[2]};
            } else {
                const Quat invP   = QInverse(wq[std::size_t(p)]);
                const Quat localR = QNormalize(QMul(invP, wq[b]));
                const Vec3 localT = QRotate(invP, Sub(pos, wp[std::size_t(p)]));
                dest.r = {localR[0], localR[1], localR[2], localR[3]};
                dest.t = {localT[0], localT[1], localT[2]};
            }
        }

        // Twist bones take their share of the driver's twist about the limb, on top of their aligned rest.
        for (const Twist& t : twists) {
            const Quat driver = Q(out.At(f, t.driver).r);
            const Quat aligned = alignedLocal[std::size_t(t.driver)];
            const Quat since  = QNormalize(t.own ? QMul(QInverse(aligned), driver) : QMul(driver, QInverse(aligned)));
            const Quat local  = QNormalize(QMul(QAngleAxis(t.fraction * TwistAngle(since, t.axis), t.axis), alignedLocal[std::size_t(t.bone)]));
            out.At(f, t.bone).r = {local[0], local[1], local[2], local[3]};
        }
    }

    // ── Foot lock ────────────────────────────────────────────────────────────
    if (options.lockFeet && !map.targetLeftFoot.empty() && !map.targetRightFoot.empty()) {
        const std::array<int, 2> feet{target.BoneIndex(map.targetLeftFoot), target.BoneIndex(map.targetRightFoot)};
        if (feet[0] >= 0 && feet[1] >= 0 && srcOf[std::size_t(feet[0])] >= 0 && srcOf[std::size_t(feet[1])] >= 0) {
            // Plants are found on the source, whose feet stand where the performer's did; thresholds are in target units.
            PlantThresholds th = options.plants;
            th.plantHeight /= double(scale);
            th.plantSpeed /= double(scale);
            const FootPlants plants = DetectFootPlants(clip, source,
                                                       {source.bones[std::size_t(headingBone)].name,
                                                        source.bones[std::size_t(srcOf[std::size_t(feet[0])])].name,
                                                        source.bones[std::size_t(srcOf[std::size_t(feet[1])])].name},
                                                       th);
            if (plants.hasFeet) {
                const LockResult lock = LockFeet(out, target, feet, plants.ranges, std::max(options.lockBlendFrames, 0));
                rep.lockedFrames      = lock.lockedFrames;
                rep.lockMiss          = lock.miss;
            }
        }
    }

    // Make root XY start-relative (frame 0 at the origin).
    float startX = 0.f, startY = 0.f;
    if (out.numFrames > 0) {
        startX = out.At(0, tRoot).t[0];
        startY = out.At(0, tRoot).t[1];
        if (startX != 0.f || startY != 0.f)
            for (int f = 0; f < out.numFrames; ++f) {
                out.At(f, tRoot).t[0] -= startX;
                out.At(f, tRoot).t[1] -= startY;
            }
    }

    // Every track stays in one hemisphere, so interpolating between two keys never goes the long way round.
    for (int f = 1; f < out.numFrames; ++f)
        for (std::size_t b = 0; b < nT; ++b) {
            auto&       r    = out.At(f, int(b)).r;
            const auto& prev = out.At(f - 1, int(b)).r;
            if (r[0] * prev[0] + r[1] * prev[1] + r[2] * prev[2] + r[3] * prev[3] < 0.f) r = {-r[0], -r[1], -r[2], -r[3]};
        }

    rep.scale        = scale;
    rep.facing       = {facing[0], facing[1], facing[2], facing[3]};
    rep.sourceGround = srcGround;
    rep.targetGround = tgtGround;
    rep.startOffset  = {startX, startY};
    if (report) *report = std::move(rep);
    return out;
}

std::expected<Clip, std::string> RetargetClip(const Skeleton& source, const Clip& clip, const Skeleton& target,
                                              const BoneMap& map, RetargetReport* report) {
    return RetargetClip(source, clip, target, map, RetargetOptions{}, report);
}

std::array<float, 3> PlaceSourcePoint(const RetargetReport& r, const std::array<float, 3>& p) {
    const Vec3 faced = QRotate(Q(r.facing), {p[0], p[1], p[2] - r.sourceGround});
    return {r.scale * faced[0] - r.startOffset[0], r.scale * faced[1] - r.startOffset[1], r.scale * faced[2] + r.targetGround};
}

std::array<float, 4> TurnPoseOffset(const std::array<float, 4>& offset, const std::array<float, 4>& boneWorld,
                                    const std::array<float, 4>& turn) {
    // The offset right-multiplies the bone's aligned rest, so the bone shows at W * offset: turning it to Q * W takes
    // offset' = offset * (W^-1 Q W).
    const Quat w = QNormalize(Q(boneWorld));
    const Quat o = QNormalize(QMul(Q(offset), QMul(QInverse(w), QMul(QNormalize(Q(turn)), w))));
    return {o[0], o[1], o[2], o[3]};
}

} // namespace targeter
