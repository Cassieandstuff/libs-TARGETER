#include <targeter/Retarget.h>
#include "MathOps.h"

#include <algorithm>
#include <cmath>
#include <format>

namespace targeter {

namespace {
using namespace detail;

struct World {
    std::vector<Vec3> p;
    std::vector<Quat> q;
};

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
            w.q[i] = QNormalize({t.r[0], t.r[1], t.r[2], t.r[3]});
            w.p[i] = {t.t[0], t.t[1], t.t[2]};
        } else {
            w.q[i] = QNormalize(QMul(w.q[std::size_t(p)], {t.r[0], t.r[1], t.r[2], t.r[3]}));
            w.p[i] = Add(w.p[std::size_t(p)], QRotate(w.q[std::size_t(p)], {t.t[0], t.t[1], t.t[2]}));
        }
    }
    return w;
}

} // namespace

std::expected<Clip, std::string> RetargetClip(
    const Skeleton& source,
    const Clip&     clip,
    const Skeleton& target,
    const BoneMap&  map,
    RetargetReport* report)
{
    const std::size_t nT = target.bones.size();
    const std::size_t nS = source.bones.size();

    // ── Bone correspondence ───────────────────────────────────────────────────
    std::vector<int> srcOf(nT, -1);
    for (std::size_t b = 0; b < nT; ++b) {
        const std::string s = map.SourceOf(target.bones[b].name);
        if (!s.empty()) srcOf[b] = source.BoneIndex(s);
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

    // ── Source rest: facing, ground, scale ───────────────────────────────────
    const World srcRest = Fk(source, [&](std::size_t i) -> const Transform& { return source.bones[i].rest; });

    // Facing = rotation that maps the source lateral axis (LeftHip − RightHip) → −X.
    Quat facing = {0.f, 0.f, 0.f, 1.f};
    if (sLeft >= 0 && sRight >= 0) {
        Vec3 lateral = {srcRest.p[std::size_t(sLeft)][0] - srcRest.p[std::size_t(sRight)][0],
                        srcRest.p[std::size_t(sLeft)][1] - srcRest.p[std::size_t(sRight)][1], 0.f};
        if (LenSq(lateral) > 1e-8f)
            facing = QFromTo(Normalize(lateral), {-1.f, 0.f, 0.f});
    }

    float srcGround = srcRest.p[0][2];
    for (const auto& p : srcRest.p) srcGround = std::min(srcGround, p[2]);

    const World tgtRest = Fk(target, [&](std::size_t i) -> const Transform& { return target.bones[i].rest; });
    float tgtGround = tgtRest.p[0][2];
    for (const auto& p : tgtRest.p) tgtGround = std::min(tgtGround, p[2]);

    const float srcHipH = srcRest.p[std::size_t(sHips)][2] - srcGround;
    if (srcHipH <= 1e-4f) return std::unexpected("the source hips are not above its feet at rest");
    const float tgtHipH = tgtRest.p[std::size_t(tHips)][2] - tgtGround;
    const float scale   = tgtHipH / srcHipH;

    // Source rest in facing-corrected world space.
    std::vector<Quat> srcRestQ(nS);
    std::vector<Vec3> srcRestP(nS);
    for (std::size_t j = 0; j < nS; ++j) {
        srcRestQ[j] = QNormalize(QMul(facing, srcRest.q[j]));
        srcRestP[j] = QRotate(facing, {srcRest.p[j][0], srcRest.p[j][1], srcRest.p[j][2] - srcGround});
    }

    // placeP: facing-corrected, scaled, grounded source world position → target world position.
    auto placeP = [&](Vec3 src) -> Vec3 {
        Vec3 faced = QRotate(facing, {src[0], src[1], src[2] - srcGround});
        return {scale * faced[0], scale * faced[1], scale * faced[2] + tgtGround};
    };

    // ── Rest alignment ───────────────────────────────────────────────────────
    // alignedWorld[b] = target rest world orientation, adjusted so the direction
    // toward the aim-child matches the source bone's direction in facing-corrected space.
    std::vector<Quat> alignedWorld(nT), alignedLocal(nT);
    std::vector<Vec3> alignedPos(nT);

    for (std::size_t b = 0; b < nT; ++b) {
        const int p      = target.bones[b].parent;
        const Vec3 restT = {target.bones[b].rest.t[0], target.bones[b].rest.t[1], target.bones[b].rest.t[2]};
        alignedLocal[b]  = QNormalize({target.bones[b].rest.r[0], target.bones[b].rest.r[1],
                                        target.bones[b].rest.r[2], target.bones[b].rest.r[3]});
        alignedWorld[b]  = p < 0 ? alignedLocal[b] : QNormalize(QMul(alignedWorld[std::size_t(p)], alignedLocal[b]));
        alignedPos[b]    = p < 0 ? restT : Add(alignedPos[std::size_t(p)], QRotate(alignedWorld[std::size_t(p)], restT));

        if (srcOf[b] < 0) continue;

        // Aim child: from map.aim, or first mapped child bone.
        int aimChild = -1;
        if (const auto it = map.aim.find(target.bones[b].name); it != map.aim.end())
            aimChild = target.BoneIndex(it->second);
        for (std::size_t c = b + 1; aimChild < 0 && c < nT; ++c)
            if (target.bones[c].parent == int(b) && srcOf[c] >= 0) aimChild = int(c);
        if (aimChild < 0 || srcOf[std::size_t(aimChild)] < 0) continue;

        // Target bone vector toward aim child (in target world space).
        const Vec3 dT = QRotate(alignedWorld[b],
            {target.bones[std::size_t(aimChild)].rest.t[0],
             target.bones[std::size_t(aimChild)].rest.t[1],
             target.bones[std::size_t(aimChild)].rest.t[2]});
        // Source bone vector toward corresponding aim child (in facing-corrected world space).
        const Vec3 dS = Sub(srcRestP[std::size_t(srcOf[std::size_t(aimChild)])],
                             srcRestP[std::size_t(srcOf[b])]);
        if (LenSq(dT) < 1e-8f || LenSq(dS) < 1e-8f) continue;

        alignedWorld[b] = QNormalize(QMul(QFromTo(Normalize(dT), Normalize(dS)), alignedWorld[b]));
        alignedLocal[b] = p < 0 ? alignedWorld[b]
                                 : QNormalize(QMul(QInverse(alignedWorld[std::size_t(p)]), alignedWorld[b]));
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

    int mapped = 0, unmapped = 0;
    for (std::size_t b = 0; b < nT; ++b) {
        if (srcOf[b] >= 0) ++mapped; else ++unmapped;
    }

    // ── Per-frame ────────────────────────────────────────────────────────────
    std::vector<Quat> wq(nT);
    std::vector<Vec3> wp(nT);

    for (int f = 0; f < clip.numFrames; ++f) {
        const World src = Fk(source, [&](std::size_t i) -> const Transform& {
            const int ci = clip.BoneIndex(source.bones[i].name);
            return ci >= 0 ? clip.At(f, ci) : source.bones[i].rest;
        });

        // Facing-corrected source world rotation: facing * srcFrame.q
        auto srcQ = [&](int j) -> Quat {
            return QNormalize(QMul(facing, src.q[std::size_t(j)]));
        };

        const Vec3 hipsPos = placeP(src.p[std::size_t(sHips)]);

        for (std::size_t b = 0; b < nT; ++b) {
            const int p  = target.bones[b].parent;
            const int si = srcOf[b];
            Vec3      pos;

            if (int(b) == tRoot) {
                if (sRoot >= 0) {
                    wq[b] = QNormalize(QMul(QMul(srcQ(sRoot), QInverse(srcRestQ[std::size_t(sRoot)])),
                                            alignedWorld[b]));
                    pos   = placeP(src.p[std::size_t(sRoot)]);
                } else {
                    // No root bone: yaw from hips facing direction, placed on ground.
                    const Vec3 hipsWorld = QRotate(
                        QNormalize(QMul(srcQ(sHips), QInverse(srcRestQ[std::size_t(sHips)]))),
                        QRotate(alignedWorld[b], {0.f, 1.f, 0.f}));
                    wq[b] = QNormalize(QMul(
                        QAngleAxis(YawOf(hipsWorld), {0.f, 0.f, 1.f}), alignedWorld[b]));
                    pos   = {hipsPos[0], hipsPos[1], tgtGround};
                }
            } else {
                wq[b] = si >= 0
                    ? QNormalize(QMul(QMul(srcQ(si), QInverse(srcRestQ[std::size_t(si)])), alignedWorld[b]))
                    : (p < 0 ? alignedLocal[b] : QNormalize(QMul(wq[std::size_t(p)], alignedLocal[b])));
                pos = int(b) == tHips ? hipsPos
                    : p < 0 ? Vec3{target.bones[b].rest.t[0], target.bones[b].rest.t[1], target.bones[b].rest.t[2]}
                    : Add(wp[std::size_t(p)], QRotate(wq[std::size_t(p)],
                          {target.bones[b].rest.t[0], target.bones[b].rest.t[1], target.bones[b].rest.t[2]}));
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
    }

    // Make root XY start-relative (frame 0 at the origin).
    if (tRoot >= 0 && out.numFrames > 0) {
        const float dx = out.At(0, tRoot).t[0];
        const float dy = out.At(0, tRoot).t[1];
        if (dx != 0.f || dy != 0.f)
            for (int f = 0; f < out.numFrames; ++f) {
                out.At(f, tRoot).t[0] -= dx;
                out.At(f, tRoot).t[1] -= dy;
            }
    }

    if (report) {
        report->mappedBones   = mapped;
        report->unmappedBones = unmapped;
    }
    return out;
}

} // namespace targeter
