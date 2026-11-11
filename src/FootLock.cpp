#include "FootLock.h"
#include "MathOps.h"

#include <algorithm>
#include <cmath>

namespace targeter::detail {

namespace {

Quat Q(const std::array<float, 4>& q) { return {q[0], q[1], q[2], q[3]}; }
Vec3 V(const std::array<float, 3>& v) { return {v[0], v[1], v[2]}; }

float Smoothstep(float x) {
    x = std::clamp(x, 0.f, 1.f);
    return x * x * (3.f - 2.f * x);
}

/// @brief How much the lock holds on `frame`, and which plant it holds to: 1 inside a plant, easing to 0 over `blend` frames
///        either side of it.
std::pair<float, int> Weight(const std::vector<FrameRange>& ranges, int frame, int blend) {
    float best = 0.f;
    int   at   = -1;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        const auto [first, last] = ranges[i];
        float w = 0.f;
        if (frame >= first && frame < last) w = 1.f;
        else {
            const int away = frame < first ? first - frame : frame - (last - 1);
            if (away <= blend) w = Smoothstep(1.f - float(away) / float(blend + 1));
        }
        if (w > best) {
            best = w;
            at   = int(i);
        }
    }
    return {best, at};
}

} // namespace

LockResult LockFeet(Clip& clip, const Skeleton& skeleton, const std::array<int, 2>& feet,
                    const std::array<std::vector<FrameRange>, 2>& plants, int blend) {
    LockResult        result;
    const std::size_t n = skeleton.bones.size();
    std::array<std::array<int, 3>, 2> leg{};   // thigh, calf, foot
    std::array<bool, 2>               ok{};
    for (std::size_t s = 0; s < 2; ++s) {
        const int foot = feet[s];
        const int calf = foot >= 0 ? skeleton.bones[std::size_t(foot)].parent : -1;
        const int thigh = calf >= 0 ? skeleton.bones[std::size_t(calf)].parent : -1;
        ok[s]  = thigh >= 0;
        leg[s] = {thigh, calf, foot};
    }

    auto local = [&](int f) {
        std::vector<Transform> l(n);
        for (std::size_t b = 0; b < n; ++b) l[b] = clip.At(f, int(b));
        return l;
    };

    // Each plant stands where its foot is on the plant's first frame, before any lock moves it.
    std::array<std::vector<Vec3>, 2> anchor;
    for (std::size_t s = 0; s < 2; ++s) {
        if (!ok[s]) continue;
        for (const auto& [first, last] : plants[s]) {
            const int  f     = std::clamp(first, 0, std::max(clip.numFrames - 1, 0));
            const auto model = ComputeModelSpace(skeleton, local(f));
            anchor[s].push_back(V(model[std::size_t(leg[s][2])].t));
        }
    }

    for (int f = 0; f < clip.numFrames; ++f) {
        bool locked = false;
        for (std::size_t s = 0; s < 2; ++s) {
            if (!ok[s]) continue;
            const auto [w, at] = Weight(plants[s], f, blend);
            if (w <= 0.f || at < 0) continue;
            locked = true;

            const auto model = ComputeModelSpace(skeleton, local(f));
            const auto [thigh, calf, foot] = leg[s];
            const Vec3 a = V(model[std::size_t(thigh)].t), b = V(model[std::size_t(calf)].t), c = V(model[std::size_t(foot)].t);
            const Vec3 goal = Lerp(c, anchor[s][std::size_t(at)], w);

            const float l1 = Len(Sub(b, a)), l2 = Len(Sub(c, b));
            if (l1 < 1e-4f || l2 < 1e-4f) continue;
            const Vec3  toGoal = Sub(goal, a);
            const float want   = Len(toGoal);
            if (want < 1e-4f) continue;
            const float dist = std::clamp(want, std::fabs(l1 - l2) + 1e-3f, l1 + l2 - 1e-3f);
            const Vec3  u    = Scale(toGoal, 1.f / want);

            // The knee stays on the side it bends to now; a straight leg bends toward the root's forward.
            Vec3 side = Sub(Sub(b, a), Scale(u, Dot(Sub(b, a), u)));
            if (LenSq(side) < 1e-6f * l1 * l1) {
                const Vec3 fwd = QRotate(Q(model[0].r), {0.f, 1.f, 0.f});
                side           = Sub(fwd, Scale(u, Dot(fwd, u)));
            }
            side = Normalize(side);
            const float cosA = std::clamp((l1 * l1 + dist * dist - l2 * l2) / (2.f * l1 * dist), -1.f, 1.f);
            const float sinA = std::sqrt(std::max(0.f, 1.f - cosA * cosA));
            const Vec3  knee = Add(a, Add(Scale(u, l1 * cosA), Scale(side, l1 * sinA)));
            const Vec3  end  = Add(a, Scale(u, dist));
            result.miss      = std::max(result.miss, Len(Sub(end, goal)));

            const Quat thighQ = Q(model[std::size_t(thigh)].r), calfQ = Q(model[std::size_t(calf)].r), footQ = Q(model[std::size_t(foot)].r);
            const Quat swing1 = QFromTo(Normalize(Sub(b, a)), Normalize(Sub(knee, a)));
            const Quat thighN = QNormalize(QMul(swing1, thighQ));
            const Vec3 shin   = QRotate(swing1, Sub(c, b));
            const Quat swing2 = QFromTo(Normalize(shin), Normalize(Sub(end, knee)));
            const Quat calfN  = QNormalize(QMul(swing2, QMul(swing1, calfQ)));

            const int  hipBone = skeleton.bones[std::size_t(thigh)].parent;
            const Quat parentQ = hipBone >= 0 ? Q(model[std::size_t(hipBone)].r) : Quat{0.f, 0.f, 0.f, 1.f};
            const Quat tl = QNormalize(QMul(QInverse(parentQ), thighN));
            const Quat cl = QNormalize(QMul(QInverse(thighN), calfN));
            const Quat fl = QNormalize(QMul(QInverse(calfN), footQ));
            clip.At(f, thigh).r = {tl[0], tl[1], tl[2], tl[3]};
            clip.At(f, calf).r  = {cl[0], cl[1], cl[2], cl[3]};
            clip.At(f, foot).r  = {fl[0], fl[1], fl[2], fl[3]};
        }
        if (locked) ++result.lockedFrames;
    }
    return result;
}

} // namespace targeter::detail
