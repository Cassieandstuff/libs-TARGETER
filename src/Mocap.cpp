#include <targeter/Mocap.h>
#include "MathOps.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <map>
#include <unordered_map>

namespace targeter {

namespace {
using namespace detail;

constexpr Vec3 kAim{0.f, 1.f, 0.f};   // a source bone's local aim axis
constexpr Vec3 kUp{0.f, 0.f, 1.f};

bool Seen(const std::array<float, 3>& p) { return !std::isnan(p[0]); }

/// World rotation taking local +Y along `fwd` and local +Z toward `ref` (made perpendicular to `fwd`).
Quat OrientAimRoll(Vec3 fwd, Vec3 ref) {
    const Vec3 f = Normalize(fwd);
    Vec3       r = Sub(ref, Scale(f, Dot(ref, f)));
    if (LenSq(r) < 1e-10f) {
        const Vec3 seed = std::fabs(f[0]) < 0.9f ? Vec3{1.f, 0.f, 0.f} : Vec3{0.f, 1.f, 0.f};
        r = Sub(seed, Scale(f, Dot(seed, f)));
    }
    r = Normalize(r);
    return QFromColumns(Cross(f, r), f, r);
}

float Yaw(Quat q) { return YawOf(QRotate(q, kAim)); }

/// The layout bound to one take's markers: every marker a slot (reals first, then virtuals), bones by slot.
struct Bound {
    int numReal = 0;
    std::vector<int> takeIndex;   // per real slot
    struct Virtual { std::vector<std::pair<int, float>> components; };
    std::vector<Virtual> virtuals;
    struct Bone { int parent = -1, origin = -1, aim = -1, roll = -1, hinge = -1; };
    std::vector<Bone> bones;
    std::vector<std::string> missing;
    Quat  capture{0.f, 0.f, 0.f, 1.f};
    float unitScale = 1.f;
};

Bound Bind(const MocapLayout& layout, const MarkerTake& take, std::string_view prefix) {
    Bound b;
    b.capture   = QNormalize({layout.captureRotation[0], layout.captureRotation[1], layout.captureRotation[2],
                              layout.captureRotation[3]});
    b.unitScale = layout.unitScale;
    std::unordered_map<std::string, int> slot;
    const std::vector<std::string> reals = layout.RealMarkers();
    b.numReal = int(reals.size());
    for (const std::string& r : reals) {
        const int i = take.MarkerIndex(std::string(prefix) + r);
        if (i < 0) b.missing.push_back(r);
        slot.emplace(r, int(b.takeIndex.size()));
        b.takeIndex.push_back(i);
    }
    auto find = [&](const std::string& name) {
        if (name.empty()) return -1;
        const auto it = slot.find(name);
        return it == slot.end() ? -1 : it->second;
    };
    for (const VirtualMarker& v : layout.virtuals) {
        Bound::Virtual bv;
        for (const auto& [name, w] : v.components) bv.components.emplace_back(find(name), w);
        slot.emplace(v.name, b.numReal + int(b.virtuals.size()));
        b.virtuals.push_back(std::move(bv));
    }
    std::unordered_map<std::string, int> bone;
    for (const MocapBone& mb : layout.bones) {
        Bound::Bone bb;
        if (!mb.parent.empty()) {
            const auto it = bone.find(mb.parent);
            bb.parent = it == bone.end() ? -1 : it->second;
        }
        bb.origin = find(mb.origin);
        bb.aim    = find(mb.aim);
        bb.roll   = find(mb.roll);
        bb.hinge  = find(mb.hinge);
        bone.emplace(mb.name, int(b.bones.size()));
        b.bones.push_back(bb);
    }
    return b;
}

struct Points {
    std::vector<Vec3> p;
    std::vector<char> ok;
};

Points FramePoints(const Bound& b, const MarkerTake& take, int frame) {
    Points pts;
    const std::size_t n = std::size_t(b.numReal) + b.virtuals.size();
    pts.p.assign(n, {0.f, 0.f, 0.f});
    pts.ok.assign(n, 0);
    for (int i = 0; i < b.numReal; ++i) {
        const int t = b.takeIndex[std::size_t(i)];
        if (t < 0) continue;
        const auto& raw = take.At(frame, t);
        if (!Seen(raw)) continue;
        pts.p[std::size_t(i)]  = Scale(QRotate(b.capture, {raw[0], raw[1], raw[2]}), b.unitScale);
        pts.ok[std::size_t(i)] = 1;
    }
    for (std::size_t v = 0; v < b.virtuals.size(); ++v) {
        Vec3 acc{0.f, 0.f, 0.f};
        bool ok = !b.virtuals[v].components.empty();
        for (const auto& [s, w] : b.virtuals[v].components) {
            if (s < 0 || !pts.ok[std::size_t(s)]) { ok = false; break; }
            acc = Add(acc, Scale(pts.p[std::size_t(s)], w));
        }
        pts.p[std::size_t(b.numReal) + v]  = acc;
        pts.ok[std::size_t(b.numReal) + v] = ok ? 1 : 0;
    }
    return pts;
}

struct Pose {
    std::vector<Quat> q;
    std::vector<Vec3> p;
    std::vector<char> ok;
};

Pose Solve(const Bound& b, const Points& pts) {
    const std::size_t n = b.bones.size();
    Pose pose;
    pose.q.assign(n, {0.f, 0.f, 0.f, 1.f});
    pose.p.assign(n, {0.f, 0.f, 0.f});
    pose.ok.assign(n, 0);
    auto have = [&](int s) { return s >= 0 && pts.ok[std::size_t(s)]; };
    for (std::size_t i = 0; i < n; ++i) {
        const Bound::Bone& bb = b.bones[i];
        const bool needRoll  = bb.roll >= 0 || bb.hinge >= 0;
        // A named roll or hinge that is missing would let the twist jump, so the bone is lost with it.
        const bool ok = have(bb.origin) && have(bb.aim) && (!needRoll || have(bb.hinge) || (bb.hinge < 0 && have(bb.roll)));
        if (ok) {
            const Vec3 o   = pts.p[std::size_t(bb.origin)];
            const Vec3 fwd = Sub(pts.p[std::size_t(bb.aim)], o);
            Vec3       ref{0.f, 0.f, 0.f};
            if (bb.hinge >= 0) ref = Cross(fwd, Sub(pts.p[std::size_t(bb.hinge)], o));
            else if (bb.roll >= 0) ref = Sub(pts.p[std::size_t(bb.roll)], o);
            if (LenSq(fwd) > 1e-10f) {
                pose.q[i]  = OrientAimRoll(fwd, ref);
                pose.p[i]  = o;
                pose.ok[i] = 1;
                continue;
            }
        }
        if (bb.parent >= 0) {
            pose.q[i] = pose.q[std::size_t(bb.parent)];
            pose.p[i] = have(bb.origin) ? pts.p[std::size_t(bb.origin)] : pose.p[std::size_t(bb.parent)];
        }
    }
    return pose;
}

/// Fill each marker's gaps of up to `maxFrames` frames by linear interpolation between the samples around them.
void FillGaps(MarkerTake& take, int maxFrames) {
    const std::size_t m = take.markers.size();
    for (std::size_t k = 0; k < m; ++k) {
        int last = -1;
        for (int f = 0; f < take.numFrames; ++f) {
            auto& p = take.points[std::size_t(f) * m + k];
            if (!Seen(p)) continue;
            if (last >= 0 && f - last > 1 && f - last - 1 <= maxFrames) {
                const auto& a = take.points[std::size_t(last) * m + k];
                for (int g = last + 1; g < f; ++g) {
                    const float u = float(g - last) / float(f - last);
                    auto&       q = take.points[std::size_t(g) * m + k];
                    for (int c = 0; c < 3; ++c) q[c] = a[c] + (p[c] - a[c]) * u;
                }
            }
            last = f;
        }
    }
}

} // namespace

int MarkerTake::MarkerIndex(std::string_view marker) const {
    for (std::size_t i = 0; i < markers.size(); ++i)
        if (markers[i] == marker) return int(i);
    return -1;
}

std::vector<std::string> MocapLayout::RealMarkers() const {
    std::vector<std::string> out;
    auto isVirtual = [&](const std::string& n) {
        return std::any_of(virtuals.begin(), virtuals.end(), [&](const VirtualMarker& v) { return v.name == n; });
    };
    auto add = [&](const std::string& n) {
        if (n.empty() || isVirtual(n) || std::find(out.begin(), out.end(), n) != out.end()) return;
        out.push_back(n);
    };
    for (const VirtualMarker& v : virtuals)
        for (const auto& c : v.components) add(c.first);
    for (const MocapBone& b : bones) {
        add(b.origin);
        add(b.aim);
        add(b.roll);
        add(b.hinge);
    }
    return out;
}

std::string MarkerPrefix(const MarkerTake& take, const MocapLayout& layout) {
    const std::vector<std::string> reals = layout.RealMarkers();
    std::map<std::string, int> votes;
    for (const std::string& m : take.markers)
        for (const std::string& r : reals)
            if (m.size() >= r.size() && m.compare(m.size() - r.size(), r.size(), r) == 0) {
                ++votes[m.substr(0, m.size() - r.size())];
                break;
            }
    std::string best;
    int         most = 0;
    for (const auto& [prefix, n] : votes)
        if (n > most) {
            best = prefix;
            most = n;
        }
    return best;
}

MarkerTake ResampleTake(const MarkerTake& take, float frameDuration, float maxGap) {
    MarkerTake filled = take;
    if (take.frameDuration > 0.f) FillGaps(filled, int(maxGap / take.frameDuration));
    MarkerTake out;
    out.name          = take.name;
    out.markers       = take.markers;
    out.frameDuration = frameDuration;
    if (take.numFrames <= 0 || frameDuration <= 0.f || take.frameDuration <= 0.f) return out;
    const float span = float(take.numFrames - 1) * take.frameDuration;
    out.numFrames    = int(std::floor(span / frameDuration + 1e-4f)) + 1;
    const std::size_t m = take.markers.size();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    out.points.assign(std::size_t(out.numFrames) * m, {nan, nan, nan});
    for (int k = 0; k < out.numFrames; ++k) {
        // A window centred on the output time (narrowed at the ends), so the average is not shifted.
        const float t  = float(k) * frameDuration;
        const float r  = std::min({0.5f * frameDuration, t, span - t});
        const int   f0 = std::max(0, int(std::ceil((t - r) / take.frameDuration - 1e-3f)));
        const int   f1 = std::min(take.numFrames - 1, int(std::floor((t + r) / take.frameDuration + 1e-3f)));
        for (std::size_t j = 0; j < m; ++j) {
            std::array<double, 3> acc{0.0, 0.0, 0.0};
            int                   n = 0;
            for (int f = f0; f <= f1; ++f) {
                const auto& p = filled.points[std::size_t(f) * m + j];
                if (!Seen(p)) continue;
                for (int c = 0; c < 3; ++c) acc[std::size_t(c)] += p[std::size_t(c)];
                ++n;
            }
            if (n > 0)
                out.points[std::size_t(k) * m + j] = {float(acc[0] / n), float(acc[1] / n), float(acc[2] / n)};
        }
    }
    return out;
}

std::vector<Transform> SourcePose(const MocapLayout& layout, const MarkerTake& take, int frame,
                                  std::string_view markerPrefix, std::vector<char>* valid) {
    const Bound bound = Bind(layout, take, markerPrefix);
    const Pose  pose  = Solve(bound, FramePoints(bound, take, frame));
    std::vector<Transform> out(pose.q.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i].t = pose.p[i];
        out[i].r = pose.q[i];
    }
    if (valid) *valid = pose.ok;
    return out;
}

std::expected<Clip, std::string> RetargetMocap(const MocapLayout& layout, const MocapMap& map, const MarkerTake& take,
                                               const Skeleton& target, const MocapRetargetOptions& options,
                                               MocapReport* report) {
    const std::size_t nT = target.bones.size();
    if (nT == 0) return std::unexpected(std::string("the target skeleton has no bones"));
    if (take.numFrames <= 0) return std::unexpected(std::format("take '{}' has no frames", take.name));
    const int root   = target.BoneIndex(map.root);
    const int anchor = target.BoneIndex(map.anchor);
    if (root < 0) return std::unexpected(std::format("the target has no root bone '{}'", map.root));
    if (anchor < 0) return std::unexpected(std::format("the target has no anchor bone '{}'", map.anchor));
    int anchorSrc = -1;
    for (std::size_t i = 0; i < layout.bones.size(); ++i)
        if (layout.bones[i].name == map.anchorSource) anchorSrc = int(i);
    if (anchorSrc < 0) return std::unexpected(std::format("the layout has no anchor source bone '{}'", map.anchorSource));

    MocapReport rep;
    const MarkerTake  clipTake = ResampleTake(take, options.frameDuration, options.maxGap);
    const MarkerTake  restTake = options.restTake ? ResampleTake(*options.restTake, options.frameDuration, options.maxGap)
                                                  : clipTake;
    const std::string prefix     = options.markerPrefix.empty() ? MarkerPrefix(take, layout) : options.markerPrefix;
    const std::string restPrefix = options.markerPrefix.empty() ? MarkerPrefix(restTake, layout) : options.markerPrefix;
    const Bound       bound      = Bind(layout, clipTake, prefix);
    const Bound       restBound  = Bind(layout, restTake, restPrefix);
    rep.missing = bound.missing;

    // Target rest, local and model space.
    const std::vector<Transform> bindLocal = target.RestLocal();
    const std::vector<Transform> bindModel = ComputeModelSpace(target, bindLocal);
    auto pos = [&](int b) { const auto& t = bindModel[std::size_t(b)].t; return Vec3{t[0], t[1], t[2]}; };
    auto rot = [&](const Transform& t) { return Quat{t.r[0], t.r[1], t.r[2], t.r[3]}; };

    // Pairs by target bone.
    std::unordered_map<std::string, int> srcByName;
    for (std::size_t i = 0; i < layout.bones.size(); ++i) srcByName.emplace(layout.bones[i].name, int(i));
    struct Drive { int src = -1, aim = -1; bool align = false; };
    std::vector<Drive> drive(nT);
    for (const MocapPair& p : map.pairs) {
        const auto s = srcByName.find(p.source);
        const int  t = target.BoneIndex(p.target);
        if (s == srcByName.end()) { rep.missing.push_back(p.source); continue; }
        if (t < 0) { rep.missing.push_back(p.target); continue; }
        drive[std::size_t(t)] = {s->second, p.targetAim.empty() ? -1 : target.BoneIndex(p.targetAim), p.alignRest};
        if (!p.targetAim.empty() && drive[std::size_t(t)].aim < 0) rep.missing.push_back(p.targetAim);
    }

    // The rest frame: given, or the one whose bones point most like the target's rest (heading removed).
    auto canonical = [&](const Pose& pose) {
        const Quat q = QAngleAxis(-Yaw(pose.q[std::size_t(anchorSrc)]), kUp);
        std::vector<Quat> out(pose.q.size());
        for (std::size_t i = 0; i < out.size(); ++i) out[i] = QNormalize(QMul(q, pose.q[i]));
        return out;
    };
    auto mismatch = [&](const Pose& pose) {
        if (!pose.ok[std::size_t(anchorSrc)]) return std::numeric_limits<float>::max();
        const std::vector<Quat> c = canonical(pose);
        float sum = 0.f;
        for (std::size_t t = 0; t < nT; ++t) {
            const Drive& d = drive[t];
            if (d.src < 0) continue;
            if (!pose.ok[std::size_t(d.src)]) return std::numeric_limits<float>::max();
            if (d.aim < 0) continue;
            const Vec3 want = Normalize(Sub(pos(d.aim), pos(int(t))));
            sum += std::acos(std::clamp(Dot(QRotate(c[std::size_t(d.src)], kAim), want), -1.f, 1.f));
        }
        return sum;
    };
    int restFrame = options.restFrame;
    if (restFrame < 0) {
        float best = std::numeric_limits<float>::max();
        for (int f = 0; f < restTake.numFrames; ++f) {
            const float m = mismatch(Solve(restBound, FramePoints(restBound, restTake, f)));
            if (m < best) {
                best      = m;
                restFrame = f;
            }
        }
        if (restFrame < 0) return std::unexpected(std::string("no frame of the rest take has every mapped bone"));
    }
    if (restFrame >= restTake.numFrames) return std::unexpected(std::format("rest frame {} is past the take", restFrame));
    rep.restFrame = restFrame;
    const Pose              rest      = Solve(restBound, FramePoints(restBound, restTake, restFrame));
    const std::vector<Quat> restCanon = canonical(rest);
    if (!rest.ok[std::size_t(anchorSrc)]) return std::unexpected(std::string("the anchor source bone is missing in the rest frame"));

    // A pair whose source bone is missing at rest has nothing to measure motion from.
    std::vector<Quat> aligned(nT);
    for (std::size_t t = 0; t < nT; ++t) {
        Drive& d   = drive[t];
        aligned[t] = rot(bindModel[t]);
        if (d.src < 0) continue;
        if (!rest.ok[std::size_t(d.src)]) {
            d.src = -1;
            continue;
        }
        ++rep.mappedBones;
        if (d.align && d.aim >= 0) {
            const Vec3 from = Normalize(Sub(pos(d.aim), pos(int(t))));
            const Vec3 to   = QRotate(restCanon[std::size_t(d.src)], kAim);
            aligned[t]      = QNormalize(QMul(QFromTo(from, to), aligned[t]));
        }
    }

    const float restHeight = rest.p[std::size_t(anchorSrc)][2];
    float       scale      = options.scale;
    if (scale <= 0.f) {
        if (restHeight <= 1e-3f) return std::unexpected(std::string("the anchor source bone is not above the ground at rest"));
        scale = pos(anchor)[2] / restHeight;
    }
    rep.scale = scale;

    const int first = std::clamp(int(std::lround(options.start / options.frameDuration)), 0, clipTake.numFrames - 1);
    const int last  = options.end > 0.f
                          ? std::clamp(int(std::lround(options.end / options.frameDuration)), first, clipTake.numFrames - 1)
                          : clipTake.numFrames - 1;

    Clip clip;
    clip.name          = take.name;
    clip.skeleton      = target.bones[0].name;
    clip.frameDuration = options.frameDuration;
    for (const SkeletonBone& b : target.bones) clip.bones.push_back(b.name);
    clip.Resize(last - first + 1);

    const Vec3 anchorFromRoot = Sub(pos(anchor), pos(root));
    float      yaw0 = 0.f, yaw = 0.f;
    Vec3       origin0{0.f, 0.f, 0.f}, travel{0.f, 0.f, 0.f};
    float      height = restHeight;
    std::vector<Quat> delta(layout.bones.size()), world(nT);
    std::vector<Vec3> worldPos(nT);
    for (int f = first; f <= last; ++f) {
        const Pose pose = Solve(bound, FramePoints(bound, clipTake, f));
        if (pose.ok[std::size_t(anchorSrc)]) {
            yaw    = Yaw(pose.q[std::size_t(anchorSrc)]);
            travel = pose.p[std::size_t(anchorSrc)];
            height = travel[2];
        }
        if (f == first) {
            yaw0    = yaw;
            origin0 = travel;
        }
        const Quat facing = QAngleAxis(-yaw0, kUp);
        for (std::size_t s = 0; s < delta.size(); ++s) {
            const int parent = bound.bones[s].parent;
            if (pose.ok[s] && rest.ok[s])
                delta[s] = QNormalize(QMul(QMul(facing, pose.q[s]), QInverse(restCanon[s])));
            else
                delta[s] = parent >= 0 ? delta[std::size_t(parent)] : Quat{0.f, 0.f, 0.f, 1.f};
        }

        const Quat rootRot = QAngleAxis(yaw - yaw0, kUp);
        Vec3       rootPos = Scale(QRotate(facing, Sub(travel, origin0)), scale);
        rootPos[2]         = 0.f;
        const Vec3 anchorPos = Add(Add(rootPos, QRotate(rootRot, anchorFromRoot)), {0.f, 0.f, scale * (height - restHeight)});

        for (std::size_t t = 0; t < nT; ++t) {
            const int        parent = target.bones[t].parent;
            const Transform& bl     = bindLocal[t];
            const Quat       pq     = parent >= 0 ? world[std::size_t(parent)] : Quat{0.f, 0.f, 0.f, 1.f};
            const Vec3       pp     = parent >= 0 ? worldPos[std::size_t(parent)] : Vec3{0.f, 0.f, 0.f};
            if (int(t) == root) world[t] = rootRot;
            else if (drive[t].src >= 0) world[t] = QNormalize(QMul(delta[std::size_t(drive[t].src)], aligned[t]));
            else world[t] = QNormalize(QMul(pq, rot(bl)));

            Transform& out = clip.At(f - first, int(t));
            out.r          = parent >= 0 ? QNormalize(QMul(QInverse(pq), world[t])) : world[t];
            out.s          = bl.s;
            if (int(t) == root) out.t = parent >= 0 ? QRotate(QInverse(pq), Sub(rootPos, pp)) : rootPos;
            else if (int(t) == anchor) out.t = QRotate(QInverse(pq), Sub(anchorPos, pp));
            else out.t = bl.t;
            worldPos[t] = Add(pp, QRotate(pq, out.t));
        }
    }
    if (report) *report = std::move(rep);
    return clip;
}

} // namespace targeter
