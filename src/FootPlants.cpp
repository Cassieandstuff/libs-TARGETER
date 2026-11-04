#include <targeter/FootPlants.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace targeter {

namespace {

std::vector<double> Smooth(const std::vector<double>& v, int half) {
    std::vector<double> out(v.size());
    const int           n = int(v.size());
    for (int i = 0; i < n; ++i) {
        const int a = std::max(0, i - half), b = std::min(n - 1, i + half);
        double    sum = 0;
        for (int k = a; k <= b; ++k) sum += v[std::size_t(k)];
        out[std::size_t(i)] = sum / double(b - a + 1);
    }
    return out;
}

/// @brief Each frame's speed along a path: central differences, one-sided at the ends.
std::vector<double> SpeedOf(const std::vector<std::array<float, 3>>& path, double dt, bool ground) {
    const int           n = int(path.size());
    std::vector<double> out(path.size(), 0.0);
    if (n < 2 || dt <= 0) return out;
    for (int i = 0; i < n; ++i) {
        const auto& a = path[std::size_t(std::max(0, i - 1))];
        const auto& b = path[std::size_t(std::min(n - 1, i + 1))];
        const double span = double(std::min(n - 1, i + 1) - std::max(0, i - 1)) * dt;
        const double dx = b[0] - a[0], dy = b[1] - a[1], dz = ground ? 0.0 : double(b[2] - a[2]);
        out[std::size_t(i)] = std::sqrt(dx * dx + dy * dy + dz * dz) / span;
    }
    return out;
}

std::vector<FrameRange> RangesOf(std::vector<char> on, int gap, int minFrames) {
    const int n = int(on.size());
    for (int i = 0; i < n;) {
        if (on[std::size_t(i)]) {
            ++i;
            continue;
        }
        int j = i;
        while (j < n && !on[std::size_t(j)]) ++j;
        if (i > 0 && j < n && j - i <= gap) std::fill(on.begin() + i, on.begin() + j, char(1));
        i = j;
    }
    std::vector<FrameRange> out;
    for (int i = 0; i < n;) {
        if (!on[std::size_t(i)]) {
            ++i;
            continue;
        }
        int j = i;
        while (j < n && on[std::size_t(j)]) ++j;
        if (j - i >= minFrames) out.emplace_back(i, j);
        i = j;
    }
    return out;
}

} // namespace

FootPlants DetectFootPlants(const Clip& clip, const Skeleton& skeleton, const PlantBones& bones, const PlantThresholds& th) {
    FootPlants  out;
    const int   rootBone = skeleton.BoneIndex(bones.root);
    const std::array<int, 2> foot{ skeleton.BoneIndex(bones.leftFoot), skeleton.BoneIndex(bones.rightFoot) };
    if (foot[0] < 0 || foot[1] < 0 || clip.numFrames <= 0) return out;
    out.hasFeet = true;

    std::vector<int> track(skeleton.bones.size(), -1);
    for (std::size_t b = 0; b < skeleton.bones.size(); ++b) track[b] = clip.BoneIndex(skeleton.bones[b].name);

    const std::size_t      frames = std::size_t(clip.numFrames);
    std::vector<Transform> local(skeleton.bones.size());
    out.foot[0].resize(frames);
    out.foot[1].resize(frames);
    out.root.resize(frames);
    for (std::size_t f = 0; f < frames; ++f) {
        for (std::size_t b = 0; b < skeleton.bones.size(); ++b) local[b] = track[b] >= 0 ? clip.At(int(f), track[b]) : skeleton.bones[b].rest;
        const std::vector<Transform> model = ComputeModelSpace(skeleton, local);
        out.foot[0][f] = model[std::size_t(foot[0])].t;
        out.foot[1][f] = model[std::size_t(foot[1])].t;
        out.root[f]    = rootBone >= 0 ? model[std::size_t(rootBone)].t : std::array<float, 3>{ 0.f, 0.f, 0.f };
    }

    const std::vector<double> rootSpeed = Smooth(SpeedOf(out.root, clip.frameDuration, true), th.rootSmoothHalf);
    for (std::size_t s = 0; s < 2; ++s) {
        const auto& path = out.foot[s];
        float       lowest = path[0][2];
        for (const auto& p : path) lowest = std::min(lowest, p[2]);
        const std::vector<double> speed = Smooth(SpeedOf(path, clip.frameDuration, false), th.footSmoothHalf);
        const auto low = [&](std::size_t f) { return double(path[f][2] - lowest) <= th.plantHeight; };

        std::vector<char> on(frames, 0);
        for (std::size_t f = 0; f < frames; ++f)
            on[f] = low(f) && speed[f] <= std::max(th.plantSpeed, th.plantSpeedScale * rootSpeed[f]);
        std::vector<FrameRange> ranges = RangesOf(std::move(on), th.gapFrames, th.minFrames);

        /// @note The clip's edges are gaps too: a foot low from frame 0 was already down, even while it settles.
        const auto lowThrough = [&](int a, int b) {
            for (int k = a; k < b; ++k)
                if (!low(std::size_t(k))) return false;
            return true;
        };
        if (!ranges.empty()) {
            if (ranges.front().first <= th.gapFrames && lowThrough(0, ranges.front().first)) ranges.front().first = 0;
            if (ranges.back().second >= clip.numFrames - th.gapFrames && lowThrough(ranges.back().second, clip.numFrames)) ranges.back().second = clip.numFrames;
        }
        out.ranges[s] = std::move(ranges);
    }
    return out;
}

std::array<float, 2> PlantCenter(const std::vector<std::array<float, 3>>& foot, FrameRange range) {
    const int first = std::clamp(range.first, 0, int(foot.size()));
    const int last  = std::clamp(range.second, first, int(foot.size()));
    if (first >= last) return { 0.f, 0.f };
    double x = 0, y = 0;
    for (int f = first; f < last; ++f) {
        x += foot[std::size_t(f)][0];
        y += foot[std::size_t(f)][1];
    }
    return { float(x / double(last - first)), float(y / double(last - first)) };
}

} // namespace targeter
