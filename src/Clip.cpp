#include <targeter/Clip.h>

#include <algorithm>

namespace targeter {

int Clip::BoneIndex(std::string_view bone) const {
    for (int i = 0; i < static_cast<int>(bones.size()); ++i)
        if (bones[i] == bone) return i;
    return -1;
}

void Clip::Resize(int frames) {
    numFrames = frames;
    poses.assign(std::size_t(frames) * bones.size(), Transform{});
}

void NormalizeRanges(RangeMap& tags) {
    for (auto& [name, ranges] : tags) {
        // Sort by start frame
        std::sort(ranges.begin(), ranges.end());

        // Merge overlapping/adjacent and drop empty
        std::vector<FrameRange> merged;
        for (auto& r : ranges) {
            if (r.first >= r.second) continue; // drop empty
            if (!merged.empty() && r.first <= merged.back().second)
                merged.back().second = std::max(merged.back().second, r.second);
            else
                merged.push_back(r);
        }
        ranges = std::move(merged);
    }
}

} // namespace targeter
