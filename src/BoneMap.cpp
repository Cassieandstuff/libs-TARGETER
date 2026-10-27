#include <targeter/BoneMap.h>

namespace targeter {

std::string BoneMap::SourceOf(std::string_view targetBone) const {
    auto it = bones.find(std::string(targetBone));
    return it != bones.end() ? it->second : std::string{};
}

} // namespace targeter
