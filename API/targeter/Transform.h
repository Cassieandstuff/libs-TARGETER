#pragma once
// Transform — translation, rotation (XYZW quaternion), scale.
// Havok QsTransform layout: scale is applied first, then rotate, then translate.
// Axis convention: Z up, character faces +Y, Skyrim game units unless noted.

#include <array>

namespace targeter {

struct Transform {
    std::array<float, 3> t{0.f, 0.f, 0.f};
    std::array<float, 4> r{0.f, 0.f, 0.f, 1.f};   // x, y, z, w
    std::array<float, 3> s{1.f, 1.f, 1.f};
    bool operator==(const Transform&) const = default;
};

} // namespace targeter
