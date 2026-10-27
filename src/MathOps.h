#pragma once
// Private math helpers — not installed, not part of the public API.
// Vec3 = float[3] (x,y,z), Quat = float[4] (x,y,z,w — Havok/xyzw convention).

#include <array>
#include <cmath>

namespace targeter::detail {

using Vec3 = std::array<float, 3>;
using Quat = std::array<float, 4>;  // x, y, z, w

// ── Vec3 ─────────────────────────────────────────────────────────────────────

[[nodiscard]] inline float Dot(Vec3 a, Vec3 b) noexcept {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
[[nodiscard]] inline float LenSq(Vec3 v) noexcept { return Dot(v, v); }
[[nodiscard]] inline float Len(Vec3 v) noexcept { return std::sqrt(LenSq(v)); }

[[nodiscard]] inline Vec3 Normalize(Vec3 v) noexcept {
    float len = Len(v);
    if (len < 1e-10f) return {0.f, 0.f, 0.f};
    return {v[0]/len, v[1]/len, v[2]/len};
}
[[nodiscard]] inline Vec3 Cross(Vec3 a, Vec3 b) noexcept {
    return {a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]};
}
[[nodiscard]] inline Vec3 Add(Vec3 a, Vec3 b) noexcept {
    return {a[0]+b[0], a[1]+b[1], a[2]+b[2]};
}
[[nodiscard]] inline Vec3 Sub(Vec3 a, Vec3 b) noexcept {
    return {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}
[[nodiscard]] inline Vec3 Scale(Vec3 v, float s) noexcept {
    return {v[0]*s, v[1]*s, v[2]*s};
}
[[nodiscard]] inline Vec3 Lerp(Vec3 a, Vec3 b, float t) noexcept {
    return Add(Scale(a, 1.f - t), Scale(b, t));
}

// ── Quat (xyzw) ──────────────────────────────────────────────────────────────

[[nodiscard]] inline float QDot(Quat a, Quat b) noexcept {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}
[[nodiscard]] inline Quat QNormalize(Quat q) noexcept {
    float len = std::sqrt(QDot(q, q));
    if (len < 1e-10f) return {0.f, 0.f, 0.f, 1.f};
    return {q[0]/len, q[1]/len, q[2]/len, q[3]/len};
}
/// Hamilton product: q * r (apply r first, then q in world space).
[[nodiscard]] inline Quat QMul(Quat q, Quat r) noexcept {
    return {
         q[3]*r[0] + q[0]*r[3] + q[1]*r[2] - q[2]*r[1],
         q[3]*r[1] - q[0]*r[2] + q[1]*r[3] + q[2]*r[0],
         q[3]*r[2] + q[0]*r[1] - q[1]*r[0] + q[2]*r[3],
         q[3]*r[3] - q[0]*r[0] - q[1]*r[1] - q[2]*r[2],
    };
}
[[nodiscard]] inline Quat QInverse(Quat q) noexcept {
    return {-q[0], -q[1], -q[2], q[3]};
}
/// Rotate vector v by quaternion q.
[[nodiscard]] inline Vec3 QRotate(Quat q, Vec3 v) noexcept {
    // t = 2 * cross(q.xyz, v)
    Vec3  qv  = {q[0], q[1], q[2]};
    float qw  = q[3];
    Vec3  t   = Scale(Cross(qv, v), 2.f);
    // result = v + qw*t + cross(qv, t)
    return Add(v, Add(Scale(t, qw), Cross(qv, t)));
}

/// Shortest-arc quaternion rotating from unit vector `from` to `to`.
[[nodiscard]] inline Quat QFromTo(Vec3 from, Vec3 to) noexcept {
    Vec3  axis = Cross(from, to);
    float d    = Dot(from, to);
    if (d < -1.f + 1e-6f) {
        // 180° — pick any perpendicular axis
        Vec3 perp = (std::fabs(from[0]) < 0.9f)
                    ? Vec3{1.f, 0.f, 0.f}
                    : Vec3{0.f, 1.f, 0.f};
        axis = Normalize(Cross(from, perp));
        return {axis[0], axis[1], axis[2], 0.f};
    }
    float s = std::sqrt((1.f + d) * 2.f);
    float inv = 1.f / s;
    return QNormalize({axis[0]*inv, axis[1]*inv, axis[2]*inv, s * 0.5f});
}

/// Rotation of `angle` radians around (normalised) `axis`.
[[nodiscard]] inline Quat QAngleAxis(float angle, Vec3 axis) noexcept {
    float half = angle * 0.5f;
    float s    = std::sin(half);
    return {axis[0]*s, axis[1]*s, axis[2]*s, std::cos(half)};
}

/// Rotation whose matrix columns are `x`, `y`, `z` (the images of the local axes). Columns must be orthonormal.
[[nodiscard]] inline Quat QFromColumns(Vec3 x, Vec3 y, Vec3 z) noexcept {
    const float m00 = x[0], m10 = x[1], m20 = x[2];
    const float m01 = y[0], m11 = y[1], m21 = y[2];
    const float m02 = z[0], m12 = z[1], m22 = z[2];
    const float tr  = m00 + m11 + m22;
    if (tr > 0.f) {
        const float s = std::sqrt(tr + 1.f) * 2.f;
        return QNormalize({(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s});
    }
    if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.f + m00 - m11 - m22) * 2.f;
        return QNormalize({0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s});
    }
    if (m11 > m22) {
        const float s = std::sqrt(1.f + m11 - m00 - m22) * 2.f;
        return QNormalize({(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s});
    }
    const float s = std::sqrt(1.f + m22 - m00 - m11) * 2.f;
    return QNormalize({(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s});
}

// ── Utilities ─────────────────────────────────────────────────────────────────

/// Extract the yaw (rotation around Z) from a direction vector.
/// Result is atan2(-x, y) — gives 0 when facing +Y (Skyrim convention).
[[nodiscard]] inline float YawOf(Vec3 dir) noexcept {
    return std::atan2(-dir[0], dir[1]);
}

} // namespace targeter::detail
