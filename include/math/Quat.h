#pragma once
/// @file Quat.h
/// @brief Unit quaternion for 3D rotations.

#include "Vec3.h"
#include "Mat3.h"
#include "Mat4.h"
#include <cassert>
#include <cmath>

namespace physics {

/// A quaternion (w + xi + yj + zk) — w is the scalar part.
struct Quat {
    float w; ///< Scalar (real) part
    float x; ///< i component
    float y; ///< j component
    float z; ///< k component

    // ── Constructors ──────────────────────────────────────────────────────

    /// Default constructor — identity quaternion (no rotation).
    Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    /// Construct from explicit components.
    Quat(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    // ── Static factories ──────────────────────────────────────────────────

    /// Return the identity quaternion.
    static Quat identity() { return Quat{1.0f, 0.0f, 0.0f, 0.0f}; }

    /// Build a quaternion from an axis (must be unit-length) and angle in radians.
    static Quat fromAxisAngle(const Vec3& axis, float angleRadians) {
        const float half = angleRadians * 0.5f;
        const float s = std::sin(half);
        const float c = std::cos(half);
        return {c, axis.x * s, axis.y * s, axis.z * s};
    }

    // ── Hamilton product ──────────────────────────────────────────────────

    /// Quaternion multiplication (Hamilton product).
    Quat operator*(const Quat& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z,
                w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x,
                w * q.z + x * q.y - y * q.x + z * q.w};
    }

    /// Multiply-assign quaternion.
    Quat& operator*=(const Quat& q) { *this = *this * q; return *this; }

    // ── Quaternion operations ─────────────────────────────────────────────

    /// Return the conjugate (negated vector part).
    Quat conjugate() const { return {w, -x, -y, -z}; }

    /// Quaternion norm (length).
    float norm() const { return std::sqrt(normSq()); }

    /// Squared norm (avoids sqrt).
    float normSq() const { return w * w + x * x + y * y + z * z; }

    /// Return a unit-length quaternion; asserts norm > 1e-10.
    Quat normalized() const {
        const float n = norm();
        assert(n > 1e-10f && "Cannot normalize a near-zero quaternion");
        const float inv = 1.0f / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }

    /// Rotate a vector by this quaternion: v' = q * v * q*.
    Vec3 rotate(const Vec3& v) const {
        // Optimised form: v' = v + 2w(t) + 2(qv × t), where t = qv × v
        const Vec3 qv{x, y, z};
        const Vec3 t = qv.cross(v) * 2.0f;
        return v + t * w + qv.cross(t);
    }

    /// Convert to a 3×3 rotation matrix.
    Mat3 toMat3() const {
        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        const float wx = w * x, wy = w * y, wz = w * z;

        Mat3 m;
        m(0, 0) = 1.0f - 2.0f * (yy + zz);
        m(0, 1) = 2.0f * (xy - wz);
        m(0, 2) = 2.0f * (xz + wy);

        m(1, 0) = 2.0f * (xy + wz);
        m(1, 1) = 1.0f - 2.0f * (xx + zz);
        m(1, 2) = 2.0f * (yz - wx);

        m(2, 0) = 2.0f * (xz - wy);
        m(2, 1) = 2.0f * (yz + wx);
        m(2, 2) = 1.0f - 2.0f * (xx + yy);
        return m;
    }

    /// Integrate angular velocity: q' = normalize(q + 0.5 * dt * [0, omega] * q).
    Quat integrated(const Vec3& omega, float dt) const {
        const Quat omegaQ{0.0f, omega.x, omega.y, omega.z};
        const Quat dq = omegaQ * (*this);
        return Quat{w + 0.5f * dt * dq.w,
                     x + 0.5f * dt * dq.x,
                     y + 0.5f * dt * dq.y,
                     z + 0.5f * dt * dq.z}.normalized();
    }
};

// ── Deferred Mat4::TRS definition (needs complete Quat) ───────────────────

inline Mat4 Mat4::TRS(const Vec3& pos, const Quat& rot, float s) {
    return translation(pos) * fromMat3(rot.toMat3()) * scale(s);
}

} // namespace physics
