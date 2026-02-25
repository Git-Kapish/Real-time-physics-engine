#pragma once
/// @file Mat4.h
/// @brief Column-major 4×4 matrix for transforms and OpenGL projection.

#include "Vec3.h"
#include "Mat3.h"
#include <cassert>
#include <cmath>

// Forward-declare Quat so Mat4::TRS can accept it.
namespace physics { struct Quat; }

namespace physics {

/// A 4×4 matrix stored in column-major order (OpenGL convention).
struct Mat4 {
    float data[16]; ///< Column-major storage: col c, row r -> data[c*4 + r]

    // ── Constructors ──────────────────────────────────────────────────────

    /// Default constructor — zero matrix.
    Mat4() : data{} {}

    // ── Element access ────────────────────────────────────────────────────

    /// Read-write element at (row, col).
    float& operator()(int row, int col) {
        assert(row >= 0 && row < 4 && col >= 0 && col < 4);
        return data[col * 4 + row];
    }

    /// Read-only element at (row, col).
    float operator()(int row, int col) const {
        assert(row >= 0 && row < 4 && col >= 0 && col < 4);
        return data[col * 4 + row];
    }

    /// Raw pointer to column-major data (for OpenGL uniform uploads).
    const float* ptr() const { return data; }

    // ── Arithmetic ────────────────────────────────────────────────────────

    /// Matrix-matrix multiplication (this * m).
    Mat4 operator*(const Mat4& m) const {
        Mat4 r;
        for (int col = 0; col < 4; ++col) {
            for (int row = 0; row < 4; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += (*this)(row, k) * m(k, col);
                }
                r(row, col) = sum;
            }
        }
        return r;
    }

    // ── Static factories ──────────────────────────────────────────────────

    /// Return the 4×4 identity matrix.
    static Mat4 identity() {
        Mat4 m;
        m(0, 0) = 1.0f; m(1, 1) = 1.0f; m(2, 2) = 1.0f; m(3, 3) = 1.0f;
        return m;
    }

    /// Build a translation matrix.
    static Mat4 translation(const Vec3& t) {
        Mat4 m = identity();
        m(0, 3) = t.x;
        m(1, 3) = t.y;
        m(2, 3) = t.z;
        return m;
    }

    /// Build a uniform scale matrix.
    static Mat4 scale(float s) {
        Mat4 m;
        m(0, 0) = s; m(1, 1) = s; m(2, 2) = s; m(3, 3) = 1.0f;
        return m;
    }

    /// Build a non-uniform scale matrix.
    static Mat4 scale(const Vec3& s) {
        Mat4 m;
        m(0, 0) = s.x; m(1, 1) = s.y; m(2, 2) = s.z; m(3, 3) = 1.0f;
        return m;
    }

    /// Embed a Mat3 in the top-left 3×3; rest is identity.
    static Mat4 fromMat3(const Mat3& m3) {
        Mat4 m = identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                m(r, c) = m3(r, c);
        return m;
    }

    /// Build a TRS (Translation-Rotation-Scale) matrix.
    /// Implementation provided after Quat is fully defined (see Quat.h).
    static Mat4 TRS(const Vec3& pos, const Quat& rot, float s);

    /// Build a symmetric perspective projection matrix (OpenGL convention).
    static Mat4 perspective(float fovY, float aspect, float near, float far) {
        const float tanHalf = std::tan(fovY / 2.0f);
        Mat4 m;
        m(0, 0) = 1.0f / (aspect * tanHalf);
        m(1, 1) = 1.0f / tanHalf;
        m(2, 2) = -(far + near) / (far - near);
        m(2, 3) = -(2.0f * far * near) / (far - near);
        m(3, 2) = -1.0f;
        return m;
    }

    /// Build a lookAt view matrix (right-handed, OpenGL convention).
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        const Vec3 f = (center - eye).normalized();  // forward
        const Vec3 r = f.cross(up).normalized();      // right
        const Vec3 u = r.cross(f);                     // recomputed up

        Mat4 m = identity();
        m(0, 0) =  r.x;  m(0, 1) =  r.y;  m(0, 2) =  r.z;
        m(1, 0) =  u.x;  m(1, 1) =  u.y;  m(1, 2) =  u.z;
        m(2, 0) = -f.x;  m(2, 1) = -f.y;  m(2, 2) = -f.z;
        m(0, 3) = -r.dot(eye);
        m(1, 3) = -u.dot(eye);
        m(2, 3) =  f.dot(eye);
        return m;
    }
};

} // namespace physics
