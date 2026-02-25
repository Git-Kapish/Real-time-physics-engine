#pragma once
/// @file Vec3.h
/// @brief 3-component floating-point vector for physics math.

#include <cassert>
#include <cmath>
#include <iostream>

namespace physics {

/// A 3D vector with float components.
struct Vec3 {
    float x; ///< X component
    float y; ///< Y component
    float z; ///< Z component

    // ── Constructors ──────────────────────────────────────────────────────

    /// Default constructor — initializes to (0, 0, 0).
    Vec3() : x(0.0f), y(0.0f), z(0.0f) {}

    /// Construct from individual components.
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    /// Construct with all components set to the same scalar.
    explicit Vec3(float s) : x(s), y(s), z(s) {}

    // ── Arithmetic operators (Vec3) ───────────────────────────────────────

    /// Component-wise addition.
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }

    /// Component-wise subtraction.
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }

    /// Component-wise multiplication by another vector.
    Vec3 operator*(const Vec3& v) const { return {x * v.x, y * v.y, z * v.z}; }

    /// Component-wise division by another vector.
    Vec3 operator/(const Vec3& v) const { return {x / v.x, y / v.y, z / v.z}; }

    // ── Arithmetic operators (scalar) ─────────────────────────────────────

    /// Multiply all components by a scalar.
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }

    /// Divide all components by a scalar.
    Vec3 operator/(float s) const {
        const float inv = 1.0f / s;
        return {x * inv, y * inv, z * inv};
    }

    /// Unary negation.
    Vec3 operator-() const { return {-x, -y, -z}; }

    // ── Compound assignment ───────────────────────────────────────────────

    /// Add another vector in-place.
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }

    /// Subtract another vector in-place.
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }

    /// Multiply by scalar in-place.
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }

    /// Divide by scalar in-place.
    Vec3& operator/=(float s) {
        const float inv = 1.0f / s;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }

    // ── Comparison ────────────────────────────────────────────────────────

    /// Exact equality.
    bool operator==(const Vec3& v) const { return x == v.x && y == v.y && z == v.z; }

    /// Exact inequality.
    bool operator!=(const Vec3& v) const { return !(*this == v); }

    // ── Geometric operations ──────────────────────────────────────────────

    /// Dot product with another vector.
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

    /// Cross product with another vector.
    Vec3 cross(const Vec3& v) const {
        return {y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x};
    }

    /// Euclidean length (magnitude).
    float length() const { return std::sqrt(lengthSq()); }

    /// Squared length (avoids sqrt).
    float lengthSq() const { return x * x + y * y + z * z; }

    /// Return a unit-length copy; asserts that length > 1e-10.
    Vec3 normalized() const {
        const float len = length();
        assert(len > 1e-10f && "Cannot normalize a near-zero vector");
        return *this / len;
    }

    /// Check whether this vector is approximately zero.
    bool isZero(float eps = 1e-6f) const { return lengthSq() < eps * eps; }

    /// Component-wise (Hadamard) product.
    Vec3 cwiseProduct(const Vec3& v) const { return {x * v.x, y * v.y, z * v.z}; }

    /// Component-wise absolute value.
    Vec3 cwiseAbs() const { return {std::fabs(x), std::fabs(y), std::fabs(z)}; }

    /// Largest component value.
    float maxCoeff() const { return std::fmax(x, std::fmax(y, z)); }

    /// Smallest component value.
    float minCoeff() const { return std::fmin(x, std::fmin(y, z)); }

    // ── Indexed access ────────────────────────────────────────────────────

    /// Read-write access by index (0=x, 1=y, 2=z).
    float& operator[](int i) {
        assert(i >= 0 && i < 3 && "Vec3 index out of range");
        return (&x)[i];
    }

    /// Read-only access by index.
    float operator[](int i) const {
        assert(i >= 0 && i < 3 && "Vec3 index out of range");
        return (&x)[i];
    }

    // ── Static factory helpers ────────────────────────────────────────────

    /// Return the zero vector (0, 0, 0).
    static Vec3 zero()  { return {0.0f, 0.0f, 0.0f}; }

    /// Return the X-axis unit vector (1, 0, 0).
    static Vec3 unitX() { return {1.0f, 0.0f, 0.0f}; }

    /// Return the Y-axis unit vector (0, 1, 0).
    static Vec3 unitY() { return {0.0f, 1.0f, 0.0f}; }

    /// Return the Z-axis unit vector (0, 0, 1).
    static Vec3 unitZ() { return {0.0f, 0.0f, 1.0f}; }

    // ── Friend: scalar * Vec3 ─────────────────────────────────────────────

    /// Left-multiply by scalar (s * v).
    friend Vec3 operator*(float s, const Vec3& v) { return v * s; }

    // ── Stream output ─────────────────────────────────────────────────────

    /// Print as "(x, y, z)".
    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
};

// ── Free-function convenience wrappers ────────────────────────────────────

/// Free-function dot product.
inline float dot(const Vec3& a, const Vec3& b) { return a.dot(b); }

/// Free-function cross product.
inline Vec3 cross(const Vec3& a, const Vec3& b) { return a.cross(b); }

/// Free-function length.
inline float length(const Vec3& v) { return v.length(); }

/// Free-function normalize.
inline Vec3 normalize(const Vec3& v) { return v.normalized(); }

} // namespace physics
