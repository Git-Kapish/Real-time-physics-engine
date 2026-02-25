#pragma once
/// @file Mat3.h
/// @brief Column-major 3×3 matrix for physics math.

#include "Vec3.h"
#include <cassert>
#include <cmath>

namespace physics {

/// A 3×3 matrix stored in column-major order.
struct Mat3 {
    float data[9]; ///< Column-major storage: col c, row r -> data[c*3 + r]

    // ── Constructors ──────────────────────────────────────────────────────

    /// Default constructor — zero matrix.
    Mat3() : data{} {}

    // ── Element access ────────────────────────────────────────────────────

    /// Read-write element at (row, col), column-major layout.
    float& operator()(int row, int col) {
        assert(row >= 0 && row < 3 && col >= 0 && col < 3);
        return data[col * 3 + row];
    }

    /// Read-only element at (row, col).
    float operator()(int row, int col) const {
        assert(row >= 0 && row < 3 && col >= 0 && col < 3);
        return data[col * 3 + row];
    }

    // ── Arithmetic ────────────────────────────────────────────────────────

    /// Matrix addition.
    Mat3 operator+(const Mat3& m) const {
        Mat3 r;
        for (int i = 0; i < 9; ++i) r.data[i] = data[i] + m.data[i];
        return r;
    }

    /// Scalar multiplication.
    Mat3 operator*(float s) const {
        Mat3 r;
        for (int i = 0; i < 9; ++i) r.data[i] = data[i] * s;
        return r;
    }

    /// Matrix-matrix multiplication (this * m).
    Mat3 operator*(const Mat3& m) const {
        Mat3 r;
        for (int col = 0; col < 3; ++col) {
            for (int row = 0; row < 3; ++row) {
                float sum = 0.0f;
                for (int k = 0; k < 3; ++k) {
                    sum += (*this)(row, k) * m(k, col);
                }
                r(row, col) = sum;
            }
        }
        return r;
    }

    /// Matrix-vector multiplication (this * v).
    Vec3 operator*(const Vec3& v) const {
        return {(*this)(0, 0) * v.x + (*this)(0, 1) * v.y + (*this)(0, 2) * v.z,
                (*this)(1, 0) * v.x + (*this)(1, 1) * v.y + (*this)(1, 2) * v.z,
                (*this)(2, 0) * v.x + (*this)(2, 1) * v.y + (*this)(2, 2) * v.z};
    }

    // ── Matrix operations ─────────────────────────────────────────────────

    /// Return the transpose.
    Mat3 transposed() const {
        Mat3 r;
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                r(row, col) = (*this)(col, row);
        return r;
    }

    /// Compute the determinant.
    float determinant() const {
        const float a = (*this)(0, 0), b = (*this)(0, 1), c = (*this)(0, 2);
        const float d = (*this)(1, 0), e = (*this)(1, 1), f = (*this)(1, 2);
        const float g = (*this)(2, 0), h = (*this)(2, 1), i = (*this)(2, 2);
        return a * (e * i - f * h)
             - b * (d * i - f * g)
             + c * (d * h - e * g);
    }

    /// Compute the inverse; asserts that the determinant is non-zero.
    Mat3 inverse() const {
        const float det = determinant();
        assert(std::fabs(det) > 1e-10f && "Cannot invert singular Mat3");

        const float invDet = 1.0f / det;

        const float a = (*this)(0, 0), b = (*this)(0, 1), c = (*this)(0, 2);
        const float d = (*this)(1, 0), e = (*this)(1, 1), f = (*this)(1, 2);
        const float g = (*this)(2, 0), h = (*this)(2, 1), i = (*this)(2, 2);

        Mat3 r;
        r(0, 0) = (e * i - f * h) * invDet;
        r(0, 1) = (c * h - b * i) * invDet;
        r(0, 2) = (b * f - c * e) * invDet;
        r(1, 0) = (f * g - d * i) * invDet;
        r(1, 1) = (a * i - c * g) * invDet;
        r(1, 2) = (c * d - a * f) * invDet;
        r(2, 0) = (d * h - e * g) * invDet;
        r(2, 1) = (b * g - a * h) * invDet;
        r(2, 2) = (a * e - b * d) * invDet;
        return r;
    }

    // ── Static factories ──────────────────────────────────────────────────

    /// Return the 3×3 identity matrix.
    static Mat3 identity() {
        Mat3 m;
        m(0, 0) = 1.0f; m(1, 1) = 1.0f; m(2, 2) = 1.0f;
        return m;
    }

    /// Return the 3×3 zero matrix.
    static Mat3 zero() { return Mat3{}; }

    /// Return the skew-symmetric (cross-product) matrix for a vector.
    /// skew(v) * u == v.cross(u)
    static Mat3 skew(const Vec3& v) {
        Mat3 m;
        m(0, 1) = -v.z;  m(0, 2) =  v.y;
        m(1, 0) =  v.z;  m(1, 2) = -v.x;
        m(2, 0) = -v.y;  m(2, 1) =  v.x;
        return m;
    }

    /// Return a diagonal matrix.
    static Mat3 diagonal(float d0, float d1, float d2) {
        Mat3 m;
        m(0, 0) = d0; m(1, 1) = d1; m(2, 2) = d2;
        return m;
    }
};

} // namespace physics
