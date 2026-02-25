/// @file test_math.cpp
/// @brief Google Test suite for the Phase 1 math library (Vec3, Mat3, Mat4, Quat).

#include <gtest/gtest.h>
#include <cmath>

#include "math/Vec3.h"
#include "math/Mat3.h"
#include "math/Mat4.h"
#include "math/Quat.h"

using namespace physics;

// ── Tolerance constant ────────────────────────────────────────────────────
static constexpr float kEps = 1e-5f;

// A small helper: check two Vec3 are approximately equal.
static void ExpectVec3Near(const Vec3& a, const Vec3& b, float eps = kEps) {
    EXPECT_NEAR(a.x, b.x, eps);
    EXPECT_NEAR(a.y, b.y, eps);
    EXPECT_NEAR(a.z, b.z, eps);
}

// A small helper: check two Mat3 are approximately equal.
static void ExpectMat3Near(const Mat3& a, const Mat3& b, float eps = kEps) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_NEAR(a(r, c), b(r, c), eps) << "at (" << r << ", " << c << ")";
}

// ═══════════════════════════════════════════════════════════════════════════
//  Vec3 Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(Vec3Tests, DefaultConstructorGivesZero) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST(Vec3Tests, ParameterizedConstructor) {
    Vec3 v(1.0f, 2.0f, 3.0f);
    EXPECT_FLOAT_EQ(v.x, 1.0f);
    EXPECT_FLOAT_EQ(v.y, 2.0f);
    EXPECT_FLOAT_EQ(v.z, 3.0f);
}

TEST(Vec3Tests, Addition) {
    Vec3 a(1, 2, 3), b(4, 5, 6);
    Vec3 c = a + b;
    ExpectVec3Near(c, Vec3(5, 7, 9));
}

TEST(Vec3Tests, Subtraction) {
    Vec3 a(5, 7, 9), b(1, 2, 3);
    Vec3 c = a - b;
    ExpectVec3Near(c, Vec3(4, 5, 6));
}

TEST(Vec3Tests, ScalarMultiply) {
    Vec3 v(1, 2, 3);
    Vec3 r = v * 2.0f;
    ExpectVec3Near(r, Vec3(2, 4, 6));
}

TEST(Vec3Tests, ScalarDivide) {
    Vec3 v(2, 4, 6);
    Vec3 r = v / 2.0f;
    ExpectVec3Near(r, Vec3(1, 2, 3));
}

TEST(Vec3Tests, UnaryMinus) {
    Vec3 v(1, -2, 3);
    Vec3 r = -v;
    ExpectVec3Near(r, Vec3(-1, 2, -3));
}

TEST(Vec3Tests, DotProductOrthogonal) {
    EXPECT_FLOAT_EQ(Vec3(1, 0, 0).dot(Vec3(0, 1, 0)), 0.0f);
}

TEST(Vec3Tests, DotProductNonOrthogonal) {
    EXPECT_FLOAT_EQ(Vec3(1, 1, 0).dot(Vec3(1, 1, 0)), 2.0f);
}

TEST(Vec3Tests, CrossProduct) {
    Vec3 r = Vec3::unitX().cross(Vec3::unitY());
    ExpectVec3Near(r, Vec3::unitZ());
}

TEST(Vec3Tests, Length) {
    EXPECT_FLOAT_EQ(Vec3(3, 4, 0).length(), 5.0f);
}

TEST(Vec3Tests, Normalize) {
    Vec3 n = Vec3(3, 4, 0).normalized();
    EXPECT_NEAR(n.length(), 1.0f, kEps);
}

TEST(Vec3Tests, CwiseProduct) {
    Vec3 a(1, 2, 3), b(4, 5, 6);
    Vec3 r = a.cwiseProduct(b);
    ExpectVec3Near(r, Vec3(4, 10, 18));
}

TEST(Vec3Tests, OperatorBracket) {
    Vec3 v(10, 20, 30);
    EXPECT_FLOAT_EQ(v[0], 10.0f);
    EXPECT_FLOAT_EQ(v[1], 20.0f);
    EXPECT_FLOAT_EQ(v[2], 30.0f);
}

TEST(Vec3Tests, IsZero) {
    EXPECT_TRUE(Vec3(0, 0, 0).isZero());
    EXPECT_FALSE(Vec3(1, 0, 0).isZero());
}

TEST(Vec3Tests, FriendScalarMultiply) {
    Vec3 v(1, 2, 3);
    Vec3 r = 3.0f * v;
    ExpectVec3Near(r, Vec3(3, 6, 9));
}

// ═══════════════════════════════════════════════════════════════════════════
//  Mat3 Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(Mat3Tests, IdentityTimesIdentity) {
    Mat3 I = Mat3::identity();
    Mat3 r = I * I;
    ExpectMat3Near(r, I);
}

TEST(Mat3Tests, IdentityTimesVector) {
    Mat3 I = Mat3::identity();
    Vec3 v(1, 2, 3);
    ExpectVec3Near(I * v, v);
}

TEST(Mat3Tests, MatrixMultiply) {
    // A = [[1,2,3],[4,5,6],[7,8,9]]
    Mat3 A;
    A(0,0)=1; A(0,1)=2; A(0,2)=3;
    A(1,0)=4; A(1,1)=5; A(1,2)=6;
    A(2,0)=7; A(2,1)=8; A(2,2)=9;

    // B = [[9,8,7],[6,5,4],[3,2,1]]
    Mat3 B;
    B(0,0)=9; B(0,1)=8; B(0,2)=7;
    B(1,0)=6; B(1,1)=5; B(1,2)=4;
    B(2,0)=3; B(2,1)=2; B(2,2)=1;

    Mat3 C = A * B;
    // Known result: A*B = [[30,24,18],[84,69,54],[138,114,90]]
    EXPECT_NEAR(C(0,0), 30,  kEps);
    EXPECT_NEAR(C(0,1), 24,  kEps);
    EXPECT_NEAR(C(0,2), 18,  kEps);
    EXPECT_NEAR(C(1,0), 84,  kEps);
    EXPECT_NEAR(C(1,1), 69,  kEps);
    EXPECT_NEAR(C(1,2), 54,  kEps);
    EXPECT_NEAR(C(2,0), 138, kEps);
    EXPECT_NEAR(C(2,1), 114, kEps);
    EXPECT_NEAR(C(2,2), 90,  kEps);
}

TEST(Mat3Tests, TransposeRoundTrip) {
    Mat3 A;
    A(0,0)=1; A(0,1)=2; A(0,2)=3;
    A(1,0)=4; A(1,1)=5; A(1,2)=6;
    A(2,0)=7; A(2,1)=8; A(2,2)=9;
    ExpectMat3Near(A.transposed().transposed(), A);
}

TEST(Mat3Tests, Determinant) {
    // A = [[6,1,1],[4,-2,5],[2,8,7]]  det = -306
    Mat3 A;
    A(0,0)=6; A(0,1)=1; A(0,2)=1;
    A(1,0)=4; A(1,1)=-2; A(1,2)=5;
    A(2,0)=2; A(2,1)=8; A(2,2)=7;
    EXPECT_NEAR(A.determinant(), -306.0f, kEps);
}

TEST(Mat3Tests, InverseTimesOriginalIsIdentity) {
    Mat3 A;
    A(0,0)=6; A(0,1)=1; A(0,2)=1;
    A(1,0)=4; A(1,1)=-2; A(1,2)=5;
    A(2,0)=2; A(2,1)=8; A(2,2)=7;
    ExpectMat3Near(A * A.inverse(), Mat3::identity());
}

TEST(Mat3Tests, SkewSymmetricProperty) {
    Vec3 v(1, 2, 3), u(4, 5, 6);
    Vec3 cross_result = v.cross(u);
    Vec3 skew_result  = Mat3::skew(v) * u;
    ExpectVec3Near(skew_result, cross_result);
}

TEST(Mat3Tests, Diagonal) {
    Mat3 D = Mat3::diagonal(2, 3, 4);
    EXPECT_FLOAT_EQ(D(0,0), 2.0f);
    EXPECT_FLOAT_EQ(D(1,1), 3.0f);
    EXPECT_FLOAT_EQ(D(2,2), 4.0f);
    EXPECT_FLOAT_EQ(D(0,1), 0.0f);
    EXPECT_FLOAT_EQ(D(1,0), 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Mat4 Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(Mat4Tests, IdentityTimesIdentity) {
    Mat4 I = Mat4::identity();
    Mat4 r = I * I;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            EXPECT_NEAR(r(row, col), I(row, col), kEps)
                << "at (" << row << ", " << col << ")";
}

TEST(Mat4Tests, TranslationAppliedToPoint) {
    Mat4 T = Mat4::translation(Vec3(1, 2, 3));
    // Extend point (4,5,6) to homogeneous (4,5,6,1)
    // Multiply and read xyz
    float pt[4] = {4, 5, 6, 1};
    float out[4] = {};
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r] += T(r, c) * pt[c];
    EXPECT_NEAR(out[0], 5.0f, kEps);
    EXPECT_NEAR(out[1], 7.0f, kEps);
    EXPECT_NEAR(out[2], 9.0f, kEps);
    EXPECT_NEAR(out[3], 1.0f, kEps);
}

TEST(Mat4Tests, PerspectiveNegativeZ) {
    const float fov  = 3.14159265f / 4.0f; // 45 degrees
    const float asp  = 16.0f / 9.0f;
    const float near_val = 0.1f;
    const float far_val  = 100.0f;
    Mat4 P = Mat4::perspective(fov, asp, near_val, far_val);
    // m(2,2) should be negative for standard OpenGL convention
    EXPECT_LT(P(2, 2), 0.0f);
}

TEST(Mat4Tests, LookAtTranslation) {
    Vec3 eye(0, 0, 5), center(0, 0, 0), up(0, 1, 0);
    Mat4 V = Mat4::lookAt(eye, center, up);
    // The view-space translation along -Z should be -5
    EXPECT_NEAR(V(2, 3), -5.0f, kEps);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Quat Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(QuatTests, IdentityRotation) {
    Quat q = Quat::identity();
    Vec3 v(1, 0, 0);
    ExpectVec3Near(q.rotate(v), v);
}

TEST(QuatTests, Rotate90AroundZ) {
    // Rotating unitX by 90° around Z should give unitY
    const float halfPi = 3.14159265f / 2.0f;
    Quat q = Quat::fromAxisAngle(Vec3::unitZ(), halfPi);
    Vec3 r = q.rotate(Vec3::unitX());
    ExpectVec3Near(r, Vec3::unitY());
}

TEST(QuatTests, Rotate180AroundX) {
    // Rotating unitY by 180° around X should give -unitY
    const float pi = 3.14159265f;
    Quat q = Quat::fromAxisAngle(Vec3::unitX(), pi);
    Vec3 r = q.rotate(Vec3::unitY());
    ExpectVec3Near(r, -Vec3::unitY());
}

TEST(QuatTests, QTimesConjugateIsIdentity) {
    Quat q = Quat::fromAxisAngle(Vec3(0, 1, 0), 1.23f);
    Quat r = q * q.conjugate();
    EXPECT_NEAR(r.w, 1.0f, kEps);
    EXPECT_NEAR(r.x, 0.0f, kEps);
    EXPECT_NEAR(r.y, 0.0f, kEps);
    EXPECT_NEAR(r.z, 0.0f, kEps);
}

TEST(QuatTests, NormalizedHasUnitNorm) {
    Quat q(2, 3, 4, 5);
    Quat n = q.normalized();
    EXPECT_NEAR(n.norm(), 1.0f, kEps);
}

TEST(QuatTests, ToMat3MatchesKnown) {
    // 90° around Z: expected rotation matrix
    const float halfPi = 3.14159265f / 2.0f;
    Quat q = Quat::fromAxisAngle(Vec3::unitZ(), halfPi);
    Mat3 R = q.toMat3();

    // Expected: [[0,-1,0],[1,0,0],[0,0,1]]
    EXPECT_NEAR(R(0, 0),  0.0f, kEps);
    EXPECT_NEAR(R(0, 1), -1.0f, kEps);
    EXPECT_NEAR(R(0, 2),  0.0f, kEps);
    EXPECT_NEAR(R(1, 0),  1.0f, kEps);
    EXPECT_NEAR(R(1, 1),  0.0f, kEps);
    EXPECT_NEAR(R(1, 2),  0.0f, kEps);
    EXPECT_NEAR(R(2, 0),  0.0f, kEps);
    EXPECT_NEAR(R(2, 1),  0.0f, kEps);
    EXPECT_NEAR(R(2, 2),  1.0f, kEps);
}

TEST(QuatTests, IntegratedZeroOmega) {
    Quat q = Quat::identity();
    Quat r = q.integrated(Vec3(0, 0, 0), 1.0f);
    EXPECT_NEAR(r.w, 1.0f, kEps);
    EXPECT_NEAR(r.x, 0.0f, kEps);
    EXPECT_NEAR(r.y, 0.0f, kEps);
    EXPECT_NEAR(r.z, 0.0f, kEps);
}
