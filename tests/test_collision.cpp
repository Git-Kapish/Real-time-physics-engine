/// @file test_collision.cpp
/// @brief Google Test suite for Phase 3: CollisionDetector (broad phase + all shape pairs).

#include <gtest/gtest.h>
#include <cmath>

#include "physics/AABB.h"
#include "physics/RigidBody.h"
#include "physics/ContactManifold.h"
#include "physics/CollisionDetector.h"
#include "physics/PhysicsWorld.h"

using namespace physics;

static constexpr float kEps = 1e-4f;

// Helper: check two Vec3 are approximately equal.
static void ExpectVec3Near(const Vec3& a, const Vec3& b, float eps = kEps) {
    EXPECT_NEAR(a.x, b.x, eps) << "x mismatch";
    EXPECT_NEAR(a.y, b.y, eps) << "y mismatch";
    EXPECT_NEAR(a.z, b.z, eps) << "z mismatch";
}

// ═══════════════════════════════════════════════════════════════════════════
//  Broad Phase Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BroadPhaseTests, OnlyCloseSpherePairReturned) {
    // A and B overlap (close together), C is far away
    std::vector<RigidBody> bodies;
    bodies.push_back(RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0)));   // A
    bodies.push_back(RigidBody::makeSphere(1.0f, 1.0f, Vec3(1, 0, 0)));   // B – overlaps A
    bodies.push_back(RigidBody::makeSphere(1.0f, 1.0f, Vec3(100, 0, 0))); // C – far away

    auto pairs = CollisionDetector::broadPhase(bodies);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].first,  0);
    EXPECT_EQ(pairs[0].second, 1);
}

TEST(BroadPhaseTests, StaticStaticPairSkipped) {
    std::vector<RigidBody> bodies;
    bodies.push_back(RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f));
    bodies.push_back(RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f));

    auto pairs = CollisionDetector::broadPhase(bodies);
    EXPECT_EQ(pairs.size(), 0u);
}

TEST(BroadPhaseTests, DynamicStaticOverlapReturned) {
    std::vector<RigidBody> bodies;
    bodies.push_back(RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0)));
    bodies.push_back(RigidBody::makeStaticPlane(Vec3(0, 1, 0), -10.0f)); // d=-10 so sphere is above

    auto pairs = CollisionDetector::broadPhase(bodies);
    // Plane AABB is enormous so it always overlaps dynamic bodies
    EXPECT_GE(pairs.size(), 1u);
}

// ═══════════════════════════════════════════════════════════════════════════
//  sphereSphere Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(SphereSphereTests, SeparatedReturnsNullopt) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(3, 0, 0)); // dist=3 > rSum=2
    EXPECT_FALSE(CollisionDetector::sphereSphere(a, b).has_value());
}

TEST(SphereSphereTests, TouchingReturnsNullopt) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(2, 0, 0)); // dist=2 == rSum=2
    EXPECT_FALSE(CollisionDetector::sphereSphere(a, b).has_value());
}

TEST(SphereSphereTests, OverlapPenetrationDepth) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(1.5f, 0, 0)); // overlap = 2-1.5 = 0.5
    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->penetrationDepth, 0.5f, kEps);
}

TEST(SphereSphereTests, NormalPointsFromBToA) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(1.5f, 0, 0)); // overlap: dist=1.5 < rSum=2
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    // normal should point in +X direction (from b to a)
    EXPECT_GT(m->normal.x, 0.0f);
}

TEST(SphereSphereTests, ConcentricNoCrash) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    // Default normal should be unitY
    EXPECT_NEAR(m->normal.y, 1.0f, kEps);
}

TEST(SphereSphereTests, CombinedRestitution) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(1.5f, 0, 0));
    a.restitution = 0.4f; b.restitution = 0.6f;
    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->restitution, 0.4f, kEps); // min(0.4, 0.6)
}

TEST(SphereSphereTests, CombinedFriction) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(1.5f, 0, 0));
    a.friction = 0.25f; b.friction = 0.25f;
    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->friction, 0.25f, kEps); // sqrt(0.25*0.25)
}

// ═══════════════════════════════════════════════════════════════════════════
//  spherePlane Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(SpherePlaneTests, SphereAbovePlaneNoContact) {
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 5, 0)); // y=5, r=1, above y=0 plane
    RigidBody plane  = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    EXPECT_FALSE(CollisionDetector::spherePlane(sphere, plane).has_value());
}

TEST(SpherePlaneTests, SphereTouchingPlaneNoContact) {
    // signedDist = 1, radius = 1 → penetration = 0 → nullopt
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 1, 0));
    RigidBody plane  = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    EXPECT_FALSE(CollisionDetector::spherePlane(sphere, plane).has_value());
}

TEST(SpherePlaneTests, SpherePenetrating) {
    // sphere at y=0.7, radius=1 → signedDist=0.7 → penetration=0.3
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0.7f, 0));
    RigidBody plane  = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    auto m = CollisionDetector::spherePlane(sphere, plane);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->penetrationDepth, 0.3f, kEps);
}

TEST(SpherePlaneTests, NormalEqualsPlaneNormal) {
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0.5f, 0));
    RigidBody plane  = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    auto m = CollisionDetector::spherePlane(sphere, plane);
    ASSERT_TRUE(m.has_value());
    ExpectVec3Near(m->normal, Vec3(0, 1, 0));
}

TEST(SpherePlaneTests, ContactPointOnSphereSurface) {
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0.7f, 0));
    RigidBody plane  = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    auto m = CollisionDetector::spherePlane(sphere, plane);
    ASSERT_TRUE(m.has_value());
    // contact = sphere.pos - normal * radius = (0, 0.7-1, 0) = (0, -0.3, 0)
    EXPECT_NEAR(m->contactPoint.y, -0.3f, kEps);
}

// ═══════════════════════════════════════════════════════════════════════════
//  sphereBox Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(SphereBoxTests, SeparatedReturnsNullopt) {
    RigidBody sphere = RigidBody::makeSphere(0.5f, 1.0f, Vec3(5, 0, 0));
    RigidBody box    = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    EXPECT_FALSE(CollisionDetector::sphereBox(sphere, box).has_value());
}

TEST(SphereBoxTests, SphereOverlappingFace) {
    // Sphere touching +X face of unit box with slight overlap
    RigidBody sphere = RigidBody::makeSphere(0.5f, 1.0f, Vec3(1.4f, 0, 0));
    RigidBody box    = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    auto m = CollisionDetector::sphereBox(sphere, box);
    ASSERT_TRUE(m.has_value());
    EXPECT_GT(m->penetrationDepth, 0.0f);
}

TEST(SphereBoxTests, SphereCentreInsideBox) {
    // Sphere fully inside box — should still return a valid manifold
    RigidBody sphere = RigidBody::makeSphere(0.1f, 1.0f, Vec3(0, 0, 0));
    RigidBody box    = RigidBody::makeBox(Vec3(2, 2, 2), 1.0f, Vec3(0, 0, 0));
    auto m = CollisionDetector::sphereBox(sphere, box);
    ASSERT_TRUE(m.has_value());
    EXPECT_GT(m->penetrationDepth, 0.0f);
    // Normal must be unit length
    EXPECT_NEAR(m->normal.length(), 1.0f, kEps);
}

TEST(SphereBoxTests, NormalPointsTowardSphere) {
    // Sphere to the +X side of box
    RigidBody sphere = RigidBody::makeSphere(0.5f, 1.0f, Vec3(1.4f, 0, 0));
    RigidBody box    = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    auto m = CollisionDetector::sphereBox(sphere, box);
    ASSERT_TRUE(m.has_value());
    // Normal should point in +X direction (from box toward sphere)
    EXPECT_GT(m->normal.x, 0.5f);
}

// ═══════════════════════════════════════════════════════════════════════════
//  boxPlane Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BoxPlaneTests, BoxFloatingAbovePlane) {
    RigidBody box   = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 5, 0));
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    EXPECT_FALSE(CollisionDetector::boxPlane(box, plane).has_value());
}

TEST(BoxPlaneTests, BoxCornerBelowPlane) {
    // Box at y=0.5, half=1 → bottom corners at y=-0.5 → below plane at y=0
    RigidBody box   = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0.5f, 0));
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    auto m = CollisionDetector::boxPlane(box, plane);
    ASSERT_TRUE(m.has_value());
    EXPECT_GT(m->penetrationDepth, 0.0f);
}

TEST(BoxPlaneTests, DeepestCornerReturned) {
    // Box at y=0, half=1 → all 4 bottom corners at y=-1
    RigidBody box   = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    auto m = CollisionDetector::boxPlane(box, plane);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->penetrationDepth, 1.0f, kEps);
}

TEST(BoxPlaneTests, NormalEqualsPlaneNormal) {
    RigidBody box   = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    auto m = CollisionDetector::boxPlane(box, plane);
    ASSERT_TRUE(m.has_value());
    ExpectVec3Near(m->normal, Vec3(0, 1, 0));
}

// ═══════════════════════════════════════════════════════════════════════════
//  boxBox Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BoxBoxTests, SeparatedOnXAxisReturnsNullopt) {
    RigidBody a = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(5, 0, 0)); // gap on X
    EXPECT_FALSE(CollisionDetector::boxBox(a, b).has_value());
}

TEST(BoxBoxTests, OverlapOnX) {
    // half=1 each, centers 1.5 apart → overlap on X = 2+2-1.5 = ... wait half=1 so full=2
    // centers at 0 and 1.5 → overlap = 1+1-1.5 = 0.5
    RigidBody a = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(1.5f, 0, 0));
    auto m = CollisionDetector::boxBox(a, b);
    ASSERT_TRUE(m.has_value());
    EXPECT_NEAR(m->penetrationDepth, 0.5f, kEps);
}

TEST(BoxBoxTests, ColocatedBoxesNoCrash) {
    RigidBody a = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0, 0, 0));
    auto m = CollisionDetector::boxBox(a, b);
    ASSERT_TRUE(m.has_value());
    EXPECT_GT(m->penetrationDepth, 0.0f);
    EXPECT_NEAR(m->normal.length(), 1.0f, kEps);
}

TEST(BoxBoxTests, NormalPointsFromBToA) {
    // a to the +X side of b
    RigidBody a = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(1.5f, 0, 0));
    RigidBody b = RigidBody::makeBox(Vec3(1, 1, 1), 1.0f, Vec3(0,   0, 0));
    auto m = CollisionDetector::boxBox(a, b);
    ASSERT_TRUE(m.has_value());
    EXPECT_EQ(m->bodyA, &a);
    EXPECT_GT(m->normal.x, 0.0f); // points from b toward a (+X)
}

// ═══════════════════════════════════════════════════════════════════════════
//  relativeVelocityAtContact Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(RelativeVelocityTests, BothAtRestIsZero) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(1.5f, 0, 0));
    a.updateInertiaTensor(); b.updateInertiaTensor();

    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    Vec3 rv = m->relativeVelocityAtContact();
    EXPECT_NEAR(rv.x, 0.0f, kEps);
    EXPECT_NEAR(rv.y, 0.0f, kEps);
    EXPECT_NEAR(rv.z, 0.0f, kEps);
}

TEST(RelativeVelocityTests, LinearVelocityContribution) {
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(1.5f, 0, 0));
    a.linearVelocity = Vec3(1, 0, 0);
    a.updateInertiaTensor(); b.updateInertiaTensor();

    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    Vec3 rv = m->relativeVelocityAtContact();
    // vA = (1,0,0), vB = (0,0,0) → rv ≈ (1,0,0)
    EXPECT_NEAR(rv.x, 1.0f, kEps);
}

TEST(RelativeVelocityTests, AngularContribution) {
    // bodyA spinning around Z → contact point is offset in X → angular contribution in Y
    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(1.5f, 0, 0));
    a.angularVelocity = Vec3(0, 0, 1); // spin around Z
    a.updateInertiaTensor(); b.updateInertiaTensor();

    auto m = CollisionDetector::sphereSphere(a, b);
    ASSERT_TRUE(m.has_value());
    Vec3 rv = m->relativeVelocityAtContact();
    // rA = contactPoint - a.position: contactPoint is at (0.5,0,0) → rA=(0.5,0,0)
    // v_angular = omega × rA = (0,0,1) × (0.5,0,0) = (0*0-1*0, 1*0.5-0*0, 0*0-0*0.5) = (0,0.5,0)
    // So rv.y should be non-zero
    EXPECT_GT(std::fabs(rv.y), 0.05f);
}
