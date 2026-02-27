/// @file test_physics.cpp
/// @brief Google Test suite for Phase 2: AABB, RigidBody, PhysicsWorld.

#include <gtest/gtest.h>
#include <cmath>

#include "physics/AABB.h"
#include "physics/RigidBody.h"
#include "physics/PhysicsWorld.h"

using namespace physics;

static constexpr float kEps  = 1e-5f;
static constexpr float kEps5 = 5e-2f;  // 5% margin for energy test

// ═══════════════════════════════════════════════════════════════════════════
//  AABB Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(AABBTests, FromSphere) {
    AABB a = AABB::fromSphere(Vec3(0, 0, 0), 1.0f);
    EXPECT_NEAR(a.min.x, -1.0f, kEps);
    EXPECT_NEAR(a.min.y, -1.0f, kEps);
    EXPECT_NEAR(a.min.z, -1.0f, kEps);
    EXPECT_NEAR(a.max.x,  1.0f, kEps);
    EXPECT_NEAR(a.max.y,  1.0f, kEps);
    EXPECT_NEAR(a.max.z,  1.0f, kEps);
}

TEST(AABBTests, OverlapsFalseWhenSeparated) {
    AABB a(Vec3(0, 0, 0), Vec3(1, 1, 1));
    AABB b(Vec3(2, 2, 2), Vec3(3, 3, 3));
    EXPECT_FALSE(a.overlaps(b));
    EXPECT_FALSE(b.overlaps(a));
}

TEST(AABBTests, OverlapsTrueWhenIntersecting) {
    AABB a(Vec3(0, 0, 0), Vec3(2, 2, 2));
    AABB b(Vec3(1, 1, 1), Vec3(3, 3, 3));
    EXPECT_TRUE(a.overlaps(b));
    EXPECT_TRUE(b.overlaps(a));
}

TEST(AABBTests, OverlapsFalseOnTouchingEdge) {
    // Touching exactly at x=1 — strict overlap means this is NOT overlapping
    AABB a(Vec3(0, 0, 0), Vec3(1, 1, 1));
    AABB b(Vec3(1, 0, 0), Vec3(2, 1, 1));
    EXPECT_FALSE(a.overlaps(b));
}

TEST(AABBTests, MergedContainsBoth) {
    AABB a(Vec3(0, 0, 0), Vec3(1, 1, 1));
    AABB b(Vec3(-1, -1, -1), Vec3(0.5f, 0.5f, 0.5f));
    AABB m = a.merged(b);
    EXPECT_LE(m.min.x, a.min.x);  EXPECT_LE(m.min.x, b.min.x);
    EXPECT_LE(m.min.y, a.min.y);  EXPECT_LE(m.min.y, b.min.y);
    EXPECT_LE(m.min.z, a.min.z);  EXPECT_LE(m.min.z, b.min.z);
    EXPECT_GE(m.max.x, a.max.x);  EXPECT_GE(m.max.x, b.max.x);
    EXPECT_GE(m.max.y, a.max.y);  EXPECT_GE(m.max.y, b.max.y);
    EXPECT_GE(m.max.z, a.max.z);  EXPECT_GE(m.max.z, b.max.z);
}

TEST(AABBTests, SurfaceAreaKnownBox) {
    // Box: 2x4x6  ->  SA = 2*(2*4 + 2*6 + 4*6) = 2*(8+12+24) = 88
    AABB a(Vec3(0, 0, 0), Vec3(2, 4, 6));
    EXPECT_NEAR(a.surfaceArea(), 88.0f, kEps);
}

// ═══════════════════════════════════════════════════════════════════════════
//  RigidBody Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(RigidBodyTests, MakeSphereR1M1) {
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3::zero());
    EXPECT_NEAR(b.invMass, 1.0f, kEps);
    // I = 2/5 * 1 * 1 = 0.4  ->  invI diagonal = 2.5
    const float expectedInvI = 1.0f / ((2.0f / 5.0f) * 1.0f * 1.0f);
    EXPECT_NEAR(b.invInertiaTensorLocal(0, 0), expectedInvI, kEps);
    EXPECT_NEAR(b.invInertiaTensorLocal(1, 1), expectedInvI, kEps);
    EXPECT_NEAR(b.invInertiaTensorLocal(2, 2), expectedInvI, kEps);
}

TEST(RigidBodyTests, MakeSphereR2M3) {
    RigidBody b = RigidBody::makeSphere(2.0f, 3.0f, Vec3::zero());
    // I = 2/5 * 3 * 4 = 4.8
    const float expectedI = (2.0f / 5.0f) * 3.0f * 4.0f;
    EXPECT_NEAR(b.inertiaTensorLocal(0, 0), expectedI, kEps);
    EXPECT_NEAR(b.inertiaTensorLocal(1, 1), expectedI, kEps);
    EXPECT_NEAR(b.inertiaTensorLocal(2, 2), expectedI, kEps);
}

TEST(RigidBodyTests, MakeBoxHalf1M12) {
    // half=(1,1,1), m=12 -> w=h=d=2
    // Ix = Iy = Iz = (12/12)*(4+4) = 8
    RigidBody b = RigidBody::makeBox(Vec3(1, 1, 1), 12.0f, Vec3::zero());
    const float expectedI = (12.0f / 12.0f) * (4.0f + 4.0f);
    EXPECT_NEAR(b.inertiaTensorLocal(0, 0), expectedI, kEps);
    EXPECT_NEAR(b.inertiaTensorLocal(1, 1), expectedI, kEps);
    EXPECT_NEAR(b.inertiaTensorLocal(2, 2), expectedI, kEps);
}

TEST(RigidBodyTests, MakeStaticPlane) {
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    EXPECT_FLOAT_EQ(plane.invMass, 0.0f);
    EXPECT_EQ(plane.bodyType, BodyType::Static);
    EXPECT_TRUE(plane.isStatic());
}

TEST(RigidBodyTests, ApplyImpulseChangesVelocity) {
    RigidBody b = RigidBody::makeSphere(1.0f, 2.0f, Vec3::zero()); // invMass=0.5
    b.updateInertiaTensor();
    b.applyImpulse(Vec3(4.0f, 0.0f, 0.0f));
    // dv = impulse * invMass = 4 * 0.5 = 2
    EXPECT_NEAR(b.linearVelocity.x, 2.0f, kEps);
    EXPECT_NEAR(b.linearVelocity.y, 0.0f, kEps);
    EXPECT_NEAR(b.linearVelocity.z, 0.0f, kEps);
}

TEST(RigidBodyTests, StaticBodyIgnoresForce) {
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    plane.applyForce(Vec3(100, 100, 100));
    EXPECT_FLOAT_EQ(plane.forceAccum.x, 0.0f);
    EXPECT_FLOAT_EQ(plane.linearVelocity.x, 0.0f);
}

TEST(RigidBodyTests, ClearForces) {
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3::zero());
    b.applyForce(Vec3(5, 5, 5));
    b.applyTorque(Vec3(1, 2, 3));
    b.clearForces();
    EXPECT_FLOAT_EQ(b.forceAccum.x, 0.0f);
    EXPECT_FLOAT_EQ(b.forceAccum.y, 0.0f);
    EXPECT_FLOAT_EQ(b.forceAccum.z, 0.0f);
    EXPECT_FLOAT_EQ(b.torqueAccum.x, 0.0f);
    EXPECT_FLOAT_EQ(b.torqueAccum.y, 0.0f);
    EXPECT_FLOAT_EQ(b.torqueAccum.z, 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
//  PhysicsWorld Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(PhysicsWorldTests, ForceIntegrationNoGravity) {
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);  // no gravity
    PhysicsWorld world(cfg);

    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3::zero()); // invMass=1
    int id = world.addBody(sphere);

    // Apply F=(1,0,0) before step — we must do it after adding
    world.getBody(id)->applyForce(Vec3(1.0f, 0.0f, 0.0f));
    world.step();

    const RigidBody* b = world.getBody(id);
    const float dt = cfg.fixedDt;

    // Semi-implicit: v = F/m * dt = 1 * dt
    EXPECT_NEAR(b->linearVelocity.x, 1.0f * dt, kEps);
    // Position uses new velocity: x = v * dt
    EXPECT_NEAR(b->position.x, 1.0f * dt * dt, kEps);
}

TEST(PhysicsWorldTests, GravityIntegrationOneStep) {
    PhysicsConfig cfg; // default gravity (0, -9.81, 0)
    PhysicsWorld world(cfg);

    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3::zero());
    int id = world.addBody(sphere);
    world.step();

    const RigidBody* b = world.getBody(id);
    const float dt = cfg.fixedDt;
    const float g  = -9.81f;

    EXPECT_NEAR(b->linearVelocity.y, g * dt, kEps);
    EXPECT_NEAR(b->position.y,       g * dt * dt, kEps);
}

TEST(PhysicsWorldTests, StaticBodyUnaffectedByGravity) {
    PhysicsWorld world;
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    int id = world.addBody(plane);

    for (int i = 0; i < 10; ++i) world.step();

    const RigidBody* b = world.getBody(id);
    EXPECT_FLOAT_EQ(b->position.y, 0.0f);
    EXPECT_FLOAT_EQ(b->linearVelocity.y, 0.0f);
}

TEST(PhysicsWorldTests, EnergyConservationFreeFall) {
    // Drop sphere from h=10, run until y≈0, check speed ≈ sqrt(2*g*h)
    const float h  = 10.0f;
    const float g  =  9.81f;
    const float expectedSpeed = std::sqrt(2.0f * g * h);

    PhysicsConfig cfg;
    PhysicsWorld world(cfg);
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0.0f, h, 0.0f));
    int id = world.addBody(sphere);

    // Step until near ground (y <= 0)
    for (int i = 0; i < 10000; ++i) {
        world.step();
        if (world.getBody(id)->position.y <= 0.0f) break;
    }

    const float speed = std::fabs(world.getBody(id)->linearVelocity.y);
    // Semi-implicit adds a tiny bit of energy; allow 5% margin above
    EXPECT_GE(speed, expectedSpeed * 0.95f);
    EXPECT_LE(speed, expectedSpeed * 1.10f);
}

TEST(PhysicsWorldTests, TwoBodiesIndependent) {
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld world(cfg);

    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3(5, 0, 0));
    int idA = world.addBody(a);
    int idB = world.addBody(b);

    world.getBody(idA)->applyForce(Vec3(10, 0, 0));
    world.step();

    // Body B should be entirely unaffected
    EXPECT_FLOAT_EQ(world.getBody(idB)->linearVelocity.x, 0.0f);
    EXPECT_FLOAT_EQ(world.getBody(idB)->position.x, 5.0f);
}

TEST(PhysicsWorldTests, UpdateRunsExactSubsteps) {
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld world(cfg);

    world.update(cfg.fixedDt * 3.0f); // should run exactly 3 steps
    EXPECT_EQ(world.stepCount(), static_cast<uint64_t>(3));
}
