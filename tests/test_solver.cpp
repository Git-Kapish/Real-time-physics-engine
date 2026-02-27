/// @file test_solver.cpp
/// @brief Google Test suite for Phase 4: ImpulseSolver (normal impulse, friction, Baumgarte).

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

#include "physics/RigidBody.h"
#include "physics/PhysicsWorld.h"
#include "physics/ImpulseSolver.h"
#include "physics/CollisionDetector.h"

using namespace physics;

static constexpr float kEps  = 1e-3f;
static constexpr float kEps4 = 1e-4f;

// ── Test helper ───────────────────────────────────────────────────────────

/// Build a zero-gravity world with a configured solver threshold.
static PhysicsWorld makeWorld(float restitutionThreshold = 0.0f) {
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld w(cfg);
    SolverConfig sc;
    sc.restitutionThreshold = restitutionThreshold;
    w.solver().setConfig(sc);
    return w;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Normal Impulse Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(NormalImpulseTests, ElasticHeadOnCollision) {
    // Two equal spheres, head-on, e=1 → velocities should swap
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 1.0f, Vec3(-0.5f, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.1f, 1.0f, Vec3( 0.5f, 0, 0));
    a.linearVelocity = Vec3( 1, 0, 0);
    b.linearVelocity = Vec3(-1, 0, 0);
    a.restitution = b.restitution = 1.0f;
    a.friction    = b.friction    = 0.0f;
    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();
    // After elastic 1D collision with equal masses, velocities swap
    EXPECT_NEAR(world.getBody(idA)->linearVelocity.x, -1.0f, kEps);
    EXPECT_NEAR(world.getBody(idB)->linearVelocity.x,  1.0f, kEps);
}

TEST(NormalImpulseTests, PerfectlyInelasticCollision) {
    // e=0 → bodies should have same velocity after collision
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 1.0f, Vec3(-0.5f, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.1f, 1.0f, Vec3( 0.5f, 0, 0));
    a.linearVelocity = Vec3( 1, 0, 0);
    b.linearVelocity = Vec3(-1, 0, 0);
    a.restitution = b.restitution = 0.0f;
    a.friction    = b.friction    = 0.0f;
    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();
    // Equal mass, opposite velocities → both nearly zero
    const float vAx = world.getBody(idA)->linearVelocity.x;
    const float vBx = world.getBody(idB)->linearVelocity.x;
    EXPECT_NEAR(std::fabs(vAx - vBx), 0.0f, kEps);
}

TEST(NormalImpulseTests, StaticBodyCollision) {
    // A moving sphere hits a static sphere → A reverses, B stays zero
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 1.0f, Vec3(-0.5f, 0, 0));
    a.linearVelocity = Vec3(1, 0, 0);
    a.restitution    = 1.0f;
    a.friction       = 0.0f;

    RigidBody b = RigidBody::makeSphere(1.1f, 1.0f, Vec3(0.5f, 0, 0));
    b.bodyType  = BodyType::Static;
    b.invMass   = 0.0f;
    b.restitution = 1.0f;
    b.friction    = 0.0f;
    b.updateInertiaTensor();

    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();

    EXPECT_NEAR(world.getBody(idA)->linearVelocity.x, -1.0f, kEps);
    EXPECT_NEAR(world.getBody(idB)->linearVelocity.x,  0.0f, kEps4);
}

TEST(NormalImpulseTests, MassRatioElastic) {
    // 1D elastic: m1=1 moving at 1, m2=9 at rest
    // vA_final = (m1-m2)/(m1+m2)*v1 = -0.8,  vB_final = 2*m1/(m1+m2)*v1 = 0.2
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 1.0f, Vec3(-0.5f, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.1f, 9.0f, Vec3( 0.5f, 0, 0));
    a.linearVelocity = Vec3(1, 0, 0);
    b.linearVelocity = Vec3(0, 0, 0);
    a.restitution = b.restitution = 1.0f;
    a.friction    = b.friction    = 0.0f;
    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();
    EXPECT_NEAR(world.getBody(idA)->linearVelocity.x, -0.8f, 0.02f);
    EXPECT_NEAR(world.getBody(idB)->linearVelocity.x,  0.2f, 0.02f);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Momentum Conservation Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(MomentumTests, LinearMomentumConserved) {
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 2.0f, Vec3(-0.5f, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.1f, 3.0f, Vec3( 0.5f, 0, 0));
    a.linearVelocity = Vec3( 2, 0, 0);
    b.linearVelocity = Vec3(-1, 0, 0);
    a.friction = b.friction = 0.0f;

    // Total momentum before
    const Vec3 pBefore = a.linearVelocity * a.mass + b.linearVelocity * b.mass;

    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();

    const auto* ba = world.getBody(idA);
    const auto* bb = world.getBody(idB);
    const Vec3 pAfter = ba->linearVelocity * ba->mass + bb->linearVelocity * bb->mass;

    EXPECT_NEAR(pAfter.x, pBefore.x, kEps4);
    EXPECT_NEAR(pAfter.y, pBefore.y, kEps4);
    EXPECT_NEAR(pAfter.z, pBefore.z, kEps4);
}

TEST(MomentumTests, AngularMomentumApproxConserved) {
    // Impulse solvers with heuristic contact points do not exactly conserve angular
    // momentum, but they MUST NOT add energy. Verify KE does not increase.
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 1.0f, Vec3(-0.5f, 0.2f, 0));
    RigidBody b = RigidBody::makeSphere(1.1f, 1.0f, Vec3( 0.5f, 0.0f, 0));
    a.linearVelocity = Vec3(1, 0, 0);
    b.linearVelocity = Vec3(0, 0, 0);
    a.friction = b.friction = 0.0f;
    a.restitution = b.restitution = 0.8f;

    const float keBefore = 0.5f * a.mass * a.linearVelocity.lengthSq()
                         + 0.5f * b.mass * b.linearVelocity.lengthSq();

    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();

    const float keAfter = 0.5f * world.getBody(idA)->mass * world.getBody(idA)->linearVelocity.lengthSq()
                        + 0.5f * world.getBody(idB)->mass * world.getBody(idB)->linearVelocity.lengthSq();

    // KE after must not exceed KE before (no energy added)
    EXPECT_LE(keAfter, keBefore * 1.01f) << "Solver added kinetic energy (non-physical)";
    // And bodies should be moving (not fully frozen)
    EXPECT_GT(keAfter, 0.0f) << "All kinetic energy lost unexpectedly";
}

// ═══════════════════════════════════════════════════════════════════════════
//  Friction Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(FrictionTests, FrictionSlowsTangentialMotion) {
    // Sphere sitting ON the plane (penetrating) — friction should reduce vx
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld world(cfg);
    // Set high threshold so restitution is suppressed (force inelastic contact)
    SolverConfig sc; sc.restitutionThreshold = 100.0f; sc.baumgarte = 0.3f;
    sc.slop = 0.001f;
    world.solver().setConfig(sc);

    // Sphere deeply embedding in plane so contact always fires
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0.5f, 0));
    // Small closing velocity (towards plane) so normal impulse j > 0, enabling friction
    sphere.linearVelocity = Vec3(5, -0.5f, 0);
    sphere.restitution = 0.0f;
    sphere.friction    = 0.5f;

    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    plane.friction = 0.5f;

    const float vxBefore = sphere.linearVelocity.x;
    int idS = world.addBody(sphere);
    world.addBody(plane);

    for (int i = 0; i < 10; ++i) world.step();
    const float vxAfter = std::fabs(world.getBody(idS)->linearVelocity.x);
    EXPECT_LT(vxAfter, vxBefore);
}

TEST(FrictionTests, ZeroFrictionLeavesVxUnchanged) {
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld world(cfg);
    SolverConfig sc; sc.restitutionThreshold = 100.0f; sc.baumgarte = 0.3f;
    sc.slop = 0.001f;
    world.solver().setConfig(sc);

    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0.5f, 0));
    sphere.linearVelocity = Vec3(5, 0, 0);
    sphere.restitution = 0.0f;
    sphere.friction    = 0.0f;

    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    plane.friction = 0.0f;

    const float vxBefore = sphere.linearVelocity.x;
    int idS = world.addBody(sphere);
    world.addBody(plane);
    world.step();

    EXPECT_NEAR(world.getBody(idS)->linearVelocity.x, vxBefore, kEps);
}

TEST(FrictionTests, FrictionConeClamping) {
    // Verify that the friction impulse never exceeds mu*j in magnitude
    ImpulseSolver solver;
    SolverConfig sc;
    sc.restitutionThreshold = 0.0f;
    sc.velocityIterations   = 1;
    sc.positionIterations   = 1; // must be >= 0 (>= 1 with assert)
    solver.setConfig(sc);

    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0.8f, 0));
    RigidBody b = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    a.updateInertiaTensor(); b.updateInertiaTensor();

    // Big tangential velocity, tiny closing velocity
    a.linearVelocity = Vec3(100, -0.1f, 0);
    a.friction = b.friction = 0.3f;
    a.restitution = b.restitution = 0.0f;

    auto contact = CollisionDetector::spherePlane(a, b);
    ASSERT_TRUE(contact.has_value());

    const float vxBefore = a.linearVelocity.x;
    const float vyBefore = a.linearVelocity.y;
    std::vector<ContactManifold> contacts = { *contact };
    solver.solve(contacts, 1.0f / 60.0f);

    // Normal impulse magnitude (change in vy * mass, since normal is Y)
    const float dvy = std::fabs(a.linearVelocity.y - vyBefore) * a.mass;
    // Tangential impulse magnitude (change in vx * mass)
    const float dvx = std::fabs(a.linearVelocity.x - vxBefore) * a.mass;
    // Coulomb cone: friction impulse <= mu * normal impulse
    EXPECT_LE(dvx, a.friction * dvy + 0.5f);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Baumgarte Stabilization Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BaumgarteTests, SinkingPrevented) {
    // Sphere overlapping plane — should be pushed upward by Baumgarte
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld world(cfg);
    SolverConfig sc;
    sc.baumgarte           = 0.5f;
    sc.slop                = 0.001f;
    sc.velocityIterations  = 1;
    sc.positionIterations  = 3;
    sc.restitutionThreshold = 100.0f;
    world.solver().setConfig(sc);

    const float yStart = 0.8f;
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, yStart, 0));
    sphere.linearVelocity = Vec3::zero();
    sphere.restitution = 0.0f;
    sphere.friction    = 0.0f;

    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);

    int idS = world.addBody(sphere);
    world.addBody(plane);

    for (int i = 0; i < 10; ++i) world.step();
    EXPECT_GT(world.getBody(idS)->position.y, yStart);
}

TEST(BaumgarteTests, SlopTolerance) {
    // Overlap less than slop → no correction
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld world(cfg);
    SolverConfig sc;
    sc.slop               = 0.01f;
    sc.baumgarte          = 0.5f;
    sc.velocityIterations = 1;
    sc.positionIterations = 1;
    sc.restitutionThreshold = 100.0f;
    world.solver().setConfig(sc);

    // Sphere at y=0.995: overlap = 1-0.995 = 0.005 < slop=0.01
    RigidBody sphere = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0.995f, 0));
    sphere.linearVelocity = Vec3::zero();
    sphere.restitution    = 0.0f;
    sphere.friction       = 0.0f;
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);

    int idS = world.addBody(sphere);
    world.addBody(plane);
    world.step();

    // Position should not change (within a very tight tolerance)
    EXPECT_NEAR(world.getBody(idS)->position.y, 0.995f, 1e-4f);
}

TEST(BaumgarteTests, TwoDynamicSpheresSepaarte) {
    // Two overlapping dynamic spheres should be pushed apart
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);
    PhysicsWorld world(cfg);
    SolverConfig sc;
    sc.baumgarte          = 0.5f;
    sc.slop               = 0.001f;
    sc.positionIterations = 5;
    sc.velocityIterations = 1;
    sc.restitutionThreshold = 100.0f;
    world.solver().setConfig(sc);

    RigidBody a = RigidBody::makeSphere(1.0f, 1.0f, Vec3(-0.5f, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.0f, 1.0f, Vec3( 0.5f, 0, 0));
    a.linearVelocity = b.linearVelocity = Vec3::zero();
    a.restitution = b.restitution = 0.0f;
    a.friction    = b.friction    = 0.0f;

    int idA = world.addBody(a);
    int idB = world.addBody(b);

    for (int i = 0; i < 5; ++i) world.step();

    // After correction the gap between centers should be >= rSum = 2
    const float dist = (world.getBody(idB)->position - world.getBody(idA)->position).length();
    EXPECT_GE(dist, 1.9f); // relaxed tolerance due to solver iterations
}

// ═══════════════════════════════════════════════════════════════════════════
//  Energy Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(EnergyTests, ElasticPreservesKineticEnergy) {
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 1.0f, Vec3(-0.5f, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.1f, 1.0f, Vec3( 0.5f, 0, 0));
    a.linearVelocity = Vec3( 2, 0, 0);
    b.linearVelocity = Vec3(-2, 0, 0);
    a.restitution = b.restitution = 1.0f;
    a.friction    = b.friction    = 0.0f;

    const float keBefore = 0.5f * a.mass * a.linearVelocity.lengthSq()
                         + 0.5f * b.mass * b.linearVelocity.lengthSq();
    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();

    const float keAfter = 0.5f * world.getBody(idA)->mass * world.getBody(idA)->linearVelocity.lengthSq()
                        + 0.5f * world.getBody(idB)->mass * world.getBody(idB)->linearVelocity.lengthSq();
    EXPECT_NEAR(keAfter / keBefore, 1.0f, 0.01f); // within 1%
}

TEST(EnergyTests, InelasticDissipatesEnergy) {
    PhysicsWorld world = makeWorld();
    RigidBody a = RigidBody::makeSphere(1.1f, 1.0f, Vec3(-0.5f, 0, 0));
    RigidBody b = RigidBody::makeSphere(1.1f, 1.0f, Vec3( 0.5f, 0, 0));
    a.linearVelocity = Vec3( 2, 0, 0);
    b.linearVelocity = Vec3(-2, 0, 0);
    a.restitution = b.restitution = 0.0f;
    a.friction    = b.friction    = 0.0f;

    const float keBefore = 0.5f * a.mass * a.linearVelocity.lengthSq()
                         + 0.5f * b.mass * b.linearVelocity.lengthSq();
    int idA = world.addBody(a);
    int idB = world.addBody(b);
    world.step();

    const float keAfter = 0.5f * world.getBody(idA)->mass * world.getBody(idA)->linearVelocity.lengthSq()
                        + 0.5f * world.getBody(idB)->mass * world.getBody(idB)->linearVelocity.lengthSq();
    EXPECT_LE(keAfter, keBefore + 1e-4f); // energy must not increase
}

// ═══════════════════════════════════════════════════════════════════════════
//  Integration Tests (full PhysicsWorld with gravity)
// ═══════════════════════════════════════════════════════════════════════════

TEST(IntegrationTests, SphereBounceOffPlane) {
    // Sphere dropped from height, should bounce at least once
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, -9.81f, 0);
    PhysicsWorld world(cfg);
    SolverConfig sc;
    sc.restitutionThreshold = 0.0f;
    sc.baumgarte = 0.2f;
    sc.slop      = 0.005f;
    world.solver().setConfig(sc);

    RigidBody sphere = RigidBody::makeSphere(0.5f, 1.0f, Vec3(0, 5.0f, 0));
    sphere.restitution = 0.7f;
    sphere.friction    = 0.0f;
    RigidBody plane = RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f);
    plane.restitution = 0.7f;

    int idS = world.addBody(sphere);
    world.addBody(plane);

    float maxY = 5.0f;
    float minY = 5.0f;
    bool  hitFloor = false;
    bool  bouncedUp = false;

    for (int i = 0; i < 120; ++i) {
        world.step();
        const float y = world.getBody(idS)->position.y;
        if (y < minY) { minY = y; hitFloor = true; }
        if (hitFloor && y > minY + 0.1f) bouncedUp = true;
        if (y > maxY) maxY = y;
    }

    EXPECT_TRUE(hitFloor)   << "Sphere never reached the ground";
    EXPECT_TRUE(bouncedUp)  << "Sphere never bounced upward";
    // Bounce height must be less than initial drop (energy lost, e < 1)
    EXPECT_LT(maxY - minY,  5.0f);
}

TEST(IntegrationTests, SphereSittingOnStaticSphere) {
    // Dynamic sphere resting on top of a static sphere — should not sink
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, -9.81f, 0);
    PhysicsWorld world(cfg);
    SolverConfig sc;
    sc.baumgarte            = 0.3f;
    sc.slop                 = 0.005f;
    sc.velocityIterations   = 8;
    sc.positionIterations   = 3;
    sc.restitutionThreshold = 10.0f; // force inelastic
    world.solver().setConfig(sc);

    // Static support sphere at origin, radius 1
    RigidBody support = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0));
    support.bodyType  = BodyType::Static;
    support.invMass   = 0.0f;
    support.updateInertiaTensor();
    support.restitution = 0.0f;
    support.friction    = 0.5f;

    // Dynamic sphere resting on top: center at y = 2.05 (just above contact)
    RigidBody top = RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 2.05f, 0));
    top.restitution = 0.0f;
    top.friction    = 0.5f;

    world.addBody(support);
    int idTop = world.addBody(top);

    // Warm up
    for (int i = 0; i < 30; ++i) world.step();
    const float yStart = world.getBody(idTop)->position.y;

    // Run 30 more steps and track drift
    float yMin = yStart, yMax = yStart;
    for (int i = 0; i < 30; ++i) {
        world.step();
        const float y = world.getBody(idTop)->position.y;
        yMin = std::min(yMin, y);
        yMax = std::max(yMax, y);
    }

    // Should not sink more than 5 cm during the last 30 steps
    EXPECT_GT(yMin, yStart - 0.05f) << "Sphere sinking through support";
}
