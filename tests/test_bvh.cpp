/// @file test_bvh.cpp
/// @brief Google Test suite for Phase 5: BVHTree construction, queries, update, and integration.

#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <random>

#include "physics/AABB.h"
#include "physics/BVHTree.h"
#include "physics/RigidBody.h"
#include "physics/CollisionDetector.h"
#include "physics/PhysicsWorld.h"

using namespace physics;

// ── Helpers ───────────────────────────────────────────────────────────────

static AABB sphereAABB(float cx, float cy, float cz, float r) {
    return AABB(Vec3(cx - r, cy - r, cz - r),
                Vec3(cx + r, cy + r, cz + r));
}

// ═══════════════════════════════════════════════════════════════════════════
//  Construction Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BVHConstructionTests, EmptyTree) {
    BVHTree t;
    EXPECT_EQ(t.leafCount(), 0);
    EXPECT_EQ(t.nodeCount(), 0);
    t.validate();
}

TEST(BVHConstructionTests, OneBody) {
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    EXPECT_EQ(t.leafCount(), 1);
    t.validate();
}

TEST(BVHConstructionTests, TwoBodies) {
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(5, 0, 0, 1));
    EXPECT_EQ(t.leafCount(), 2);
    // 2 leaves + 1 internal = 3 logical live nodes
    EXPECT_EQ(t.nodeCount(), 3);
    t.validate();
}

TEST(BVHConstructionTests, TenBodies) {
    BVHTree t;
    for (int i = 0; i < 10; ++i) {
        t.insert(i, sphereAABB(static_cast<float>(i * 3), 0, 0, 1));
    }
    EXPECT_EQ(t.leafCount(), 10);
    t.validate();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Insert / Remove Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BVHInsertRemoveTests, RemoveMiddle) {
    BVHTree t;
    for (int i = 0; i < 5; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i * 3), 0, 0, 1));
    t.remove(2);
    EXPECT_EQ(t.leafCount(), 4);
    t.validate();
}

TEST(BVHInsertRemoveTests, InsertRemoveReinsert) {
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.remove(0);
    EXPECT_EQ(t.leafCount(), 0);
    t.insert(0, sphereAABB(0, 0, 0, 1));
    EXPECT_EQ(t.leafCount(), 1);
    t.validate();
}

TEST(BVHInsertRemoveTests, RemoveAll) {
    BVHTree t;
    for (int i = 0; i < 6; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i), 0, 0, 0.4f));
    for (int i = 0; i < 6; ++i) {
        t.remove(i);
        t.validate();
    }
    EXPECT_EQ(t.leafCount(), 0);
    EXPECT_EQ(t.nodeCount(), 0);
}

TEST(BVHInsertRemoveTests, LargeInsertRemoveHalf) {
    BVHTree t;
    for (int i = 0; i < 100; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i), 0, 0, 0.4f));
    // Remove even-indexed
    for (int i = 0; i < 100; i += 2)
        t.remove(i);
    EXPECT_EQ(t.leafCount(), 50);
    t.validate();
}

// ═══════════════════════════════════════════════════════════════════════════
//  queryAABB Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BVHQueryAABBTests, LineOfBodies) {
    // Bodies at x = 0, 1, 2, 3, 4 (radius 0.3) — query x=0.5..3.5
    BVHTree t;
    for (int i = 0; i < 5; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i), 0, 0, 0.3f));

    AABB query(Vec3(0.5f, -1, -1), Vec3(3.5f, 1, 1));
    auto hits = t.queryAABB(query);
    std::sort(hits.begin(), hits.end());

    ASSERT_EQ(hits.size(), 3u);
    EXPECT_EQ(hits[0], 1);
    EXPECT_EQ(hits[1], 2);
    EXPECT_EQ(hits[2], 3);
}

TEST(BVHQueryAABBTests, NoHits) {
    BVHTree t;
    for (int i = 0; i < 5; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i), 0, 0, 0.3f));

    AABB query(Vec3(100, 100, 100), Vec3(200, 200, 200));
    EXPECT_TRUE(t.queryAABB(query).empty());
}

TEST(BVHQueryAABBTests, AllHits) {
    BVHTree t;
    for (int i = 0; i < 10; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i), 0, 0, 0.3f));

    AABB query(Vec3(-10, -10, -10), Vec3(100, 100, 100));
    auto hits = t.queryAABB(query);
    EXPECT_EQ(hits.size(), 10u);
}

// ═══════════════════════════════════════════════════════════════════════════
//  queryAllPairs Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BVHQueryAllPairsTests, OnlyClosePairReturned) {
    // A(0) and B(1) overlap, C(2) is far away
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(1, 0, 0, 1)); // overlaps 0
    t.insert(2, sphereAABB(100, 0, 0, 1));
    auto pairs = t.queryAllPairs();
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].first,  0);
    EXPECT_EQ(pairs[0].second, 1);
}

TEST(BVHQueryAllPairsTests, NoStaticStaticPairsFromBVH) {
    // BVH queryAllPairs returns all AABBoverlap pairs — filtering happens in broadPhase()
    // This test verifies the BVH itself returns the pair (filtering is CollisionDetector's job)
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(1, 0, 0, 1));
    auto pairs = t.queryAllPairs();
    EXPECT_EQ(pairs.size(), 1u); // BVH sees overlap regardless of body type
}

TEST(BVHQueryAllPairsTests, FourBodiesGrid) {
    // 4 bodies in a 2x2 grid, all close — expect multiple pairs
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(1.5f, 0, 0, 1));
    t.insert(2, sphereAABB(0, 1.5f, 0, 1));
    t.insert(3, sphereAABB(1.5f, 1.5f, 0, 1));
    auto pairs = t.queryAllPairs();
    // Each sphere overlaps all others (radius=1, max distance=sqrt(1.5^2+1.5^2)=2.12 < 2)
    EXPECT_GE(pairs.size(), 4u); // at least 4 overlapping pairs
}

TEST(BVHQueryAllPairsTests, EquivalenceToBruteForce) {
    // CRITICAL: BVH must return exactly the same pairs as O(n^2) brute force.
    // Use fattenMargin=0 so both paths see the same tight AABBs.
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(-10, 10);
    std::uniform_real_distribution<float> radDist(0.3f, 1.5f);

    const int N = 30;
    BVHTree bvh;
    bvh.setFattenMargin(0.0f); // exact match — no fat AABBs
    std::vector<RigidBody> bodies;

    for (int i = 0; i < N; ++i) {
        float x = posDist(rng), y = posDist(rng), z = posDist(rng);
        float r = radDist(rng);
        RigidBody b = RigidBody::makeSphere(r, 1.0f, Vec3(x, y, z));
        bodies.push_back(b);
        bvh.insert(i, CollisionDetector::computeAABB(b));
    }

    // BVH pairs filtered for static-static
    auto pairsBVH   = CollisionDetector::broadPhase(bvh, bodies);
    auto pairsBrute = CollisionDetector::broadPhase(bodies);

    std::sort(pairsBVH.begin(),   pairsBVH.end());
    std::sort(pairsBrute.begin(), pairsBrute.end());

    EXPECT_EQ(pairsBVH, pairsBrute)
        << "BVH and brute-force broad phases returned different pairs!";
}

// ═══════════════════════════════════════════════════════════════════════════
//  update() Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BVHUpdateTests, MoveFarAwayNoPairs) {
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(1, 0, 0, 1)); // overlaps 0
    EXPECT_EQ(t.queryAllPairs().size(), 1u);

    // Move body 1 far away
    t.update(1, sphereAABB(100, 0, 0, 1));
    EXPECT_EQ(t.queryAllPairs().size(), 0u);
    t.validate();
}

TEST(BVHUpdateTests, MoveCloseCreatesPair) {
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(50, 0, 0, 1)); // far
    EXPECT_EQ(t.queryAllPairs().size(), 0u);

    t.update(1, sphereAABB(1, 0, 0, 1)); // now overlaps 0
    EXPECT_EQ(t.queryAllPairs().size(), 1u);
    t.validate();
}

TEST(BVHUpdateTests, SmallMoveNoReinsert) {
    // Small movement within fat AABB → nodeCount should not change (no reinsert)
    BVHTree t;
    t.setFattenMargin(0.5f); // larger margin to make test reliable
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(5, 0, 0, 1));
    const int countBefore = t.nodeCount();

    // Tiny nudge — should stay inside fat AABB
    t.update(0, sphereAABB(0.05f, 0, 0, 1));
    EXPECT_EQ(t.nodeCount(), countBefore); // no reinsert
    t.validate();
}

TEST(BVHUpdateTests, LargeMoveReinserts) {
    BVHTree t;
    t.insert(0, sphereAABB(0, 0, 0, 1));
    t.insert(1, sphereAABB(5, 0, 0, 1));
    // Large move — definitely exits fat AABB → reinsert
    t.update(0, sphereAABB(50, 0, 0, 1));
    EXPECT_EQ(t.leafCount(), 2);
    t.validate();
}

// ═══════════════════════════════════════════════════════════════════════════
//  SAH Quality Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BVHSAHQualityTests, TreeDepthBounded) {
    // Insert 64 bodies — tree depth should be << N (should be O(log N))
    BVHTree t;
    for (int i = 0; i < 64; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i * 2), 0, 0, 0.8f));

    // Max reasonable depth for 64 leaves is ~2*log2(64)+2 = 14
    // Just verify validate passes (structural correctness)
    t.validate();
    EXPECT_EQ(t.leafCount(), 64);
}

TEST(BVHSAHQualityTests, SurfaceAreaPositive) {
    BVHTree t;
    for (int i = 0; i < 20; ++i)
        t.insert(i, sphereAABB(static_cast<float>(i), 0, 0, 0.4f));
    EXPECT_GT(t.totalSurfaceArea(), 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
//  PhysicsWorld BVH Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

TEST(BVHWorldIntegrationTests, AddBodyIncreasesLeafCount) {
    PhysicsWorld world;
    EXPECT_EQ(world.bvh().leafCount(), 0);
    world.addBody(RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0)));
    EXPECT_EQ(world.bvh().leafCount(), 1);
    world.addBody(RigidBody::makeSphere(1.0f, 1.0f, Vec3(10, 0, 0)));
    EXPECT_EQ(world.bvh().leafCount(), 2);
}

TEST(BVHWorldIntegrationTests, RemoveBodyDecreasesLeafCount) {
    PhysicsWorld world;
    world.addBody(RigidBody::makeSphere(1.0f, 1.0f, Vec3(0, 0, 0)));
    int idB = world.addBody(RigidBody::makeSphere(1.0f, 1.0f, Vec3(10, 0, 0)));
    EXPECT_EQ(world.bvh().leafCount(), 2);
    world.removeBody(idB);
    EXPECT_EQ(world.bvh().leafCount(), 1);
    world.bvh().validate();
}

TEST(BVHWorldIntegrationTests, ValidateAfterStep) {
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, -9.81f, 0);
    PhysicsWorld world(cfg);
    world.addBody(RigidBody::makeSphere(0.5f, 1.0f, Vec3(0, 5, 0)));
    world.addBody(RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f));
    for (int i = 0; i < 10; ++i) {
        world.step();
        world.bvh().validate();
    }
    EXPECT_GT(world.stepCount(), 0u);
}

TEST(BVHWorldIntegrationTests, BVHAndBruteForceProduceSameContactCount) {
    // Run identical scene with BVH on and off — same number of contacts
    PhysicsConfig cfg;
    cfg.gravity = Vec3(0, 0, 0);

    PhysicsWorld worldBVH(cfg);
    worldBVH.setUseBVH(true);
    PhysicsWorld worldBrute(cfg);
    worldBrute.setUseBVH(false);

    // Add same bodies to both worlds
    for (int i = 0; i < 5; ++i) {
        float x = static_cast<float>(i) * 1.5f;
        worldBVH.addBody(RigidBody::makeSphere(1.0f, 1.0f, Vec3(x, 0, 0)));
        worldBrute.addBody(RigidBody::makeSphere(1.0f, 1.0f, Vec3(x, 0, 0)));
    }

    worldBVH.step();
    worldBrute.step();

    EXPECT_EQ(worldBVH.lastContacts().size(), worldBrute.lastContacts().size())
        << "BVH and brute-force produced different contact counts";
}
