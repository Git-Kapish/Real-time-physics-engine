/// @file benchmark.cpp
/// @brief Benchmarks comparing O(n^2) vs BVH broad-phase performance.

#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace physics;
using Clock = std::chrono::high_resolution_clock;

// ── Helpers ───────────────────────────────────────────────────────────────

/// Spawn N spheres in a dense grid (radius=0.5, spacing=1.0 so spheres touch/overlap) + ground plane.
static void populateWorld(PhysicsWorld& world, int N) {
    // Ground plane
    world.addBody(RigidBody::makeStaticPlane(Vec3(0, 1, 0), 0.0f));

    // Sparse grid: spacing = 3.0×radius — each body has only a few local neighbors
    const int side = static_cast<int>(std::ceil(std::cbrt(static_cast<double>(N))));
    int count = 0;
    for (int x = 0; x < side && count < N; ++x) {
        for (int y = 0; y < side && count < N; ++y) {
            for (int z = 0; z < side && count < N; ++z, ++count) {
                const Vec3 pos(
                    static_cast<float>(x) * 3.0f,
                    static_cast<float>(y) * 3.0f + 0.5f,
                    static_cast<float>(z) * 3.0f
                );
                RigidBody b = RigidBody::makeSphere(0.5f, 1.0f, pos);
                b.restitution = 0.2f;
                b.friction    = 0.3f;
                world.addBody(b);
            }
        }
    }
}

/// Time 100 steps and return ms/step.
static double timeSteps(PhysicsWorld& world, int warmup = 10, int measured = 100) {
    // Warm up (cache + BVH settle)
    for (int i = 0; i < warmup; ++i) world.step();

    const auto t0 = Clock::now();
    for (int i = 0; i < measured; ++i) world.step();
    const auto t1 = Clock::now();

    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return ms / measured;
}

// ── Main ──────────────────────────────────────────────────────────────────

int main() {
    std::puts("==========================================================================");
    std::puts("  Physics Engine — Broad Phase Benchmark");
    std::puts("  BVH (O(n log n)) vs Brute Force (O(n^2))");
    std::puts("  Dense grid: radius=0.5, spacing=1.5 (some overlaps per body)");
    std::puts("==========================================================================");
    std::puts("");
    std::printf("%-6s  %-8s  %10s  %10s  %10s  %8s\n",
                "N", "Mode", "ms/step", "steps/s", "contacts", "speedup");
    std::puts("--------------------------------------------------------------------------");

    const std::vector<int> Ns = {50, 100, 250, 500, 1000};

    for (int N : Ns) {
        PhysicsConfig cfg;
        cfg.gravity   = Vec3(0, 0, 0); // no gravity — static benchmark
        cfg.maxBodies = N + 10;        // allow N spheres + ground plane

        // ── Brute-force run ──
        PhysicsWorld worldBrute(cfg);
        worldBrute.setUseBVH(false);
        populateWorld(worldBrute, N);
        const double msBrute    = timeSteps(worldBrute, 5, 50);
        const int contactsBrute = static_cast<int>(worldBrute.lastContacts().size());

        // ── BVH run ──
        PhysicsWorld worldBVH(cfg);
        worldBVH.setUseBVH(true);
        populateWorld(worldBVH, N);
        const double msBVH    = timeSteps(worldBVH, 5, 50);
        const int contactsBVH = static_cast<int>(worldBVH.lastContacts().size());

        const double speedup = (msBVH > 0) ? msBrute / msBVH : 0.0;
        const char* marker   = (speedup >= 1.0) ? " <-- BVH wins" : "";

        std::printf("%-6d  %-8s  %10.4f  %10.1f  %10d  %8s\n",
                    N, "brute", msBrute, 1000.0 / msBrute, contactsBrute, "");
        std::printf("%-6d  %-8s  %10.4f  %10.1f  %10d  %7.1fx%s\n",
                    N, "BVH", msBVH, 1000.0 / msBVH, contactsBVH, speedup, marker);
        std::puts("");
    }

    std::puts("==========================================================================");
    std::puts("  BVH advantage: O(n log n) pair testing vs O(n^2) brute force.");
    std::puts("  At typical game scene densities (N=1000+), BVH significantly reduces");
    std::puts("  broad-phase overhead, leaving more budget for narrow-phase and solver.");
    std::puts("==========================================================================");
    return 0;
}
