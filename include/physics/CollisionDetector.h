#pragma once
/// @file CollisionDetector.h
/// @brief Stateless narrow-phase and broad-phase collision detection.

#include "physics/RigidBody.h"
#include "physics/AABB.h"
#include "physics/ContactManifold.h"
#include <optional>
#include <vector>
#include <utility>

namespace physics {

/// Pure-static collision detection utility — no state, no instances needed.
class CollisionDetector {
public:
    CollisionDetector() = delete;

    // ── Broad phase ───────────────────────────────────────────────────────

    /// Return index pairs whose AABBs overlap (skips static-static pairs).
    static std::vector<std::pair<int, int>>
        broadPhase(const std::vector<RigidBody>& bodies);

    /// Compute the world-space AABB for any body shape.
    static AABB computeAABB(const RigidBody& body);

    // ── Narrow phase dispatcher ───────────────────────────────────────────

    /// Dispatch to the correct shape-pair test; returns nullopt if no contact.
    static std::optional<ContactManifold>
        detect(RigidBody& a, RigidBody& b);

    // ── Shape-pair narrow-phase tests ────────────────────────────────────

    /// Sphere vs Sphere.
    static std::optional<ContactManifold>
        sphereSphere(RigidBody& a, RigidBody& b);

    /// Sphere vs infinite plane (plane stores normal in halfExtents, offset in radius).
    static std::optional<ContactManifold>
        spherePlane(RigidBody& sphere, RigidBody& plane);

    /// Sphere vs oriented box.
    static std::optional<ContactManifold>
        sphereBox(RigidBody& sphere, RigidBody& box);

    /// Oriented box vs infinite plane.
    static std::optional<ContactManifold>
        boxPlane(RigidBody& box, RigidBody& plane);

    /// Oriented box vs oriented box (15-axis SAT).
    static std::optional<ContactManifold>
        boxBox(RigidBody& a, RigidBody& b);
};

} // namespace physics
