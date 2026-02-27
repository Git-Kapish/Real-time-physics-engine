/// @file CollisionDetector.cpp
/// @brief Broad-phase filtering and narrow-phase collision tests for all shape pairs.

#include "physics/CollisionDetector.h"
#include "physics/BVHTree.h"
#include <cmath>
#include <algorithm>
#include <cassert>

namespace physics {

// ── Helpers ───────────────────────────────────────────────────────────────

/// Clamp a value between lo and hi.
static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ── AABB computation ──────────────────────────────────────────────────────

AABB CollisionDetector::computeAABB(const RigidBody& body) {
    switch (body.shape.type) {
        case ShapeType::Sphere:
            return AABB::fromSphere(body.position, body.shape.radius);
        case ShapeType::Box:
            return AABB::fromBox(body.position, body.orientation, body.shape.halfExtents);
        case ShapeType::Plane:
            // Infinite plane always participates; return an enormous AABB
            return AABB(Vec3(-1e9f, -1e9f, -1e9f), Vec3(1e9f, 1e9f, 1e9f));
    }
    return AABB(Vec3(-1e9f, -1e9f, -1e9f), Vec3(1e9f, 1e9f, 1e9f));
}

// ── Broad phase ───────────────────────────────────────────────────────────

std::vector<std::pair<int, int>>
CollisionDetector::broadPhase(const std::vector<RigidBody>& bodies) {
    std::vector<std::pair<int, int>> pairs;
    const int n = static_cast<int>(bodies.size());

    // Pre-compute all AABBs
    std::vector<AABB> aabbs;
    aabbs.reserve(static_cast<std::size_t>(n));
    for (const auto& b : bodies) {
        aabbs.push_back(computeAABB(b));
    }

    for (int i = 0; i < n - 1; ++i) {
        for (int j = i + 1; j < n; ++j) {
            // Skip static-static pairs — they never need collision response
            if (bodies[i].isStatic() && bodies[j].isStatic()) continue;
            if (aabbs[i].overlaps(aabbs[j])) {
                pairs.emplace_back(i, j);
            }
        }
    }
    return pairs;
}

// ── BVH broad phase ───────────────────────────────────────────────────────

std::vector<std::pair<int, int>>
CollisionDetector::broadPhase(const BVHTree& bvh, const std::vector<RigidBody>& bodies) {
    auto pairs = bvh.queryAllPairs();
    // Filter out static-static pairs (bodyIndex is the body's array index)
    pairs.erase(
        std::remove_if(pairs.begin(), pairs.end(),
            [&bodies](const std::pair<int,int>& p) {
                return bodies[p.first].isStatic() && bodies[p.second].isStatic();
            }),
        pairs.end());
    return pairs;
}

// ── Narrow-phase dispatcher ───────────────────────────────────────────────

std::optional<ContactManifold>
CollisionDetector::detect(RigidBody& a, RigidBody& b) {
    const ShapeType ta = a.shape.type;
    const ShapeType tb = b.shape.type;

    // Sphere - Sphere
    if (ta == ShapeType::Sphere && tb == ShapeType::Sphere)
        return sphereSphere(a, b);

    // Sphere - Plane (either order)
    if (ta == ShapeType::Sphere && tb == ShapeType::Plane)
        return spherePlane(a, b);
    if (ta == ShapeType::Plane && tb == ShapeType::Sphere) {
        auto m = spherePlane(b, a);
        if (m) m->normal = -m->normal; // re-orient: from b toward a
        return m;
    }

    // Sphere - Box (either order)
    if (ta == ShapeType::Sphere && tb == ShapeType::Box)
        return sphereBox(a, b);
    if (ta == ShapeType::Box && tb == ShapeType::Sphere) {
        auto m = sphereBox(b, a);
        if (m) {
            m->normal = -m->normal;
            std::swap(m->bodyA, m->bodyB);
        }
        return m;
    }

    // Box - Plane (either order)
    if (ta == ShapeType::Box && tb == ShapeType::Plane)
        return boxPlane(a, b);
    if (ta == ShapeType::Plane && tb == ShapeType::Box) {
        auto m = boxPlane(b, a);
        if (m) {
            m->normal = -m->normal;
            std::swap(m->bodyA, m->bodyB);
        }
        return m;
    }

    // Box - Box
    if (ta == ShapeType::Box && tb == ShapeType::Box)
        return boxBox(a, b);

    return std::nullopt; // unsupported pair
}

// ── sphereSphere ──────────────────────────────────────────────────────────

std::optional<ContactManifold>
CollisionDetector::sphereSphere(RigidBody& a, RigidBody& b) {
    const Vec3  delta = a.position - b.position;
    const float dist  = delta.length();
    const float rSum  = a.shape.radius + b.shape.radius;

    if (dist >= rSum) return std::nullopt;

    ContactManifold m;
    m.bodyA            = &a;
    m.bodyB            = &b;
    m.normal           = (dist > 1e-6f) ? delta / dist : Vec3::unitY();
    m.penetrationDepth = rSum - dist;
    m.contactPoint     = b.position + m.normal * b.shape.radius;
    m.restitution      = std::min(a.restitution, b.restitution);
    m.friction         = std::sqrt(a.friction * b.friction);
    return m;
}

// ── spherePlane ───────────────────────────────────────────────────────────

std::optional<ContactManifold>
CollisionDetector::spherePlane(RigidBody& sphere, RigidBody& plane) {
    // Plane: normal stored in halfExtents, offset (d) stored in radius
    const Vec3  planeNormal = plane.shape.halfExtents;
    const float planeOffset = plane.shape.radius;

    const float signedDist  = planeNormal.dot(sphere.position) - planeOffset;
    const float penetration = sphere.shape.radius - signedDist;

    if (penetration <= 0.0f) return std::nullopt;

    ContactManifold m;
    m.bodyA            = &sphere;
    m.bodyB            = &plane;
    m.normal           = planeNormal;   // points away from plane, toward sphere
    m.penetrationDepth = penetration;
    m.contactPoint     = sphere.position - planeNormal * sphere.shape.radius;
    m.restitution      = std::min(sphere.restitution, plane.restitution);
    m.friction         = std::sqrt(sphere.friction * plane.friction);
    return m;
}

// ── sphereBox ─────────────────────────────────────────────────────────────

std::optional<ContactManifold>
CollisionDetector::sphereBox(RigidBody& sphere, RigidBody& box) {
    // Transform sphere centre into box local space
    const Vec3 localCenter = box.orientation.conjugate().rotate(sphere.position - box.position);
    const Vec3 half        = box.shape.halfExtents;

    // Clamp to get closest point on (or inside) box
    const Vec3 closest(
        clampf(localCenter.x, -half.x, half.x),
        clampf(localCenter.y, -half.y, half.y),
        clampf(localCenter.z, -half.z, half.z)
    );

    const Vec3  delta  = localCenter - closest;
    const float distSq = delta.lengthSq();

    Vec3  worldNormal;
    Vec3  worldContact;
    float penetration;

    if (distSq > 1e-12f) {
        // Sphere centre is OUTSIDE the box
        const float dist = std::sqrt(distSq);
        if (dist >= sphere.shape.radius) return std::nullopt;

        const Vec3 localNormal = delta / dist;
        worldNormal  = box.orientation.rotate(localNormal);
        worldContact = box.position + box.orientation.rotate(closest);
        penetration  = sphere.shape.radius - dist;
    } else {
        // Sphere centre is INSIDE the box — find smallest exit axis
        const float overlaps[3] = {
            half.x - std::fabs(localCenter.x),
            half.y - std::fabs(localCenter.y),
            half.z - std::fabs(localCenter.z)
        };
        int minAxis = 0;
        if (overlaps[1] < overlaps[minAxis]) minAxis = 1;
        if (overlaps[2] < overlaps[minAxis]) minAxis = 2;

        Vec3 localNormal;
        localNormal[minAxis] = (localCenter[minAxis] >= 0.0f) ? 1.0f : -1.0f;

        worldNormal  = box.orientation.rotate(localNormal);
        penetration  = sphere.shape.radius + overlaps[minAxis];
        worldContact = sphere.position - worldNormal * sphere.shape.radius;
    }

    ContactManifold m;
    m.bodyA            = &sphere;
    m.bodyB            = &box;
    m.normal           = worldNormal;
    m.penetrationDepth = penetration;
    m.contactPoint     = worldContact;
    m.restitution      = std::min(sphere.restitution, box.restitution);
    m.friction         = std::sqrt(sphere.friction * box.friction);
    return m;
}

// ── boxPlane ──────────────────────────────────────────────────────────────

std::optional<ContactManifold>
CollisionDetector::boxPlane(RigidBody& box, RigidBody& plane) {
    const Vec3  planeNormal = plane.shape.halfExtents;
    const float planeOffset = plane.shape.radius;
    const Vec3& h           = box.shape.halfExtents;

    // Generate all 8 box corners in local space
    const float sx[2] = {-h.x, h.x};
    const float sy[2] = {-h.y, h.y};
    const float sz[2] = {-h.z, h.z};

    float deepestPen   = 0.0f;
    Vec3  deepestPoint;
    bool  anyContact   = false;

    for (int ix = 0; ix < 2; ++ix) {
        for (int iy = 0; iy < 2; ++iy) {
            for (int iz = 0; iz < 2; ++iz) {
                // World position of this corner
                const Vec3 localCorner(sx[ix], sy[iy], sz[iz]);
                const Vec3 worldCorner = box.position + box.orientation.rotate(localCorner);
                const float signedDist = planeNormal.dot(worldCorner) - planeOffset;

                if (signedDist < 0.0f) {
                    const float pen = -signedDist;
                    if (pen > deepestPen || !anyContact) {
                        deepestPen   = pen;
                        deepestPoint = worldCorner;
                        anyContact   = true;
                    }
                }
            }
        }
    }

    if (!anyContact) return std::nullopt;

    ContactManifold m;
    m.bodyA            = &box;
    m.bodyB            = &plane;
    m.normal           = planeNormal;
    m.penetrationDepth = deepestPen;
    m.contactPoint     = deepestPoint;
    m.restitution      = std::min(box.restitution, plane.restitution);
    m.friction         = std::sqrt(box.friction * plane.friction);
    return m;
}

// ── boxBox (15-axis SAT) ──────────────────────────────────────────────────

std::optional<ContactManifold>
CollisionDetector::boxBox(RigidBody& a, RigidBody& b) {
    // Rotation matrices (columns = box local axes in world space)
    const Mat3 Ra = a.orientation.toMat3();
    const Mat3 Rb = b.orientation.toMat3();

    // Box axes (3 each)
    Vec3 axA[3], axB[3];
    for (int i = 0; i < 3; ++i) {
        axA[i] = Vec3(Ra(0, i), Ra(1, i), Ra(2, i));
        axB[i] = Vec3(Rb(0, i), Rb(1, i), Rb(2, i));
    }

    const Vec3  centerDelta = b.position - a.position;
    const Vec3& hA          = a.shape.halfExtents;
    const Vec3& hB          = b.shape.halfExtents;

    float minOverlap = 1e38f;
    Vec3  minAxis;

    // Project an OBB onto an axis: sum of |half[i] * (axis · boxAxis[i])|
    auto projectOBB = [](const Vec3 axes[3], const Vec3& half, const Vec3& ax) -> float {
        return std::fabs(half.x * ax.dot(axes[0]))
             + std::fabs(half.y * ax.dot(axes[1]))
             + std::fabs(half.z * ax.dot(axes[2]));
    };

    auto testAxis = [&](Vec3 axis) -> bool {
        const float len = axis.length();
        if (len < 1e-6f) return true; // degenerate → skip (no separation on this axis)
        axis = axis / len;

        const float projA    = projectOBB(axA, hA, axis);
        const float projB    = projectOBB(axB, hB, axis);
        const float dist     = std::fabs(centerDelta.dot(axis));
        const float overlap  = projA + projB - dist;

        if (overlap <= 0.0f) return false; // separating axis found
        if (overlap < minOverlap) {
            minOverlap = overlap;
            // centerDelta = b - a; axis must point FROM b TOWARD a (i.e., opposite to centerDelta)
            // If centerDelta · axis >= 0, axis points toward b → flip it
            minAxis = (centerDelta.dot(axis) >= 0.0f) ? -axis : axis;
        }
        return true;
    };

    // Test 6 face axes (3 from A, 3 from B)
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(axA[i])) return std::nullopt;
        if (!testAxis(axB[i])) return std::nullopt;
    }

    // Test 9 edge cross products
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!testAxis(axA[i].cross(axB[j]))) return std::nullopt;
        }
    }

    // All 15 axes overlap — minAxis is the MTV direction (from b toward a)
    // Contact point heuristic: support point of B in the -minAxis direction
    const Vec3 supportDir = -minAxis;
    Vec3 contactPoint     = b.position;
    for (int i = 0; i < 3; ++i) {
        contactPoint += axB[i] * (hB[i] * (supportDir.dot(axB[i]) >= 0.0f ? 1.0f : -1.0f));
    }

    ContactManifold m;
    m.bodyA            = &a;
    m.bodyB            = &b;
    m.normal           = minAxis;
    m.penetrationDepth = minOverlap;
    m.contactPoint     = contactPoint;
    m.restitution      = std::min(a.restitution, b.restitution);
    m.friction         = std::sqrt(a.friction * b.friction);
    return m;
}

} // namespace physics
