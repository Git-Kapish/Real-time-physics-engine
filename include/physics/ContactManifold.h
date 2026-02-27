#pragma once
/// @file ContactManifold.h
/// @brief Describes a collision contact between two rigid bodies.

#include "physics/RigidBody.h"

namespace physics {

/// A contact point between two rigid bodies, computed by the collision detector.
struct ContactManifold {
    RigidBody* bodyA = nullptr; ///< First body (never null)
    RigidBody* bodyB = nullptr; ///< Second body (never null; may be a static body)

    Vec3  contactPoint;     ///< World-space contact point
    Vec3  normal;           ///< World-space separation normal, pointing FROM bodyB TO bodyA
    float penetrationDepth; ///< Positive overlap depth (0 = just touching)

    float restitution; ///< Combined: min(bodyA->restitution, bodyB->restitution)
    float friction;    ///< Combined: sqrt(bodyA->friction * bodyB->friction)

    /// Velocity of bodyA contact point relative to bodyB contact point.
    /// Accounts for both linear and angular contributions.
    Vec3 relativeVelocityAtContact() const {
        const Vec3 rA = contactPoint - bodyA->position;
        const Vec3 rB = contactPoint - bodyB->position;
        const Vec3 vA = bodyA->linearVelocity + bodyA->angularVelocity.cross(rA);
        const Vec3 vB = bodyB->linearVelocity + bodyB->angularVelocity.cross(rB);
        return vA - vB;
    }
};

} // namespace physics
