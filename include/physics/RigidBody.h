#pragma once
/// @file RigidBody.h
/// @brief Rigid body definition with mass, inertia, and force accumulation.

#include "math/Vec3.h"
#include "math/Mat3.h"
#include "math/Quat.h"

namespace physics {

/// Distinguishes how the physics world treats a body.
enum class BodyType {
    Dynamic,    ///< Fully simulated: affected by forces and gravity
    Static,     ///< Immovable: infinite mass, never integrated
    Kinematic   ///< Moved manually: not affected by forces, but can push dynamic bodies
};

/// Primitive shape types for a rigid body.
enum class ShapeType {
    Sphere, ///< Spherical collider
    Box,    ///< Box collider
    Plane   ///< Infinite static plane
};

/// Describes the collision shape attached to a rigid body.
struct Shape {
    ShapeType type        = ShapeType::Sphere; ///< Primitive type
    Vec3      halfExtents;                     ///< Box half-dimensions or (r,r,r) for sphere
    float     radius      = 0.0f;              ///< Sphere radius (also plane offset for Plane)
};

/// A rigid body: the fundamental simulation object.
struct RigidBody {
    // ── Identity ──────────────────────────────────────────────────────────

    int      id       = -1;               ///< Unique ID assigned by PhysicsWorld
    BodyType bodyType = BodyType::Dynamic; ///< Simulation category

    // ── Shape ─────────────────────────────────────────────────────────────

    Shape shape; ///< Collision shape

    // ── State ─────────────────────────────────────────────────────────────

    Vec3 position        = Vec3::zero();  ///< World-space position (centre of mass)
    Quat orientation     = Quat::identity(); ///< World-space orientation
    Vec3 linearVelocity  = Vec3::zero();  ///< Linear velocity (m/s)
    Vec3 angularVelocity = Vec3::zero();  ///< Angular velocity (rad/s)

    // ── Mass properties ───────────────────────────────────────────────────

    float mass    = 1.0f; ///< Total mass (kg); 0 for static
    float invMass = 1.0f; ///< 1/mass; 0 for static/infinite-mass bodies

    Mat3 inertiaTensorLocal;       ///< Inertia tensor in body space
    Mat3 invInertiaTensorLocal;    ///< Inverse inertia tensor in body space
    Mat3 invInertiaTensorWorld;    ///< Inverse inertia tensor in world space (recomputed each step)

    // ── Force accumulators (cleared every step) ───────────────────────────

    Vec3 forceAccum  = Vec3::zero(); ///< Sum of forces applied this step
    Vec3 torqueAccum = Vec3::zero(); ///< Sum of torques applied this step

    // ── Material ─────────────────────────────────────────────────────────

    float restitution = 0.3f; ///< Coefficient of restitution (bounciness 0–1)
    float friction    = 0.5f; ///< Coefficient of friction

    // ── Queries ───────────────────────────────────────────────────────────

    /// True if this body is static (infinite mass, not simulated).
    bool isStatic() const { return bodyType == BodyType::Static; }

    /// True if this body is dynamically simulated.
    bool isDynamic() const { return bodyType == BodyType::Dynamic; }

    // ── Force application ─────────────────────────────────────────────────

    /// Apply a world-space force (accumulates until end of step).
    void applyForce(const Vec3& force) {
        if (!isDynamic()) return;
        forceAccum += force;
    }

    /// Apply a world-space torque (accumulates until end of step).
    void applyTorque(const Vec3& torque) {
        if (!isDynamic()) return;
        torqueAccum += torque;
    }

    /// Apply a force at a world-space point, generating both force and torque.
    void applyForceAtPoint(const Vec3& force, const Vec3& worldPoint) {
        if (!isDynamic()) return;
        forceAccum  += force;
        torqueAccum += (worldPoint - position).cross(force);
    }

    // ── Impulse application ───────────────────────────────────────────────

    /// Apply a linear impulse, instantly changing linearVelocity.
    void applyImpulse(const Vec3& impulse) {
        if (!isDynamic()) return;
        linearVelocity += impulse * invMass;
    }

    /// Apply an angular impulse, instantly changing angularVelocity.
    void applyAngularImpulse(const Vec3& impulse) {
        if (!isDynamic()) return;
        angularVelocity += invInertiaTensorWorld * impulse;
    }

    /// Apply an impulse at a world-space point, affecting both linear and angular velocity.
    void applyImpulseAtPoint(const Vec3& impulse, const Vec3& worldPoint) {
        if (!isDynamic()) return;
        applyImpulse(impulse);
        angularVelocity += invInertiaTensorWorld * (worldPoint - position).cross(impulse);
    }

    // ── Housekeeping ──────────────────────────────────────────────────────

    /// Zero out all accumulated forces and torques.
    void clearForces() {
        forceAccum  = Vec3::zero();
        torqueAccum = Vec3::zero();
    }

    /// Recompute invInertiaTensorWorld from current orientation.
    /// Call this after each integration step to keep it up to date.
    void updateInertiaTensor();

    // ── Static factories (implemented in RigidBody.cpp) ───────────────────

    /// Create a sphere with given radius, mass, and initial position.
    static RigidBody makeSphere(float radius, float mass, const Vec3& position);

    /// Create a box with given half-extents, mass, and initial position.
    static RigidBody makeBox(const Vec3& halfExtents, float mass, const Vec3& position);

    /// Create a static infinite plane. Normal stored in halfExtents, offset in radius.
    static RigidBody makeStaticPlane(const Vec3& normal, float offset);
};

} // namespace physics
