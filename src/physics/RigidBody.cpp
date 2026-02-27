/// @file RigidBody.cpp
/// @brief Factory methods and inertia tensor computation for RigidBody.

#include "physics/RigidBody.h"
#include <cassert>
#include <cmath>

namespace physics {

void RigidBody::updateInertiaTensor() {
    if (!isDynamic()) {
        // Static/kinematic: world inertia stays zero (no rotation accumulation)
        invInertiaTensorWorld = Mat3::zero();
        return;
    }
    // Rotate the local inverse inertia into world space: R * I^-1_local * R^T
    const Mat3 R = orientation.toMat3();
    invInertiaTensorWorld = R * invInertiaTensorLocal * R.transposed();
}

RigidBody RigidBody::makeSphere(float radius, float massVal, const Vec3& pos) {
    assert(radius > 0.0f && "Sphere radius must be positive");
    assert(massVal > 0.0f && "Sphere mass must be positive");

    RigidBody body;
    body.shape.type        = ShapeType::Sphere;
    body.shape.radius      = radius;
    body.shape.halfExtents = Vec3(radius, radius, radius);
    body.position          = pos;
    body.mass              = massVal;
    body.invMass           = 1.0f / massVal;

    // I = (2/5) * m * r^2 on all 3 axes
    const float I = (2.0f / 5.0f) * massVal * radius * radius;
    body.inertiaTensorLocal    = Mat3::diagonal(I, I, I);
    body.invInertiaTensorLocal = Mat3::diagonal(1.0f / I, 1.0f / I, 1.0f / I);
    body.invInertiaTensorWorld = body.invInertiaTensorLocal; // starts aligned
    return body;
}

RigidBody RigidBody::makeBox(const Vec3& halfExtents, float massVal, const Vec3& pos) {
    assert(halfExtents.x > 0.0f && halfExtents.y > 0.0f && halfExtents.z > 0.0f
           && "Box half-extents must all be positive");
    assert(massVal > 0.0f && "Box mass must be positive");

    RigidBody body;
    body.shape.type        = ShapeType::Box;
    body.shape.halfExtents = halfExtents;
    body.shape.radius      = 0.0f;
    body.position          = pos;
    body.mass              = massVal;
    body.invMass           = 1.0f / massVal;

    // Full dimensions
    const float w = 2.0f * halfExtents.x; // width
    const float h = 2.0f * halfExtents.y; // height
    const float d = 2.0f * halfExtents.z; // depth

    const float c   = massVal / 12.0f;
    const float Ix  = c * (h * h + d * d);
    const float Iy  = c * (w * w + d * d);
    const float Iz  = c * (w * w + h * h);

    body.inertiaTensorLocal    = Mat3::diagonal(Ix, Iy, Iz);
    body.invInertiaTensorLocal = Mat3::diagonal(1.0f / Ix, 1.0f / Iy, 1.0f / Iz);
    body.invInertiaTensorWorld = body.invInertiaTensorLocal;
    return body;
}

RigidBody RigidBody::makeStaticPlane(const Vec3& normal, float offset) {
    RigidBody body;
    body.bodyType          = BodyType::Static;
    body.shape.type        = ShapeType::Plane;
    body.shape.halfExtents = normal; // normal packed into halfExtents
    body.shape.radius      = offset; // d (plane equation: n·x = d)
    body.mass              = 0.0f;
    body.invMass           = 0.0f;

    // Static: zero inertia tensors
    body.inertiaTensorLocal    = Mat3::zero();
    body.invInertiaTensorLocal = Mat3::zero();
    body.invInertiaTensorWorld = Mat3::zero();
    return body;
}

} // namespace physics
