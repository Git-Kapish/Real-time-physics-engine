/// @file ImpulseSolver.cpp
/// @brief Impulse-based velocity resolution (normal + friction) and Baumgarte position correction.

#include "physics/ImpulseSolver.h"
#include <algorithm>
#include <cmath>
#include <cassert>

namespace physics {

ImpulseSolver::ImpulseSolver(SolverConfig cfg) : config_(cfg) {
    assert(cfg.velocityIterations >= 1 && "velocityIterations must be >= 1");
    assert(cfg.positionIterations >= 0 && "positionIterations must be >= 0");
    assert(cfg.baumgarte >= 0.0f && cfg.baumgarte <= 1.0f
           && "baumgarte must be in [0, 1]");
}

void ImpulseSolver::setConfig(SolverConfig cfg) {
    assert(cfg.velocityIterations >= 1);
    assert(cfg.positionIterations >= 0); // 0 = skip position correction
    assert(cfg.baumgarte >= 0.0f && cfg.baumgarte <= 1.0f);
    config_ = cfg;
}

// ── Primary solve loop ────────────────────────────────────────────────────

void ImpulseSolver::solve(std::vector<ContactManifold>& contacts, float /*dt*/) {
    // Velocity resolution — iterate to handle multiple simultaneous contacts
    for (int iter = 0; iter < config_.velocityIterations; ++iter) {
        for (auto& contact : contacts) {
            resolveVelocity(contact);
        }
    }

    // Position correction — Baumgarte stabilization
    for (int iter = 0; iter < config_.positionIterations; ++iter) {
        for (auto& contact : contacts) {
            resolvePosition(contact);
        }
    }
}

// ── Velocity resolution ───────────────────────────────────────────────────

void ImpulseSolver::resolveVelocity(ContactManifold& contact) {
    RigidBody* bodyA = contact.bodyA;
    RigidBody* bodyB = contact.bodyB;

    // Step 1: Relative velocity at contact point
    const Vec3 rA     = contact.contactPoint - bodyA->position;
    const Vec3 rB     = contact.contactPoint - bodyB->position;
    const Vec3 vA     = bodyA->linearVelocity + bodyA->angularVelocity.cross(rA);
    const Vec3 vB     = bodyB->linearVelocity + bodyB->angularVelocity.cross(rB);
    const Vec3 relVel = vA - vB;

    // Step 2: Relative velocity along normal
    const float vn = relVel.dot(contact.normal);
    if (vn > 0.0f) return; // bodies already separating

    // Step 3: Effective mass along normal
    const float invMassSum = bodyA->invMass + bodyB->invMass;

    const Vec3 rAxN         = rA.cross(contact.normal);
    const Vec3 rBxN         = rB.cross(contact.normal);
    const float angularTermA = contact.normal.dot(
        (bodyA->invInertiaTensorWorld * rAxN).cross(rA));
    const float angularTermB = contact.normal.dot(
        (bodyB->invInertiaTensorWorld * rBxN).cross(rB));

    const float effectiveMass = invMassSum + angularTermA + angularTermB;
    if (effectiveMass < 1e-10f) return;

    // Step 4: Restitution — suppress bounce for slow collisions
    const float e = (std::fabs(vn) < config_.restitutionThreshold)
                    ? 0.0f
                    : contact.restitution;

    // Step 5: Normal impulse magnitude — always non-negative (push apart only)
    float j = -(1.0f + e) * vn / effectiveMass;
    j = std::max(j, 0.0f);

    // Step 6: Apply normal impulse
    const Vec3 impulse = contact.normal * j;
    bodyA->applyImpulseAtPoint( impulse, contact.contactPoint);
    bodyB->applyImpulseAtPoint(-impulse, contact.contactPoint);

    // Step 7: Friction impulse — recompute relVel after normal impulse
    const Vec3 vA2      = bodyA->linearVelocity + bodyA->angularVelocity.cross(rA);
    const Vec3 vB2      = bodyB->linearVelocity + bodyB->angularVelocity.cross(rB);
    const Vec3 relVel2  = vA2 - vB2;

    Vec3 tangent = relVel2 - contact.normal * relVel2.dot(contact.normal);
    if (tangent.lengthSq() < 1e-8f) return; // no tangential motion
    tangent = tangent.normalized();

    // Effective mass along tangent
    const Vec3  rAxT         = rA.cross(tangent);
    const Vec3  rBxT         = rB.cross(tangent);
    const float angularTermTA = tangent.dot(
        (bodyA->invInertiaTensorWorld * rAxT).cross(rA));
    const float angularTermTB = tangent.dot(
        (bodyB->invInertiaTensorWorld * rBxT).cross(rB));
    const float effectiveMassT = invMassSum + angularTermTA + angularTermTB;
    if (effectiveMassT < 1e-10f) return;

    const float vt = relVel2.dot(tangent);
    float jt = -vt / effectiveMassT;

    // Coulomb friction cone clamp: |jt| <= mu * |j|
    const float muCombined = contact.friction;
    jt = std::clamp(jt, -muCombined * j, muCombined * j);

    const Vec3 frictionImpulse = tangent * jt;
    bodyA->applyImpulseAtPoint( frictionImpulse, contact.contactPoint);
    bodyB->applyImpulseAtPoint(-frictionImpulse, contact.contactPoint);
}

// ── Position correction (Baumgarte) ──────────────────────────────────────

void ImpulseSolver::resolvePosition(ContactManifold& contact) {
    RigidBody* bodyA = contact.bodyA;
    RigidBody* bodyB = contact.bodyB;

    const float totalInvMass = bodyA->invMass + bodyB->invMass;
    if (totalInvMass < 1e-10f) return; // both static/infinite mass

    // Baumgarte: correct only the overlap beyond the slop threshold
    const float pen = std::max(contact.penetrationDepth - config_.slop, 0.0f);
    if (pen < 1e-10f) return;

    const float correction        = pen * config_.baumgarte / totalInvMass;
    const Vec3  correctionVec     = contact.normal * correction;

    if (bodyA->isDynamic()) bodyA->position += correctionVec * bodyA->invMass;
    if (bodyB->isDynamic()) bodyB->position -= correctionVec * bodyB->invMass;
}

} // namespace physics
