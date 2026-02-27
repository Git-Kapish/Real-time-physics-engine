/// @file PhysicsWorld.cpp
/// @brief Physics world: fixed-timestep simulation loop with semi-implicit Euler.

#include "physics/PhysicsWorld.h"
#include "physics/CollisionDetector.h"
#include <algorithm>
#include <cassert>

namespace physics {

PhysicsWorld::PhysicsWorld(PhysicsConfig cfg)
    : config_(cfg) {}

// ── Body management ───────────────────────────────────────────────────────

int PhysicsWorld::addBody(RigidBody body) {
    assert(static_cast<int>(bodies_.size()) < config_.maxBodies
           && "PhysicsWorld: maxBodies limit reached");
    body.id = nextId_++;
    // Initialise world-space inertia tensor
    body.updateInertiaTensor();
    bodies_.push_back(std::move(body));
    return bodies_.back().id;
}

void PhysicsWorld::removeBody(int id) {
    // Swap-and-pop to keep the vector dense
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
        if (bodies_[i].id == id) {
            if (i != bodies_.size() - 1) {
                bodies_[i] = std::move(bodies_.back());
            }
            bodies_.pop_back();
            return;
        }
    }
    // id not found — no-op
}

RigidBody* PhysicsWorld::getBody(int id) {
    for (auto& b : bodies_) {
        if (b.id == id) return &b;
    }
    return nullptr;
}

const RigidBody* PhysicsWorld::getBody(int id) const {
    for (const auto& b : bodies_) {
        if (b.id == id) return &b;
    }
    return nullptr;
}

// ── Simulation control ────────────────────────────────────────────────────

void PhysicsWorld::update(float dt) {
    accumulator_ += dt;
    while (accumulator_ >= config_.fixedDt) {
        step();
        accumulator_ -= config_.fixedDt;
    }
}

void PhysicsWorld::step() {
    // 1. Apply gravity as a force on every dynamic body
    applyGravity();

    // 2. Semi-implicit Euler: v += (F/m) * dt
    integrateForces(config_.fixedDt);

    // 3. Semi-implicit Euler: x += v * dt  (uses NEWLY updated velocity)
    //    Also integrates orientation from angular velocity.
    integrateVelocities(config_.fixedDt);

    // 4. Recompute world-space inertia tensor after orientation change
    for (auto& body : bodies_) {
        body.updateInertiaTensor();
    }

    // 5. Collision detection — broad phase then narrow phase
    lastContacts_.clear();
    const auto pairs = CollisionDetector::broadPhase(bodies_);
    for (const auto& [i, j] : pairs) {
        auto contact = CollisionDetector::detect(bodies_[i], bodies_[j]);
        if (contact.has_value()) {
            lastContacts_.push_back(std::move(*contact));
        }
    }

    // 6. Impulse resolution: normal + friction + Baumgarte correction
    solver_.solve(lastContacts_, config_.fixedDt);

    // 7. Reset accumulators for next step
    clearAllForces();

    ++stepCount_;
}

void PhysicsWorld::reset() {
    bodies_.clear();
    accumulator_ = 0.0f;
    stepCount_   = 0;
    nextId_      = 0;
}

// ── Private integration helpers ───────────────────────────────────────────

void PhysicsWorld::applyGravity() {
    for (auto& body : bodies_) {
        if (!body.isDynamic()) continue;
        // F_gravity = m * g   (gravity is an acceleration, scale by mass)
        body.forceAccum += config_.gravity * body.mass;
    }
}

void PhysicsWorld::integrateForces(float dt) {
    for (auto& body : bodies_) {
        if (!body.isDynamic()) continue;

        // Linear: v += (F / m) * dt = F * invMass * dt
        body.linearVelocity += body.forceAccum * body.invMass * dt;

        // Angular: omega += I^-1_world * tau * dt
        body.angularVelocity += body.invInertiaTensorWorld * body.torqueAccum * dt;
    }
}

void PhysicsWorld::integrateVelocities(float dt) {
    for (auto& body : bodies_) {
        if (!body.isDynamic()) continue;

        // Position uses the UPDATED velocity (semi-implicit Euler)
        body.position += body.linearVelocity * dt;

        // Orientation integration: q' = normalize(q + 0.5*dt*[0,omega]*q)
        body.orientation = body.orientation.integrated(body.angularVelocity, dt);
    }
}

void PhysicsWorld::clearAllForces() {
    for (auto& body : bodies_) {
        body.clearForces();
    }
}

} // namespace physics
