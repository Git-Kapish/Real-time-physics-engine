#pragma once
/// @file PhysicsWorld.h
/// @brief Fixed-timestep physics world managing rigid body simulation.

#include "physics/RigidBody.h"
#include <vector>
#include <cstdint>

namespace physics {

/// Global configuration for the physics simulation.
struct PhysicsConfig {
    Vec3  gravity              = Vec3(0.0f, -9.81f, 0.0f); ///< Gravitational acceleration (m/s²)
    float fixedDt              = 1.0f / 60.0f;              ///< Fixed simulation timestep (s)
    int   maxBodies            = 1024;                      ///< Maximum number of bodies
    float restitutionThreshold = 0.5f;                      ///< Relative vel below this → no bounce
};

/// The main physics world: manages rigid bodies and steps the simulation.
class PhysicsWorld {
public:
    /// Construct with optional configuration.
    explicit PhysicsWorld(PhysicsConfig cfg = {});

    // ── Body management ───────────────────────────────────────────────────

    /// Add a body to the world; assigns and returns its unique ID.
    int addBody(RigidBody body);

    /// Remove a body by ID (swap-and-pop; IDs of other bodies are stable).
    void removeBody(int id);

    /// Get a mutable pointer to a body by ID; returns nullptr if not found.
    RigidBody* getBody(int id);

    /// Get a const pointer to a body by ID; returns nullptr if not found.
    const RigidBody* getBody(int id) const;

    /// Read-only access to the full body list.
    const std::vector<RigidBody>& bodies() const { return bodies_; }

    // ── Simulation ────────────────────────────────────────────────────────

    /// Advance with a variable timestep; internally runs fixed sub-steps.
    void update(float dt);

    /// Advance exactly one fixedDt step.
    void step();

    /// Remove all bodies and reset the accumulator and step counter.
    void reset();

    // ── Accessors ─────────────────────────────────────────────────────────

    /// Total number of fixed steps taken since construction (or last reset).
    uint64_t stepCount() const { return stepCount_; }

    /// Read-only access to the current configuration.
    const PhysicsConfig& config() const { return config_; }

private:
    std::vector<RigidBody> bodies_;    ///< All bodies in the world
    PhysicsConfig          config_;    ///< Simulation parameters
    float                  accumulator_ = 0.0f; ///< Leftover dt from variable-timestep updates
    uint64_t               stepCount_  = 0;     ///< Number of step() calls so far
    int                    nextId_     = 0;      ///< Counter for assigning unique IDs

    // ── Internal integration steps ────────────────────────────────────────

    /// Add gravity force to all dynamic bodies.
    void applyGravity();

    /// Semi-implicit Euler: v += (F/m) * dt for each dynamic body.
    void integrateForces(float dt);

    /// Semi-implicit Euler: x += v * dt (using updated velocity) + integrate orientation.
    void integrateVelocities(float dt);

    /// Zero all force and torque accumulators.
    void clearAllForces();
};

} // namespace physics
