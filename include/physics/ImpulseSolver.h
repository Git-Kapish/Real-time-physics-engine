#pragma once
/// @file ImpulseSolver.h
/// @brief Impulse-based collision response with Coulomb friction and Baumgarte stabilization.

#include "physics/ContactManifold.h"
#include <vector>
#include <cassert>

namespace physics {

/// Configuration for the impulse solver.
struct SolverConfig {
    int   velocityIterations    = 8;     ///< Impulse solve iterations per step
    int   positionIterations    = 3;     ///< Baumgarte position correction iterations
    float baumgarte             = 0.2f;  ///< Position correction factor [0, 1]
    float slop                  = 0.01f; ///< Penetration slop — tiny overlaps ignored
    float restitutionThreshold  = 0.5f;  ///< Closing speed below this → inelastic (e=0)
    float sleepLinearThreshold  = 0.05f; ///< Linear speed below this → candidate for sleep
    float sleepAngularThreshold = 0.05f; ///< Angular speed below this → candidate for sleep
};

/// Resolves contact manifolds by applying corrective impulses and Baumgarte position correction.
class ImpulseSolver {
public:
    /// Construct with optional solver configuration.
    explicit ImpulseSolver(SolverConfig cfg = {});

    // ── Main interface ────────────────────────────────────────────────────

    /// Solve all contacts: velocity resolution then position correction.
    void solve(std::vector<ContactManifold>& contacts, float dt);

    // ── Configuration ─────────────────────────────────────────────────────

    /// Read-only access to the current solver configuration.
    const SolverConfig& config() const { return config_; }

    /// Replace the solver configuration.
    void setConfig(SolverConfig cfg);

private:
    SolverConfig config_; ///< Active solver parameters

    /// Apply normal impulse and Coulomb friction to separate one contact.
    void resolveVelocity(ContactManifold& contact);

    /// Nudge body positions to correct positional drift (Baumgarte).
    void resolvePosition(ContactManifold& contact);
};

} // namespace physics
