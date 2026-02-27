#pragma once
/// @file AABB.h
/// @brief Axis-Aligned Bounding Box for broad-phase and queries.

#include "math/Vec3.h"
#include "math/Quat.h"
#include <algorithm>
#include <cmath>

namespace physics {

/// Axis-Aligned Bounding Box defined by min and max corners.
struct AABB {
    Vec3 min; ///< Minimum corner (smallest x, y, z)
    Vec3 max; ///< Maximum corner (largest x, y, z)

    // ── Constructors ──────────────────────────────────────────────────────

    /// Default: degenerate/empty box.
    AABB() : min(0.0f), max(0.0f) {}

    /// Construct from explicit min/max corners.
    AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}

    // ── Queries ───────────────────────────────────────────────────────────

    /// True if this AABB overlaps another on ALL 3 axes (touching edges = no overlap).
    bool overlaps(const AABB& other) const {
        return (min.x < other.max.x && max.x > other.min.x) &&
               (min.y < other.max.y && max.y > other.min.y) &&
               (min.z < other.max.z && max.z > other.min.z);
    }

    /// Return the center point of the AABB.
    Vec3 center() const {
        return (min + max) * 0.5f;
    }

    /// Return the half-dimensions (extents) of the AABB.
    Vec3 extents() const {
        return (max - min) * 0.5f;
    }

    /// Surface area: 2*(w*h + w*d + h*d).
    float surfaceArea() const {
        const Vec3 d = max - min;
        return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
    }

    /// Return the smallest AABB that contains both this and other.
    AABB merged(const AABB& other) const {
        return {Vec3(std::fmin(min.x, other.min.x),
                     std::fmin(min.y, other.min.y),
                     std::fmin(min.z, other.min.z)),
                Vec3(std::fmax(max.x, other.max.x),
                     std::fmax(max.y, other.max.y),
                     std::fmax(max.z, other.max.z))};
    }

    /// True if the point is strictly inside (not on the boundary).
    bool contains(const Vec3& p) const {
        return (p.x > min.x && p.x < max.x) &&
               (p.y > min.y && p.y < max.y) &&
               (p.z > min.z && p.z < max.z);
    }

    // ── Static factories ──────────────────────────────────────────────────

    /// Construct the AABB for a sphere.
    static AABB fromSphere(const Vec3& center, float radius) {
        const Vec3 r(radius, radius, radius);
        return {center - r, center + r};
    }

    /// Construct the world-space AABB for an oriented box by projecting all 8 corners.
    static AABB fromBox(const Vec3& center, const Quat& orientation, const Vec3& halfExtents) {
        // Build the 3 world-space axes scaled by halfExtents
        const Vec3 ax = orientation.rotate(Vec3(halfExtents.x, 0.0f, 0.0f));
        const Vec3 ay = orientation.rotate(Vec3(0.0f, halfExtents.y, 0.0f));
        const Vec3 az = orientation.rotate(Vec3(0.0f, 0.0f, halfExtents.z));

        // Project onto each world axis to find max extent (Arvo's method)
        const Vec3 extent(
            std::fabs(ax.x) + std::fabs(ay.x) + std::fabs(az.x),
            std::fabs(ax.y) + std::fabs(ay.y) + std::fabs(az.y),
            std::fabs(ax.z) + std::fabs(ay.z) + std::fabs(az.z)
        );
        return {center - extent, center + extent};
    }
};

} // namespace physics
