#pragma once
/// @file Camera.h
/// @brief Orbiting perspective camera controlled by mouse drag and scroll.

#include "math/Vec3.h"
#include "math/Mat4.h"

namespace renderer {

using physics::Vec3;
using physics::Mat4;

/**
 * Spherical-coordinate orbit camera.
 *
 * Pitch and yaw are stored in degrees and clamped so the camera never flips
 * past the poles.  The target point, distance and field-of-view are all
 * adjustable at runtime.
 */
class Camera {
public:
    Camera() = default;

    /**
     * Construct with explicit orbit parameters.
     * @param target    World-space look-at point.
     * @param distance  Initial distance from target (metres).
     * @param yaw       Horizontal orbit angle (degrees).
     * @param pitch     Vertical orbit angle (degrees, clamped to ±89°).
     */
    Camera(Vec3 target, float distance, float yaw, float pitch);

    // ── Matrices ──────────────────────────────────────────────────────────

    /// Build the view matrix from the current orbit state.
    Mat4 viewMatrix() const;

    /// Build a perspective projection matrix for the given aspect ratio.
    Mat4 projMatrix(float aspect) const;

    // ── Input handlers ────────────────────────────────────────────────────

    /// Orbit the camera: dx rotates yaw, dy rotates pitch.
    void onMouseDrag(float dx, float dy);

    /// Zoom by changing the orbit distance (positive delta → zoom in).
    void onScroll(float delta);

    /// Notify the camera of a viewport resize (stored for future use).
    void onResize(int w, int h);

    // ── Accessors / mutators ──────────────────────────────────────────────

    Vec3  position()  const { return computePosition(); }
    Vec3  target()    const { return target_; }
    float distance()  const { return distance_; }

    void  setTarget(Vec3 t) { target_ = t; }

    /// Reset to construction-time defaults.
    void reset();

private:
    Vec3  target_    = {0.f, 0.f, 0.f};
    float distance_  = 15.f;
    float yaw_       = 45.f;   ///< degrees
    float pitch_     = 30.f;   ///< degrees, clamped to [-89, 89]
    float fovY_      = 60.f;   ///< degrees
    float nearPlane_ = 0.1f;
    float farPlane_  = 500.f;
    float sensitivity_ = 0.3f;
    float zoomSpeed_   = 1.0f;

    /// Default values (used by reset()).
    Vec3  defaultTarget_   = {0.f, 0.f, 0.f};
    float defaultDistance_ = 15.f;
    float defaultYaw_      = 45.f;
    float defaultPitch_    = 30.f;

    /// Compute the eye position from spherical coordinates.
    Vec3 computePosition() const;
};

} // namespace renderer
