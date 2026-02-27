#pragma once
/// @file DebugRenderer.h
/// @brief Wireframe renderer for the rigid-body physics simulation.

#include "renderer/Shader.h"
#include "renderer/Mesh.h"
#include "renderer/Camera.h"
#include "physics/RigidBody.h"
#include "physics/ContactManifold.h"
#include "physics/PhysicsWorld.h"
#include "physics/AABB.h"

namespace renderer {

using physics::Vec3;
using physics::Mat4;
using physics::RigidBody;
using physics::ContactManifold;
using physics::PhysicsWorld;
using physics::AABB;

/**
 * Debug wireframe renderer for the physics engine.
 *
 * !!  init() MUST be called after creating the GL context (gladLoadGL()) !!
 *
 * Typical frame:
 *   renderer.setTime(timer.elapsed());
 *   renderer.beginFrame(camera, window.aspect());
 *   renderer.drawWorld(world, showContacts, showVelocities, showAABBs);
 *   renderer.endFrame();
 *
 * Visual features:
 *   - Gradient background (deep space blue/purple)
 *   - Per-body unique hue (golden-ratio color cycling for dynamic bodies)
 *   - Static bodies: warm red, Kinematic: cyan
 *   - Multi-layer contact glow (3 overlapping point halos: yellow/white core)
 *   - Velocity vectors color-coded by speed (green->yellow->red)
 *   - Ground plane grid: X-axis red, Z-axis blue, grid dim gray
 *   - World-origin axis cross (XYZ = R/G/B)
 *   - Thicker line widths for better visibility
 *   - AABB overlay with subtle transparency
 */
class DebugRenderer {
public:
    DebugRenderer()  = default;
    ~DebugRenderer() = default;

    DebugRenderer(const DebugRenderer&)            = delete;
    DebugRenderer& operator=(const DebugRenderer&) = delete;

    /**
     * Compile shaders and build GPU meshes.
     * Must be called once after the OpenGL context is current.
     */
    bool init();

    /// Release all GL resources.
    void shutdown();

    // ── Per-frame API ──────────────────────────────────────────────────────

    /// Update the internal time (seconds since app start) for animated effects.
    void setTime(float t) { time_ = t; }

    /// Draw gradient background + cache view/proj matrices. Call first each frame.
    void beginFrame(const Camera& cam, float aspect);

    /// Restore default GL state. Call after all draw* calls.
    void endFrame();

    // ── Draw calls ─────────────────────────────────────────────────────────

    void drawBody    (const RigidBody&       body);
    void drawContact (const ContactManifold& contact);
    void drawVelocity(const RigidBody&       body);
    void drawAABB    (const AABB& aabb, const Vec3& color);

    void drawWorld(const PhysicsWorld& world,
                   bool showContacts,
                   bool showVelocities,
                   bool showAABBs);

private:
    // ── Shaders ───────────────────────────────────────────────────────────
    Shader shader_;    ///< Main 3-D debug shader (MVP + color + alpha)
    Shader bgShader_;  ///< Screen-space gradient background shader

    // ── Meshes ────────────────────────────────────────────────────────────
    Mesh sphereMesh_;
    Mesh boxMesh_;
    Mesh lineMesh_;
    Mesh pointMesh_;
    Mesh planeMesh_;
    Mesh bgMesh_;      ///< Full-screen quad for gradient background

    // ── Frame state ───────────────────────────────────────────────────────
    Mat4  view_ = Mat4::identity();
    Mat4  proj_ = Mat4::identity();
    float time_ = 0.f;

    // ── Private helpers ───────────────────────────────────────────────────

    void drawMesh(const Mesh&  mesh,
                  const Mat4&  model,
                  const Vec3&  color,
                  float        alpha    = 1.f,
                  unsigned int polyMode = 0x1B01 /* GL_LINE */);

    void drawBackground();
    void drawWorldAxes();
};

} // namespace renderer
