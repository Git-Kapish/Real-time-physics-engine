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
 *   renderer.beginFrame(camera, window.aspect());
 *   renderer.drawWorld(world, showContacts, showVelocities, showAABBs);
 *   renderer.endFrame();
 *
 * Colour scheme:
 *   Dynamic   (0.85, 0.85, 0.85)  near-white
 *   Static    (0.80, 0.25, 0.25)  red
 *   Kinematic (0.25, 0.55, 0.85)  blue
 *   Contact   (1.00, 0.85, 0.00)  yellow point
 *   Normal    (1.00, 0.50, 0.00)  orange line
 *   Velocity  (0.30, 0.90, 0.40)  green line
 *   AABB      (0.40, 0.40, 0.90)  muted blue, alpha 0.3
 */
class DebugRenderer {
public:
    DebugRenderer()  = default;
    ~DebugRenderer() = default;

    // Non-copyable
    DebugRenderer(const DebugRenderer&)            = delete;
    DebugRenderer& operator=(const DebugRenderer&) = delete;

    /**
     * Compile shaders and build GPU meshes.
     * Must be called once, after the OpenGL context is current.
     * @return true on success.
     */
    bool init();

    /// Explicitly release all GL resources (also happens in destructors).
    void shutdown();

    // ── Per-frame API ──────────────────────────────────────────────────────

    /// Cache the current view/projection matrices.  Call before any draw*.
    void beginFrame(const Camera& cam, float aspect);

    /// Restore default GL state.  Call after all draw* calls.
    void endFrame();

    // ── Draw calls ─────────────────────────────────────────────────────────

    void drawBody    (const RigidBody&       body);
    void drawContact (const ContactManifold& contact);
    void drawVelocity(const RigidBody&       body);
    void drawAABB    (const AABB& aabb, const Vec3& color);

    /**
     * Convenience: iterate all bodies and contacts in a PhysicsWorld.
     * @param showContacts    Draw yellow dots + orange normals.
     * @param showVelocities  Draw green velocity vectors.
     * @param showAABBs       Draw muted-blue AABB overlays.
     */
    void drawWorld(const PhysicsWorld& world,
                   bool showContacts,
                   bool showVelocities,
                   bool showAABBs);

private:
    Shader shader_;
    Mesh   sphereMesh_;
    Mesh   boxMesh_;
    Mesh   lineMesh_;
    Mesh   pointMesh_;
    Mesh   planeMesh_;

    Mat4   view_ = Mat4::identity();
    Mat4   proj_ = Mat4::identity();

    /// Draw a mesh with a full transform, colour, alpha and polygon mode.
    void drawMesh(const Mesh&  mesh,
                  const Mat4&  model,
                  const Vec3&  color,
                  float        alpha    = 1.f,
                  unsigned int polyMode = 0x1B01 /* GL_LINE */);
};

} // namespace renderer
