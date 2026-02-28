#pragma once
/// @file WebGLRenderer.h
/// @brief Minimal WebGL 2.0 wireframe renderer for the WASM demo.
///
/// Mirrors the visual look of the native DebugRenderer (gradient background,
/// per-body colour cycling, contact halos, velocity arrows, AABB overlays)
/// but targets GLSL ES 3.0 / WebGL 2 via Emscripten's OpenGL ES translation.
///
/// Design notes
/// ─────────────
/// • glPolygonMode does NOT exist in WebGL 2.  All wireframe drawing is done
///   by building GL_LINES geometry in the Mesh factory functions.
/// • Shaders are embedded as string literals (no filesystem I/O at runtime).
/// • The Camera orbit is driven by Emscripten HTML5 mouse callbacks that live
///   in main_wasm.cpp; the renderer only needs view / proj matrices.

#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "physics/ContactManifold.h"
#include "physics/AABB.h"
#include "math/Vec3.h"
#include "math/Mat4.h"

#include <vector>
#include <cstdint>

// Emscripten provides its own GL ES 3 headers; include order matters.
#ifdef __EMSCRIPTEN__
    #include <GLES3/gl3.h>
#else
    #include <GL/gl.h>
    #include <GL/glext.h>
#endif

namespace web {

using physics::Vec3;
using physics::Mat4;
using physics::RigidBody;
using physics::ContactManifold;
using physics::PhysicsWorld;
using physics::AABB;

// ─────────────────────────────────────────────────────────────────────────────
// GPU mesh handle
// ─────────────────────────────────────────────────────────────────────────────

/// Minimal VAO + VBO + EBO bundle.  Non-copyable, move-only.
struct WMesh {
    GLuint vao      = 0;
    GLuint vbo      = 0;
    GLuint ebo      = 0;
    int    count    = 0;
    GLenum drawMode = GL_TRIANGLES;

    WMesh() = default;
    ~WMesh();
    WMesh(const WMesh&)            = delete;
    WMesh& operator=(const WMesh&) = delete;
    WMesh(WMesh&&) noexcept;
    WMesh& operator=(WMesh&&) noexcept;

    void upload(const std::vector<float>&        positions,
                const std::vector<unsigned int>& indices,
                GLenum                           mode);
    void draw() const;
    bool valid() const { return vao != 0; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Shader wrapper
// ─────────────────────────────────────────────────────────────────────────────

struct WShader {
    GLuint program = 0;

    WShader() = default;
    ~WShader();
    WShader(const WShader&)            = delete;
    WShader& operator=(const WShader&) = delete;
    WShader(WShader&&) noexcept;
    WShader& operator=(WShader&&) noexcept;

    bool build(const char* vertSrc, const char* fragSrc);
    void use()  const;
    bool valid() const { return program != 0; }

    void setMat4 (const char* name, const Mat4& m) const;
    void setVec3 (const char* name, float x, float y, float z) const;
    void setFloat(const char* name, float v)                   const;
};

// ─────────────────────────────────────────────────────────────────────────────
// Camera
// ─────────────────────────────────────────────────────────────────────────────

/// Spherical-coordinate orbit camera matching the native Camera class interface.
class WCamera {
public:
    WCamera() = default;
    WCamera(Vec3 target, float dist, float yaw, float pitch);

    Mat4 viewMatrix() const;
    Mat4 projMatrix(float aspect) const;

    void onMouseDrag(float dx, float dy);
    void onScroll   (float delta);
    void update     (float dt);
    void reset();

    Vec3  position() const;
    Vec3  target()   const { return target_; }
    float distance() const { return smoothDist_; }
    void  setTarget(Vec3 t){ target_ = t; }

private:
    Vec3  target_  = {0.f, 0.f, 0.f};
    float distance_ = 20.f;
    float yaw_      = 45.f;
    float pitch_    = 30.f;
    float fovY_     = 60.f;
    float nearZ_    = 0.1f;
    float farZ_     = 500.f;
    float sensitiv_ = 0.3f;
    float zoomSpd_  = 1.2f;

    float smoothYaw_   = 45.f;
    float smoothPitch_ = 30.f;
    float smoothDist_  = 20.f;
    float smoothK_     = 16.f;
};

// ─────────────────────────────────────────────────────────────────────────────
// Debug renderer
// ─────────────────────────────────────────────────────────────────────────────

class WebGLRenderer {
public:
    WebGLRenderer()  = default;
    ~WebGLRenderer() = default;
    WebGLRenderer(const WebGLRenderer&)            = delete;
    WebGLRenderer& operator=(const WebGLRenderer&) = delete;

    /// Build shaders and GPU meshes.  Call once after GL context is ready.
    bool init();

    /// Release GL resources.
    void shutdown();

    // ── Per-frame ──────────────────────────────────────────────────────────

    void setTime(float t) { time_ = t; }

    /// Clear + cache view/proj.  Call first each frame.
    void beginFrame(const WCamera& cam, float aspect);

    /// Restore default GL state.
    void endFrame();

    // ── Draw calls ─────────────────────────────────────────────────────────

    void drawBody    (const RigidBody&       body,  int bodyIndex);
    void drawContact (const ContactManifold& c);
    void drawVelocity(const RigidBody&       body);
    void drawAABB    (const AABB&            aabb,  const Vec3& color);

    void drawWorld(const PhysicsWorld& world,
                   bool showContacts,
                   bool showVelocities,
                   bool showAABBs);

private:
    WShader shader_;    ///< Main 3-D: MVP + color + alpha
    WShader bgShader_;  ///< Full-screen gradient background

    WMesh sphereMesh_;
    WMesh boxMesh_;
    WMesh lineMesh_;
    WMesh pointMesh_;
    WMesh planeMesh_;
    WMesh bgMesh_;      ///< Full-screen quad

    Mat4  view_ = Mat4::identity();
    Mat4  proj_ = Mat4::identity();
    float time_ = 0.f;

    void drawMesh(const WMesh& mesh,
                  const Mat4&  model,
                  const Vec3&  color,
                  float        alpha = 1.f);

    void drawBackground();
    void drawWorldAxes();
    void drawGroundGrid();

    // ── Mesh factories ─────────────────────────────────────────────────────
    static WMesh makeWireframeSphere(int segs = 24);
    static WMesh makeWireframeBox();
    static WMesh makePoint();
    static WMesh makeLine();
    static WMesh makePlaneGrid();
    static WMesh makeFullscreenQuad();
};

} // namespace web
