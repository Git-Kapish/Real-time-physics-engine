/// @file main_wasm.cpp
/// @brief Emscripten WebAssembly entry point for the physics engine browser demo.
///
/// Architecture
/// ─────────────
/// • A single PhysicsWorld and WebGLRenderer are stored as module-level globals.
/// • emscripten_set_main_loop drives physics stepping + rendering at 60 fps.
/// • Mouse / wheel events on the <canvas> feed an orbit WCamera.
/// • JS bindings (Embind) expose addBox, addSphere, setGravity, step, reset,
///   getBodyTransforms, getBodyCount, getFPS and the toggle flags.
///
/// Build command (see README_WASM.md):
///   emcmake cmake <repo>/physics_engine -B build_wasm -DCMAKE_BUILD_TYPE=Release
///   cmake --build build_wasm --target physics_engine_wasm

#include "WebGLRenderer.h"

#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "math/Vec3.h"
#include "math/Mat4.h"

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <emscripten/html5.h>
#include <GLES3/gl3.h>

#include <cstdio>
#include <cmath>
#include <string>
#include <chrono>
#include <random>

using namespace physics;
using namespace web;

// ─────────────────────────────────────────────────────────────────────────────
// Module-level state
// ─────────────────────────────────────────────────────────────────────────────

static PhysicsWorld  g_world;
static WebGLRenderer g_renderer;
static WCamera       g_camera({0,3,0}, 22.f, 45.f, 28.f);
static int           g_width  = 800;
static int           g_height = 600;
static bool          g_paused = false;

// FPS tracking
static double g_fpsAccum    = 0.0;
static int    g_fpsFrames   = 0;
static float  g_fps         = 0.f;
static double g_lastTime    = 0.0;

// Mouse state for orbit
static bool  g_mouseDown = false;
static float g_lastMouseX = 0.f, g_lastMouseY = 0.f;

// Visualisation toggles (readable from JS)
static bool g_showContacts   = true;
static bool g_showVelocities = false;
static bool g_showAABBs      = false;
static bool g_gravityOn      = true;

static std::mt19937 g_rng(12345);

// ─────────────────────────────────────────────────────────────────────────────
// Scene helpers
// ─────────────────────────────────────────────────────────────────────────────

static void buildDefaultScene() {
    // Ground + 4 wall planes
    g_world.addBody(RigidBody::makeStaticPlane({0,1,0},  0.f));
    g_world.addBody(RigidBody::makeStaticPlane({ 1,0,0}, -8.f));
    g_world.addBody(RigidBody::makeStaticPlane({-1,0,0}, -8.f));
    g_world.addBody(RigidBody::makeStaticPlane({ 0,0, 1},-8.f));
    g_world.addBody(RigidBody::makeStaticPlane({ 0,0,-1},-8.f));

    // Central platform
    RigidBody plat = RigidBody::makeBox({3.f,0.5f,3.f}, 1.f, {0.f,2.f,0.f});
    plat.bodyType             = BodyType::Static;
    plat.invMass              = 0.f;
    plat.invInertiaTensorLocal = Mat3{};
    plat.invInertiaTensorWorld = Mat3{};
    g_world.addBody(plat);

    // Initial batch of spheres
    std::uniform_real_distribution<float> posD(-4.f, 4.f);
    std::uniform_real_distribution<float> radD(0.3f, 0.7f);
    std::uniform_real_distribution<float> resD(0.2f, 0.8f);
    for (int i = 0; i < 15; ++i) {
        float r  = radD(g_rng);
        Vec3  p  = {posD(g_rng), 8.f + i*0.6f, posD(g_rng)};
        auto  b  = RigidBody::makeSphere(r, 1.f, p);
        b.restitution = resD(g_rng);
        b.friction    = 0.5f;
        g_world.addBody(b);
    }
    for (int i = 0; i < 6; ++i) {
        std::uniform_real_distribution<float> halfD(0.3f, 0.8f);
        Vec3 h = {halfD(g_rng), halfD(g_rng), halfD(g_rng)};
        Vec3 p = {posD(g_rng), 14.f + i, posD(g_rng)};
        auto b = RigidBody::makeBox(h, 1.f, p);
        b.restitution = 0.3f; b.friction = 0.6f;
        g_world.addBody(b);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Emscripten input callbacks
// ─────────────────────────────────────────────────────────────────────────────

static EM_BOOL onMouseDown(int /*etype*/, const EmscriptenMouseEvent* e, void*) {
    g_mouseDown  = (e->button == 0);
    g_lastMouseX = static_cast<float>(e->clientX);
    g_lastMouseY = static_cast<float>(e->clientY);
    return EM_TRUE;
}

static EM_BOOL onMouseUp(int /*etype*/, const EmscriptenMouseEvent* /*e*/, void*) {
    g_mouseDown = false;
    return EM_TRUE;
}

static EM_BOOL onMouseMove(int /*etype*/, const EmscriptenMouseEvent* e, void*) {
    if (!g_mouseDown) return EM_FALSE;
    float cx = static_cast<float>(e->clientX);
    float cy = static_cast<float>(e->clientY);
    g_camera.onMouseDrag(cx - g_lastMouseX, cy - g_lastMouseY);
    g_lastMouseX = cx;
    g_lastMouseY = cy;
    return EM_TRUE;
}

static EM_BOOL onWheel(int /*etype*/, const EmscriptenWheelEvent* e, void*) {
    g_camera.onScroll(static_cast<float>(-e->deltaY) * 0.05f);
    return EM_TRUE;
}

static EM_BOOL onResize(int /*etype*/, const EmscriptenUiEvent* /*e*/, void*) {
    // Let JS update the canvas size; C++ re-reads it next frame
    double w, h;
    emscripten_get_element_css_size("#canvas", &w, &h);
    g_width  = static_cast<int>(w);
    g_height = static_cast<int>(h);
    emscripten_set_canvas_element_size("#canvas", g_width, g_height);
    return EM_FALSE;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main loop
// ─────────────────────────────────────────────────────────────────────────────

static void mainLoop() {
    // ── Timing ──────────────────────────────────────────────────────────────
    double now = emscripten_get_now() * 0.001; // ms → s
    float  dt  = static_cast<float>(now - g_lastTime);
    // Clamp delta to avoid spiral-of-death after tab was hidden
    if (dt > 0.1f) dt = 0.1f;
    g_lastTime = now;

    // FPS
    g_fpsAccum  += dt;
    g_fpsFrames += 1;
    if (g_fpsAccum >= 0.5) {
        g_fps       = static_cast<float>(g_fpsFrames) / static_cast<float>(g_fpsAccum);
        g_fpsAccum  = 0.0;
        g_fpsFrames = 0;
    }

    // ── Canvas size sync ─────────────────────────────────────────────────────
    // JS may resize the canvas via CSS; keep GL viewport in sync.
    {
        int w = 0, h = 0;
        emscripten_get_canvas_element_size("#canvas", &w, &h);
        if (w != g_width || h != g_height) {
            g_width  = w;
            g_height = h;
        }
        glViewport(0, 0, g_width, g_height);
    }

    // ── Physics ───────────────────────────────────────────────────────────────
    g_camera.update(dt);
    if (!g_paused) {
        g_world.update(dt);
    }

    // ── Render ───────────────────────────────────────────────────────────────
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    float aspect = (g_height > 0)
                   ? static_cast<float>(g_width) / static_cast<float>(g_height)
                   : 1.f;

    g_renderer.setTime(static_cast<float>(now));
    g_renderer.beginFrame(g_camera, aspect);
    g_renderer.drawWorld(g_world, g_showContacts, g_showVelocities, g_showAABBs);
    g_renderer.endFrame();
}

// ─────────────────────────────────────────────────────────────────────────────
// JS-callable API (also used by Embind bindings below)
// ─────────────────────────────────────────────────────────────────────────────

/// Add a box at (x,y,z) with the given half-extents and mass.
/// Returns the body ID assigned by the world.
static int js_addBox(float x, float y, float z,
                     float hx, float hy, float hz,
                     float mass)
{
    if (mass <= 0.f) mass = 1.f;
    Vec3 pos  = {x, y, z};
    Vec3 half = {hx, hy, hz};
    auto b    = RigidBody::makeBox(half, mass, pos);
    b.restitution = 0.3f;
    b.friction    = 0.5f;
    return g_world.addBody(b);
}

/// Add a sphere at (x,y,z) with the given radius and mass.
static int js_addSphere(float x, float y, float z, float radius, float mass) {
    if (mass <= 0.f) mass = 1.f;
    Vec3 pos = {x, y, z};
    auto b   = RigidBody::makeSphere(radius, mass, pos);
    b.restitution = 0.4f;
    b.friction    = 0.4f;
    return g_world.addBody(b);
}

/// Spawn a random box above the scene centre.
static int js_spawnRandomBox() {
    std::uniform_real_distribution<float> posD(-5.f, 5.f);
    std::uniform_real_distribution<float> halfD(0.3f, 0.9f);
    Vec3 h  = {halfD(g_rng), halfD(g_rng), halfD(g_rng)};
    Vec3 p  = {posD(g_rng), 14.f + std::uniform_real_distribution<float>(0,3)(g_rng), posD(g_rng)};
    return js_addBox(p.x, p.y, p.z, h.x, h.y, h.z, 1.f);
}

/// Spawn a random sphere above the scene centre.
static int js_spawnRandomSphere() {
    std::uniform_real_distribution<float> posD(-5.f, 5.f);
    std::uniform_real_distribution<float> radD(0.25f, 0.7f);
    float r = radD(g_rng);
    Vec3  p = {posD(g_rng), 14.f + std::uniform_real_distribution<float>(0,3)(g_rng), posD(g_rng)};
    return js_addSphere(p.x, p.y, p.z, r, 1.f);
}

static void js_setGravity(float gx, float gy, float gz) {
    g_world.config().gravity = {gx, gy, gz};
    g_gravityOn = (gy < -0.1f);
}

/// Toggle gravity on/off (uses ±9.81 m/s²).
static void js_toggleGravity() {
    g_gravityOn = !g_gravityOn;
    g_world.config().gravity = g_gravityOn ? Vec3(0,-9.81f,0) : Vec3(0,0,0);
}

static void js_step(float dt) {
    if (dt <= 0.f) dt = 1.f / 60.f;
    g_world.update(dt);
}

static void js_reset() {
    g_world.reset();
    buildDefaultScene();
    g_camera.reset();
}

static void js_setPaused(bool p)  { g_paused = p; }
static bool js_isPaused()         { return g_paused; }
static void js_togglePaused()     { g_paused = !g_paused; }

static int  js_getBodyCount()     { return static_cast<int>(g_world.bodies().size()); }
static int  js_getContactCount()  { return static_cast<int>(g_world.lastContacts().size()); }
static float js_getFPS()          { return g_fps; }
static uint64_t js_getStepCount() { return g_world.stepCount(); }

static void js_setShowContacts  (bool v) { g_showContacts   = v; }
static void js_setShowVelocities(bool v) { g_showVelocities = v; }
static void js_setShowAABBs     (bool v) { g_showAABBs      = v; }

/// Returns an Emscripten val (JS array) of transform objects:
///   { id, px,py,pz, qx,qy,qz,qw, shapeType, radius, hx,hy,hz }
static emscripten::val js_getBodyTransforms() {
    auto arr = emscripten::val::array();
    for (const auto& b : g_world.bodies()) {
        if (b.shape.type == physics::ShapeType::Plane) continue;
        auto obj = emscripten::val::object();
        obj.set("id",        b.id);
        obj.set("px",        b.position.x);
        obj.set("py",        b.position.y);
        obj.set("pz",        b.position.z);
        obj.set("qx",        b.orientation.x);
        obj.set("qy",        b.orientation.y);
        obj.set("qz",        b.orientation.z);
        obj.set("qw",        b.orientation.w);
        obj.set("shapeType", static_cast<int>(b.shape.type));
        obj.set("radius",    b.shape.radius);
        obj.set("hx",        b.shape.halfExtents.x);
        obj.set("hy",        b.shape.halfExtents.y);
        obj.set("hz",        b.shape.halfExtents.z);
        arr.call<void>("push", obj);
    }
    return arr;
}

// ─────────────────────────────────────────────────────────────────────────────
// Embind bindings — accessed as Module.funcName(...) in JavaScript
// ─────────────────────────────────────────────────────────────────────────────

EMSCRIPTEN_BINDINGS(physics_engine_wasm) {
    emscripten::function("addBox",             &js_addBox);
    emscripten::function("addSphere",          &js_addSphere);
    emscripten::function("spawnRandomBox",     &js_spawnRandomBox);
    emscripten::function("spawnRandomSphere",  &js_spawnRandomSphere);
    emscripten::function("setGravity",         &js_setGravity);
    emscripten::function("toggleGravity",      &js_toggleGravity);
    emscripten::function("step",               &js_step);
    emscripten::function("reset",              &js_reset);
    emscripten::function("setPaused",          &js_setPaused);
    emscripten::function("isPaused",           &js_isPaused);
    emscripten::function("togglePaused",       &js_togglePaused);
    emscripten::function("getBodyCount",       &js_getBodyCount);
    emscripten::function("getContactCount",    &js_getContactCount);
    emscripten::function("getFPS",             &js_getFPS);
    emscripten::function("getStepCount",       &js_getStepCount);
    emscripten::function("setShowContacts",    &js_setShowContacts);
    emscripten::function("setShowVelocities",  &js_setShowVelocities);
    emscripten::function("setShowAABBs",       &js_setShowAABBs);
    emscripten::function("getBodyTransforms",  &js_getBodyTransforms);
}

// ─────────────────────────────────────────────────────────────────────────────
// Entry point
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::puts("[wasm] Physics engine initialising...");

    // ── WebGL context ────────────────────────────────────────────────────────
    EmscriptenWebGLContextAttributes attrs;
    emscripten_webgl_init_context_attributes(&attrs);
    attrs.majorVersion       = 2;  // WebGL 2
    attrs.minorVersion       = 0;
    attrs.alpha              = 0;
    attrs.depth              = 1;
    attrs.stencil            = 0;
    attrs.antialias          = 1;
    attrs.premultipliedAlpha = 0;
    attrs.powerPreference    = EM_WEBGL_POWER_PREFERENCE_HIGH_PERFORMANCE;

    EMSCRIPTEN_WEBGL_CONTEXT_HANDLE ctx =
        emscripten_webgl_create_context("#canvas", &attrs);
    if (ctx <= 0) {
        std::puts("[wasm] Failed to create WebGL 2 context");
        return 1;
    }
    emscripten_webgl_make_context_current(ctx);

    // ── Canvas size ──────────────────────────────────────────────────────────
    {
        double w = 0, h = 0;
        emscripten_get_element_css_size("#canvas", &w, &h);
        g_width  = w  > 0 ? static_cast<int>(w)  : 800;
        g_height = h  > 0 ? static_cast<int>(h) : 600;
        emscripten_set_canvas_element_size("#canvas", g_width, g_height);
    }

    // ── Renderer ─────────────────────────────────────────────────────────────
    if (!g_renderer.init()) {
        std::puts("[wasm] WebGLRenderer init failed");
        return 1;
    }
    glClearColor(0.08f, 0.06f, 0.16f, 1.f);

    // ── Physics world ────────────────────────────────────────────────────────
    g_world.config().gravity = {0, -9.81f, 0};
    buildDefaultScene();

    // ── Input callbacks ──────────────────────────────────────────────────────
    emscripten_set_mousedown_callback ("#canvas", nullptr, EM_TRUE, onMouseDown);
    emscripten_set_mouseup_callback   ("#canvas", nullptr, EM_TRUE, onMouseUp);
    emscripten_set_mousemove_callback ("#canvas", nullptr, EM_TRUE, onMouseMove);
    emscripten_set_wheel_callback     ("#canvas", nullptr, EM_TRUE, onWheel);
    emscripten_set_resize_callback    (EMSCRIPTEN_EVENT_TARGET_WINDOW,
                                       nullptr, EM_FALSE, onResize);

    g_lastTime = emscripten_get_now() * 0.001;

    // ── Start render loop ────────────────────────────────────────────────────
    // 0 fps = run as fast as possible (browser rAF-limited to display refresh).
    emscripten_set_main_loop(mainLoop, 0, 1 /* simulate infinite loop */);

    return 0;
}
