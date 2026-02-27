/// @file main.cpp
/// @brief Phase 6 demo: OpenGL debug renderer driving the complete physics engine.
///
/// Controls
/// ────────
///   SPACE  — pause / resume
///   R      — reset scene
///   G      — toggle gravity
///   C      — toggle contact visualisation
///   V      — toggle velocity vectors
///   B      — toggle AABB overlay
///   ESC    — quit
///   Mouse drag  — orbit camera
///   Scroll      — zoom in / out

#include "core/Window.h"
#include "core/Timer.h"
#include "renderer/Camera.h"
#include "renderer/DebugRenderer.h"

#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"

#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <random>
#include <algorithm>

using namespace physics;
using namespace renderer;

/* ── Scene setup ─────────────────────────────────────────────────────── */

static void setupScene(PhysicsWorld& world)
{
    // Ground plane (normal = +Y, offset = 0)
    world.addBody(RigidBody::makeStaticPlane({0.f, 1.f, 0.f}, 0.f));

    // Four wall planes forming a bowl (-8 unit radius)
    world.addBody(RigidBody::makeStaticPlane({ 1.f, 0.f, 0.f}, -8.f)); // left
    world.addBody(RigidBody::makeStaticPlane({-1.f, 0.f, 0.f}, -8.f)); // right
    world.addBody(RigidBody::makeStaticPlane({ 0.f, 0.f, 1.f}, -8.f)); // front
    world.addBody(RigidBody::makeStaticPlane({ 0.f, 0.f,-1.f}, -8.f)); // back

    // Static platform box in the centre (mass=1 so makeBox's assert passes;
    // bodyType=Static makes the world treat it as immovable)
    RigidBody platform = RigidBody::makeBox({3.f, 0.5f, 3.f}, 1.f, {0.f, 2.f, 0.f});
    platform.bodyType              = BodyType::Static;
    platform.invMass               = 0.f; // infinite effective mass
    platform.invInertiaTensorLocal = physics::Mat3{};
    platform.invInertiaTensorWorld = physics::Mat3{};
    world.addBody(platform);

    // 20 spheres dropped from above with random XZ offsets
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posD(-4.f,  4.f);
    std::uniform_real_distribution<float> radD( 0.3f, 0.7f);
    std::uniform_real_distribution<float> resD( 0.2f, 0.8f);

    for (int i = 0; i < 20; ++i) {
        const float r   = radD(rng);
        const Vec3  pos = { posD(rng), 8.f + static_cast<float>(i) * 0.5f, posD(rng) };
        RigidBody   s   = RigidBody::makeSphere(r, 1.f, pos);
        s.restitution   = resD(rng);
        s.friction      = 0.5f;
        world.addBody(s);
    }

    // 8 boxes dropped in
    std::uniform_real_distribution<float> halfD(0.3f, 0.8f);
    for (int i = 0; i < 8; ++i) {
        const Vec3 half = { halfD(rng), halfD(rng), halfD(rng) };
        const Vec3 pos  = { posD(rng), 15.f + static_cast<float>(i) * 1.f, posD(rng) };
        RigidBody  b    = RigidBody::makeBox(half, 1.f, pos);
        b.restitution   = 0.3f;
        b.friction      = 0.6f;
        world.addBody(b);
    }
}

static void resetScene(PhysicsWorld& world) {
    world.reset();
    setupScene(world);
}

/* ── Entry point ─────────────────────────────────────────────────────── */

int main()
{
    std::cout <<
        "Physics Engine — Debug Renderer\n"
        "Controls:\n"
        "  SPACE  — pause/resume\n"
        "  R      — reset scene\n"
        "  G      — toggle gravity\n"
        "  C      — toggle contact visualisation\n"
        "  V      — toggle velocity vectors\n"
        "  B      — toggle AABB overlay\n"
        "  ESC    — quit\n"
        "  Mouse drag  — orbit camera\n"
        "  Scroll      — zoom in/out\n"
        "\n";

    // ── Create window + GL context ────────────────────────────────────────
    Window window(1280, 720, "Physics Engine");

    // ── Build the physics world ───────────────────────────────────────────
    PhysicsWorld world;
    world.config().gravity = { 0.f, -9.81f, 0.f };
    setupScene(world);

    // ── Renderer (must come after GL context) ─────────────────────────────
    DebugRenderer renderer;
    if (!renderer.init()) {
        std::cerr << "[main] DebugRenderer::init() failed.\n";
        return 1;
    }

    // ── Camera and timer ──────────────────────────────────────────────────
    Camera camera({ 0.f, 5.f, 0.f }, 25.f, 45.f, 25.f);
    Timer  timer;

    // ── Application state ─────────────────────────────────────────────────
    bool paused         = false;
    bool showContacts   = true;
    bool showVelocities = false;
    bool showAABBs      = false;
    bool gravityOn      = true;

    // ── Render loop ───────────────────────────────────────────────────────
    while (!window.shouldClose())
    {
        const float dt = timer.tick();

        // Input
        window.pollEvents();

        if (window.isKeyJustPressed(GLFW_KEY_ESCAPE)) { break; }
        if (window.isKeyJustPressed(GLFW_KEY_SPACE))  { paused = !paused; }
        if (window.isKeyJustPressed(GLFW_KEY_R))      { resetScene(world); }
        if (window.isKeyJustPressed(GLFW_KEY_C))      { showContacts   = !showContacts;   }
        if (window.isKeyJustPressed(GLFW_KEY_V))      { showVelocities = !showVelocities; }
        if (window.isKeyJustPressed(GLFW_KEY_B))      { showAABBs      = !showAABBs;      }
        if (window.isKeyJustPressed(GLFW_KEY_G)) {
            gravityOn = !gravityOn;
            world.config().gravity = gravityOn ? Vec3(0.f, -9.81f, 0.f) : Vec3(0.f, 0.f, 0.f);
        }

        // Camera orbit
        if (window.isMouseButtonDown(GLFW_MOUSE_BUTTON_LEFT)) {
            camera.onMouseDrag(window.mouseDX(), window.mouseDY());
        } else {
            window.mouseDX();   // consume accumulated deltas
            window.mouseDY();
        }
        camera.onScroll(window.scrollDY());
        camera.update(dt);

        // Physics step
        if (!paused) {
            world.update(dt);
        }

        // Render
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        renderer.setTime(timer.elapsed());
        renderer.beginFrame(camera, window.aspect());
        renderer.drawWorld(world, showContacts, showVelocities, showAABBs);
        renderer.endFrame();

        window.swapBuffers();

        // Window title with FPS / body / contact info
        const std::string title =
            "Physics Engine  |  FPS: " + std::to_string(static_cast<int>(timer.fps()))
            + "  |  Bodies: "   + std::to_string(world.bodies().size())
            + "  |  Contacts: " + std::to_string(world.lastContacts().size())
            + (paused ? "  |  PAUSED" : "")
            + (gravityOn ? "" : "  |  GRAVITY OFF");
        window.setTitle(title);
    }

    renderer.shutdown();
    return 0;
}
