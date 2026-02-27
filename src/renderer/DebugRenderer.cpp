/// @file DebugRenderer.cpp
#include "renderer/DebugRenderer.h"
#include <glad/glad.h>
#include "math/Quat.h"
#include "physics/BVHTree.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace renderer {

using physics::Quat;
using physics::BodyType;
using physics::ShapeType;

// ── Embedded background gradient shaders ─────────────────────────────────────

static const char* const kBgVert =
    "#version 330 core\n"
    "layout(location = 0) in vec3 aPos;\n"
    "out float vY;\n"
    "void main() {\n"
    "    gl_Position = vec4(aPos.xy, 0.9999, 1.0);\n"
    "    vY = aPos.y;\n"
    "}\n";

static const char* const kBgFrag =
    "#version 330 core\n"
    "in float vY;\n"
    "out vec4 FragColor;\n"
    "uniform vec3 uTop;\n"
    "uniform vec3 uBot;\n"
    "void main() {\n"
    "    float t = vY * 0.5 + 0.5;\n"
    "    t = t * t;\n"
    "    FragColor = vec4(mix(uBot, uTop, t), 1.0);\n"
    "}\n";

// ── Colour helpers ─────────────────────────────────────────────────────────────

/// HSV (all components in [0,1]) → RGB.
static Vec3 hsvToRgb(float h, float s, float v) {
    const float c = v * s;
    const float x = c * (1.f - std::abs(std::fmod(h * 6.f, 2.f) - 1.f));
    const float m = v - c;
    float r = 0.f, g = 0.f, b = 0.f;
    const int sector = static_cast<int>(h * 6.f) % 6;
    switch (sector) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2:         g = c; b = x; break;
        case 3:         g = x; b = c; break;
        case 4: r = x;         b = c; break;
        default: r = c;         b = x; break;
    }
    return Vec3(r + m, g + m, b + m);
}

/// Per-body unique colour via golden-ratio hue cycling.
static Vec3 dynamicBodyColor(int bodyId) {
    const float h = std::fmod(static_cast<float>(std::abs(bodyId)) * 0.618033988f, 1.0f);
    return hsvToRgb(h, 0.72f, 0.95f);
}

static const Vec3 kColorStatic     { 0.90f, 0.28f, 0.23f }; // warm red
static const Vec3 kColorKinematic  { 0.20f, 0.85f, 0.90f }; // cyan
static const Vec3 kColorContact    { 1.00f, 0.90f, 0.10f }; // bright yellow
static const Vec3 kColorNormal     { 1.00f, 0.45f, 0.05f }; // orange
static const Vec3 kColorGrid       { 0.20f, 0.20f, 0.28f }; // dim blue-gray
static const Vec3 kColorAxisX      { 0.95f, 0.25f, 0.25f }; // red
static const Vec3 kColorAxisY      { 0.25f, 0.95f, 0.35f }; // green
static const Vec3 kColorAxisZ      { 0.25f, 0.45f, 0.95f }; // blue
static const Vec3 kColorAABB       { 0.45f, 0.45f, 0.95f }; // muted blue

static Vec3 bodyColor(const RigidBody& body) {
    switch (body.bodyType) {
        case BodyType::Static:    return kColorStatic;
        case BodyType::Kinematic: return kColorKinematic;
        default:                  return dynamicBodyColor(body.id);
    }
}

/// Speed-based velocity colour: green → yellow → red.
static Vec3 velocityColor(float speed) {
    const float t = std::min(speed / 8.f, 1.f);
    if (t < 0.5f) {
        const float s = t * 2.f;
        return Vec3(s, 1.f, 0.f);
    } else {
        const float s = (t - 0.5f) * 2.f;
        return Vec3(1.f, 1.f - s, 0.f);
    }
}

// ── Rotation helpers ──────────────────────────────────────────────────────────

static Quat rotateXToDir(Vec3 dir) {
    const float len = dir.length();
    if (len < 1e-5f) return Quat::identity();
    dir = dir * (1.f / len);
    const Vec3  xAxis{ 1.f, 0.f, 0.f };
    const Vec3  axis  = xAxis.cross(dir);
    const float dotXD = xAxis.dot(dir);
    if (axis.lengthSq() < 1e-8f) {
        if (dotXD > 0.f) return Quat::identity();
        return Quat::fromAxisAngle(Vec3{ 0.f, 1.f, 0.f }, static_cast<float>(M_PI));
    }
    return Quat::fromAxisAngle(axis.normalized(),
                               std::acos(std::clamp(dotXD, -1.f, 1.f)));
}

static Quat rotateYToDir(Vec3 dir) {
    const float len = dir.length();
    if (len < 1e-5f) return Quat::identity();
    dir = dir * (1.f / len);
    const Vec3  yAxis{ 0.f, 1.f, 0.f };
    const Vec3  axis  = yAxis.cross(dir);
    const float dotYD = yAxis.dot(dir);
    if (axis.lengthSq() < 1e-8f) {
        if (dotYD > 0.f) return Quat::identity();
        return Quat::fromAxisAngle(Vec3{ 1.f, 0.f, 0.f }, static_cast<float>(M_PI));
    }
    return Quat::fromAxisAngle(axis.normalized(),
                               std::acos(std::clamp(dotYD, -1.f, 1.f)));
}

// ── init / shutdown ───────────────────────────────────────────────────────────

bool DebugRenderer::init() {
    shader_   = Shader::makeDefault();
    bgShader_ = Shader(kBgVert, kBgFrag);

    if (!shader_.valid()) {
        std::cerr << "[DebugRenderer] Main shader compilation failed.\n";
        return false;
    }
    if (!bgShader_.valid()) {
        std::cerr << "[DebugRenderer] Background shader compilation failed.\n";
        return false;
    }

    sphereMesh_ = Mesh::makeWireframeSphere(2);
    boxMesh_    = Mesh::makeWireframeBox();
    lineMesh_   = Mesh::makeLine();
    pointMesh_  = Mesh::makePoint();
    planeMesh_  = Mesh::makePlaneGrid();
    bgMesh_     = Mesh::makeFullscreenQuad();

    return true;
}

void DebugRenderer::shutdown() {
    shader_     = Shader();
    bgShader_   = Shader();
    sphereMesh_ = Mesh();
    boxMesh_    = Mesh();
    lineMesh_   = Mesh();
    pointMesh_  = Mesh();
    planeMesh_  = Mesh();
    bgMesh_     = Mesh();
}

// ── Per-frame ─────────────────────────────────────────────────────────────────

void DebugRenderer::beginFrame(const Camera& cam, float aspect) {
    view_ = cam.viewMatrix();
    proj_ = cam.projMatrix(aspect);
    drawBackground();
}

void DebugRenderer::endFrame() {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_BLEND);
    glLineWidth(1.f);
    glPointSize(1.f);
}

// ── Background gradient ───────────────────────────────────────────────────────

void DebugRenderer::drawBackground() {
    if (!bgShader_.valid() || !bgMesh_.valid()) return;

    // Top colour slowly cycles through hue for a subtle living effect
    const float hue = std::fmod(time_ * 0.04f, 1.0f);
    const Vec3  top = hsvToRgb(hue, 0.35f, 0.18f);   // desaturated blue-purple
    const Vec3  bot { 0.03f, 0.03f, 0.05f };          // near-black

    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);

    bgShader_.use();
    bgShader_.setVec3("uTop", top);
    bgShader_.setVec3("uBot", bot);
    bgMesh_.draw();

    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
}

// ── Internal draw helper ──────────────────────────────────────────────────────

void DebugRenderer::drawMesh(const Mesh&  mesh,
                              const Mat4&  model,
                              const Vec3&  color,
                              float        alpha,
                              unsigned int polyMode)
{
    if (!shader_.valid() || !mesh.valid()) return;

    const Mat4 mvp = proj_ * view_ * model;
    shader_.use();
    shader_.setMat4 ("uMVP",   mvp);
    shader_.setVec3 ("uColor", color);
    shader_.setFloat("uAlpha", alpha);

    glPolygonMode(GL_FRONT_AND_BACK, polyMode);

    if (alpha < 0.99f) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else {
        glDisable(GL_BLEND);
    }

    mesh.draw();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

// ── World-origin axes ─────────────────────────────────────────────────────────

void DebugRenderer::drawWorldAxes() {
    const float len = 2.5f;
    glLineWidth(2.5f);

    // +X  (red)
    drawMesh(lineMesh_,
             Mat4::scale(Vec3(len, 1.f, 1.f)),
             kColorAxisX);

    // +Y  (green) — rotate X-aligned line to Y
    {
        const Quat rot = Quat::fromAxisAngle(Vec3{0.f, 0.f, 1.f},
                                             static_cast<float>(M_PI) * 0.5f);
        drawMesh(lineMesh_,
                 Mat4::fromMat3(rot.toMat3()) * Mat4::scale(Vec3(len, 1.f, 1.f)),
                 kColorAxisY);
    }

    // +Z  (blue) — rotate X-aligned line to Z
    {
        const Quat rot = Quat::fromAxisAngle(Vec3{0.f, 1.f, 0.f},
                                             -static_cast<float>(M_PI) * 0.5f);
        drawMesh(lineMesh_,
                 Mat4::fromMat3(rot.toMat3()) * Mat4::scale(Vec3(len, 1.f, 1.f)),
                 kColorAxisZ);
    }

    glLineWidth(1.f);
}

// ── drawBody ──────────────────────────────────────────────────────────────────

void DebugRenderer::drawBody(const RigidBody& body) {
    const Vec3 color = bodyColor(body);

    if (body.shape.type == ShapeType::Sphere) {
        const Mat4 model = Mat4::TRS(body.position, body.orientation,
                                     body.shape.radius);
        // Ghost fill + crisp wireframe
        glLineWidth(1.f);
        drawMesh(sphereMesh_, model, color, 0.15f, GL_LINE);
        glLineWidth(1.8f);
        drawMesh(sphereMesh_, model, color, 1.f,   GL_LINE);

    } else if (body.shape.type == ShapeType::Box) {
        const Vec3 he    = body.shape.halfExtents;
        const Mat4 model = Mat4::translation(body.position)
                         * Mat4::fromMat3(body.orientation.toMat3())
                         * Mat4::scale(he);
        glLineWidth(1.f);
        drawMesh(boxMesh_, model, color, 0.12f, GL_LINE);
        glLineWidth(2.0f);
        drawMesh(boxMesh_, model, color, 1.f,   GL_LINE);

    } else if (body.shape.type == ShapeType::Plane) {
        const Vec3  n      = body.shape.halfExtents;
        const float offset = body.shape.radius;
        const Vec3  center = n * offset;
        const Quat  rot    = rotateYToDir(n);
        const Mat4  model  = Mat4::translation(center)
                           * Mat4::fromMat3(rot.toMat3())
                           * Mat4::scale(50.f);

        // Dim grid
        glLineWidth(1.f);
        drawMesh(planeMesh_, model, kColorGrid, 0.65f, GL_LINE);

        // X-axis stripe
        {
            const Mat4 axModel = Mat4::translation(center)
                               * Mat4::fromMat3(rot.toMat3())
                               * Mat4::scale(Vec3(50.f, 1.f, 0.01f));
            glLineWidth(2.f);
            drawMesh(lineMesh_, axModel, kColorAxisX, 0.70f, GL_LINE);
        }
        // Z-axis stripe
        {
            const Quat rotZ = rotateYToDir(n)
                            * Quat::fromAxisAngle(Vec3{0.f, 1.f, 0.f},
                                                  static_cast<float>(M_PI) * 0.5f);
            const Mat4 azModel = Mat4::translation(center)
                               * Mat4::fromMat3(rotZ.toMat3())
                               * Mat4::scale(Vec3(50.f, 1.f, 0.01f));
            glLineWidth(2.f);
            drawMesh(lineMesh_, azModel, kColorAxisZ, 0.70f, GL_LINE);
        }
        glLineWidth(1.f);
    }
}

// ── drawContact ───────────────────────────────────────────────────────────────

void DebugRenderer::drawContact(const ContactManifold& contact) {
    const Mat4 ptModel = Mat4::translation(contact.contactPoint);

    // 4-layer glow effect
    glPointSize(22.f);
    drawMesh(pointMesh_, ptModel, kColorContact, 0.08f, GL_LINE);
    glPointSize(13.f);
    drawMesh(pointMesh_, ptModel, kColorContact, 0.28f, GL_LINE);
    glPointSize(7.f);
    drawMesh(pointMesh_, ptModel, kColorContact, 0.75f, GL_LINE);
    glPointSize(3.f);
    drawMesh(pointMesh_, ptModel, Vec3(1.f, 1.f, 1.f), 1.0f, GL_LINE);
    glPointSize(1.f);

    // Normal arrow
    const Vec3  dir       = contact.normal.normalized();
    const Quat  rot       = rotateXToDir(dir);
    const Mat4  lineModel = Mat4::translation(contact.contactPoint)
                          * Mat4::fromMat3(rot.toMat3())
                          * Mat4::scale(Vec3(0.4f, 0.01f, 0.01f));
    glLineWidth(2.f);
    drawMesh(lineMesh_, lineModel, kColorNormal, 0.9f, GL_LINE);
    glLineWidth(1.f);
}

// ── drawVelocity ──────────────────────────────────────────────────────────────

void DebugRenderer::drawVelocity(const RigidBody& body) {
    const float speed = body.linearVelocity.length();
    if (speed < 0.05f) return;

    const float drawLen = std::min(speed, 5.f);
    const Vec3  dir     = body.linearVelocity * (1.f / speed);
    const Vec3  color   = velocityColor(speed);
    const Quat  rot     = rotateXToDir(dir);
    const Mat4  model   = Mat4::translation(body.position)
                        * Mat4::fromMat3(rot.toMat3())
                        * Mat4::scale(Vec3(drawLen, 0.04f, 0.04f));
    glLineWidth(2.f);
    drawMesh(lineMesh_, model, color, 0.9f, GL_LINE);
    glLineWidth(1.f);
}

// ── drawAABB ──────────────────────────────────────────────────────────────────

void DebugRenderer::drawAABB(const AABB& aabb, const Vec3& color) {
    const Vec3 center  = (aabb.min + aabb.max) * 0.5f;
    const Vec3 halfExt = (aabb.max - aabb.min) * 0.5f;
    const Mat4 model   = Mat4::translation(center) * Mat4::scale(halfExt);
    glLineWidth(1.f);
    drawMesh(boxMesh_, model, color, 0.25f, GL_LINE);
}

// ── drawWorld ─────────────────────────────────────────────────────────────────

void DebugRenderer::drawWorld(const PhysicsWorld& world,
                               bool showContacts,
                               bool showVelocities,
                               bool showAABBs)
{
    drawWorldAxes();

    for (const RigidBody& body : world.bodies()) {
        drawBody(body);

        if (showVelocities && body.isDynamic()) {
            drawVelocity(body);
        }

        if (showAABBs && body.shape.type != ShapeType::Plane) {
            AABB aabb;
            if (body.shape.type == ShapeType::Sphere) {
                aabb = AABB::fromSphere(body.position, body.shape.radius);
            } else {
                aabb = AABB::fromBox(body.position,
                                     body.orientation,
                                     body.shape.halfExtents);
            }
            drawAABB(aabb, kColorAABB);
        }
    }

    if (showContacts) {
        for (const ContactManifold& c : world.lastContacts()) {
            drawContact(c);
        }
    }
}

} // namespace renderer
