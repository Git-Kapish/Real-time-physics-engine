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

/* ── Colour palette ──────────────────────────────────────────────────── */
static const Vec3 kColorDynamic    { 0.85f, 0.85f, 0.85f };
static const Vec3 kColorStatic     { 0.80f, 0.25f, 0.25f };
static const Vec3 kColorKinematic  { 0.25f, 0.55f, 0.85f };
static const Vec3 kColorContact    { 1.00f, 0.85f, 0.00f };
static const Vec3 kColorNormal     { 1.00f, 0.50f, 0.00f };
static const Vec3 kColorVelocity   { 0.30f, 0.90f, 0.40f };
static const Vec3 kColorAABB       { 0.40f, 0.40f, 0.90f };

static Vec3 bodyColor(BodyType bt) {
    switch (bt) {
        case BodyType::Static:    return kColorStatic;
        case BodyType::Kinematic: return kColorKinematic;
        default:                  return kColorDynamic;
    }
}

/* ── Rotation helpers ────────────────────────────────────────────────── */

/// Build a quaternion that rotates (1,0,0) onto `dir` (does not need to be
/// unit length; returns identity if dir is near zero).
static Quat rotateXToDir(Vec3 dir) {
    const float len = dir.length();
    if (len < 1e-5f) return Quat::identity();
    dir = dir * (1.f / len);

    const Vec3  xAxis{ 1.f, 0.f, 0.f };
    const Vec3  axis  = xAxis.cross(dir);
    const float dotXD = xAxis.dot(dir);

    if (axis.lengthSq() < 1e-8f) {
        // Parallel or anti-parallel
        if (dotXD > 0.f) return Quat::identity();
        return Quat::fromAxisAngle(Vec3{ 0.f, 1.f, 0.f },
                                   static_cast<float>(M_PI));
    }
    const float angle = std::acos(std::clamp(dotXD, -1.f, 1.f));
    return Quat::fromAxisAngle(axis.normalized(), angle);
}

/// Build a quaternion that rotates (0,1,0) onto `dir`.
static Quat rotateYToDir(Vec3 dir) {
    const float len = dir.length();
    if (len < 1e-5f) return Quat::identity();
    dir = dir * (1.f / len);

    const Vec3  yAxis{ 0.f, 1.f, 0.f };
    const Vec3  axis  = yAxis.cross(dir);
    const float dotYD = yAxis.dot(dir);

    if (axis.lengthSq() < 1e-8f) {
        if (dotYD > 0.f) return Quat::identity();
        return Quat::fromAxisAngle(Vec3{ 1.f, 0.f, 0.f },
                                   static_cast<float>(M_PI));
    }
    const float angle = std::acos(std::clamp(dotYD, -1.f, 1.f));
    return Quat::fromAxisAngle(axis.normalized(), angle);
}

/* ── init / shutdown ────────────────────────────────────────────────── */

bool DebugRenderer::init() {
    // Build shader from embedded source strings — guaranteed to work regardless
    // of working directory.  If you want to iterate on shader source during
    // development, swap in the file-path constructor below instead.
    shader_ = Shader::makeDefault();
    if (!shader_.valid()) {
        std::cerr << "[DebugRenderer] Shader compilation failed.\n";
        return false;
    }

    sphereMesh_ = Mesh::makeWireframeSphere(2);
    boxMesh_    = Mesh::makeWireframeBox();
    lineMesh_   = Mesh::makeLine();
    pointMesh_  = Mesh::makePoint();
    planeMesh_  = Mesh::makePlaneGrid();

    return true;
}

void DebugRenderer::shutdown() {
    shader_     = Shader();
    sphereMesh_ = Mesh();
    boxMesh_    = Mesh();
    lineMesh_   = Mesh();
    pointMesh_  = Mesh();
    planeMesh_  = Mesh();
}

/* ── Per-frame ───────────────────────────────────────────────────────── */

void DebugRenderer::beginFrame(const Camera& cam, float aspect) {
    view_ = cam.viewMatrix();
    proj_ = cam.projMatrix(aspect);
}

void DebugRenderer::endFrame() {
    // No persistent state to reset beyond ensuring clean polygon mode
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_BLEND);
}

/* ── Internal draw helper ────────────────────────────────────────────── */

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

/* ── drawBody ────────────────────────────────────────────────────────── */

void DebugRenderer::drawBody(const RigidBody& body) {
    if (body.shape.type == ShapeType::Sphere) {
        const Vec3 color = bodyColor(body.bodyType);
        // TRS with uniform scale = radius; sphere mesh has unit radius
        const Mat4 model = Mat4::TRS(body.position, body.orientation,
                                     body.shape.radius);
        drawMesh(sphereMesh_, model, color, 1.f, GL_LINE);

    } else if (body.shape.type == ShapeType::Box) {
        const Vec3 color = bodyColor(body.bodyType);
        // Non-uniform scale: box mesh has corners at ±1, scale to halfExtents
        const Vec3 he = body.shape.halfExtents;
        const Mat4 model = Mat4::translation(body.position)
                         * Mat4::fromMat3(body.orientation.toMat3())
                         * Mat4::scale(he);
        drawMesh(boxMesh_, model, color, 1.f, GL_LINE);

    } else if (body.shape.type == ShapeType::Plane) {
        // Plane normal is stored in halfExtents, offset in radius
        const Vec3  n      = body.shape.halfExtents; // normalised plane normal
        const float offset = body.shape.radius;
        const Vec3  center = n * offset;

        // Rotate the XZ plane grid so its Y-up aligns with the plane normal
        const Quat  rot   = rotateYToDir(n);
        const Mat4  model = Mat4::translation(center)
                          * Mat4::fromMat3(rot.toMat3())
                          * Mat4::scale(50.f);

        drawMesh(planeMesh_, model, kColorStatic, 1.f, GL_LINE);
    }
}

/* ── drawContact ─────────────────────────────────────────────────────── */

void DebugRenderer::drawContact(const ContactManifold& contact) {
    // Yellow point at the contact location
    glPointSize(6.f);
    const Mat4 ptModel = Mat4::translation(contact.contactPoint);
    drawMesh(pointMesh_, ptModel, kColorContact, 1.f, GL_LINE);
    glPointSize(1.f);

    // Orange line from contact point along the contact normal (length 0.3)
    const Vec3  dir      = contact.normal.normalized();
    const float lineLen  = 0.3f;
    const Quat  rot      = rotateXToDir(dir);
    const Mat4  lineModel = Mat4::translation(contact.contactPoint)
                           * Mat4::fromMat3(rot.toMat3())
                           * Mat4::scale(Vec3(lineLen, 0.01f, 0.01f));
    drawMesh(lineMesh_, lineModel, kColorNormal, 1.f, GL_LINE);
}

/* ── drawVelocity ────────────────────────────────────────────────────── */

void DebugRenderer::drawVelocity(const RigidBody& body) {
    const float speed = body.linearVelocity.length();
    if (speed < 0.001f) return;

    const float drawLen = std::min(speed, 5.f);
    const Vec3  dir     = body.linearVelocity * (1.f / speed);
    const Quat  rot     = rotateXToDir(dir);
    const Mat4  model   = Mat4::translation(body.position)
                        * Mat4::fromMat3(rot.toMat3())
                        * Mat4::scale(Vec3(drawLen, 0.05f, 0.05f));
    drawMesh(lineMesh_, model, kColorVelocity, 1.f, GL_LINE);
}

/* ── drawAABB ────────────────────────────────────────────────────────── */

void DebugRenderer::drawAABB(const AABB& aabb, const Vec3& color) {
    const Vec3  center  = (aabb.min + aabb.max) * 0.5f;
    const Vec3  halfExt = (aabb.max - aabb.min) * 0.5f;
    const Mat4  model   = Mat4::translation(center) * Mat4::scale(halfExt);
    drawMesh(boxMesh_, model, color, 0.3f, GL_LINE);
}

/* ── drawWorld ───────────────────────────────────────────────────────── */

void DebugRenderer::drawWorld(const PhysicsWorld& world,
                               bool showContacts,
                               bool showVelocities,
                               bool showAABBs)
{
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
