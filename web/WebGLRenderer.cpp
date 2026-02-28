/// @file WebGLRenderer.cpp
/// @brief WebGL 2.0 (GLSL ES 3.0) implementation of the wireframe renderer.
///
/// Key differences from the native DebugRenderer
/// ──────────────────────────────────────────────
/// • glPolygonMode is absent in WebGL 2 → all wireframes use GL_LINES geometry.
/// • Shaders use "#version 300 es" with explicit float precision.
/// • No glad loader: Emscripten provides GL symbols via -sUSE_WEBGL2=1.
/// • Point size is set via gl_PointSize in the vertex shader (no glPointSize).

#include "WebGLRenderer.h"
#include "math/Quat.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ─────────────────────────────────────────────────────────────────────────────
// GLSL ES 3.0 shaders (embedded — no file I/O)
// ─────────────────────────────────────────────────────────────────────────────

static const char* kVertSrc = R"GLSL(#version 300 es
precision mediump float;
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
uniform vec3 uColor;
uniform float uAlpha;
uniform float uPointSize;
out vec4 vColorAlpha;
void main() {
    gl_Position  = uMVP * vec4(aPos, 1.0);
    gl_PointSize = uPointSize;
    vColorAlpha  = vec4(uColor, uAlpha);
}
)GLSL";

static const char* kFragSrc = R"GLSL(#version 300 es
precision mediump float;
in  vec4 vColorAlpha;
out vec4 fragColor;
void main() {
    fragColor = vColorAlpha;
}
)GLSL";

// Full-screen gradient (deep-space blue/purple, matching native renderer)
static const char* kBgVertSrc = R"GLSL(#version 300 es
precision mediump float;
layout(location = 0) in vec2 aPos;
out vec2 vUV;
void main() {
    vUV         = aPos * 0.5 + 0.5;   // [0,1]
    gl_Position = vec4(aPos, 0.9999, 1.0);
}
)GLSL";

static const char* kBgFragSrc = R"GLSL(#version 300 es
precision mediump float;
in  vec2 vUV;
out vec4 fragColor;
uniform float uTime;
void main() {
    // Top: deep space blue, bottom: dark purple — gentle animated shimmer
    vec3 top    = vec3(0.04, 0.06, 0.18);
    vec3 bottom = vec3(0.10, 0.04, 0.14);
    float wave  = 0.012 * sin(uTime * 0.4 + vUV.x * 3.1416);
    fragColor   = vec4(mix(bottom, top, vUV.y + wave), 1.0);
}
)GLSL";

namespace web {

using physics::Vec3;
using physics::Mat4;
using physics::Quat;
using physics::RigidBody;
using physics::BodyType;
using physics::ShapeType;
using physics::ContactManifold;
using physics::PhysicsWorld;
using physics::AABB;

// ─────────────────────────────────────────────────────────────────────────────
// Math helpers — thin wrappers around the engine's own Mat4 factories
// ─────────────────────────────────────────────────────────────────────────────

static Mat4 matTranslate(Vec3 t) { return Mat4::translation(t); }
static Mat4 matScale    (Vec3 s) { return Mat4::scale(s); }

static Mat4 quatToMat4(const Quat& q) { return Mat4::fromMat3(q.toMat3()); }

static Mat4 bodyModelMatrix(const RigidBody& b, Vec3 sc = {1,1,1}) {
    return Mat4::translation(b.position) * quatToMat4(b.orientation) * Mat4::scale(sc);
}

/// Golden-ratio hue cycling — same scheme as native DebugRenderer.
static Vec3 bodyColor(int idx, BodyType type) {
    if (type == BodyType::Static)    return {0.85f, 0.25f, 0.20f}; // warm red
    if (type == BodyType::Kinematic) return {0.10f, 0.85f, 0.85f}; // cyan
    // Dynamic: golden-ratio hue
    const float hue = std::fmod(static_cast<float>(idx) * 0.6180339887f, 1.0f);
    // HSV → RGB (S=0.75, V=0.85)
    float h6 = hue * 6.0f;
    int   hi  = static_cast<int>(h6);
    float f   = h6 - static_cast<float>(hi);
    float p   = 0.85f * (1.f - 0.75f);
    float q   = 0.85f * (1.f - 0.75f*f);
    float tv  = 0.85f * (1.f - 0.75f*(1.f-f));
    switch (hi % 6) {
        case 0: return {0.85f,   tv,    p};
        case 1: return {q,      0.85f,  p};
        case 2: return {p,      0.85f,  tv};
        case 3: return {p,      q,      0.85f};
        case 4: return {tv,     p,      0.85f};
        default:return {0.85f,  p,      q};
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// WMesh
// ─────────────────────────────────────────────────────────────────────────────

WMesh::~WMesh() {
    if (ebo) glDeleteBuffers(1, &ebo);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (vao) glDeleteVertexArrays(1, &vao);
}

WMesh::WMesh(WMesh&& o) noexcept
    : vao(o.vao), vbo(o.vbo), ebo(o.ebo), count(o.count), drawMode(o.drawMode)
{
    o.vao = o.vbo = o.ebo = 0;
    o.count = 0;
}

WMesh& WMesh::operator=(WMesh&& o) noexcept {
    if (this != &o) {
        if (ebo) glDeleteBuffers(1, &ebo);
        if (vbo) glDeleteBuffers(1, &vbo);
        if (vao) glDeleteVertexArrays(1, &vao);
        vao = o.vao; vbo = o.vbo; ebo = o.ebo;
        count = o.count; drawMode = o.drawMode;
        o.vao = o.vbo = o.ebo = 0; o.count = 0;
    }
    return *this;
}

void WMesh::upload(const std::vector<float>&        positions,
                   const std::vector<unsigned int>& indices,
                   GLenum mode)
{
    if (vao) {
        glDeleteBuffers(1, &ebo);
        glDeleteBuffers(1, &vbo);
        glDeleteVertexArrays(1, &vao);
        vao = vbo = ebo = 0;
    }
    drawMode = mode;
    count    = static_cast<int>(indices.size());
    if (count == 0) return;

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(positions.size() * sizeof(float)),
                 positions.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                 indices.data(), GL_STATIC_DRAW);

    // layout(location = 0) in vec3 aPos
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    glBindVertexArray(0);
}

void WMesh::draw() const {
    if (!vao) return;
    glBindVertexArray(vao);
    glDrawElements(drawMode, count, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

// ─────────────────────────────────────────────────────────────────────────────
// WShader
// ─────────────────────────────────────────────────────────────────────────────

WShader::~WShader() {
    if (program) glDeleteProgram(program);
}

void WShader::use() const {
    if (program) glUseProgram(program);
}

WShader::WShader(WShader&& o) noexcept : program(o.program) { o.program = 0; }

WShader& WShader::operator=(WShader&& o) noexcept {
    if (this != &o) {
        if (program) glDeleteProgram(program);
        program = o.program;
        o.program = 0;
    }
    return *this;
}

bool WShader::build(const char* vert, const char* frag) {
    auto compile = [](GLenum type, const char* src) -> GLuint {
        GLuint s = glCreateShader(type);
        glShaderSource(s, 1, &src, nullptr);
        glCompileShader(s);
        GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512]; glGetShaderInfoLog(s, 512, nullptr, log);
            std::printf("[WShader] compile error: %s\n", log);
        }
        return s;
    };
    GLuint v = compile(GL_VERTEX_SHADER,   vert);
    GLuint f = compile(GL_FRAGMENT_SHADER, frag);
    program = glCreateProgram();
    glAttachShader(program, v);
    glAttachShader(program, f);
    glLinkProgram(program);
    GLint ok = 0; glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512]; glGetProgramInfoLog(program, 512, nullptr, log);
        std::printf("[WShader] link error: %s\n", log);
        glDeleteProgram(program); program = 0;
    }
    glDeleteShader(v);
    glDeleteShader(f);
    return program != 0;
}

void WShader::setMat4(const char* name, const Mat4& m) const {
    GLint loc = glGetUniformLocation(program, name);
    // Mat4::data is already column-major (OpenGL convention) — pass directly.
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, m.data);
}

void WShader::setVec3(const char* name, float x, float y, float z) const {
    GLint loc = glGetUniformLocation(program, name);
    if (loc >= 0) glUniform3f(loc, x, y, z);
}

void WShader::setFloat(const char* name, float v) const {
    GLint loc = glGetUniformLocation(program, name);
    if (loc >= 0) glUniform1f(loc, v);
}

// ─────────────────────────────────────────────────────────────────────────────
// WCamera
// ─────────────────────────────────────────────────────────────────────────────

WCamera::WCamera(Vec3 target, float dist, float yaw, float pitch)
    : target_(target), distance_(dist), yaw_(yaw), pitch_(pitch),
      smoothYaw_(yaw), smoothPitch_(pitch), smoothDist_(dist)
{}

static Vec3 sphericalToCart(float yawDeg, float pitchDeg, float r) {
    const float y = yawDeg   * static_cast<float>(M_PI) / 180.f;
    const float p = pitchDeg * static_cast<float>(M_PI) / 180.f;
    return { r * std::cos(p) * std::sin(y),
             r * std::sin(p),
             r * std::cos(p) * std::cos(y) };
}

Vec3 WCamera::position() const {
    return target_ + sphericalToCart(smoothYaw_, smoothPitch_, smoothDist_);
}

void WCamera::onMouseDrag(float dx, float dy) {
    yaw_   += dx * sensitiv_;
    pitch_ -= dy * sensitiv_;
    pitch_  = std::max(-89.f, std::min(89.f, pitch_));
}

void WCamera::onScroll(float delta) {
    distance_ -= delta * zoomSpd_;
    distance_  = std::max(1.5f, std::min(200.f, distance_));
}

void WCamera::update(float dt) {
    float k = 1.f - std::exp(-smoothK_ * dt);
    smoothYaw_   += (yaw_      - smoothYaw_)   * k;
    smoothPitch_ += (pitch_    - smoothPitch_) * k;
    smoothDist_  += (distance_ - smoothDist_)  * k;
}

void WCamera::reset() {
    yaw_ = 45.f; pitch_ = 30.f; distance_ = 20.f;
    smoothYaw_ = yaw_; smoothPitch_ = pitch_; smoothDist_ = distance_;
    target_ = {0,0,0};
}

// --- view/proj --- delegate to Mat4 static factories

Mat4 WCamera::viewMatrix() const {
    return Mat4::lookAt(position(), target_, {0,1,0});
}

Mat4 WCamera::projMatrix(float aspect) const {
    // Mat4::perspective expects fovY in radians
    const float fovRad = fovY_ * static_cast<float>(M_PI) / 180.f;
    return Mat4::perspective(fovRad, aspect, nearZ_, farZ_);
}

// ─────────────────────────────────────────────────────────────────────────────
// Mesh factories
// ─────────────────────────────────────────────────────────────────────────────

WMesh WebGLRenderer::makeWireframeSphere(int segs) {
    // Two great circles per axis pair using GL_LINES
    std::vector<float>        pos;
    std::vector<unsigned int> idx;
    auto addRing = [&](int axis1, int axis2) {
        unsigned int base = static_cast<unsigned int>(pos.size() / 3);
        for (int i = 0; i < segs; ++i) {
            float a = 2.f * static_cast<float>(M_PI) * i / segs;
            float v[3] = {0,0,0};
            v[axis1] = std::cos(a);
            v[axis2] = std::sin(a);
            pos.push_back(v[0]); pos.push_back(v[1]); pos.push_back(v[2]);
            idx.push_back(base + i);
            idx.push_back(base + (i+1) % segs);
        }
    };
    addRing(0, 1); // XY circle
    addRing(1, 2); // YZ circle
    addRing(0, 2); // XZ circle
    WMesh m; m.upload(pos, idx, GL_LINES); return m;
}

WMesh WebGLRenderer::makeWireframeBox() {
    // 8 corners ±1, 12 edges
    static const float c[8][3] = {
        {-1,-1,-1},{+1,-1,-1},{+1,+1,-1},{-1,+1,-1},
        {-1,-1,+1},{+1,-1,+1},{+1,+1,+1},{-1,+1,+1}
    };
    static const unsigned int e[24] = {
        0,1, 1,2, 2,3, 3,0,  // back
        4,5, 5,6, 6,7, 7,4,  // front
        0,4, 1,5, 2,6, 3,7   // sides
    };
    std::vector<float>        pos(c[0], c[0]+24);
    std::vector<unsigned int> idx(e, e+24);
    WMesh m; m.upload(pos, idx, GL_LINES); return m;
}

WMesh WebGLRenderer::makePoint() {
    std::vector<float>        pos = {0,0,0};
    std::vector<unsigned int> idx = {0};
    WMesh m; m.upload(pos, idx, GL_POINTS); return m;
}

WMesh WebGLRenderer::makeLine() {
    std::vector<float>        pos = {0,0,0, 1,0,0};
    std::vector<unsigned int> idx = {0,1};
    WMesh m; m.upload(pos, idx, GL_LINES); return m;
}

WMesh WebGLRenderer::makePlaneGrid() {
    // 21×21 grid in XZ plane, [-10,10] = 1-unit spacing
    std::vector<float>        pos;
    std::vector<unsigned int> idx;
    unsigned int id = 0;
    for (int i = -10; i <= 10; ++i) {
        float fi = static_cast<float>(i);
        // Z lines
        pos.push_back(fi); pos.push_back(0); pos.push_back(-10);
        pos.push_back(fi); pos.push_back(0); pos.push_back( 10);
        idx.push_back(id); idx.push_back(id+1); id+=2;
        // X lines
        pos.push_back(-10); pos.push_back(0); pos.push_back(fi);
        pos.push_back( 10); pos.push_back(0); pos.push_back(fi);
        idx.push_back(id); idx.push_back(id+1); id+=2;
    }
    WMesh m; m.upload(pos, idx, GL_LINES); return m;
}

WMesh WebGLRenderer::makeFullscreenQuad() {
    std::vector<float>        pos = {-1,-1,  1,-1,  1,1,  -1,1};
    std::vector<unsigned int> idx = {0,1,2, 0,2,3};
    // HACK: quad uses 2-D positions; the VAO is the same layout
    // Upload as 3-component by padding Z=0 so the stride matches.
    std::vector<float> pos3 = {-1,-1,0, 1,-1,0, 1,1,0, -1,1,0};
    WMesh m; m.upload(pos3, idx, GL_TRIANGLES); return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// WebGLRenderer public API
// ─────────────────────────────────────────────────────────────────────────────

bool WebGLRenderer::init() {
    if (!shader_.build(kVertSrc, kFragSrc)) {
        std::printf("[WebGLRenderer] main shader failed\n");
        return false;
    }
    if (!bgShader_.build(kBgVertSrc, kBgFragSrc)) {
        std::printf("[WebGLRenderer] bg shader failed\n");
        return false;
    }

    sphereMesh_ = makeWireframeSphere();
    boxMesh_    = makeWireframeBox();
    lineMesh_   = makeLine();
    pointMesh_  = makePoint();
    planeMesh_  = makePlaneGrid();
    bgMesh_     = makeFullscreenQuad();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);

    return true;
}

void WebGLRenderer::shutdown() {
    // WMesh/WShader destructors clean up GL objects automatically.
}

void WebGLRenderer::beginFrame(const WCamera& cam, float aspect) {
    view_ = cam.viewMatrix();
    proj_ = cam.projMatrix(aspect);

    drawBackground();

    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glLineWidth(1.5f); // wider lines where supported

    drawWorldAxes();
    drawGroundGrid();
}

void WebGLRenderer::endFrame() {
    glDisable(GL_BLEND);
    glEnable(GL_BLEND); // keep blend on for next frame
}

// ── Private draw helpers ─────────────────────────────────────────────────────

void WebGLRenderer::drawMesh(const WMesh& mesh,
                              const Mat4&  model,
                              const Vec3&  color,
                              float        alpha)
{
    Mat4 mvp = proj_ * view_ * model;
    shader_.use();
    shader_.setMat4 ("uMVP",       mvp);
    shader_.setVec3 ("uColor",     color.x, color.y, color.z);
    shader_.setFloat("uAlpha",     alpha);
    shader_.setFloat("uPointSize", 6.f);
    mesh.draw();
}

void WebGLRenderer::drawBackground() {
    glDisable(GL_DEPTH_TEST);
    bgShader_.use();
    bgShader_.setFloat("uTime", time_);
    bgMesh_.draw();
    glEnable(GL_DEPTH_TEST);
}

void WebGLRenderer::drawWorldAxes() {
    // X axis (red): line from origin along +X, scaled to length 4
    drawMesh(lineMesh_, Mat4::translation({0,0,0}) * Mat4::scale({4,1,1}), {0.9f,0.2f,0.2f});
    // Y axis (green): rotate the +X unit line 90° around Z so it points up, scale length 4
    {
        // column-major, data[col*4+row]: rotate +X→+Y
        Mat4 rot = Mat4::identity();
        rot(0,0)=0; rot(1,0)=1;   // col0 (was X) now Y
        rot(0,1)=-1; rot(1,1)=0;  // col1 (was Y) now -X
        drawMesh(lineMesh_, Mat4::scale(Vec3{4,4,4}) * rot, {0.2f,0.9f,0.2f});
    }
    // Z axis (blue): rotate +X→+Z (90° around Y)
    {
        Mat4 rot = Mat4::identity();
        rot(0,0)=0; rot(2,0)=1;
        rot(0,2)=-1; rot(2,2)=0;
        drawMesh(lineMesh_, Mat4::scale(Vec3{4,4,4}) * rot, {0.2f,0.4f,0.9f});
    }
}

void WebGLRenderer::drawGroundGrid() {
    drawMesh(planeMesh_, Mat4::identity(), {0.3f,0.3f,0.35f}, 0.5f);
}

// ── Public draw calls ────────────────────────────────────────────────────────

void WebGLRenderer::drawBody(const RigidBody& body, int bodyIndex) {
    if (body.shape.type == physics::ShapeType::Plane) return; // infinite planes skipped

    Vec3 col = bodyColor(bodyIndex, body.bodyType);

    if (body.shape.type == physics::ShapeType::Sphere) {
        float r = body.shape.radius;
        Mat4  model = matTranslate(body.position) *
                      quatToMat4(body.orientation) *
                      matScale({r, r, r});
        // Ghost fill: opaque-ish transparent sphere face (approximate via point-set)
        // WebGL has no polygon-mode fill → draw hollow wireframe + faint inner tint
        drawMesh(sphereMesh_, model, col, 0.85f);
        // Bright highlight ring at equator scaled up slightly
        drawMesh(sphereMesh_, matTranslate(body.position) * matScale({r*1.02f,r*1.02f,r*1.02f}),
                 col * Vec3(1.4f,1.4f,1.4f), 0.25f);
    } else if (body.shape.type == physics::ShapeType::Box) {
        Vec3  h     = body.shape.halfExtents;
        Mat4  model = bodyModelMatrix(body, h);
        drawMesh(boxMesh_, model, col, 0.85f);
        drawMesh(boxMesh_, bodyModelMatrix(body, h * Vec3(1.03f,1.03f,1.03f)),
                 col * Vec3(1.3f,1.3f,1.3f), 0.20f);
    }
}

void WebGLRenderer::drawContact(const ContactManifold& c) {
    Vec3 p = c.contactPoint;          // single contact point per manifold
    // Three overlapping halos (yellow → white) — same as native renderer
    for (int layer = 0; layer < 3; ++layer) {
        float s = 0.04f + 0.04f * static_cast<float>(layer);
        Mat4  m = Mat4::translation(p) * Mat4::scale(Vec3{s,s,s});
        Vec3  col = (layer < 2) ? Vec3{1.f,0.9f,0.2f} : Vec3{1.f,1.f,1.f};
        float a   = 1.0f - 0.25f * static_cast<float>(layer);
        shader_.use();
        shader_.setMat4 ("uMVP",       proj_ * view_ * m);
        shader_.setVec3 ("uColor",     col.x, col.y, col.z);
        shader_.setFloat("uAlpha",     a);
        shader_.setFloat("uPointSize", 8.f + 4.f * static_cast<float>(layer));
        pointMesh_.draw();
    }
}

void WebGLRenderer::drawVelocity(const RigidBody& body) {
    if (!body.isDynamic()) return;
    float speed = std::sqrt(body.linearVelocity.x*body.linearVelocity.x +
                            body.linearVelocity.y*body.linearVelocity.y +
                            body.linearVelocity.z*body.linearVelocity.z);
    if (speed < 0.01f) return;
    // Color: green (slow) → yellow → red (fast)
    float t = std::min(speed / 10.f, 1.f);
    Vec3 col = (t < 0.5f)
               ? Vec3{2.f*t, 1.f, 0.f}
               : Vec3{1.f, 2.f-2.f*t, 0.f};

    Vec3 dir  = body.linearVelocity * (1.f / (speed + 1e-6f));
    float len = std::min(speed * 0.25f, 3.f);

    // Build a rotation from +X to dir
    Vec3  xAxis = {1.f,0.f,0.f};
    Vec3  axis  = { xAxis.y*dir.z - xAxis.z*dir.y,
                    xAxis.z*dir.x - xAxis.x*dir.z,
                    xAxis.x*dir.y - xAxis.y*dir.x };
    float crossLen = std::sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z);
    float dot       = xAxis.x*dir.x + xAxis.y*dir.y + xAxis.z*dir.z;

    Mat4 rot = Mat4::identity();
    if (crossLen > 1e-6f) {
        axis = axis * (1.f/crossLen);
        float ang  = std::atan2(crossLen, dot);
        float c    = std::cos(ang), s = std::sin(ang), ic = 1.f-c;
        // Rodrigues rotation matrix
        // Rodrigues formula — fill column-major via operator()(row, col)
        rot(0,0) = c + axis.x*axis.x*ic;
        rot(1,0) = axis.x*axis.y*ic - axis.z*s;
        rot(2,0) = axis.x*axis.z*ic + axis.y*s;
        rot(0,1) = axis.y*axis.x*ic + axis.z*s;
        rot(1,1) = c + axis.y*axis.y*ic;
        rot(2,1) = axis.y*axis.z*ic - axis.x*s;
        rot(0,2) = axis.z*axis.x*ic - axis.y*s;
        rot(1,2) = axis.z*axis.y*ic + axis.x*s;
        rot(2,2) = c + axis.z*axis.z*ic;
    }

    Mat4 model = matTranslate(body.position) * rot * matScale({len,1,1});
    drawMesh(lineMesh_, model, col, 0.9f);
}

void WebGLRenderer::drawAABB(const AABB& aabb, const Vec3& color) {
    Vec3 center = (aabb.min + aabb.max) * 0.5f;
    Vec3 half   = (aabb.max - aabb.min) * 0.5f;
    Mat4 model  = Mat4::translation(center) * Mat4::scale(half);
    drawMesh(boxMesh_, model, color, 0.35f);
}

void WebGLRenderer::drawWorld(const PhysicsWorld& world,
                               bool showContacts,
                               bool showVelocities,
                               bool showAABBs)
{
    int dynIdx = 0;
    for (const auto& body : world.bodies()) {
        int colorIdx = (body.isDynamic()) ? dynIdx++ : -1;
        drawBody(body, colorIdx);

        if (showVelocities) drawVelocity(body);

        if (showAABBs && body.shape.type != physics::ShapeType::Plane) {
            AABB aabb;
            if (body.shape.type == physics::ShapeType::Sphere)
                aabb = AABB::fromSphere(body.position, body.shape.radius);
            else
                aabb = AABB::fromBox(body.position, body.orientation, body.shape.halfExtents);
            drawAABB(aabb, {0.45f, 0.45f, 0.95f});
        }
    }

    if (showContacts) {
        for (const auto& c : world.lastContacts())
            drawContact(c);
    }
}

} // namespace web
