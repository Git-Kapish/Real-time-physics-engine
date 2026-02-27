/// @file Mesh.cpp
#include "renderer/Mesh.h"
#include <glad/glad.h>
#include <cmath>
#include <utility>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace renderer {

/* ── Destructor / move ────────────────────────────────────────────────── */

Mesh::~Mesh() {
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (ebo_) glDeleteBuffers(1, &ebo_);
}

Mesh::Mesh(Mesh&& o) noexcept
    : vao_(o.vao_), vbo_(o.vbo_), ebo_(o.ebo_)
    , indexCount_(o.indexCount_), drawMode_(o.drawMode_)
{
    o.vao_ = 0; o.vbo_ = 0; o.ebo_ = 0; o.indexCount_ = 0;
}

Mesh& Mesh::operator=(Mesh&& o) noexcept {
    if (this != &o) {
        if (vao_) glDeleteVertexArrays(1, &vao_);
        if (vbo_) glDeleteBuffers(1, &vbo_);
        if (ebo_) glDeleteBuffers(1, &ebo_);

        vao_        = o.vao_;
        vbo_        = o.vbo_;
        ebo_        = o.ebo_;
        indexCount_ = o.indexCount_;
        drawMode_   = o.drawMode_;

        o.vao_ = 0; o.vbo_ = 0; o.ebo_ = 0; o.indexCount_ = 0;
    }
    return *this;
}

/* ── upload / draw ────────────────────────────────────────────────────── */

void Mesh::upload(const std::vector<Vertex>&       verts,
                  const std::vector<unsigned int>& indices,
                  unsigned int                     drawMode)
{
    // Release any existing GL objects
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    if (vbo_) { glDeleteBuffers(1, &vbo_);      vbo_ = 0; }
    if (ebo_) { glDeleteBuffers(1, &ebo_);      ebo_ = 0; }

    drawMode_   = drawMode;
    indexCount_ = static_cast<int>(indices.size());

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(verts.size() * sizeof(Vertex)),
                 verts.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
                 indices.data(), GL_STATIC_DRAW);

    // layout(location = 0): vec3 position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex),
                          reinterpret_cast<const void*>(0));
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void Mesh::draw() const {
    if (!valid()) return;
    glBindVertexArray(vao_);
    glDrawElements(drawMode_, indexCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

/* ── Static factory methods ───────────────────────────────────────────── */

Mesh Mesh::makeWireframeSphere(int subdivisions) {
    const int rings    = 8  * subdivisions;
    const int segments = 16 * subdivisions;

    std::vector<Vertex>       verts;
    std::vector<unsigned int> indices;

    verts.reserve(static_cast<std::size_t>((rings + 1) * (segments + 1)));

    // Generate vertex grid: lat (rings) × lon (segments)
    for (int r = 0; r <= rings; ++r) {
        const float phi    = static_cast<float>(M_PI) * static_cast<float>(r) / static_cast<float>(rings);
        const float y      = std::cos(phi);
        const float sinPhi = std::sin(phi);

        for (int s = 0; s <= segments; ++s) {
            const float theta = 2.f * static_cast<float>(M_PI) * static_cast<float>(s) / static_cast<float>(segments);
            verts.push_back({{ sinPhi * std::cos(theta), y, sinPhi * std::sin(theta) }});
        }
    }

    // Ring (latitude) lines: connect adjacent vertices in the same row
    for (int r = 0; r <= rings; ++r) {
        for (int s = 0; s < segments; ++s) {
            const unsigned int v0 = static_cast<unsigned int>(r * (segments + 1) + s);
            const unsigned int v1 = v0 + 1u;
            indices.push_back(v0);
            indices.push_back(v1);
        }
    }

    // Longitude (meridian) lines: connect rows in the same column
    for (int s = 0; s < segments; ++s) {
        for (int r = 0; r < rings; ++r) {
            const unsigned int v0 = static_cast<unsigned int>(r       * (segments + 1) + s);
            const unsigned int v1 = static_cast<unsigned int>((r + 1) * (segments + 1) + s);
            indices.push_back(v0);
            indices.push_back(v1);
        }
    }

    Mesh m;
    m.upload(verts, indices, GL_LINES);
    return m;
}

Mesh Mesh::makeWireframeBox() {
    // 8 corners of a unit cube (±1, ±1, ±1)
    const std::vector<Vertex> verts = {
        {{ -1.f, -1.f, -1.f }}, // 0
        {{  1.f, -1.f, -1.f }}, // 1
        {{  1.f,  1.f, -1.f }}, // 2
        {{ -1.f,  1.f, -1.f }}, // 3
        {{ -1.f, -1.f,  1.f }}, // 4
        {{  1.f, -1.f,  1.f }}, // 5
        {{  1.f,  1.f,  1.f }}, // 6
        {{ -1.f,  1.f,  1.f }}, // 7
    };
    // 12 edges × 2 indices = 24 indices
    const std::vector<unsigned int> indices = {
        0,1,  1,2,  2,3,  3,0,  // front face (z=-1)
        4,5,  5,6,  6,7,  7,4,  // back face  (z=+1)
        0,4,  1,5,  2,6,  3,7   // connecting edges
    };
    Mesh m;
    m.upload(verts, indices, GL_LINES);
    return m;
}

Mesh Mesh::makeAxes() {
    const std::vector<Vertex> verts = {
        {{ 0.f, 0.f, 0.f }}, {{ 1.f, 0.f, 0.f }},  // +X
        {{ 0.f, 0.f, 0.f }}, {{ 0.f, 1.f, 0.f }},  // +Y
        {{ 0.f, 0.f, 0.f }}, {{ 0.f, 0.f, 1.f }},  // +Z
    };
    const std::vector<unsigned int> indices = { 0,1, 2,3, 4,5 };
    Mesh m;
    m.upload(verts, indices, GL_LINES);
    return m;
}

Mesh Mesh::makePoint() {
    const std::vector<Vertex>       verts   = { {{ 0.f, 0.f, 0.f }} };
    const std::vector<unsigned int> indices = { 0u };
    Mesh m;
    m.upload(verts, indices, GL_POINTS);
    return m;
}

Mesh Mesh::makeLine() {
    const std::vector<Vertex>       verts   = { {{ 0.f, 0.f, 0.f }}, {{ 1.f, 0.f, 0.f }} };
    const std::vector<unsigned int> indices = { 0u, 1u };
    Mesh m;
    m.upload(verts, indices, GL_LINES);
    return m;
}

Mesh Mesh::makePlaneGrid() {
    constexpr int N = 10; // N+1 grid lines in each direction → N cells
    std::vector<Vertex>       verts;
    std::vector<unsigned int> indices;

    verts.reserve(static_cast<std::size_t>((N + 1) * 4));
    indices.reserve(static_cast<std::size_t>((N + 1) * 4));

    unsigned int idx = 0u;
    for (int i = 0; i <= N; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(N) * 2.f - 1.f; // [-1, 1]

        // Line parallel to X axis at z = t
        verts.push_back({{ -1.f, 0.f,  t }});
        verts.push_back({{  1.f, 0.f,  t }});
        indices.push_back(idx++);
        indices.push_back(idx++);

        // Line parallel to Z axis at x = t
        verts.push_back({{  t,   0.f, -1.f }});
        verts.push_back({{  t,   0.f,  1.f }});
        indices.push_back(idx++);
        indices.push_back(idx++);
    }

    Mesh m;
    m.upload(verts, indices, GL_LINES);
    return m;
}

} // namespace renderer
