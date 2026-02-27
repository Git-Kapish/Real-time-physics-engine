#pragma once
/// @file Mesh.h
/// @brief GPU mesh: uploads geometry to VAO/VBO/EBO and draws it.

#include "math/Vec3.h"
#include <vector>

/* Forward-declare so Mesh.h does not pull in glad.h everywhere. */
namespace renderer { class Mesh; }

namespace renderer {

using physics::Vec3;

/// A single vertex carrying only a 3-D position (all colour comes from uniforms).
struct Vertex {
    Vec3 position;
};

/**
 * Lightweight RAII wrapper around one VAO + VBO + EBO.
 *
 * !!  upload() and the static factory methods must only be called after
 *     gladLoadGL() has succeeded and a GL context is current  !!
 *
 * Move-only (no copies); VAO/VBO/EBO are freed in the destructor.
 */
class Mesh {
public:
    Mesh()  = default;
    ~Mesh();

    Mesh(const Mesh&)            = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) noexcept;
    Mesh& operator=(Mesh&&) noexcept;

    // ── Upload ────────────────────────────────────────────────────────────

    /**
     * Upload vertex and index data to the GPU.
     * Calling upload() a second time replaces the existing buffers.
     *
     * @param verts     Vertex array (positions only).
     * @param indices   Index array (triplets for GL_TRIANGLES, pairs for GL_LINES, etc.).
     * @param drawMode  GL_TRIANGLES, GL_LINES, GL_POINTS, … (default GL_TRIANGLES).
     */
    void upload(const std::vector<Vertex>&       verts,
                const std::vector<unsigned int>& indices,
                unsigned int drawMode = 0x0004 /* GL_TRIANGLES */);

    /// Issue the indexed draw call.  No-op if not valid().
    void draw() const;

    /// True once upload() has completed successfully.
    bool valid() const { return vao_ != 0; }

    /// Returns the draw mode passed to upload().
    unsigned int drawMode() const { return drawMode_; }

    // ── Static factory methods ────────────────────────────────────────────

    /// UV-sphere wireframe.  subdivisions controls ring/segment count.
    static Mesh makeWireframeSphere(int subdivisions = 3);

    /// Unit cube wireframe (corners at ±1 on each axis).
    static Mesh makeWireframeBox();

    /// Three axis lines from origin: X(1,0,0), Y(0,1,0), Z(0,0,1).
    static Mesh makeAxes();

    /// Single vertex at the origin (drawn as GL_POINTS).
    static Mesh makePoint();

    /// Line from (0,0,0) to (1,0,0) — transform to draw any line segment.
    static Mesh makeLine();

    /// 10×10 flat grid in the XZ plane from -1 to +1 (scale by 50 for a 50-unit plane).
    static Mesh makePlaneGrid();

    /// Full-screen quad in NDC [-1,1]x[-1,1], z=0.  Used for background gradient.
    /// Draw with depth-test OFF so it never occludes geometry.
    static Mesh makeFullscreenQuad();

private:
    unsigned int vao_        = 0;
    unsigned int vbo_        = 0;
    unsigned int ebo_        = 0;
    int          indexCount_ = 0;
    unsigned int drawMode_   = 0x0004; // GL_TRIANGLES
};

} // namespace renderer
