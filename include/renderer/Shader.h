#pragma once
/// @file Shader.h
/// @brief OpenGL shader program wrapper with file-load / embedded-source fallback.

#include "math/Vec3.h"
#include "math/Mat4.h"
#include <string>

namespace renderer {

using physics::Vec3;
using physics::Mat4;

/**
 * RAII wrapper around an OpenGL shader program.
 *
 * Construction attempts to load GLSL source from file paths.  If a file
 * cannot be opened, the embedded hard-coded shader source is used as a
 * fallback, so the renderer always has working shaders regardless of the
 * working directory.
 *
 * !!  Must only be constructed after gladLoadGL() has succeeded  !!
 */
class Shader {
public:
    /// Default constructor — produces an invalid (no-op) shader.
    Shader() = default;

    /**
     * Load and compile from source files.
     * Falls back to embedded source if either file cannot be opened.
     */
    Shader(const std::string& vertPath, const std::string& fragPath);

    /// Compile directly from source-code strings (no file I/O).
    Shader(const char* vertSrc, const char* fragSrc);

    /// Build a shader from the built-in embedded debug GLSL sources.
    /// Always succeeds on any OpenGL 3.3 Core driver.
    static Shader makeDefault();

    ~Shader();

    // Non-copyable, movable
    Shader(const Shader&)            = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& o) noexcept;
    Shader& operator=(Shader&& o) noexcept;

    /// Bind this program for subsequent draw calls.
    void use() const;

    /// True if the program was compiled and linked successfully.
    bool valid() const { return program_ != 0; }

    // ── Uniform setters ───────────────────────────────────────────────────

    void setMat4 (const std::string& name, const Mat4& m)                    const;
    void setVec3 (const std::string& name, const Vec3& v)                    const;
    void setVec4 (const std::string& name, float x, float y, float z, float w) const;
    void setFloat(const std::string& name, float v)                          const;
    void setInt  (const std::string& name, int v)                            const;

private:
    unsigned int program_ = 0;

    unsigned int compileShader(unsigned int type, const char* src);
    void         checkErrors(unsigned int obj, bool isProgram);
    void         buildProgram(const char* vertSrc, const char* fragSrc);
};

} // namespace renderer
