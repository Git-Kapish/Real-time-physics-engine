/// @file Shader.cpp
#include "renderer/Shader.h"
#include <glad/glad.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <utility>

namespace renderer {

/* ── Embedded fallback sources ──────────────────────────────────────── */

static const char* const kEmbeddedVert =
    "#version 330 core\n"
    "layout(location = 0) in vec3 aPos;\n"
    "uniform mat4 uMVP;\n"
    "uniform vec3 uColor;\n"
    "out vec3 vColor;\n"
    "void main() {\n"
    "    gl_Position = uMVP * vec4(aPos, 1.0);\n"
    "    vColor = uColor;\n"
    "}\n";

static const char* const kEmbeddedFrag =
    "#version 330 core\n"
    "in vec3 vColor;\n"
    "out vec4 FragColor;\n"
    "uniform float uAlpha;\n"
    "void main() {\n"
    "    FragColor = vec4(vColor, uAlpha);\n"
    "}\n";

/* ── Helpers ─────────────────────────────────────────────────────────── */

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/* ── Constructors / destructor ───────────────────────────────────────── */

Shader::Shader(const std::string& vertPath, const std::string& fragPath) {
    const std::string vSrc = readFile(vertPath);
    const std::string fSrc = readFile(fragPath);

    const char* vertSrc = vSrc.empty() ? kEmbeddedVert : vSrc.c_str();
    const char* fragSrc = fSrc.empty() ? kEmbeddedFrag : fSrc.c_str();

    if (vSrc.empty())
        std::cerr << "[Shader] Could not open '" << vertPath
                  << "', using embedded fallback.\n";
    if (fSrc.empty())
        std::cerr << "[Shader] Could not open '" << fragPath
                  << "', using embedded fallback.\n";

    buildProgram(vertSrc, fragSrc);
}

Shader::Shader(const char* vertSrc, const char* fragSrc) {
    buildProgram(vertSrc, fragSrc);
}

Shader Shader::makeDefault() {
    return Shader(kEmbeddedVert, kEmbeddedFrag);
}

Shader::~Shader() {
    if (program_) {
        glDeleteProgram(program_);
    }
}

Shader::Shader(Shader&& o) noexcept : program_(o.program_) {
    o.program_ = 0;
}

Shader& Shader::operator=(Shader&& o) noexcept {
    if (this != &o) {
        if (program_) glDeleteProgram(program_);
        program_   = o.program_;
        o.program_ = 0;
    }
    return *this;
}

/* ── Public interface ────────────────────────────────────────────────── */

void Shader::use() const {
    if (program_) glUseProgram(program_);
}

void Shader::setMat4(const std::string& name, const Mat4& m) const {
    glUniformMatrix4fv(glGetUniformLocation(program_, name.c_str()),
                       1, GL_FALSE, m.ptr());
}

void Shader::setVec3(const std::string& name, const Vec3& v) const {
    glUniform3f(glGetUniformLocation(program_, name.c_str()), v.x, v.y, v.z);
}

void Shader::setVec4(const std::string& name,
                     float x, float y, float z, float w) const {
    glUniform4f(glGetUniformLocation(program_, name.c_str()), x, y, z, w);
}

void Shader::setFloat(const std::string& name, float v) const {
    glUniform1f(glGetUniformLocation(program_, name.c_str()), v);
}

void Shader::setInt(const std::string& name, int v) const {
    glUniform1i(glGetUniformLocation(program_, name.c_str()), v);
}

/* ── Private helpers ─────────────────────────────────────────────────── */

unsigned int Shader::compileShader(unsigned int type, const char* src) {
    const unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &src, nullptr);
    glCompileShader(id);
    checkErrors(id, false);
    return id;
}

void Shader::checkErrors(unsigned int obj, bool isProgram) {
    int ok = 0;
    if (isProgram) {
        glGetProgramiv(obj, GL_LINK_STATUS, &ok);
        if (!ok) {
            int len = 0;
            glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &len);
            if (len > 1) {
                std::string log(static_cast<std::size_t>(len), '\0');
                glGetProgramInfoLog(obj, len, nullptr, log.data());
                std::cerr << "[Shader] Link error:\n" << log << '\n';
            }
        }
    } else {
        glGetShaderiv(obj, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            int len = 0;
            glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &len);
            if (len > 1) {
                std::string log(static_cast<std::size_t>(len), '\0');
                glGetShaderInfoLog(obj, len, nullptr, log.data());
                std::cerr << "[Shader] Compile error:\n" << log << '\n';
            }
        }
    }
}

void Shader::buildProgram(const char* vertSrc, const char* fragSrc) {
    const unsigned int vert = compileShader(GL_VERTEX_SHADER,   vertSrc);
    const unsigned int frag = compileShader(GL_FRAGMENT_SHADER, fragSrc);

    program_ = glCreateProgram();
    glAttachShader(program_, vert);
    glAttachShader(program_, frag);
    glLinkProgram(program_);
    checkErrors(program_, true);

    glDeleteShader(vert);
    glDeleteShader(frag);

    // If linking failed, release the broken program so valid() returns false.
    int ok = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        glDeleteProgram(program_);
        program_ = 0;
    }
}

} // namespace renderer
