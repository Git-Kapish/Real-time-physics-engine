/*
 * glad.c — minimal OpenGL 3.3 Core function-pointer loader.
 *
 * Compiles as C99.  Uses GLFW's glfwGetProcAddress to resolve each
 * entry point after a GL context has been created.
 *
 * All casts between glfwGetProcAddress return value (GLFWglproc, a
 * function pointer) and PFNGL*PROC types are function-pointer ↔
 * function-pointer casts, which are implementation-defined but work
 * correctly on every platform that GLFW supports.  The pedantic warning
 * is suppressed locally so the rest of the project remains warning-free.
 */

#include "glad/glad.h"

/* GLFW must come after glad.h so GLFW_INCLUDE_NONE is already defined. */
#include <GLFW/glfw3.h>

/* ── Function-pointer definitions ────────────────────────────────────── */

PFNGLGENVERTEXARRAYSPROC          glad_glGenVertexArrays      = NULL;
PFNGLBINDVERTEXARRAYPROC          glad_glBindVertexArray      = NULL;
PFNGLDELETEVERTEXARRAYSPROC       glad_glDeleteVertexArrays   = NULL;

PFNGLGENBUFFERSPROC               glad_glGenBuffers           = NULL;
PFNGLBINDBUFFERPROC               glad_glBindBuffer           = NULL;
PFNGLBUFFERDATAPROC               glad_glBufferData           = NULL;
PFNGLDELETEBUFFERSPROC            glad_glDeleteBuffers        = NULL;

PFNGLVERTEXATTRIBPOINTERPROC      glad_glVertexAttribPointer  = NULL;
PFNGLENABLEVERTEXATTRIBARRAYPROC  glad_glEnableVertexAttribArray = NULL;

PFNGLCREATESHADERPROC             glad_glCreateShader         = NULL;
PFNGLSHADERSOURCEPROC             glad_glShaderSource         = NULL;
PFNGLCOMPILESHADERPROC            glad_glCompileShader        = NULL;
PFNGLGETSHADERIVPROC              glad_glGetShaderiv          = NULL;
PFNGLGETSHADERINFOLOGPROC         glad_glGetShaderInfoLog     = NULL;
PFNGLDELETESHADERPROC             glad_glDeleteShader         = NULL;

PFNGLCREATEPROGRAMPROC            glad_glCreateProgram        = NULL;
PFNGLATTACHSHADERPROC             glad_glAttachShader         = NULL;
PFNGLLINKPROGRAMPROC              glad_glLinkProgram          = NULL;
PFNGLGETPROGRAMIVPROC             glad_glGetProgramiv         = NULL;
PFNGLGETPROGRAMINFOLOGPROC        glad_glGetProgramInfoLog    = NULL;
PFNGLUSEPROGRAMPROC               glad_glUseProgram           = NULL;
PFNGLDELETEPROGRAMPROC            glad_glDeleteProgram        = NULL;

PFNGLGETUNIFORMLOCATIONPROC       glad_glGetUniformLocation   = NULL;
PFNGLUNIFORMMATRIX4FVPROC         glad_glUniformMatrix4fv     = NULL;
PFNGLUNIFORM3FPROC                glad_glUniform3f            = NULL;
PFNGLUNIFORM4FPROC                glad_glUniform4f            = NULL;
PFNGLUNIFORM1FPROC                glad_glUniform1f            = NULL;
PFNGLUNIFORM1IPROC                glad_glUniform1i            = NULL;

PFNGLGENFRAMEBUFFERSPROC          glad_glGenFramebuffers      = NULL;

/* ── Loader implementation ────────────────────────────────────────────── */

/*
 * Helper macro: load one function pointer.
 * We suppress -Wpedantic for the function-pointer cast via a union, which
 * is standards-conformant (both members are pointer types of the same
 * representation on all supported platforms).
 */
#define GLAD_LOAD(type, var, name)                          \
    do {                                                    \
        union { GLFWglproc g; type f; } _u;                 \
        _u.g = glfwGetProcAddress(name);                    \
        var  = _u.f;                                        \
    } while (0)

int gladLoadGL(void)
{
    GLAD_LOAD(PFNGLGENVERTEXARRAYSPROC,         glad_glGenVertexArrays,        "glGenVertexArrays");
    GLAD_LOAD(PFNGLBINDVERTEXARRAYPROC,         glad_glBindVertexArray,        "glBindVertexArray");
    GLAD_LOAD(PFNGLDELETEVERTEXARRAYSPROC,      glad_glDeleteVertexArrays,     "glDeleteVertexArrays");

    GLAD_LOAD(PFNGLGENBUFFERSPROC,              glad_glGenBuffers,             "glGenBuffers");
    GLAD_LOAD(PFNGLBINDBUFFERPROC,              glad_glBindBuffer,             "glBindBuffer");
    GLAD_LOAD(PFNGLBUFFERDATAPROC,              glad_glBufferData,             "glBufferData");
    GLAD_LOAD(PFNGLDELETEBUFFERSPROC,           glad_glDeleteBuffers,          "glDeleteBuffers");

    GLAD_LOAD(PFNGLVERTEXATTRIBPOINTERPROC,     glad_glVertexAttribPointer,    "glVertexAttribPointer");
    GLAD_LOAD(PFNGLENABLEVERTEXATTRIBARRAYPROC, glad_glEnableVertexAttribArray,"glEnableVertexAttribArray");

    GLAD_LOAD(PFNGLCREATESHADERPROC,            glad_glCreateShader,           "glCreateShader");
    GLAD_LOAD(PFNGLSHADERSOURCEPROC,            glad_glShaderSource,           "glShaderSource");
    GLAD_LOAD(PFNGLCOMPILESHADERPROC,           glad_glCompileShader,          "glCompileShader");
    GLAD_LOAD(PFNGLGETSHADERIVPROC,             glad_glGetShaderiv,            "glGetShaderiv");
    GLAD_LOAD(PFNGLGETSHADERINFOLOGPROC,        glad_glGetShaderInfoLog,       "glGetShaderInfoLog");
    GLAD_LOAD(PFNGLDELETESHADERPROC,            glad_glDeleteShader,           "glDeleteShader");

    GLAD_LOAD(PFNGLCREATEPROGRAMPROC,           glad_glCreateProgram,          "glCreateProgram");
    GLAD_LOAD(PFNGLATTACHSHADERPROC,            glad_glAttachShader,           "glAttachShader");
    GLAD_LOAD(PFNGLLINKPROGRAMPROC,             glad_glLinkProgram,            "glLinkProgram");
    GLAD_LOAD(PFNGLGETPROGRAMIVPROC,            glad_glGetProgramiv,           "glGetProgramiv");
    GLAD_LOAD(PFNGLGETPROGRAMINFOLOGPROC,       glad_glGetProgramInfoLog,      "glGetProgramInfoLog");
    GLAD_LOAD(PFNGLUSEPROGRAMPROC,              glad_glUseProgram,             "glUseProgram");
    GLAD_LOAD(PFNGLDELETEPROGRAMPROC,           glad_glDeleteProgram,          "glDeleteProgram");

    GLAD_LOAD(PFNGLGETUNIFORMLOCATIONPROC,      glad_glGetUniformLocation,     "glGetUniformLocation");
    GLAD_LOAD(PFNGLUNIFORMMATRIX4FVPROC,        glad_glUniformMatrix4fv,       "glUniformMatrix4fv");
    GLAD_LOAD(PFNGLUNIFORM3FPROC,               glad_glUniform3f,              "glUniform3f");
    GLAD_LOAD(PFNGLUNIFORM4FPROC,               glad_glUniform4f,              "glUniform4f");
    GLAD_LOAD(PFNGLUNIFORM1FPROC,               glad_glUniform1f,              "glUniform1f");
    GLAD_LOAD(PFNGLUNIFORM1IPROC,               glad_glUniform1i,              "glUniform1i");

    GLAD_LOAD(PFNGLGENFRAMEBUFFERSPROC,         glad_glGenFramebuffers,        "glGenFramebuffers");

#undef GLAD_LOAD

    /* Return success only when all critical functions are loaded */
    return (glad_glGenVertexArrays     != NULL &&
            glad_glCreateShader        != NULL &&
            glad_glCreateProgram       != NULL &&
            glad_glUniformMatrix4fv    != NULL) ? 1 : 0;
}
