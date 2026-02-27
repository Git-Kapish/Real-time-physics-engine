#ifndef __glad_h_
#define __glad_h_

/*
 * Minimal GLAD-style OpenGL 3.3 Core loader for the physics engine renderer.
 *
 * Usage:
 *   1. #include <glad/glad.h>  BEFORE any GLFW or GL headers
 *   2. After creating the GL context, call gladLoadGL()
 *
 * This header includes the system GL headers (OpenGL 1.1 declared as normal
 * functions) and then declares function-pointer variables for all GL 2.0–3.3
 * entry points used by the renderer.  Convenience macros map the standard
 * gl* names to the glad_gl* pointer variables so all call sites are unchanged.
 *
 * Telling GLFW not to pull in its own GL headers:
 */
#define GLFW_INCLUDE_NONE

/* ── Windows / MinGW setup ────────────────────────────────────────────── */
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
   /* windef.h defines 'near' and 'far' as empty macros which clash with
    * parameter names in the physics-engine math headers.  Undefine them. */
#  ifdef near
#    undef near
#  endif
#  ifdef far
#    undef far
#  endif
#endif

/* ── System OpenGL headers ────────────────────────────────────────────── */
#include <GL/gl.h>    /* OpenGL 1.1 declarations + Win32 types             */
#include <GL/glext.h> /* PFNGL*PROC typedefs for all extension functions   */

#ifdef __cplusplus
extern "C" {
#endif

/* ── Function-pointer declarations ────────────────────────────────────── */

/* Vertex Array Objects */
extern PFNGLGENVERTEXARRAYSPROC          glad_glGenVertexArrays;
extern PFNGLBINDVERTEXARRAYPROC          glad_glBindVertexArray;
extern PFNGLDELETEVERTEXARRAYSPROC       glad_glDeleteVertexArrays;

/* Buffer Objects */
extern PFNGLGENBUFFERSPROC               glad_glGenBuffers;
extern PFNGLBINDBUFFERPROC               glad_glBindBuffer;
extern PFNGLBUFFERDATAPROC               glad_glBufferData;
extern PFNGLDELETEBUFFERSPROC            glad_glDeleteBuffers;

/* Vertex Attributes */
extern PFNGLVERTEXATTRIBPOINTERPROC      glad_glVertexAttribPointer;
extern PFNGLENABLEVERTEXATTRIBARRAYPROC  glad_glEnableVertexAttribArray;

/* Shaders */
extern PFNGLCREATESHADERPROC             glad_glCreateShader;
extern PFNGLSHADERSOURCEPROC             glad_glShaderSource;
extern PFNGLCOMPILESHADERPROC            glad_glCompileShader;
extern PFNGLGETSHADERIVPROC              glad_glGetShaderiv;
extern PFNGLGETSHADERINFOLOGPROC         glad_glGetShaderInfoLog;
extern PFNGLDELETESHADERPROC             glad_glDeleteShader;

/* Shader Programs */
extern PFNGLCREATEPROGRAMPROC            glad_glCreateProgram;
extern PFNGLATTACHSHADERPROC             glad_glAttachShader;
extern PFNGLLINKPROGRAMPROC              glad_glLinkProgram;
extern PFNGLGETPROGRAMIVPROC             glad_glGetProgramiv;
extern PFNGLGETPROGRAMINFOLOGPROC        glad_glGetProgramInfoLog;
extern PFNGLUSEPROGRAMPROC               glad_glUseProgram;
extern PFNGLDELETEPROGRAMPROC            glad_glDeleteProgram;

/* Uniforms */
extern PFNGLGETUNIFORMLOCATIONPROC       glad_glGetUniformLocation;
extern PFNGLUNIFORMMATRIX4FVPROC         glad_glUniformMatrix4fv;
extern PFNGLUNIFORM3FPROC                glad_glUniform3f;
extern PFNGLUNIFORM4FPROC                glad_glUniform4f;
extern PFNGLUNIFORM1FPROC                glad_glUniform1f;
extern PFNGLUNIFORM1IPROC                glad_glUniform1i;

/* Framebuffers (optional, loaded for completeness) */
extern PFNGLGENFRAMEBUFFERSPROC          glad_glGenFramebuffers;

/* ── Convenience macros ────────────────────────────────────────────────── */
#define glGenVertexArrays        glad_glGenVertexArrays
#define glBindVertexArray        glad_glBindVertexArray
#define glDeleteVertexArrays     glad_glDeleteVertexArrays

#define glGenBuffers             glad_glGenBuffers
#define glBindBuffer             glad_glBindBuffer
#define glBufferData             glad_glBufferData
#define glDeleteBuffers          glad_glDeleteBuffers

#define glVertexAttribPointer    glad_glVertexAttribPointer
#define glEnableVertexAttribArray glad_glEnableVertexAttribArray

#define glCreateShader           glad_glCreateShader
#define glShaderSource           glad_glShaderSource
#define glCompileShader          glad_glCompileShader
#define glGetShaderiv            glad_glGetShaderiv
#define glGetShaderInfoLog       glad_glGetShaderInfoLog
#define glDeleteShader           glad_glDeleteShader

#define glCreateProgram          glad_glCreateProgram
#define glAttachShader           glad_glAttachShader
#define glLinkProgram            glad_glLinkProgram
#define glGetProgramiv           glad_glGetProgramiv
#define glGetProgramInfoLog      glad_glGetProgramInfoLog
#define glUseProgram             glad_glUseProgram
#define glDeleteProgram          glad_glDeleteProgram

#define glGetUniformLocation     glad_glGetUniformLocation
#define glUniformMatrix4fv       glad_glUniformMatrix4fv
#define glUniform3f              glad_glUniform3f
#define glUniform4f              glad_glUniform4f
#define glUniform1f              glad_glUniform1f
#define glUniform1i              glad_glUniform1i

#define glGenFramebuffers        glad_glGenFramebuffers

/* ── Loader ────────────────────────────────────────────────────────────── */

/**
 * Load all OpenGL 3.3 core function pointers.
 * Must be called after a GL context has been made current (e.g. after
 * glfwMakeContextCurrent).  Uses glfwGetProcAddress internally.
 *
 * @return 1 on success, 0 if any critical function could not be loaded.
 */
int gladLoadGL(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __glad_h_ */
