cmake_minimum_required(VERSION 3.16)
project(PhysicsEngine LANGUAGES CXX C)

# ── C++17 standard ──────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler warnings ──────────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    # glad.c uses function-pointer casts that only warn under -Wpedantic; the
    # file suppresses them internally, but we disable the C-specific flag globally
    # for C sources so the rest of the project stays warning-free.
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-pedantic")
endif()

# ── Per-configuration flags ────────────────────────────────────────────────
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# ── Header-only math library (interface target) ───────────────────────────
add_library(math_lib INTERFACE)
target_include_directories(math_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ── Physics engine static library (needed by both native and WASM) ───────
add_subdirectory(src)

if(NOT EMSCRIPTEN)
    # ── OpenGL + GLFW ─────────────────────────────────────────────────────
    # Add the MSYS2/MinGW prefix so find_package locates the system packages
    # when building from a plain Windows terminal.
    if(WIN32)
        list(APPEND CMAKE_PREFIX_PATH "C:/msys64/mingw64")
    endif()

    find_package(OpenGL REQUIRED)
    find_package(glfw3  CONFIG REQUIRED)

    # ── GLAD (vendored, compiled as a plain C static library) ─────────────
    add_library(glad STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/external/glad/src/glad.c
    )
    target_include_directories(glad PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/external/glad/include
    )
    target_link_libraries(glad PRIVATE OpenGL::GL)
    target_link_libraries(glad PRIVATE glfw)

    # ── Google Test via FetchContent ───────────────────────────────────────
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # ── Tests ──────────────────────────────────────────────────────────────
    enable_testing()
    add_subdirectory(tests)

    # ── Native executable (renderer + demo) ───────────────────────────────
    add_executable(physics_engine
        src/main.cpp
        src/renderer/Shader.cpp
        src/renderer/Mesh.cpp
        src/renderer/DebugRenderer.cpp
        src/renderer/Camera.cpp
        src/core/Window.cpp
        src/core/Timer.cpp
    )

    target_include_directories(physics_engine PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/external/glad/include
    )

    target_link_libraries(physics_engine PRIVATE
        physics_engine_lib
        glad
        glfw
        OpenGL::GL
        ${CMAKE_DL_LIBS}
    )

    target_compile_features(physics_engine PRIVATE cxx_std_17)

    # ── Copy shaders next to the executable ───────────────────────────────
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders/debug.vert
        ${CMAKE_CURRENT_BINARY_DIR}/shaders/debug.vert
        COPYONLY
    )
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/shaders/debug.frag
        ${CMAKE_CURRENT_BINARY_DIR}/shaders/debug.frag
        COPYONLY
    )
endif()

# ── WebAssembly / Emscripten build ─────────────────────────────────────────
# Activated when:  emcmake cmake <source_dir> -B build_wasm ...
if(EMSCRIPTEN)
    add_subdirectory(web)
endif()