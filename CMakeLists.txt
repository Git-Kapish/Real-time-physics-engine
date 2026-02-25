cmake_minimum_required(VERSION 3.16)
project(PhysicsEngine LANGUAGES CXX)

# ── C++17 standard ──────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler warnings ──────────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Per-configuration flags ────────────────────────────────────────────────
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG   "-g -O0")

# ── Header-only math library (interface target) ───────────────────────────
add_library(math_lib INTERFACE)
target_include_directories(math_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ── Google Test via FetchContent ──────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
# Prevent GoogleTest from overriding our compiler/linker settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# ── Tests ─────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
