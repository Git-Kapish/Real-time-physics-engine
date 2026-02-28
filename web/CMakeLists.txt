cmake_minimum_required(VERSION 3.16)
# This file is only processed when EMSCRIPTEN is the active toolchain.
# Invoke with:
#   emcmake cmake <repo>/physics_engine \
#           -B <repo>/build_wasm       \
#           -DCMAKE_BUILD_TYPE=Release
#   cmake --build <repo>/build_wasm --target physics_engine_wasm

# ── WASM demo executable ──────────────────────────────────────────────────
add_executable(physics_engine_wasm
    ${CMAKE_CURRENT_SOURCE_DIR}/main_wasm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/WebGLRenderer.cpp
)

target_include_directories(physics_engine_wasm PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}           # web/ headers
    ${CMAKE_CURRENT_SOURCE_DIR}/../include # engine headers
)

target_link_libraries(physics_engine_wasm PRIVATE physics_engine_lib)
target_compile_features(physics_engine_wasm PRIVATE cxx_std_17)

# ── Emscripten output & linker flags ─────────────────────────────────────
#   -sUSE_WEBGL2=1            → map OpenGL ES 3 calls to WebGL 2
#   -sMAX_WEBGL_VERSION=2     → enable WebGL 2 context creation
#   -sMIN_WEBGL_VERSION=2     → require WebGL 2 (no legacy fallback)
#   -sALLOW_MEMORY_GROWTH=1   → heap can grow as bodies accumulate
#   -sMODULARIZE=1            → wrap output in a factory function
#   -sEXPORT_NAME=PhysicsEngine→ factory function name for index.html
#   --bind                    → compile in Embind JS bindings
#   -sFILESYSTEM=0            → no VFS needed (shaders are embedded)
#   -sNO_EXIT_RUNTIME=1       → keep runtime alive (Emscripten loop)
#   -sFORCE_FILESYSTEM=0      → double-confirm no FS overhead
#   -O3                       → optimised build for 60 fps target
set_target_properties(physics_engine_wasm PROPERTIES
    OUTPUT_NAME "physics_engine"
    SUFFIX      ".js"
)

target_link_options(physics_engine_wasm PRIVATE
    -sUSE_WEBGL2=1
    -sMAX_WEBGL_VERSION=2
    -sMIN_WEBGL_VERSION=2
    -sALLOW_MEMORY_GROWTH=1
    -sMODULARIZE=1
    "-sEXPORT_NAME=PhysicsEngine"
    --bind
    -sFILESYSTEM=0
    -sNO_EXIT_RUNTIME=1
    -sFORCE_FILESYSTEM=0
    "$<IF:$<CONFIG:Release>,-O3,-O0>"
    "$<$<CONFIG:Debug>:-g>"
    "$<$<CONFIG:Release>:--closure=1>"
)

# Profiling build: add -sPROFILING_FUNCS for flamegraph capture
# target_link_options(physics_engine_wasm PRIVATE -sPROFILING_FUNCS=1)

# ── Copy HTML shell to the binary dir ────────────────────────────────────
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/index.html
    ${CMAKE_CURRENT_BINARY_DIR}/index.html
    COPYONLY
)
