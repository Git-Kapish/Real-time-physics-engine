add_library(physics_engine_lib STATIC
    physics/RigidBody.cpp
    physics/PhysicsWorld.cpp
    physics/CollisionDetector.cpp
    physics/ImpulseSolver.cpp
    physics/BVHTree.cpp
)

target_include_directories(physics_engine_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(physics_engine_lib PUBLIC math_lib)

target_compile_features(physics_engine_lib PUBLIC cxx_std_17)

# ── Benchmark executable ──────────────────────────────────────────────────
add_executable(physics_benchmark benchmark.cpp)
target_link_libraries(physics_benchmark PRIVATE physics_engine_lib)
target_compile_features(physics_benchmark PRIVATE cxx_std_17)
