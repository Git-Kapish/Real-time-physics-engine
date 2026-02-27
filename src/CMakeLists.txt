add_library(physics_engine_lib STATIC
    physics/RigidBody.cpp
    physics/PhysicsWorld.cpp
)

target_include_directories(physics_engine_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(physics_engine_lib PUBLIC math_lib)

target_compile_features(physics_engine_lib PUBLIC cxx_std_17)
