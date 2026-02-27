add_executable(test_math test_math.cpp)
target_link_libraries(test_math PRIVATE math_lib GTest::gtest_main)

add_executable(test_physics test_physics.cpp)
target_link_libraries(test_physics PRIVATE physics_engine_lib GTest::gtest_main)

add_executable(test_collision test_collision.cpp)
target_link_libraries(test_collision PRIVATE physics_engine_lib GTest::gtest_main)

add_executable(test_solver test_solver.cpp)
target_link_libraries(test_solver PRIVATE physics_engine_lib GTest::gtest_main)

add_executable(test_bvh test_bvh.cpp)
target_link_libraries(test_bvh PRIVATE physics_engine_lib GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_math)
gtest_discover_tests(test_physics)
gtest_discover_tests(test_collision)
gtest_discover_tests(test_solver)
gtest_discover_tests(test_bvh)
