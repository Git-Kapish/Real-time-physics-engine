add_executable(test_math test_math.cpp)
target_link_libraries(test_math PRIVATE math_lib GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_math)
