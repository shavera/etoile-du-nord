cmake_minimum_required(VERSION 3.11)

include(FetchContent)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG b796f7d # v1.13.0
)

FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(orb_mech_tests)

target_sources(orb_mech_tests
    PRIVATE
        BodyTest.cpp)

target_link_libraries(orb_mech_tests PRIVATE orb_mech GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(orb_mech_tests)
