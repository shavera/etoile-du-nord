cmake_minimum_required(VERSION 3.11)

add_executable(orb_mech_tests)

target_sources(orb_mech_tests
    PRIVATE
        BodyTest.cpp
        CartesianVectorTest.cpp
        ElementsGeneratorTest.cpp
#        OrbitalPhysicsMatrixTest.cpp
        OrbitalKernelTest.cpp
        StateGeneratorTest.cpp
        UnitsTest.cpp)

target_link_libraries(orb_mech_tests PRIVATE orb_mech GTest::gmock_main)
include(GoogleTest)
gtest_discover_tests(orb_mech_tests)
