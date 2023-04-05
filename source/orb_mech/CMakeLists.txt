cmake_minimum_required(VERSION 3.11)

add_library(orb_mech)

find_package(Eigen3 3.4 REQUIRED)

target_sources(orb_mech
    PRIVATE
        Body.cpp
        CartesianVector.cpp
        ElementsGenerator.cpp
        OrbitalKernel.cpp
        StateGenerator.cpp)

target_include_directories(orb_mech PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(orb_mech PUBLIC Eigen3::Eigen)

target_compile_features(orb_mech PUBLIC cxx_std_20)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
