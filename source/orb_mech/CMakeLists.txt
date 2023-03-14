cmake_minimum_required(VERSION 3.11)

add_library(orb_mech)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

target_sources(orb_mech
    PRIVATE
        Body.cpp
        CartesianVector.cpp
        Orbit.cpp)

target_include_directories(orb_mech PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(orb_mech PUBLIC Eigen3::Eigen)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
