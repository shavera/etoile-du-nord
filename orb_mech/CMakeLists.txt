cmake_minimum_required(VERSION 3.11)

add_library(orb_mech)

target_sources(orb_mech
    PRIVATE
        Body.cpp)

target_include_directories(orb_mech PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

