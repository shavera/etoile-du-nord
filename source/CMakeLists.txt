cmake_minimum_required(VERSION 3.11)

project(etoile-du-nord)

option(BUILD_TESTS "Build tests with project" ON)

if(${BUILD_TESTS})
    include(FetchContent)

    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG b796f7d # v1.13.0
    )

    FetchContent_MakeAvailable(googletest)

    enable_testing()
endif()

add_subdirectory(orb_mech)
