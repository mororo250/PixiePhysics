cmake_minimum_required(VERSION 3.30)

include(FetchContent)
include(ExternalProject)

set(FETCHCONTENT_QUIET NO)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        release-1.12.1
        GIT_PROGRESS   true
)

FetchContent_MakeAvailable(googletest)

add_library(GTest::GTest INTERFACE IMPORTED)
include_directories(${googletest_SOURCE_DIR}/googletest/include)
include_directories(${googletest_SOURCE_DIR}/googlemock/include)

set_target_properties (gtest PROPERTIES FOLDER extern/gtest)
set_target_properties (gtest_main PROPERTIES FOLDER extern/gtest)
set_target_properties (gmock PROPERTIES FOLDER extern/gtest)
set_target_properties (gmock_main PROPERTIES FOLDER extern/gtest)
