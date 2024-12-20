cmake_minimum_required(VERSION 3.30)

include(FetchContent)
include(ExternalProject)

set(FETCHCONTENT_QUIET NO)

FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG          5.5
        GIT_PROGRESS	 true
)

FetchContent_MakeAvailable(raylib)
include_directories(${RAYLIB_SOURCE_DIR}/include)
