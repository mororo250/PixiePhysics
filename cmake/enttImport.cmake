cmake_minimum_required(VERSION 3.30)

include(FetchContent)
include(ExternalProject)

set(FETCHCONTENT_QUIET NO)

FetchContent_Declare(
        entt
        GIT_REPOSITORY   https://github.com/skypjack/entt.git
        GIT_TAG          master
        GIT_PROGRESS	 true
)

FetchContent_MakeAvailable(entt)
include_directories(${entt_SOURCE_DIR}/src)
