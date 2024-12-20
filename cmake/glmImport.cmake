cmake_minimum_required(VERSION 3.30)

include(FetchContent)
include(ExternalProject)

set(FETCHCONTENT_QUIET NO)

FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG          1.0.1
        GIT_PROGRESS	 true
)

FetchContent_MakeAvailable(glm)
include_directories(${GLM_SOURCE_DIR}/include)
