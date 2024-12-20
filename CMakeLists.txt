cmake_minimum_required(VERSION 3.30)
project(PixiePhysics)

set(CMAKE_CXX_STANDARD 23)

include(cmake/raylibImport.cmake)
include(cmake/glmImport.cmake)
include(cmake/enttImport.cmake)

# Create a library target for the shared code
add_library(PixiePhysicsLib STATIC
        src/physics/components/SphereShape.hpp
        src/physics/components/RigidBody.hpp
        src/scene/Scene.cpp
        src/scene/Scene.hpp
        src/physics/components/Transform.hpp
        src/rendering/systems/RenderSystem.cpp
        src/rendering/systems/RenderSystem.hpp
        src/physics/systems/ResolveCollisions.cpp
        src/physics/systems/ResolveCollisions.hpp
        src/physics/ContactPoint.hpp
        src/physics/systems/UpdateTransform.cpp
        src/physics/systems/UpdateTransform.hpp)

target_link_libraries(PixiePhysicsLib PUBLIC raylib)
target_link_libraries(PixiePhysicsLib PUBLIC glm::glm)
target_link_libraries(PixiePhysicsLib PUBLIC EnTT::EnTT)

add_executable(PixiePhysics main.cpp
        example/Lights.h
)

target_link_libraries(PixiePhysics PRIVATE PixiePhysicsLib)

add_subdirectory(test)

# This can get really expensive with textures and models
# Can we optimize this somehow?
file(COPY "${CMAKE_SOURCE_DIR}/assets" DESTINATION "${CMAKE_BINARY_DIR}")