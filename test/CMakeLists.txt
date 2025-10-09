project(PixiePhysicsTest)

include(../cmake/gtestImport.cmake)

add_executable(PixiePhysicsTest firstTest.cpp
        GjkUnitTest.cpp
        BaryCentricCoordUnitTest.cpp)


target_link_libraries(PixiePhysicsTest PRIVATE
        PixiePhysicsLib    # Sua própria biblioteca
        glm                # Biblioteca GLM
        raylib             # Biblioteca RayLib
        gtest              # Google Test
        gtest_main         # Biblioteca principal do Google Test
)


