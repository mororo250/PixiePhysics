project(PixiePhysicsTest)

include(../cmake/gtestImport.cmake)

add_executable(PixiePhysicsTest firstTest.cpp)

#addTest(PixiePhysicsTest firstTest.cpp)
target_link_libraries(PixiePhysicsTest PRIVATE GTest::GTest)
target_link_libraries(PixiePhysicsTest PRIVATE glm::glm)
