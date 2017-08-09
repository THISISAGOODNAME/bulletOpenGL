cmake_minimum_required(VERSION 3.8)
project(bulletOpenGL)

set(CMAKE_CXX_STANDARD 11)

# config OpenGL
FIND_PACKAGE(OpenGL REQUIRED)
LIST(APPEND LIBS ${OPENGL_gl_LIBRARY})
LIST(APPEND LIBS ${OPENGL_glu_LIBRARY})
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
message(warning ${OPENGL_INCLUDE_DIR})

# config freeglut
# TODO: NEED modify
add_subdirectory(3rd_party/freeglut/freeglut/freeglut)
INCLUDE_DIRECTORIES(3rd_party/freeglut/freeglut/freeglut/include)
#LIST(APPEND LIBS freeglut)
LIST(APPEND LIBS glut)

# config bullet
option(BUILD_BULLET2_DEMOS OFF)
option(BUILD_CPU_DEMOS OFF)
option(BUILD_EXTRAS OFF)
option(BUILD_OPENGL3_DEMOS OFF)
option(BUILD_UNIT_TESTS OFF)
add_subdirectory(3rd_party/bullet)
INCLUDE_DIRECTORIES(3rd_party/bullet/src)
LIST(APPEND LIBS BulletDynamics BulletCollision BulletCollision LinearMath)

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -std=c++11")
endif()

add_subdirectory(CH1.2_TheApplicationLayer)
add_subdirectory(CH2.1_RenderingTheScene)
add_subdirectory(CH2.2_BasicRenderingAndLighting)
add_subdirectory(CH2.3_UserInputAndCameraControl)
add_subdirectory(CH3.1_TheCoreBulletObjects)
add_subdirectory(CH3.2_CreatingOurFirstPhysicsObject)
add_subdirectory(CH4.1_HandlingMultipleObjects)
add_subdirectory(CH4.2_DebugDrawer)
add_subdirectory(CH5.1_Raycasting)
add_subdirectory(CH5.2_Constraints)
add_subdirectory(CH6.1_CollisionEvents)