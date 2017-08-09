set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h
        BasicDemo.cpp
        BasicDemo.h
        OpenGLMotionState.h)
add_executable(CH3.2_CreatingOurFirstPhysicsObject ${SOURCE_FILES})
target_link_libraries(CH3.2_CreatingOurFirstPhysicsObject ${LIBS})