set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h
        BasicDemo.cpp
        BasicDemo.h
        OpenGLMotionState.h
        GameObject.cpp
        GameObject.h
        DebugDrawer.cpp
        DebugDrawer.h)
add_executable(CH5.2_Constraints ${SOURCE_FILES})
target_link_libraries(CH5.2_Constraints ${LIBS})