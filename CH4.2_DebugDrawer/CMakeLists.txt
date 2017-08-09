set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h
        BasicDemo.cpp
        BasicDemo.h
        OpenGLMotionState.h
        GameObject.cpp
        GameObject.h DebugDrawer.cpp DebugDrawer.h)
add_executable(CH4.2_DebugDrawer ${SOURCE_FILES})
target_link_libraries(CH4.2_DebugDrawer ${LIBS})