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
add_executable(CH7.2_ConvexHulls ${SOURCE_FILES})
target_link_libraries(CH7.2_ConvexHulls ${LIBS})