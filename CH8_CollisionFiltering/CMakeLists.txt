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
        DebugDrawer.h
        CollisionFilteringDemo.cpp
        CollisionFilteringDemo.h)
add_executable(CH8_CollisionFiltering ${SOURCE_FILES})
target_link_libraries(CH8_CollisionFiltering ${LIBS})