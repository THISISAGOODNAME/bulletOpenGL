set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h
        OpenGLMotionState.h
        GameObject.cpp
        GameObject.h
        DebugDrawer.cpp
        DebugDrawer.h
        SoftBodyDemo.cpp
        SoftBodyDemo.h)
LIST(APPEND LIBS BulletSoftBody)
add_executable(CH9_SoftBodyDynamics ${SOURCE_FILES})
target_link_libraries(CH9_SoftBodyDynamics ${LIBS})