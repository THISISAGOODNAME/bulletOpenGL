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
add_executable(CH6.2_TriggerVolumes ${SOURCE_FILES})
target_link_libraries(CH6.2_TriggerVolumes ${LIBS})