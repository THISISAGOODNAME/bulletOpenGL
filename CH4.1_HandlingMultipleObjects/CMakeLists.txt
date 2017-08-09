set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h
        BasicDemo.cpp
        BasicDemo.h
        OpenGLMotionState.h GameObject.cpp GameObject.h)
add_executable(CH4.1_HandlingMultipleObject ${SOURCE_FILES})
target_link_libraries(CH4.1_HandlingMultipleObject ${LIBS})