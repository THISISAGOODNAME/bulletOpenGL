set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h
        BasicDemo.cpp
        BasicDemo.h)
add_executable(CH3.1_TheCoreBulletObjects ${SOURCE_FILES})
target_link_libraries(CH3.1_TheCoreBulletObjects ${LIBS})