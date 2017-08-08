set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h)
add_executable(CH1.2_TheApplicationLayer ${SOURCE_FILES})
target_link_libraries(CH1.2_TheApplicationLayer ${LIBS})