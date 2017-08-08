set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h)
add_executable(CH2.2_BasicRenderingAndLighting ${SOURCE_FILES})
target_link_libraries(CH2.2_BasicRenderingAndLighting ${LIBS})