set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h)
add_executable(CH2.1_RenderingTheScene ${SOURCE_FILES})
target_link_libraries(CH2.1_RenderingTheScene ${LIBS})