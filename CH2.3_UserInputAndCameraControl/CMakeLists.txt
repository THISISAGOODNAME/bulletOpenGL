set(SOURCE_FILES main.cpp
        FreeGLUTCallbacks.h
        BulletOpenGLApplication.cpp
        BulletOpenGLApplication.h)
add_executable(CH2.3_UserInputAndCameraControl ${SOURCE_FILES})
target_link_libraries(CH2.3_UserInputAndCameraControl ${LIBS})