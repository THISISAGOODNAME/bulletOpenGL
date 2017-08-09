//
// Created by AICDG on 2017/8/9.
//

//#include "BasicDemo.h"
#include "CollisionFilteringDemo.h"
#include "FreeGLUTCallbacks.h"

int main(int argc, char** argv) {
//    BasicDemo demo;
    CollisionFilteringDemo demo;
    return glutmain(argc, argv, 800, 600, "bullet and opengl", &demo);
}