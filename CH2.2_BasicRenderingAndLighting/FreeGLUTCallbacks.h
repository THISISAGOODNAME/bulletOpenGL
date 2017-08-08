//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_FREEGLUTCALLBACKS_H
#define BULLETOPENGL_FREEGLUTCALLBACKS_H

#include "BulletOpenGLApplication.h"

// global pointer to our application object
static BulletOpenGLApplication* g_pApp;

// Various static functions that will be handed to FreeGLUT to be called
// during various events (our callbacks). Each calls an equivalent function
// in our (global) application object.
static void KeyboardCallback(unsigned char key, int x, int y) {
	g_pApp->Keyboard(key, x, y);
}
static void KeyboardUpCallback(unsigned char key, int x, int y) {
	g_pApp->KeyboardUp(key, x, y);
}
static void SpecialCallback(int key, int x, int y) {
	g_pApp->Special(key, x, y);
}
static void SpecialUpCallback(int key, int x, int y) {
	g_pApp->SpecialUp(key, x, y);
}
static void ReshapeCallback(int w, int h) {
	g_pApp->Reshape(w, h);
}
static void IdleCallback() {
	g_pApp->Idle();
}
static void MouseCallback(int button, int state, int x, int y) {
	g_pApp->Mouse(button, state, x, y);
}
static void MotionCallback(int x,int y) {
	g_pApp->Motion(x, y);
}
static void DisplayCallback(void) {
	g_pApp->Display();
}


// our custom-built 'main' function, which accepts a reference to a
// BulletOpenGLApplication object.
int glutmain(int argc, char **argv, int width, int height, const char* title, BulletOpenGLApplication* pApp) {
    // store the application object so we can
	// access it globally
	g_pApp = pApp;

	// initialize the window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(width, height);
	glutCreateWindow(title);
	glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	// perform custom initialization our of application
	g_pApp->Initialize();

	// give our static
	glutKeyboardFunc(KeyboardCallback);
	glutKeyboardUpFunc(KeyboardUpCallback);
	glutSpecialFunc(SpecialCallback);
	glutSpecialUpFunc(SpecialUpCallback);
	glutReshapeFunc(ReshapeCallback);
	glutIdleFunc(IdleCallback);
	glutMouseFunc(MouseCallback);
	glutPassiveMotionFunc(MotionCallback);
	glutMotionFunc(MotionCallback);
	glutDisplayFunc(DisplayCallback );

	// perform one render before we launch the application
	g_pApp->Idle();

	// hand application control over to the FreeGLUT library.
	// This function remains in a while-loop until the
	// application is exited.
	glutMainLoop();
	return 0;
}

#endif //BULLETOPENGL_FREEGLUTCALLBACKS_H
