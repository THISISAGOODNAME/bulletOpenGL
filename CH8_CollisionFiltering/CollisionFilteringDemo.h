//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_COLLISIONFILTERINGDEMO_H
#define BULLETOPENGL_COLLISIONFILTERINGDEMO_H


#include "BulletOpenGLApplication.h"

#include <btBulletDynamicsCommon.h>

class CollisionFilteringDemo : public BulletOpenGLApplication {
public:
	CollisionFilteringDemo();

	virtual void InitializePhysics() override;
	virtual void ShutdownPhysics() override;
	void CreateObjects();
};


#endif //BULLETOPENGL_COLLISIONFILTERINGDEMO_H
