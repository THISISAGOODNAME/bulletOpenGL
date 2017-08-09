//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_BASICDEMO_H
#define BULLETOPENGL_BASICDEMO_H

#include "BulletOpenGLApplication.h"
#include <btBulletDynamicsCommon.h>

class BasicDemo : public BulletOpenGLApplication {
public:
	virtual void InitializePhysics() override;
	virtual void ShutdownPhysics() override;
};


#endif //BULLETOPENGL_BASICDEMO_H
