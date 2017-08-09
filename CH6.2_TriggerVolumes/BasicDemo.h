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

	void CreateObjects();

	virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1) override;

protected:
	// our box to lift
	GameObject* m_pBox;

	// a simple trigger volume
	btCollisionObject* m_pTrigger;
};


#endif //BULLETOPENGL_BASICDEMO_H
