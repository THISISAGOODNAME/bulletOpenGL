//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_BASICDEMO_H
#define BULLETOPENGL_BASICDEMO_H

#include "BulletOpenGLApplication.h"
#include <btBulletDynamicsCommon.h>

#define EXPLOSION_STRENGTH 50.0f

class BasicDemo : public BulletOpenGLApplication {
public:
	BasicDemo();
	
	virtual void InitializePhysics() override;
	virtual void ShutdownPhysics() override;

	void CreateObjects();

	virtual void Keyboard(unsigned char key, int x, int y) override;
	virtual void KeyboardUp(unsigned char key, int x, int y) override;
	virtual void UpdateScene(float dt);

	virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1) override;

protected:
	// our box to lift
	GameObject* m_pBox;

	// a simple trigger volume
	btCollisionObject* m_pTrigger;

	// keeps track of whether we're holding down the 'g' key
	bool m_bApplyForce;

	// explosion variables
	btCollisionObject* m_pExplosion;
	bool m_bCanExplode;
};


#endif //BULLETOPENGL_BASICDEMO_H
