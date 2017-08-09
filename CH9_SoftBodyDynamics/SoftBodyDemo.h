//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_SOFTBODYDEMO_H
#define BULLETOPENGL_SOFTBODYDEMO_H

#include "BulletOpenGLApplication.h"

#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>

class SoftBodyDemo : public BulletOpenGLApplication {
public:
	SoftBodyDemo();

	virtual void InitializePhysics() override;
	virtual void ShutdownPhysics() override;
	virtual void RenderScene() override;
	void CreateObjects();

protected:
	// a pointer to our world, typecast into its soft body type
	btSoftRigidDynamicsWorld*  m_pSoftBodyWorld;

	// the soft body world info. Needed for proper contact generation
	btSoftBodyWorldInfo  m_softBodyWorldInfo;
};


#endif //BULLETOPENGL_SOFTBODYDEMO_H
