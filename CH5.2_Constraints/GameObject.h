//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_GAMEOBJECT_H
#define BULLETOPENGL_GAMEOBJECT_H

#include <btBulletDynamicsCommon.h>
#include "OpenGLMotionState.h"


class GameObject {
public:
	GameObject(btCollisionShape* pShape, float mass, const btVector3 &color, const btVector3 &initialPosition = btVector3(0,0,0), const btQuaternion &initialRotation = btQuaternion(0,0,1,1));
	~GameObject();

	// accessors
	btCollisionShape* GetShape() { return m_pShape; }

	btRigidBody* GetRigidBody() { return m_pBody; }

	btMotionState* GetMotionState() { return m_pMotionState; }

	void GetTransform(btScalar* transform) {
		if (m_pMotionState) m_pMotionState->GetWorldTransform(transform);
	}

	btVector3 GetColor() { return m_color; }

protected:
	btCollisionShape*  m_pShape;
	btRigidBody*    m_pBody;
	OpenGLMotionState*  m_pMotionState;
	btVector3      m_color;
};


#endif //BULLETOPENGL_GAMEOBJECT_H
