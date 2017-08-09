//
// Created by AICDG on 2017/8/9.
//

#include "BasicDemo.h"

void BasicDemo::InitializePhysics() {
	// create the collision configuration
	m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
	// create the dispatcher
	m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
	// create the broadphase
	m_pBroadphase = new btDbvtBroadphase();
	// create the constraint solver
	m_pSolver = new btSequentialImpulseConstraintSolver();
	// create the world
	m_pWorld = new btDiscreteDynamicsWorld(m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);

	// create our scene's physics objects
	CreateObjects();
}

void BasicDemo::ShutdownPhysics() {
	delete m_pWorld;
	delete m_pSolver;
	delete m_pBroadphase;
	delete m_pDispatcher;
	delete m_pCollisionConfiguration;
}

void BasicDemo::CreateObjects() {
	// create a box shape of size (1,1,1)
	btBoxShape* pBoxShape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
	// give our box an initial position of (0,0,0)
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(0.0f, 0.0f, 0.0f));
	// create a motion state
	m_pMotionState = new OpenGLMotionState(transform);
	// create the rigid body construction info object, giving it a 
	// mass of 1, the motion state, and the shape
	btRigidBody::btRigidBodyConstructionInfo rbInfo(1.0f, m_pMotionState, pBoxShape);
	btRigidBody* pRigidBody = new btRigidBody(rbInfo);
	// inform our world that we just created a new rigid body for 
	// it to manage
	m_pWorld->addRigidBody(pRigidBody);
}