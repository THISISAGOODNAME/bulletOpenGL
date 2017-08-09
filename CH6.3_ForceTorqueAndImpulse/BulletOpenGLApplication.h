//
// Created by AICDG on 2017/8/9.
//

#ifndef BULLETOPENGL_BULLETOPENGLAPPLICATION_H
#define BULLETOPENGL_BULLETOPENGLAPPLICATION_H


#if defined(__APPLE__)
# include <OpenGL/gl.h>
# include <OpenGL/glu.h>
#else
# include <GL/gl.h>
# include <GL/glu.h>
#endif
#include <GL/freeglut.h>

#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

// include our custom Motion State object
#include "OpenGLMotionState.h"

#include "GameObject.h"
#include <vector>
#include <set>
#include <iterator>
#include <algorithm>

// Our custom debug renderer
#include "DebugDrawer.h"

// a convenient typedef to reference an STL vector of GameObjects
typedef std::vector<GameObject*> GameObjects;

// convenient typedefs for collision events
typedef std::pair<const btRigidBody*, const btRigidBody*> CollisionPair;
typedef std::set<CollisionPair> CollisionPairs;

// struct to store our raycasting results
struct RayResult {
	btRigidBody* pBody;
	btVector3 hitPoint;
};

class BulletOpenGLApplication {
public:
	BulletOpenGLApplication();
	~BulletOpenGLApplication();
	void Initialize();
	// FreeGLUT callbacks //
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void KeyboardUp(unsigned char key, int x, int y);
	virtual void Special(int key, int x, int y);
	virtual void SpecialUp(int key, int x, int y);
	virtual void Reshape(int w, int h);
	virtual void Idle();
	virtual void Mouse(int button, int state, int x, int y);
	virtual void PassiveMotion(int x, int y);
	virtual void Motion(int x, int y);
	virtual void Display();

	// rendering. Can be overrideen by derived classes
	virtual void RenderScene();

	// scene updating. Can be overridden by derived classes
	virtual void UpdateScene(float dt);

	// physics functions. Can be overriden by derived classes (like BasicDemo)
	virtual void InitializePhysics() {};
	virtual void ShutdownPhysics() {};
	
	// camera functions
	void UpdateCamera();
	void RotateCamera(float &angle, float value);
	void ZoomCamera(float distance);

	// drawing functions
	void DrawBox(const btVector3 &halfSize);
	void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color);

	// object functions
	GameObject* CreateGameObject(btCollisionShape* pShape,
			const float &mass,
			const btVector3 &color = btVector3(1.0f,1.0f,1.0f),
			const btVector3 &initialPosition = btVector3(0.0f,0.0f,0.0f),
			const btQuaternion &initialRotation = btQuaternion(0,0,1,1));

	void ShootBox(const btVector3 &direction);
	void DestroyGameObject(btRigidBody* pBody);
	GameObject* FindGameObject(btRigidBody* pBody);

	// picking functions
	btVector3 GetPickingRay(int x, int y);
	bool Raycast(const btVector3 &startPosition, const btVector3 &direction, RayResult &output, bool includeStatic = false);

	// constraint functions
	void CreatePickingConstraint(int x, int y);
	void RemovePickingConstraint();


	// collision event functions
	void CheckForCollisionEvents();
	virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody * pBody1);
	virtual void SeparationEvent(btRigidBody * pBody0, btRigidBody * pBody1);

protected:
	// camera control
	btVector3 m_cameraPosition; // the camera's current position
	btVector3 m_cameraTarget;	 // the camera's lookAt target
	float m_nearPlane; // minimum distance the camera will render
	float m_farPlane; // farthest distance the camera will render
	btVector3 m_upVector; // keeps the camera rotated correctly
	float m_cameraDistance; // distance from the camera to its target
	float m_cameraPitch; // pitch of the camera 
	float m_cameraYaw; // yaw of the camera

	int m_screenWidth;
	int m_screenHeight;

	// core Bullet components
	btBroadphaseInterface* m_pBroadphase;
	btCollisionConfiguration* m_pCollisionConfiguration;
	btCollisionDispatcher* m_pDispatcher;
	btConstraintSolver* m_pSolver;
	btDynamicsWorld* m_pWorld;

	// a simple clock for counting time
	btClock m_clock;

	// an array of our game objects
	GameObjects m_objects;

	// debug renderer
	DebugDrawer* m_pDebugDrawer;

	// constraint variables
	btRigidBody* m_pPickedBody;				// the body we picked up
	btTypedConstraint*  m_pPickConstraint;	// the constraint the body is attached to
	btScalar m_oldPickingDist;				// the distance from the camera to the hit point (so we can move the object up, down, left and right from our view)

	// collision event variables
	CollisionPairs m_pairsLastUpdate;
};


#endif //BULLETOPENGL_BULLETOPENGLAPPLICATION_H
