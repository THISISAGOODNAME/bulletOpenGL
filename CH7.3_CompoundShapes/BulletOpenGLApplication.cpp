//
// Created by AICDG on 2017/8/9.
//

#include "BulletOpenGLApplication.h"
#include <iostream>

// Some constants for 3D math and the camera speed
#define RADIANS_PER_DEGREE 0.01745329f
#define CAMERA_STEP_SIZE 5.0f

BulletOpenGLApplication::BulletOpenGLApplication() :
        m_cameraPosition(10.0f, 5.0f, 0.0f),
    	m_cameraTarget(0.0f, 0.0f, 0.0f),
		m_cameraDistance(15.0f),
		m_cameraPitch(20.0f),
		m_cameraYaw(0.0f),
    	m_upVector(0.0f, 1.0f, 0.0f),
    	m_nearPlane(1.0f),
    	m_farPlane(1000.0f),
		m_pBroadphase(nullptr),
		m_pCollisionConfiguration(nullptr),
		m_pDispatcher(nullptr),
		m_pSolver(nullptr),
		m_pWorld(nullptr),
		m_pPickedBody(nullptr),
		m_pPickConstraint(nullptr)
{

}

BulletOpenGLApplication::~BulletOpenGLApplication() {
	// shutdown the physics system
	ShutdownPhysics();
}

void BulletOpenGLApplication::Initialize() {
	// this function is called inside glutmain() after
	// creating the window, but before handing control
	// to FreeGLUT

	// create some floats for our ambient, diffuse, specular and position
	GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f }; // dark grey
	GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
	GLfloat position[] = { 5.0f, 10.0f, 1.0f, 0.0f };

	// set the ambient, diffuse, specular and position for LIGHT0
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glEnable(GL_LIGHTING); // enables lighting
	glEnable(GL_LIGHT0); // enables the 0th light
	glEnable(GL_COLOR_MATERIAL); // colors materials when lighting is enabled

	// enable specular lighting via materials
	glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
	glMateriali(GL_FRONT, GL_SHININESS, 15);

	// enable smooth shading
	glShadeModel(GL_SMOOTH);

	// enable depth testing to be 'less than'
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	// set the backbuffer clearing color to a lightish blue
	glClearColor(0.6, 0.65, 0.85, 0);

	// initialize the physics system
	InitializePhysics();

	// create the debug drawer
	m_pDebugDrawer = new DebugDrawer();
	// set the initial debug level to 0
	m_pDebugDrawer->setDebugMode(0);
	// add the debug drawer to the world
	m_pWorld->setDebugDrawer(m_pDebugDrawer);
}

void BulletOpenGLApplication::Keyboard(unsigned char key, int x, int y) {
	// This function is called by FreeGLUT whenever
	// generic keys are pressed down.
	switch(key) {
		// 'z' zooms in
	case 'z': ZoomCamera(+CAMERA_STEP_SIZE); break;
		// 'x' zoom out
	case 'x': ZoomCamera(-CAMERA_STEP_SIZE); break;
	case 'w':
		// toggle wireframe debug drawing
		m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawWireframe);
		break;
	case 'b':
		// toggle AABB debug drawing
		m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawAabb);
		break;
	case 'd':
		// create a temp object to store the raycast result
		RayResult result;
		// perform the raycast
		if (!Raycast(m_cameraPosition, GetPickingRay(x, y), result))
			return; // return if the test failed
		// destroy the corresponding game object
		DestroyGameObject(result.pBody);
		break;

	}
}

void BulletOpenGLApplication::KeyboardUp(unsigned char key, int x, int y) {}

void BulletOpenGLApplication::Special(int key, int x, int y) {
	// This function is called by FreeGLUT whenever special keys
	// are pressed down, like the arrow keys, or Insert, Delete etc.
	switch(key) {
		// the arrow keys rotate the camera up/down/left/right
	case GLUT_KEY_LEFT:
		RotateCamera(m_cameraYaw, +CAMERA_STEP_SIZE); break;
	case GLUT_KEY_RIGHT:
		RotateCamera(m_cameraYaw, -CAMERA_STEP_SIZE); break;
	case GLUT_KEY_UP:
		RotateCamera(m_cameraPitch, +CAMERA_STEP_SIZE); break;
	case GLUT_KEY_DOWN:
		RotateCamera(m_cameraPitch, -CAMERA_STEP_SIZE); break;
	}	
}

void BulletOpenGLApplication::SpecialUp(int key, int x, int y) {}

void BulletOpenGLApplication::Reshape(int w, int h) {
    // this function is called once during application intialization
	// and again every time we resize the window

	// grab the screen width/height
	m_screenWidth = w;
	m_screenHeight = h;
	// set the viewport
	glViewport(0, 0, w, h);
	// update the camera
	UpdateCamera();
}

void BulletOpenGLApplication::Idle() {
	// this function is called frequently, whenever FreeGlut
	// isn't busy processing its own events. It should be used
	// to perform any updating and rendering tasks

	// clear the backbuffer
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// get the time since the last iteration
	float dt = m_clock.getTimeMilliseconds();
	// reset the clock to 0
	m_clock.reset();
	// update the scene (convert ms to s)
	UpdateScene(dt / 1000.0f);

	// update the camera
	UpdateCamera();

	// render the scene
	RenderScene();
	
	// swap the front and back buffers
	glutSwapBuffers();
}

void BulletOpenGLApplication::Mouse(int button, int state, int x, int y) {
	switch(button) {
	case 0:  // left mouse button
		{
			if (state == 0) { // button down
				// create the picking constraint when we click the LMB
				CreatePickingConstraint(x, y);
			} else { // button up
				// remove the picking constraint when we release the LMB
				RemovePickingConstraint();
			}
			break;
		}
	case 2: // right mouse button
		{
			if (state == 0) { // pressed down
				// shoot a box
				ShootBox(GetPickingRay(x, y));
			}

			break;
		}
	}
}

void BulletOpenGLApplication::PassiveMotion(int x, int y) {}
void BulletOpenGLApplication::Motion(int x, int y) {
		// did we pick a body with the LMB?
	if (m_pPickedBody) {
		btGeneric6DofConstraint* pickCon = static_cast<btGeneric6DofConstraint*>(m_pPickConstraint);
		if (!pickCon)
			return;

		// use another picking ray to get the target direction
		btVector3 dir = GetPickingRay(x,y) - m_cameraPosition;
		dir.normalize();

		// use the same distance as when we originally picked the object
		dir *= m_oldPickingDist;
		btVector3 newPivot = m_cameraPosition + dir;

		// set the position of the constraint
		pickCon->getFrameOffsetA().setOrigin(newPivot);
	}
}

void BulletOpenGLApplication::Display() {}

void BulletOpenGLApplication::UpdateCamera() {
	// exit in erroneous situations
	if (m_screenWidth == 0 && m_screenHeight == 0)
		return;

	// select the projection matrix
	glMatrixMode(GL_PROJECTION);
	// set it to the matrix-equivalent of 1
	glLoadIdentity();
	// determine the aspect ratio of the screen
	float aspectRatio = m_screenWidth / (float)m_screenHeight;
	// create a viewing frustum based on the aspect ratio and the
	// boundaries of the camera
	glFrustum (-aspectRatio * m_nearPlane, aspectRatio * m_nearPlane, -m_nearPlane, m_nearPlane, m_nearPlane, m_farPlane);
	// the projection matrix is now set

	// select the view matrix
	glMatrixMode(GL_MODELVIEW);
	// set it to '1'
	glLoadIdentity();

	// our values represent the angles in degrees, but 3D
	// math typically demands angular values are in radians.
	float pitch = m_cameraPitch * RADIANS_PER_DEGREE;
	float yaw = m_cameraYaw * RADIANS_PER_DEGREE;

	// create a quaternion defining the angular rotation
	// around the up vector
	btQuaternion rotation(m_upVector, yaw);

	// set the camera's position to 0,0,0, then move the 'z'
	// position to the current value of m_cameraDistance.
	btVector3 cameraPosition(0,0,0);
	cameraPosition[2] = -m_cameraDistance;

	// create a Bullet Vector3 to represent the camera
	// position and scale it up if its value is too small.
	btVector3 forward(cameraPosition[0], cameraPosition[1], cameraPosition[2]);
	if (forward.length2() < SIMD_EPSILON) {
		forward.setValue(1.f,0.f,0.f);
	}

	// figure out the 'right' vector by using the cross
	// product on the 'forward' and 'up' vectors
	btVector3 right = m_upVector.cross(forward);

	// create a quaternion that represents the camera's roll
	btQuaternion roll(right, - pitch);

	// turn the rotation (around the Y-axis) and roll (around
	// the forward axis) into transformation matrices and 
	// apply them to the camera position. This gives us the 
	// final position
	cameraPosition = btMatrix3x3(rotation) * btMatrix3x3(roll) * cameraPosition;

	// save our new position in the member variable, and
	// shift it relative to the target position (so that we 
	// orbit it)
	m_cameraPosition[0] = cameraPosition.getX();
	m_cameraPosition[1] = cameraPosition.getY();
	m_cameraPosition[2] = cameraPosition.getZ();
	m_cameraPosition += m_cameraTarget;
	
	// create a view matrix based on the camera's position and where it's
	// looking
	gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], m_cameraTarget[0], m_cameraTarget[1], m_cameraTarget[2], m_upVector.getX(), m_upVector.getY(), m_upVector.getZ());
	// the view matrix is now set
}

void BulletOpenGLApplication::DrawBox(const btVector3 &halfSize) {

	float halfWidth = halfSize.x();
	float halfHeight = halfSize.y();
	float halfDepth = halfSize.z();

	// create the vertex positions
	btVector3 vertices[8]={
	btVector3( halfWidth,  halfHeight,  halfDepth),
	btVector3(-halfWidth,  halfHeight,  halfDepth),
	btVector3( halfWidth, -halfHeight,  halfDepth),
	btVector3(-halfWidth, -halfHeight,  halfDepth),
	btVector3( halfWidth,  halfHeight, -halfDepth),
	btVector3(-halfWidth,  halfHeight, -halfDepth),
	btVector3( halfWidth, -halfHeight, -halfDepth),
	btVector3(-halfWidth, -halfHeight, -halfDepth)};

	// create the indexes for each triangle, using the
	// vertices above. Make it static so we don't waste 
	// processing time recreating it over and over again
	static int indices[36] = {
		0,1,2,
		3,2,1,
		4,0,6,
		6,0,2,
		5,1,4,
		4,1,0,
		7,3,1,
		7,1,5,
		5,4,7,
		7,4,6,
		7,2,3,
		7,6,2};

	// start processing vertices as triangles
	glBegin (GL_TRIANGLES);

	// increment the loop by 3 each time since we create a
	// triangle with 3 vertices at a time.

	for (int i = 0; i < 36; i += 3) {
		// get the three vertices for the triangle based
		// on the index values set above
		// use const references so we don't copy the object
		// (a good rule of thumb is to never allocate/deallocate
		// memory during *every* render/update call. This should 
		// only happen sporadically)
		const btVector3 &vert1 = vertices[indices[i]];
		const btVector3 &vert2 = vertices[indices[i+1]];
		const btVector3 &vert3 = vertices[indices[i+2]];

		// create a normal that is perpendicular to the
		// face (use the cross product)
		btVector3 normal = (vert3-vert1).cross(vert2-vert1);
		normal.normalize ();

		// set the normal for the subsequent vertices
		glNormal3f(normal.getX(),normal.getY(),normal.getZ());

		// create the vertices
		glVertex3f (vert1.x(), vert1.y(), vert1.z());
		glVertex3f (vert2.x(), vert2.y(), vert2.z());
		glVertex3f (vert3.x(), vert3.y(), vert3.z());
	}

	// stop processing vertices
	glEnd();
}

void BulletOpenGLApplication::RotateCamera(float &angle, float value) {
	// change the value (it is passed by reference, so we
	// can edit it here)
	angle -= value;
	// keep the value within bounds
	if (angle < 0) angle += 360;
	if (angle >= 360) angle -= 360;
	// update the camera since we changed the angular value
	UpdateCamera();
}

void BulletOpenGLApplication::ZoomCamera(float distance) {
	// change the distance value
	m_cameraDistance -= distance;
	// prevent it from zooming in too far
	if (m_cameraDistance < 0.1f) m_cameraDistance = 0.1f;
	// update the camera since we changed the zoom distance
	UpdateCamera();
}

void BulletOpenGLApplication::RenderScene() {
	// create an array of 16 floats (representing a 4x4 matrix)
	btScalar transform[16];

	// iterate through all of the objects in our world
	for (GameObjects::iterator i = m_objects.begin(); i != m_objects.end(); ++i) {
		// get the object from the iterator
		GameObject* pObj = *i;

		// read the transform
		pObj->GetTransform(transform);

		// get data from the object and draw it
		DrawShape(transform, pObj->GetShape(), pObj->GetColor());
	}

	// after rendering all game objects, perform debug rendering
	// Bullet will figure out what needs to be drawn then call to
	// our DebugDrawer class to do the rendering for us
	m_pWorld->debugDrawWorld();
}

void BulletOpenGLApplication::UpdateScene(float dt) {
	// check if the world object exists
	if (m_pWorld) {
		// step the simulation through time. This is called
		// every update and the amount of elasped time was 
		// determined back in ::Idle() by our clock object.
		m_pWorld->stepSimulation(dt);

		// check for any new collisions/separations
		CheckForCollisionEvents();
	}
}

void BulletOpenGLApplication::DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color) {
	// set the color
	glColor3f(color.x(), color.y(), color.z());

	// push the matrix stack
	glPushMatrix();
	glMultMatrixf(transform);

	// make a different draw call based on the object type
	switch(pShape->getShapeType()) {
		// an internal enum used by Bullet for boxes
	case BOX_SHAPE_PROXYTYPE:
	{
		// assume the shape is a box, and typecast it
		const btBoxShape* box = static_cast<const btBoxShape*>(pShape);
		// get the 'halfSize' of the box
		btVector3 halfSize = box->getHalfExtentsWithMargin();
		// draw the box
		DrawBox(halfSize);
		break;
	}
	case SPHERE_SHAPE_PROXYTYPE:
	{
		// assume the shape is a sphere and typecast it
		const btSphereShape* sphere = static_cast<const btSphereShape*>(pShape);
		// get the sphere's size from the shape
		float radius = sphere->getMargin();
		// draw the sphere
		DrawSphere(radius);
		break;
	}
	case CYLINDER_SHAPE_PROXYTYPE:
	{
		// assume the object is a cylinder
		const btCylinderShape* pCylinder = static_cast<const btCylinderShape*>(pShape);
		// get the relevant data
		float radius = pCylinder->getRadius();
		float halfHeight = pCylinder->getHalfExtentsWithMargin()[1];
		// draw the cylinder
		DrawCylinder(radius,halfHeight);

		break;
	}
	case CONVEX_HULL_SHAPE_PROXYTYPE:
	{
		// draw the convex hull shape...whatever it is
		DrawConvexHull(pShape);
		break;
	}
    case COMPOUND_SHAPE_PROXYTYPE:
	{
		// get the shape
		const btCompoundShape* pCompound = static_cast<const btCompoundShape*>(pShape);
		// iterate through the children
		for (int i = 0; i < pCompound->getNumChildShapes(); ++i) {
			// get the transform of the sub-shape
			btTransform thisTransform = pCompound->getChildTransform(i);
			btScalar thisMatrix[16];
			thisTransform.getOpenGLMatrix(thisMatrix);
			// call drawshape recursively for each child. The matrix
			// stack takes care of positioning/orienting the object for us
			DrawShape(thisMatrix, pCompound->getChildShape(i), color);
		}
		break;
	}
	default:
		// unsupported type
		break;
	}

	// pop the stack
	glPopMatrix();
}

GameObject* BulletOpenGLApplication::CreateGameObject(btCollisionShape* pShape, const float &mass, const btVector3 &color, const btVector3 &initialPosition, const btQuaternion &initialRotation) {
	// create a new game object
	GameObject* pObject = new GameObject(pShape, mass, color, initialPosition, initialRotation);

	// push it to the back of the list
	m_objects.push_back(pObject);

	// check if the world object is valid
	if (m_pWorld) {
		// add the object's rigid body to the world
		m_pWorld->addRigidBody(pObject->GetRigidBody());
	}
	return pObject;
}

btVector3 BulletOpenGLApplication::GetPickingRay(int x, int y) {
	// calculate the field-of-view
	float tanFov = 1.0f / m_nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	// get a ray pointing forward from the
	// camera and extend it to the far plane
	btVector3 rayFrom = m_cameraPosition;
	btVector3 rayForward = (m_cameraTarget - m_cameraPosition);
	rayForward.normalize();
	rayForward*= m_farPlane;

	// find the horizontal and vertical vectors
	// relative to the current camera view
	btVector3 ver = m_upVector;
	btVector3 hor = rayForward.cross(ver);
	hor.normalize();
	ver = hor.cross(rayForward);
	ver.normalize();
	hor *= 2.f * m_farPlane * tanFov;
	ver *= 2.f * m_farPlane * tanFov;

	// calculate the aspect ratio
	btScalar aspect = m_screenWidth / (btScalar)m_screenHeight;

	// adjust the forward-ray based on
	// the X/Y coordinates that were clicked
	hor*=aspect;
	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/float(m_screenWidth);
	btVector3 dVert = ver * 1.f/float(m_screenHeight);
	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * ver;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;

	// return the final result
	return rayTo;
}

void BulletOpenGLApplication::ShootBox(const btVector3 &direction) {
	// create a new box object
	GameObject* pObject = CreateGameObject(new btBoxShape(btVector3(1, 1, 1)), 1, btVector3(0.4f, 0.f, 0.4f), m_cameraPosition);

	// calculate the velocity
	btVector3 velocity = direction;
	velocity.normalize();
	velocity *= 25.0f;

	// set the linear velocity of the box
	pObject->GetRigidBody()->setLinearVelocity(velocity);
}

bool BulletOpenGLApplication::Raycast(const btVector3 &startPosition, const btVector3 &direction, RayResult &output, bool includeStatic) {
	if (!m_pWorld)
		return false;

	// get the picking ray from where we clicked
	btVector3 rayTo = direction;
	btVector3 rayFrom = m_cameraPosition;

	// create our raycast callback object
	btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);

	// perform the raycast
	m_pWorld->rayTest(rayFrom,rayTo,rayCallback);

	// did we hit something?
	if (rayCallback.hasHit())
	{
		// if so, get the rigid body we hit
		btRigidBody* pBody = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
		if (!pBody)
			return false;

		// prevent us from picking objects
		// like the ground plane
		if (!includeStatic) // skip this check if we want it to hit static objects
			if (pBody->isStaticObject() || pBody->isKinematicObject())
				return false;

		// set the result data
		output.pBody = pBody;
		output.hitPoint = rayCallback.m_hitPointWorld;
		return true;
	}

	// we didn't hit anything
	return false;
}

void BulletOpenGLApplication::DestroyGameObject(btRigidBody* pBody) {
	// we need to search through the objects in order to 
	// find the corresponding iterator (can only erase from 
	// an std::vector by passing an iterator)
	for (GameObjects::iterator iter = m_objects.begin(); iter != m_objects.end(); ++iter) {
		if ((*iter)->GetRigidBody() == pBody) {
			GameObject* pObject = *iter;
			// remove the rigid body from the world
			m_pWorld->removeRigidBody(pObject->GetRigidBody());
			// erase the object from the list
			m_objects.erase(iter);
			// delete the object from memory
			delete pObject;
			// done
			return;
		}
	}
}

void BulletOpenGLApplication::CreatePickingConstraint(int x, int y) {
	if (!m_pWorld)
		return;

	// perform a raycast and return if it fails
	RayResult output;
	if (!Raycast(m_cameraPosition, GetPickingRay(x, y), output))
		return;

	// store the body for future reference
	m_pPickedBody = output.pBody;

	// prevent the picked object from falling asleep
	m_pPickedBody->setActivationState(DISABLE_DEACTIVATION);

	// get the hit position relative to the body we hit
	btVector3 localPivot = m_pPickedBody->getCenterOfMassTransform().inverse() * output.hitPoint;

	// create a transform for the pivot point
	btTransform pivot;
	pivot.setIdentity();
	pivot.setOrigin(localPivot);

	// create our constraint object
	btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*m_pPickedBody, pivot, true);
	bool bLimitAngularMotion = true;
	if (bLimitAngularMotion) {
		dof6->setAngularLowerLimit(btVector3(0,0,0));
		dof6->setAngularUpperLimit(btVector3(0,0,0));
	}

	// add the constraint to the world
	m_pWorld->addConstraint(dof6,true);

	// store a pointer to our constraint
	m_pPickConstraint = dof6;

	// define the 'strength' of our constraint (each axis)
	float cfm = 0.5f;
	dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,0);
	dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,1);
	dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,2);
	dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,3);
	dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,4);
	dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,5);

	// define the 'error reduction' of our constraint (each axis)
	float erp = 0.5f;
	dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,0);
	dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,1);
	dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,2);
	dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,3);
	dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,4);
	dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,5);

	// save this data for future reference
	m_oldPickingDist  = (output.hitPoint - m_cameraPosition).length();
}

void BulletOpenGLApplication::RemovePickingConstraint() {
	// exit in erroneous situations
	if (!m_pPickConstraint || !m_pWorld)
		return;

	// remove the constraint from the world
	m_pWorld->removeConstraint(m_pPickConstraint);

	// delete the constraint object
	delete m_pPickConstraint;

	// reactivate the body
	m_pPickedBody->forceActivationState(ACTIVE_TAG);
	m_pPickedBody->setDeactivationTime( 0.f );

	// clear the pointers
	m_pPickConstraint = 0;
	m_pPickedBody = 0;
}

void BulletOpenGLApplication::CheckForCollisionEvents() {
	// keep a list of the collision pairs we
	// found during the current update
	CollisionPairs pairsThisUpdate;

	// iterate through all of the manifolds in the dispatcher
	for (int i = 0; i < m_pDispatcher->getNumManifolds(); ++i) {
		// get the manifold
		btPersistentManifold* pManifold = m_pDispatcher->getManifoldByIndexInternal(i);
		
		// ignore manifolds that have 
		// no contact points.
		if (pManifold->getNumContacts() > 0) {
			// get the two rigid bodies involved in the collision
			const btRigidBody* pBody0 = static_cast<const btRigidBody*>(pManifold->getBody0());
			const btRigidBody* pBody1 = static_cast<const btRigidBody*>(pManifold->getBody1());
			
			// always create the pair in a predictable order
			// (use the pointer value..)
			bool const swapped = pBody0 > pBody1;
			const btRigidBody* pSortedBodyA = swapped ? pBody1 : pBody0;
			const btRigidBody* pSortedBodyB = swapped ? pBody0 : pBody1;
			
			// create the pair
			CollisionPair thisPair = std::make_pair(pSortedBodyA, pSortedBodyB);
			
			// insert the pair into the current list
			pairsThisUpdate.insert(thisPair);
			
			// if this pair doesn't exist in the list
			// from the previous update, it is a new
			// pair and we must send a collision event
			if (m_pairsLastUpdate.find(thisPair) == m_pairsLastUpdate.end()) {
				CollisionEvent((btRigidBody*)pBody0, (btRigidBody*)pBody1);
			}
		}
	}
	
	// create another list for pairs that
	// were removed this update
	CollisionPairs removedPairs;
	
	// this handy function gets the difference beween
	// two sets. It takes the difference between
	// collision pairs from the last update, and this 
	// update and pushes them into the removed pairs list
	std::set_difference( m_pairsLastUpdate.begin(), m_pairsLastUpdate.end(),
	pairsThisUpdate.begin(), pairsThisUpdate.end(),
	std::inserter(removedPairs, removedPairs.begin()));
	
	// iterate through all of the removed pairs
	// sending separation events for them
	for (CollisionPairs::const_iterator iter = removedPairs.begin(); iter != removedPairs.end(); ++iter) {
		SeparationEvent((btRigidBody*)iter->first, (btRigidBody*)iter->second);
	}
	
	// in the next iteration we'll want to
	// compare against the pairs we found
	// in this iteration
	m_pairsLastUpdate = pairsThisUpdate;
}

void BulletOpenGLApplication::CollisionEvent(btRigidBody * pBody0, btRigidBody * pBody1) {
//	// find the two colliding objects
//	GameObject* pObj0 = FindGameObject(pBody0);
//	GameObject* pObj1 = FindGameObject(pBody1);
//
//	// exit if we didn't find anything
//	if (!pObj0 || !pObj1) return;
//
//	// set their colors to white
//	pObj0->SetColor(btVector3(1.0,1.0,1.0));
//	pObj1->SetColor(btVector3(1.0,1.0,1.0));
}

void BulletOpenGLApplication::SeparationEvent(btRigidBody * pBody0, btRigidBody * pBody1) {
//	// get the two separating objects
//	GameObject* pObj0 = FindGameObject((btRigidBody*)pBody0);
//	GameObject* pObj1 = FindGameObject((btRigidBody*)pBody1);
//
//	// exit if we didn't find anything
//	if (!pObj0 || !pObj1) return;
//
//	// set their colors to black
//	pObj0->SetColor(btVector3(0.0,0.0,0.0));
//	pObj1->SetColor(btVector3(0.0,0.0,0.0));
}

GameObject* BulletOpenGLApplication::FindGameObject(btRigidBody* pBody) {
	// search through our list of gameobjects finding
	// the one with a rigid body that matches the given one
	for (GameObjects::iterator iter = m_objects.begin(); iter != m_objects.end(); ++iter) {
		if ((*iter)->GetRigidBody() == pBody) {
			// found the body, so return the corresponding game object
			return *iter;
		}
	}
	return 0;
}

void BulletOpenGLApplication::DrawSphere(const btScalar &radius) {
	// some constant values for more many segments to build the sphere from
	static int lateralSegments = 25;
	static int longitudinalSegments = 25;

	// iterate laterally
	for(int i = 0; i <= lateralSegments; i++) {
		// do a little math to find the angles of this segment
		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lateralSegments);
		btScalar z0  = radius*sin(lat0);
		btScalar zr0 =  radius*cos(lat0);

		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lateralSegments);
		btScalar z1 = radius*sin(lat1);
		btScalar zr1 = radius*cos(lat1);

		// start rendering strips of quads (polygons with 4 poins)
		glBegin(GL_QUAD_STRIP);
		// iterate longitudinally
		for(int j = 0; j <= longitudinalSegments; j++) {
			// determine the points of the quad from the lateral angles
			btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longitudinalSegments;
			btScalar x = cos(lng);
			btScalar y = sin(lng);
			// draw the normals and vertices for each point in the quad
			// since it is a STRIP of quads, we only need to add two points
			// each time to create a whole new quad

			// calculate the normal
			btVector3 normal = btVector3(x*zr0, y*zr0, z0);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the first vertex
			glVertex3f(x * zr0, y * zr0, z0);

			// calculate the next normal
			normal = btVector3(x*zr1, y*zr1, z1);
			normal.normalize();
			glNormal3f(normal.x(), normal.y(), normal.z());
			// create the second vertex
			glVertex3f(x * zr1, y * zr1, z1);

			// and repeat...
		}
		glEnd();
	}
}

void BulletOpenGLApplication::DrawCylinder(const btScalar &radius, const btScalar &halfHeight) {
	static int slices = 15;
	static int stacks = 10;
	// tweak the starting position of the
	// cylinder to match the physics object
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glTranslatef(0.0, 0.0, -halfHeight);
	// create a quadric object to render with
	GLUquadricObj *quadObj = gluNewQuadric();
	// set the draw style of the quadric
	gluQuadricDrawStyle(quadObj, (GLenum)GLU_FILL);
	gluQuadricNormals(quadObj, (GLenum)GLU_SMOOTH);
	// create a disk to cap the cylinder
	gluDisk(quadObj, 0, radius, slices, stacks);
	// create the main hull of the cylinder (no caps)
	gluCylinder(quadObj, radius, radius, 2.f*halfHeight, slices, stacks);
	// shift the position and rotation again
	glTranslatef(0.0f, 0.0f, 2.f*halfHeight);
	glRotatef(-180.0f, 0.0f, 1.0f, 0.0f);
	// draw the cap on the other end of the cylinder
	gluDisk(quadObj, 0, radius, slices, stacks);
	// don't need the quadric anymore, so remove it
	// to save memory
	gluDeleteQuadric(quadObj);
}

void BulletOpenGLApplication::DrawConvexHull(const btCollisionShape* shape) {
	// get the polyhedral data from the convex hull
	const btConvexPolyhedron* pPoly = shape->isPolyhedral() ? ((btPolyhedralConvexShape*) shape)->getConvexPolyhedron() : 0;
	if (!pPoly) return;

	// begin drawing triangles
	glBegin (GL_TRIANGLES);

	// iterate through all faces
	for (int i = 0; i < pPoly->m_faces.size(); i++) {
		// get the indices for the face
		int numVerts = pPoly->m_faces[i].m_indices.size();
		if (numVerts>2)	{
			// iterate through all index triplets
			for (int v = 0; v <pPoly->m_faces[i].m_indices.size()-2;v++) {
				// grab the three vertices
				btVector3 v1 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[0]];
				btVector3 v2 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v+1]];
				btVector3 v3 = pPoly->m_vertices[pPoly->m_faces[i].m_indices[v+2]];
				// calculate the normal
				btVector3 normal = (v3-v1).cross(v2-v1);
				normal.normalize ();
				// draw the triangle
				glNormal3f(normal.getX(),normal.getY(),normal.getZ());
				glVertex3f (v1.x(), v1.y(), v1.z());
				glVertex3f (v2.x(), v2.y(), v2.z());
				glVertex3f (v3.x(), v3.y(), v3.z());
			}
		}
	}
	// done drawing
	glEnd ();
}