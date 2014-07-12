/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5


//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1
#define START_POS_X 0
#define START_POS_Y 10
#define START_POS_Z -10

#include "BasicDemo.h"
#include "GlutStuff.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "math.h"
#include <stdio.h> //printf debugging
#include "GLDebugDrawer.h"
#include "LinearMath/btAabbUtil2.h"


btVector3 changecoor(btVector3 t,btMatrix3x3 m)
{
	return btVector3(m.getColumn(0).dot(t),m.getColumn(1).dot(t),m.getColumn(2).dot(t));

}


double absa2(double x)
{
	if (x>0)
	  return x/2;
	else
	  return (-x)/2;
}
static GLDebugDrawer gDebugDraw;
btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;


btCollisionObject* ddd[100];
int ncount=0;


/***********************/
//int lflag=0;
/***********************/

int objlength;

struct my_obj
{
	int flag;
	float length;
};

///The MyOverlapCallback is used to show how to collect object that overlap with a given bounding box defined by aabbMin and aabbMax. 
///See m_dynamicsWorld->getBroadphase()->aabbTest.
struct	MyOverlapCallback : public btBroadphaseAabbCallback
{
	btVector3 m_queryAabbMin;
	btVector3 m_queryAabbMax;
	
	int m_numOverlap;
	MyOverlapCallback(const btVector3& aabbMin, const btVector3& aabbMax ) : m_queryAabbMin(aabbMin),m_queryAabbMax(aabbMax),m_numOverlap(0)	{}
	virtual bool	process(const btBroadphaseProxy* proxy)
	{
		btVector3 proxyAabbMin,proxyAabbMax;
		btCollisionObject* colObj0 = (btCollisionObject*)proxy->m_clientObject;
		colObj0->getCollisionShape()->getAabb(colObj0->getWorldTransform(),proxyAabbMin,proxyAabbMax);
		if (TestAabbAgainstAabb2(proxyAabbMin,proxyAabbMax,m_queryAabbMin,m_queryAabbMax))
		{
			m_numOverlap++;
		}
		return true;
	}
};

//void myTickCallback(btDynamicsWorld *world, btScalar timeStep) {
 //  BasicDemo::myTickCallback2(world, timeStep);

//}

void myTickCallback(btDynamicsWorld *world, btScalar timeStep) {
    //printf("The world just ticked by %f seconds\n", (float)timeStep);
    
	int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* obA = const_cast<btCollisionObject*>(contactManifold->getBody0());
		btCollisionObject* obB =  const_cast<btCollisionObject*>(contactManifold->getBody1());
		if ((!(obA->getUserPointer()==NULL) && (((my_obj *)obA->getUserPointer())->flag==-1)) || (!(obB->getUserPointer()==NULL) && (((my_obj *)obB->getUserPointer())->flag==-1))) 
			continue;
		if ((!(obA->getUserPointer()==NULL) && (((my_obj *)obA->getUserPointer())->flag==1)) && (!(obB->getUserPointer()==NULL) && (((my_obj *)obB->getUserPointer())->flag==1))) 
		    continue;
		if ((obA->getUserPointer()==NULL) && (obB->getUserPointer()==NULL)) 
		    continue;

		/*if 	(!(obA->getUserPointer()==NULL)) 
		{
			printf("1 %d \n",((my_obj *)obA->getUserPointer())->flag);
		}
		if 	(!(obB->getUserPointer()==NULL)) 
		{
			printf("2 %d \n",((my_obj *)obB->getUserPointer())->flag);
		}
		printf("\n");
		continue;*/

		int numContacts = contactManifold->getNumContacts();
		int lflag=0;
		for (int j=0;(j<numContacts) && (lflag==0);j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<0.f)
			{
				const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;

				btVector3& ptA1 = pt.m_localPointA;
				btVector3& ptB1 = pt.m_localPointB;

                //if (ncount>=1) continue;

				float objle;
				btTransform ttt;
				float tx;
				btVector3& txl=ptA1;
				if ((!(obA->getUserPointer()==NULL)) && ((((my_obj *)obA->getUserPointer())->flag==1)))
				{
					ttt=obA->getWorldTransform();
				    tx=(float)ptA.getX();
					txl=ptA1;
					objle=(((my_obj *)obA->getUserPointer())->length);
					ddd[ncount]=obB;
					ncount++;
				    ddd[ncount]=obA;
				    ncount++;
				}  
				else
				{
				    ttt=obB->getWorldTransform();
					tx=(float)ptB.getX();
					txl=ptB1;
					objle=(((my_obj *)obB->getUserPointer())->length);
					ddd[ncount]=obA;
					ncount++;
				    ddd[ncount]=obB;
					ncount++;
				}
				
				//printf("%f\n",objle);

                    btRigidBody *rigidBody = btRigidBody::upcast(ddd[ncount-1]);
					btRigidBody *rigidBody2=btRigidBody::upcast(ddd[ncount-2]);

					my_obj* newbody=new my_obj;
					newbody->length=absa2(ttt.getOrigin().getX()+objle-tx);
					newbody->flag=1;
		            btBoxShape* colShape = new btBoxShape(btVector3(newbody->length,SCALING*1,SCALING*1));
					m_collisionShapes.push_back(colShape);
                    btScalar	mass(10.f*(newbody->length/10));
				    bool isDynamic = (mass != 0.f);
					btVector3 localInertia(0,0,0);
					if (isDynamic)
						colShape->calculateLocalInertia(mass,localInertia);

				    
				    btVector3 lotowo=changecoor(btVector3((objle+txl.getX())/2,txl.getY(),txl.getZ()),ddd[ncount-1]->getWorldTransform().getBasis());
					btTransform tt=ttt;
					tt.setOrigin(btVector3(ttt.getOrigin().getX()+lotowo.getX(),
											ttt.getOrigin().getY()+lotowo.getY(),
											ttt.getOrigin().getZ()+lotowo.getZ()));
					

					lotowo=changecoor(btVector3(txl.getX(),txl.getY(),txl.getZ()),ddd[ncount-1]->getWorldTransform().getBasis());
					printf("0 %f\n",tx);
					printf("1  %lf\n",lotowo.getX()+ttt.getOrigin().getX());

					btDefaultMotionState* myMotionState = new btDefaultMotionState(tt);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					body->setUserPointer(newbody);
					
					//body->setMotionState(rigidBody->getMotionState());
					body->setLinearVelocity(rigidBody->getVelocityInLocalPoint(btVector3((objle+txl.getX())/2,txl.getY(),txl.getZ())));
					world->addRigidBody(body);
                    
				    body->applyImpulse(btVector3(rigidBody2->getLinearVelocity().getX()/2,
									           rigidBody2->getLinearVelocity().getY()/2,
											   rigidBody2->getLinearVelocity().getZ()/2),btVector3(-newbody->length,0,0));
/*					printf("%f %f %f\n",rigidBody->getTotalForce().getX()*5,
									           rigidBody->getTotalForce().getY()*5,
											   rigidBody->getTotalForce().getZ()*5);
					
*/
					my_obj* newbody2=new my_obj;
					newbody2->length=absa2(ttt.getOrigin().getX()-objle-tx);
					newbody2->flag=1;
					colShape = new btBoxShape(btVector3(newbody2->length,SCALING*1,SCALING*1));
					m_collisionShapes.push_back(colShape);
					
					mass=10.f*(newbody2->length/10);
					isDynamic = (mass != 0.f);

					btVector3 localInertia2(0,0,0);
					if (isDynamic)
						colShape->calculateLocalInertia(mass,localInertia2);
					

					lotowo=changecoor(btVector3((-objle+txl.getX())/2,txl.getY(),txl.getZ()),ddd[ncount-1]->getWorldTransform().getBasis());
					tt=ttt;
					tt.setOrigin(btVector3(ttt.getOrigin().getX()+lotowo.getX(),
											ttt.getOrigin().getY()+lotowo.getY(),
											ttt.getOrigin().getZ()+lotowo.getZ()));


                    btDefaultMotionState* myMotionState2 = new btDefaultMotionState(tt);
					btRigidBody::btRigidBodyConstructionInfo rbInfo2(mass,myMotionState2,colShape,localInertia2);
					btRigidBody* body2 = new btRigidBody(rbInfo2);
					body2->setUserPointer(newbody2);
                    
					body2->setLinearVelocity(rigidBody->getVelocityInLocalPoint(btVector3((-objle+txl.getX())/2,txl.getY(),txl.getZ())));

					world->addRigidBody(body2);
				    
					body2->applyImpulse(btVector3(rigidBody2->getLinearVelocity().getX()/2,
									           rigidBody2->getLinearVelocity().getY()/2,
											   rigidBody2->getLinearVelocity().getZ()/2),btVector3(newbody2->length*0.8,0,0));
                    
					lflag=1;
			}
		}
	}
	for (int j=0;j<ncount;j++)
	  world->removeCollisionObject(ddd[j]);
	ncount=0;
	return;
}

void BasicDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();

		btVector3 aabbMin(1,1,1);
		btVector3 aabbMax(2,2,2);

		MyOverlapCallback aabbOverlap(aabbMin,aabbMax);
		m_dynamicsWorld->getBroadphase()->aabbTest(aabbMin,aabbMax,aabbOverlap);
		
		//if (aabbOverlap.m_numOverlap)
		//	printf("#aabb overlap = %d\n", aabbOverlap.m_numOverlap);
	}
		
	renderme(); 

	glFlush();

	swapBuffers();

}



void BasicDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}





void	BasicDemo::initPhysics()
{
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	//*****************
	m_dynamicsWorld->setInternalTickCallback(myTickCallback,0,true);
	//****************

	m_dynamicsWorld->setGravity(btVector3(0,-1,0));

	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	//groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);


		my_obj* obground=new my_obj;
		obground->flag=-1;
		body->setUserPointer(obground);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(SCALING*10,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(10.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(2.0*i + start_x),
										btScalar(20+2.0*k + start_y),
										btScalar(2.0*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					
					my_obj* objstack=new my_obj;
					objstack->flag=1;
					objstack->length=10;
                    body->setUserPointer(objstack);

					m_dynamicsWorld->addRigidBody(body);
				}
			}
		}
	}


}
void	BasicDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}
	

void	BasicDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




