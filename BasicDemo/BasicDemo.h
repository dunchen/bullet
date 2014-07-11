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
#ifndef BASIC_DEMO_H
#define BASIC_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"
#include "math.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
//btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;


///BasicDemo is good starting point for learning the code base and porting.

class BasicDemo : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	//btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	public:
    
	//btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	BasicDemo()
	{
	}
	virtual ~BasicDemo()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	exitPhysics();
    //static void myTickCallback2(btDynamicsWorld *world, btScalar timeStep);
	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
	virtual void	clientResetScene();
	
	static DemoApplication* Create()
	{
		BasicDemo* demo = new BasicDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	
};

#endif //BASIC_DEMO_H

typedef struct
{
              float t; // real-component
              float x; // x-component
              float y; // y-component
              float z; // z-component
} quaternion;
  

//// Bill 注：Kakezan 在日语里是 “乘法”的意思
quaternion Kakezan(quaternion left, quaternion right)
{
              quaternion ans;
              float d1, d2, d3, d4;
  
              d1 = left.t * right.t;
              d2 = -left.x * right.x;
              d3 = -left.y * right.y;
              d4 = -left.z * right.z;
              ans.t = d1+ d2+ d3+ d4;
  
              d1 = left.t * right.x;
              d2 = right.t * left.x;
              d3 = left.y * right.z;
              d4 = -left.z * right.y;
              ans.x = d1+ d2+ d3+ d4;
  
              d1 = left.t * right.y;
              d2 = right.t * left.y;
              d3 = left.z * right.x;
              d4 = -left.x * right.z;
              ans.y = d1+ d2+ d3+ d4;
  
              d1 = left.t * right.z;
              d2 = right.t * left.z;
              d3 = left.x * right.y;
              d4 = -left.y * right.x;
              ans.z = d1+ d2+ d3+ d4;
              
              return ans;
}
  
//// Make Rotational quaternion
quaternion MakeRotationalQuaternion(float radian, float AxisX, float AxisY, float AxisZ)
{
              quaternion ans;
              float norm;
              float ccc, sss;
              
              ans.t = ans.x = ans.y = ans.z = 0.0;
  
              norm = AxisX * AxisX + AxisY * AxisY + AxisZ * AxisZ;
              if(norm <= 0.0) return ans;
  
              norm = 1.0 / sqrt(norm);
              AxisX *= norm;
              AxisY *= norm;
              AxisZ *= norm;
  
              ccc = cos(0.5 * radian);
              sss = sin(0.5 * radian);
  
              ans.t = ccc;
              ans.x = sss * AxisX;
              ans.y = sss * AxisY;
              ans.z = sss * AxisZ;
  
              return ans;
}
  
//// Put XYZ into quaternion
quaternion PutXYZToQuaternion(float PosX, float PosY, float PosZ)
{
              quaternion ans;
  
              ans.t = 0.0;
              ans.x = PosX;
              ans.y = PosY;
              ans.z = PosZ;
  
              return ans;
}
  
///// main
quaterion changecoor(float px,py,pz,th,ax,ay,az)
{
              quaternion ppp, qqq, rrr;
  /*
              cout << "Point Position (x, y, z) " << endl;
              cout << " x = ";
              cin >> px;
              cout << " y = ";
              cin >> py;
              cout << " z = ";
              cin >> pz;*/
              ppp = PutXYZToQuaternion(px, py, pz);
  /*
              while(1) {
                            cout << "\nRotation Degree ? (Enter 0 to Quit) " << endl;
                            cout << " angle = ";
                            cin >> th;
                            if(th == 0.0) break;
  
                            cout << "Rotation Axis Direction ? (x, y, z) " << endl;
                            cout << " x = ";
                            cin >> ax;
                            cout << " y = ";
                            cin >> ay;
                            cout << " z = ";
                            cin >> az;
  
  */
                            th *= 3.1415926535897932384626433832795 / 180.0; /// Degree -> radian;
  
                            qqq = MakeRotationalQuaternion(th, ax, ay, az);
                            rrr = MakeRotationalQuaternion(-th, ax, ay, az);
  
                            ppp = Kakezan(rrr, ppp);
                            ppp = Kakezan(ppp, qqq);
  
                           /* cout << "\nAnser X = " << ppp.x
                                          << "\n Y = " << ppp.y
                                          << "\n Z = " << ppp.z << endl;*/
  
              }
  
              return ppp;
} 
