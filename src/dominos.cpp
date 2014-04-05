/*! \file dominos.cpp
 * \brief This file creates all the objects in the simulation.
 * */


/*Nishant Kumar Singh
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */

#include <iostream>
#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   * 
   */ 
  
  dominos_t::dominos_t()
  {
	  
	  
	  
	  
	  
    //Ground
    /*! \par Ground 
     * Var: b1, Type: b2Body*, Desc: This is a pointer to the ground. <br>
     * Var: shape, Type: b2EdgeShape, Desc: The edge corresponding to the ground.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the ground.<br>
     * 
     */ 
    b2Body* b1;  
    
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
  
      
      
      
      
      {
		  //The stopper in the trigger ****************
		
		
		b2Vec2 vertices_pb[4];
			vertices_pb[0].Set(-0.5f, 0.5f);

			vertices_pb[1].Set(0.5f, 0.5f);

			vertices_pb[2].Set(1.5f, -4.5f);
			vertices_pb[3].Set(0.5f, -4.5f);
			
		b2Vec2 vertices_pb1[4];
			vertices_pb1[0].Set(-0.5f, 0.5f);

			vertices_pb1[1].Set(4.5f, 0.5f);

			vertices_pb1[2].Set(4.5f, -0.5f);
			vertices_pb1[3].Set(-0.5f, -0.5f);
			
			
		b2Vec2 vertices3[3];
	        vertices3[0].Set(3.0f,0.0f);
			vertices3[1].Set(5.0f,0.0f);
			vertices3[2].Set(2.0f,1.8f);
		
		
		//b2Body* body_p;
		
		b2PolygonShape polygon_pb;
		polygon_pb.Set(vertices_pb,4);
		
		b2PolygonShape polygon_pb1;
		polygon_pb1.Set(vertices_pb1,4);
		
		b2PolygonShape polygon3;
		polygon3.Set(vertices3,3);
		
		b2BodyDef bd_pb;
		bd_pb.position.Set(30.0f,13.5f);
		bd_pb.type = b2_dynamicBody;
		bodyy = m_world->CreateBody(&bd_pb);
		b2FixtureDef *fd_pb = new b2FixtureDef;
		fd_pb->density = 0.1f;
		fd_pb->shape = new b2PolygonShape;
		fd_pb->shape = &polygon_pb1;
		bodyy->CreateFixture(fd_pb);
		fd_pb->shape = &polygon_pb;
		bodyy->CreateFixture(fd_pb);
		fd_pb->shape = &polygon3;
		bodyy->CreateFixture(fd_pb);
		
		
		b2BodyDef bdx3;
      bdx3.position.Set(30.5f, 13.5f);
      b2Body* bodyx3 = m_world->CreateBody(&bdx3);
	  b2RevoluteJointDef jointDef3xy;
      jointDef3xy.bodyA = bodyx3;
      jointDef3xy.bodyB = bodyy;
      jointDef3xy.localAnchorA.Set(0,0);
      jointDef3xy.localAnchorB.Set(0,0);
      jointDef3xy.collideConnected = false;
      jointDef3xy.enableMotor = true;
      jointDef3xy.motorSpeed=10;
      jointDef3xy.maxMotorTorque=2000;
      m_world->CreateJoint(&jointDef3xy);
      
      
      b2BodyDef arod;
      arod.type = b2_dynamicBody;
      b2PolygonShape shaperod;
      shaperod.SetAsBox(5.0f,0.5f);
      arod.position.Set(20.0f,10.0f);
      attachrod = m_world->CreateBody(&arod);
      b2FixtureDef *fd_arod = new b2FixtureDef;
      fd_arod->density = 0.2f;
      fd_arod->shape = new b2PolygonShape;
      fd_arod->shape = &shaperod;
      attachrod->CreateFixture(fd_arod);
      
      b2RevoluteJointDef jointDefrod;
      jointDefrod.bodyA = attachrod;
      jointDefrod.bodyB = bodyy;
      jointDefrod.localAnchorA.Set(5.0f,0.0f);
      jointDefrod.localAnchorB.Set(1.0f,-3.5f);
      jointDefrod.collideConnected = false;
      
      m_world->CreateJoint(&jointDefrod);
      
      
      b2PolygonShape shapenew;
      shapenew.SetAsBox(0.2f, 0.2f);
      b2BodyDef bdnew;
      bdnew.position.Set(25.5f, 9.0f);
      b2Body* bodynew = m_world->CreateBody(&bdnew);
      b2FixtureDef *fdnew = new b2FixtureDef;
      fdnew->density = 0.1f;
      fdnew->shape = new b2PolygonShape;
      fdnew->shape = &shapenew;
      bodynew->CreateFixture(fdnew);
      
     
		  
	  }
      
      
      
      
      
      
      
      //////////////// The Trigger /////////////////////////
      
      
      {
		  
		  b2Vec2 vertices_pb[4];
			vertices_pb[0].Set(-0.5f, 0.5f);

			vertices_pb[1].Set(0.5f, 0.5f);

			vertices_pb[2].Set(-1.5f, -3.5f);
			vertices_pb[3].Set(-2.5f, -3.5f);
			
		b2Vec2 vertices_pb1[4];
			vertices_pb1[0].Set(-0.5f, 0.5f);

			vertices_pb1[1].Set(0.5f, 0.5f);

			vertices_pb1[2].Set(-0.5f, 3.5f);
			vertices_pb1[3].Set(-1.5f, 3.5f);
		
		
		
		
		b2PolygonShape polygon_pb;
		polygon_pb.Set(vertices_pb,4);
		
		b2PolygonShape polygon_pb1;
		polygon_pb1.Set(vertices_pb1,4);
		
		b2BodyDef bd_pb;
		bd_pb.position.Set(0.0f,20.0f);
		bd_pb.type = b2_dynamicBody;
		body_t = m_world->CreateBody(&bd_pb);
		b2FixtureDef *fd_pb = new b2FixtureDef;
		fd_pb->density = 0.1f;
		fd_pb->shape = new b2PolygonShape;
		fd_pb->shape = &polygon_pb1;
		body_t->CreateFixture(fd_pb);
		fd_pb->shape = &polygon_pb;
		body_t->CreateFixture(fd_pb);
		
		
		b2BodyDef bd2;
      bd2.position.Set(22.0f, 6.5f);
      b2Body* body2 = m_world->CreateBody(&bd2);
		
		
		b2RevoluteJointDef jointDeft;
      jointDeft.bodyA = body2;
      jointDeft.bodyB = body_t;
      jointDeft.localAnchorA.Set(0.0f,0.0f);
      jointDeft.localAnchorB.Set(0.0f,0.0f);
      jointDeft.collideConnected = false;
      
      m_world->CreateJoint(&jointDeft);
      
      
		b2RevoluteJointDef jointDefrod;
      jointDefrod.bodyA = attachrod;
      jointDefrod.bodyB = body_t;
      jointDefrod.localAnchorA.Set(-4.5f,0.0f);
      jointDefrod.localAnchorB.Set(-1.0f,3.5f);
      jointDefrod.collideConnected = false;
      
      m_world->CreateJoint(&jointDefrod);
		  
		  
	  }
      
      
      
      
      
      
      //The revolving horizontal platform
    {
		
			 b2Vec2 vertices[4];
			 b2Vec2 vertices1[4];
			 b2Vec2 vertices2[3];
			 b2Vec2 vertices3[3];
			vertices[0].Set(0.0f, 0.0f);

			vertices[1].Set(2.0f, 0.0f);

			vertices[2].Set(2.0f, 8.0f);
			vertices[3].Set(0.0f, 8.0f);
			
			vertices1[0].Set(-1.0f, 6.0f);
			vertices1[1].Set(2.0f, 6.0f);
			vertices1[2].Set(2.0f, 7.0f);
			vertices1[3].Set(-1.0f, 7.0f);
			
			vertices2[0].Set(0.0f,0.0f);
			vertices2[1].Set(2.0f,0.0f);
			vertices2[2].Set(3.0f,-1.5f);
			
			vertices3[0].Set(4.5f,0.0f);
			vertices3[1].Set(6.5f,0.0f);
			vertices3[2].Set(3.5f,1.5f);
			
			b2Vec2 vertices5[4];
			vertices5[0].Set(3.0f,0.0f);
			vertices5[0].Set(4.0f,0.0f);
			vertices5[0].Set(4.0f,-2.0f);
			vertices5[0].Set(3.0f,-2.0f);
			
			int32 count = 4;
			
			int count2 = 3;
 

b2PolygonShape polygon;
b2PolygonShape polygon1;
b2PolygonShape polygon2;
b2PolygonShape polygon3;
b2PolygonShape polygon4;
b2PolygonShape polygon5;



polygon4.SetAsBox(6.0f,0.5f);
polygon3.Set(vertices3, count2);
polygon2.Set(vertices2,count2);
polygon1.Set(vertices1,count);
polygon.Set(vertices, count);
//polygon5.Set(vertices5, count);


	// The hitter
      
      // The container of the pullback. *************
      
      
      //Lower block ****************
      b2PolygonShape shape;
      shape.SetAsBox(5.2f, 0.2f);
      b2BodyDef bd3;
      bd3.position.Set(28.0f, 20.0f);
      //bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.1f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape;
      body3->CreateFixture(fd3);
      
      
      
      
     
      
      
      //upper block *****************
      b2BodyDef bd31;
      bd31.position.Set(28.0f, 23.5f);
      //bd3.type = b2_dynamicBody;
      b2Body* body31 = m_world->CreateBody(&bd31);
      b2FixtureDef *fd31 = new b2FixtureDef;
      fd31->density = 0.1f;
      fd31->shape = new b2PolygonShape;
      fd31->shape = &shape;
      body31->CreateFixture(fd31);
      
      //front stopper *******************
      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 1.0f);
      
      b2BodyDef bd32;
      bd32.position.Set(15.0f, 24.0f);
      //bd3.type = b2_dynamicBody;
      b2Body* body32 = m_world->CreateBody(&bd32);
      b2FixtureDef *fd32 = new b2FixtureDef;
      fd32->density = 0.1f;
      fd32->shape = new b2PolygonShape;
      fd32->shape = &shape2;
      body32->CreateFixture(fd32);
      //front stopper *******************
      
      b2BodyDef bd32_1;
      bd32_1.position.Set(15.0f, 20.0f);
      //bd3.type = b2_dynamicBody;
      b2Body* body32_1 = m_world->CreateBody(&bd32_1);
      b2FixtureDef *fd32_1 = new b2FixtureDef;
      fd32_1->density = 0.1f;
      fd32_1->shape = new b2PolygonShape;
      fd32_1->shape = &shape2;
      body32_1->CreateFixture(fd32_1);

		//////////// The bullet platform ///////////
	  b2PolygonShape shape_bp;
	  shape_bp.SetAsBox(4.0f,0.2f);
      b2BodyDef bd_bp;
      bd_bp.position.Set(10.0f, 20.0f);
      //bd3.type = b2_dynamicBody;
      b2Body* body_bp = m_world->CreateBody(&bd_bp);
      b2FixtureDef *fd_bp = new b2FixtureDef;
      fd_bp->density = 0.1f;
      fd_bp->shape = new b2PolygonShape;
      fd_bp->shape = &shape_bp;
      body_bp->CreateFixture(fd_bp);
      //bodyx->ApplyForce( b2Vec2(50,0), bodyx->GetWorldCenter() ,true);

		//////////// The dummy bullet ///////////
	  b2PolygonShape shape_bul;
	  shape_bul.SetAsBox(2.0f,1.0f);
      b2BodyDef bd_bul;
      bd_bul.position.Set(9.2f, 21.5f);
      bd_bul.type = b2_dynamicBody;
      body_bul = m_world->CreateBody(&bd_bul);
      b2FixtureDef *fd_bul = new b2FixtureDef;
      fd_bul->density = 0.1f;
      fd_bul->shape = new b2PolygonShape;
      fd_bul->shape = &shape_bul;
      
      //body_bul->SetUserData( this );
      int myint1 = 118;
      body_bul->SetUserData((void*)myint1);
      
      body_bul->CreateFixture(fd_bul);
      
     
     
		//The pullback mechanism  ************************
		
		
		b2Vec2 vertices_pb[4];
			vertices_pb[0].Set(0.0f, 0.0f);

			vertices_pb[1].Set(1.0f, 0.0f);

			vertices_pb[2].Set(1.0f, 2.0f);
			vertices_pb[3].Set(0.0f, 2.0f);
			
			b2Vec2 vertices_pb2[4];
			vertices_pb2[0].Set(0.0f, 2.0f);

			vertices_pb2[1].Set(1.0f, 2.0f);

			vertices_pb2[2].Set(1.0f, 4.0f);
			vertices_pb2[3].Set(0.0f, 4.0f);
			
		b2Vec2 vertices_pb1[4];
			vertices_pb1[0].Set(0.0f, 2.0f);

			vertices_pb1[1].Set(18.0f, 2.0f);

			vertices_pb1[2].Set(18.0f, 1.0f);
			vertices_pb1[3].Set(0.0f, 1.0f);
			
		b2Vec2 vertices_pb3[4];
			vertices_pb3[0].Set(0.0f, 2.0f);

			vertices_pb3[1].Set(21.0f, 2.0f);

			vertices_pb3[2].Set(21.0f, 1.0f);
			vertices_pb3[3].Set(0.0f, 1.0f);
		
		
		//b2Body* body_pb;
		
		b2PolygonShape polygon_pb;
		polygon_pb.Set(vertices_pb,count);
		
		b2PolygonShape polygon_pb1;
		polygon_pb1.Set(vertices_pb1,count);
		
		b2PolygonShape polygon_pb2;
		polygon_pb2.Set(vertices_pb2,count);
		
		
		b2PolygonShape polygon_pb3;
		polygon_pb3.Set(vertices_pb3,count);
		
		
		
		// The bullet hitter
		b2BodyDef bd_pb2;
		bd_pb2.position.Set(14.0f,22.0f);
		bd_pb2.type = b2_dynamicBody;
		body_pb2 = m_world->CreateBody(&bd_pb2);
		b2FixtureDef *fd_pb2 = new b2FixtureDef;
		fd_pb2->density = 0.1f;
		fd_pb2->shape = new b2PolygonShape;
		fd_pb2->shape = &polygon_pb3;
		body_pb2->CreateFixture(fd_pb2);
		
			int myint=108;
		body_pb2->SetUserData((void*)myint);
		//body_pb2->SetUserData( this );
		
		
		
		b2BodyDef bd_pb;
		bd_pb.position.Set(18.0f,20.0f);
		bd_pb.type = b2_dynamicBody;
		body_pb = m_world->CreateBody(&bd_pb);
		b2FixtureDef *fd_pb = new b2FixtureDef;
		fd_pb->density = 0.1f;
		fd_pb->shape = new b2PolygonShape;
		fd_pb->shape = &polygon_pb1;
		body_pb->CreateFixture(fd_pb);
		fd_pb->shape = &polygon_pb;
		body_pb->CreateFixture(fd_pb);
		

		
		b2BodyDef bd_pb1;
		bd_pb1.position.Set(18.0f,22.0f);
		bd_pb1.type = b2_dynamicBody;
		body_pb1 = m_world->CreateBody(&bd_pb1);
		b2FixtureDef *fd_pb1 = new b2FixtureDef;
		fd_pb1->density = 0.1f;
		fd_pb1->shape = new b2PolygonShape;
		fd_pb1->shape = &polygon_pb1;
		body_pb1->CreateFixture(fd_pb1);
		fd_pb1->shape = &polygon_pb2;
		body_pb1->CreateFixture(fd_pb1);
		
		
		
		 b2DistanceJointDef joint4; 
		const b2Vec2 point6(18.5f, 20.5f); 
		const b2Vec2 point7(27.5f, 20.0f); 
		joint4.Initialize(body_pb, body3, point6, point7); 
		joint4.collideConnected = true; 
		joint4.frequencyHz = 1.0f; 
		joint4.dampingRatio =0.0f;
		joint4.length = 13.0f;
		 m_world->CreateJoint(&joint4);
		 
		 
		 b2DistanceJointDef joint5; 
		const b2Vec2 point8(18.5f, 24.5f); 
		const b2Vec2 point9(27.5f, 23.5f); 
		joint5.Initialize(body_pb1, body31, point8, point9); 
		joint5.collideConnected = true; 
		joint5.frequencyHz = 1.0f; 
		joint5.dampingRatio =0.0f;
		joint5.length = 13.0f;
		 m_world->CreateJoint(&joint5);
		 
		 
		 
		 b2DistanceJointDef joint6; 
		const b2Vec2 point10(25.5f, 23.5f); 
		const b2Vec2 point11(14.5f, 23.5f); 
		joint6.Initialize(body_pb1, body_pb2, point10, point11); 
		joint6.collideConnected = true; 
		joint6.frequencyHz = 0.5f; 
		joint6.dampingRatio =1.0f;
		joint6.length = 5.0f;
		 m_world->CreateJoint(&joint6);
		 
		 
		 b2DistanceJointDef joint7; 
		const b2Vec2 point12(25.5f, 21.5f); 
		const b2Vec2 point13(14.5f, 23.5f); 
		joint7.Initialize(body_pb, body_pb2, point12, point13); 
		joint7.collideConnected = true; 
		joint7.frequencyHz = 0.5f; 
		joint7.dampingRatio =1.0f;
		joint7.length = 5.0f;
		 m_world->CreateJoint(&joint7);
		 
		 b2DistanceJointDef joint8; 
		const b2Vec2 point14(25.5f, 21.5f); 
		const b2Vec2 point15(31.5f, 23.5f); 
		joint8.Initialize(body_pb1, body_pb2, point14, point15); 
		joint8.collideConnected = true; 
		joint8.frequencyHz = 0.5f; 
		joint8.dampingRatio =1.0f;
		joint8.length = 5.0f;
		 m_world->CreateJoint(&joint8);
		 
		  b2DistanceJointDef joint9; 
		const b2Vec2 point16(25.5f, 23.5f); 
		const b2Vec2 point17(31.5f, 23.5f); 
		joint9.Initialize(body_pb, body_pb2, point16, point17); 
		joint9.collideConnected = true; 
		joint9.frequencyHz = 0.5f; 
		joint9.dampingRatio =1.0f;
		joint9.length = 5.0f;
		 m_world->CreateJoint(&joint9);
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
		 
	  b2BodyDef bdx;
      bdx.position.Set(34.0f, 20.0f);
      bdx.type = b2_dynamicBody;
      bodyx = m_world->CreateBody(&bdx);
      b2FixtureDef *fdx = new b2FixtureDef;
      fdx->density = 0.3f;
      fdx->shape = new b2PolygonShape;
      fdx->shape = &polygon;
      bodyx->CreateFixture(fdx);
      //fdx->shape = &polygon1;
      //bodyx->CreateFixture(fdx);
      fdx->shape = &polygon2;
      bodyx->CreateFixture(fdx);
      
      
    
      
      b2PolygonShape shapenew;
      shapenew.SetAsBox(0.2f, 0.2f);
      b2BodyDef bdnew;
      bdnew.position.Set(32.5f, 10.0f);
      b2Body* bodynew = m_world->CreateBody(&bdnew);
      b2FixtureDef *fdnew = new b2FixtureDef;
      fdnew->density = 0.1f;
      fdnew->shape = new b2PolygonShape;
      fdnew->shape = &shapenew;
      bodynew->CreateFixture(fdnew);
		
		
      //The hitter rotator ***********************
      //b2PolygonShape shape2;
      //shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(34.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = bodyx;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(1,1);
      jointDef.localAnchorB.Set(0,0);
      jointDef.enableMotor = true;
      jointDef.motorSpeed=-1000;
      jointDef.maxMotorTorque=3000;
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
		 
		
      }
      
      
      //myContactListenerInstance = new MyContactListener(body_pb2);
      myContactListenerInstance.MycontactListener(body_bul,body_pb,body_pb1);
	  m_world->SetContactListener(&myContactListenerInstance);
     
    
     

  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
  

}



