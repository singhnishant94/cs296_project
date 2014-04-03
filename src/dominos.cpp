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
      
     /*! \par Top horizontal shelf
      * Var: shape, Type: b2PolygonShape, Desc: It describes the shape of the shelf.<br>
      * Var: bd, Type: b2BodyDef, Desc: The actual body representing the shelf.<br>
      * Var: ground, Type: b2Body*, Desc: This is a pointer to shelf.<br>
      * */    
    //Top horizontal shelf
    /*
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }*/
	/*! \par Dominos
	 * Var: shape, Type: b2PolygonShape, Desc: It describes the shape of the dominos.<br>
	 * Var: fd, Type: b2FixtureDef, Desc: It binds the body to the specified fixture.<br>
	 * For loop for creating ten dominos placed vertically side by side.<br>
	 * ...Var: bd, Type: b2BodyDef, Desc: Provides a definition for each of the dominos.<br>
	 * ...Var: body, Type: b2Body*, Desc: This is a pointer to the dominos.<br>
	 * */
    //Dominos
    /*
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
    */
    /*! \par Another horizontal shelf 
     * Var: shape, Type: b2PolygonShape, Desc: Defines a shape for the shelf.<br>
     * Var: bd, Type: b2BodyDef, Desc: Provides a definition for the shelf.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to the shelf.<br>
     * 
     * */
    //Another horizontal shelf
    /*
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
	*/
	/*! \par The pendulum that knocks the dominos off
	 * Var: b2, Type: b2Body*, Desc: This is a pointer to the fixed bar at the bottom to which the pendulum is attached.<br>
	 * ...Var: shape, Type: b2PolygonShape, Desc: Provides a definition for the shape of the bar.<br>
	 * ...Var: bd, Type: b2BodyDef, Desc: Holds the data needed to create the bar.<br>
	 * Var: b4,Type :b2Body*, Desc: This is a pointer to bob of the pendulum.<br>
	 * ...Var:shape ,Type: b2PolygonShape, Desc: Defines the shape of the bob.<br>
	 * ...Var:bd, Type: b2BodyDef, Desc: Holds the data needed to create the bar.<br>
	 * Var: jd, Type: b2RevoluteJointDef, Desc: Definition of the joint between the bob and point to which it hangs.<br>
	 * Var: anchor, Type:b2Vec2, Desc: Is a 2D vector to define the position of the point to which the bob hangs.<br>
	 *  
	 * 
	 * */
    //The pendulum that knocks the dominos off
    /*
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
    * */
      /*! \par The train of small spheres
       * Var: spherebody, Type: b2Body*, Desc: This is a pointer to the small spheres. <br>
       * Var: circle, Type: b2CircleShape, Desc: To define the radius of the spheres.<br>
       * Var: ballfd, Type: b2FixtureDef, Desc: To define the fixture of the spheres.<br>
       * For loop:To create ten such small spheres<br>
       * ...Var: ballbd, Type: b2BodyDef, Desc: Holds the data needed to create the spheres.<br>
       * 
       * 
       * */
    //The train of small spheres
    /*
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }
    * */
	/*! \par The pulley system
	 * Var: bd, Type: b2bodyDef*, Desc: This is a pointer to bar which hangs on to the right side of the pulley.<br>
	 * Var: fd1, Type: b2FixtureDef*, Desc: This is a pointer to the fixture of one of the sides of the open box.<br>
	 * Var: bs1, Type: b2PolygonShape, Desc: To define one of the sides of the box.<br>
	 * Var: fd2, Type: b2FixtureDef*, Desc: This is a pointer to the fixture of one of the sides of the open box.<br>
	 * Var: bs2, Type: b2PolygonShape, Desc: To define one of the sides of the box.<br>
	 * Var: fd3, Type: b2FixtureDef*, Desc: This is a pointer to the fixture of one of the sides of the open box.<br>
	 * Var: bs3, Type: b2PolygonShape, Desc: To define one of the sides of the box.<br>
	 * Var: box1, Type: b2Body*, Desc: A pointer to the actual box.<br>
	 * Var: box2, Type: b2Body*, Desc: A pointer to the actual hanging bar.<br>
	 * Var: myjoint, Type:b2PulleyJointDef*, Desc: A pointer to the pulley joint.<br>
	 * Var: worldAnchorOnBody1, Type: b2Vec2, Desc: Anchor point on body 1 in world axis.<br>
	 * Var: worldAnchorOnBody2, Type: b2Vec2, Desc: Anchor point on body 2 in world axis.<br>
	 * Var: worldAnchorOnBody2, Type: b2Vec2, Desc: Anchor point on ground 1 in world axis.<br>
	 * Var: worldAnchorOnBody2, Type: b2Vec2, Desc: Anchor point on ground 2 in world axis.<br>
	 * */
    //The pulley system
    /*
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(10,15);	
      fd1->density = 34.0;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);




      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    * */

	/*! \par The revolving horizontal platform
	 * Var: shape, Type: b2PolygonShape, Desc: Defines the shape of the platform.<br>
	 * Var: bd, Type: b2BodyDef, Desc: Holds the details needed to create the platform.<br>
	 * Var: body, Type: b2Body*, Desc: This is a pointer to the actual platform.<br>
	 * Var: fd, Type: b2FixtureDef*, Desc: This is pointer to fixture of the platform.<br>
	 * Var: bd2, Type: b2BodyDef, Desc: Defines the point to which platform is attached and around which it revolves.<br>
	 * Var: body2, Type: b2Body*, Desc: A pointer to the actual joint.<br>
	 * Var: jointDef, Type: b2RevoluteJointDef, Desc: Defines the joint of the platform with the point.<br>
	 * jointDef.bodyA,jointDef.bodyB: Two Bodies which will be joined together.<br>
	 * 
	 * */


    //The revolving horizontal platform
    /*
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      //b2PolygonShape shape2;
      //shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
      
      
     
      }
      
      */
      
      
      
      
      
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
			vertices3[2].Set(2.0f,1.5f);
		
		
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
      jointDef3xy.motorSpeed=1;
      jointDef3xy.maxMotorTorque=800;
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
	  b2BodyDef bdx;
      bdx.position.Set(34.0f, 20.0f);
      bdx.type = b2_dynamicBody;
      bodyx = m_world->CreateBody(&bdx);
      b2FixtureDef *fdx = new b2FixtureDef;
      fdx->density = 0.1f;
      fdx->shape = new b2PolygonShape;
      fdx->shape = &polygon;
      bodyx->CreateFixture(fdx);
      //fdx->shape = &polygon1;
      //bodyx->CreateFixture(fdx);
      fdx->shape = &polygon2;
      bodyx->CreateFixture(fdx);
      
      
      /*
      // The stopper
      b2BodyDef bdy;
      bdy.position.Set(-30.0f, 14.0f);
      bdy.type = b2_dynamicBody;
      bodyy = m_world->CreateBody(&bdy);
      b2FixtureDef *fdy = new b2FixtureDef;
      fdy->density = 1.f;
      fdy->shape = new b2PolygonShape;
      fdy->shape = &polygon4;
      bodyy->CreateFixture(fdy);
      fdy->shape = &polygon3;
      bodyy->CreateFixture(fdy);
      //fdy->shape = &polygon5;
      //bodyy->CreateFixture(fdy);
      
		
	  b2BodyDef bdx3;
      bdx3.position.Set(-30.0f, 13.0f);
      b2Body* bodyx3 = m_world->CreateBody(&bdx3);
	  b2RevoluteJointDef jointDef3xy;
      jointDef3xy.bodyA = bodyx3;
      jointDef3xy.bodyB = bodyy;
      jointDef3xy.localAnchorA.Set(0,0);
      jointDef3xy.localAnchorB.Set(0,0);
      jointDef3xy.collideConnected = false;
      jointDef3xy.enableMotor = true;
      jointDef3xy.motorSpeed=100;
      jointDef3xy.maxMotorTorque=800;
      m_world->CreateJoint(&jointDef3xy);*/
      
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
      jointDef.maxMotorTorque=2000;
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
      
      // The container of the pullback. *************
      
      
      //Lower block ****************
      b2PolygonShape shape;
      shape.SetAsBox(4.2f, 0.2f);
      b2BodyDef bd3;
      bd3.position.Set(29.0f, 20.0f);
      //bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.1f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape;
      body3->CreateFixture(fd3);
      
      
      
      
     
      
      
      //upper block *****************
      b2BodyDef bd31;
      bd31.position.Set(29.0f, 21.5f);
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
      bd32.position.Set(16.0f, 19.0f);
      //bd3.type = b2_dynamicBody;
      b2Body* body32 = m_world->CreateBody(&bd32);
      b2FixtureDef *fd32 = new b2FixtureDef;
      fd32->density = 0.1f;
      fd32->shape = new b2PolygonShape;
      fd32->shape = &shape2;
      body32->CreateFixture(fd32);
      //bodyx->ApplyForce( b2Vec2(50,0), bodyx->GetWorldCenter() ,true);
      
      /*
        MouseJointDef def = new MouseJointDef();
		def.bodyA = m_world.GetGroundBody();
		def.bodyB = bodyx;
		def.collideConnected = true;
		b2Vec2 testPoint = bodyx.GetWorldCenter();
		
		def.target.set(testPoint.x, testPoint.y);
		def.maxForce = 10000.0f;
		def.frequencyHz=100;
		def.dampingRatio=0;
      */
     
     
		//The pullback mechanism
		
		
		b2Vec2 vertices_pb[4];
			vertices_pb[0].Set(0.0f, 0.0f);

			vertices_pb[1].Set(2.0f, 0.0f);

			vertices_pb[2].Set(2.0f, 2.0f);
			vertices_pb[3].Set(0.0f, 2.0f);
			
		b2Vec2 vertices_pb1[4];
			vertices_pb1[0].Set(0.0f, 2.0f);

			vertices_pb1[1].Set(17.0f, 2.0f);

			vertices_pb1[2].Set(17.0f, 1.0f);
			vertices_pb1[3].Set(0.0f, 1.0f);
		
		
		//b2Body* body_pb;
		
		b2PolygonShape polygon_pb;
		polygon_pb.Set(vertices_pb,count);
		
		b2PolygonShape polygon_pb1;
		polygon_pb1.Set(vertices_pb1,count);
		
		b2BodyDef bd_pb;
		bd_pb.position.Set(20.0f,20.0f);
		bd_pb.type = b2_dynamicBody;
		body_pb = m_world->CreateBody(&bd_pb);
		b2FixtureDef *fd_pb = new b2FixtureDef;
		fd_pb->density = 0.1f;
		fd_pb->shape = new b2PolygonShape;
		fd_pb->shape = &polygon_pb1;
		body_pb->CreateFixture(fd_pb);
		fd_pb->shape = &polygon_pb;
		body_pb->CreateFixture(fd_pb);
		
		
		
		 b2DistanceJointDef joint4; 
		const b2Vec2 point6(20.5f, 20.0f); 
		const b2Vec2 point7(27.5f, 20.0f); 
		joint4.Initialize(body_pb, body3, point6, point7); 
		joint4.collideConnected = true; 
		joint4.frequencyHz = 1.0f; 
		joint4.dampingRatio =0.0f;
		joint4.length = 11.0f;
		 m_world->CreateJoint(&joint4);
		
		//The pullback mechanism
		
		/*
		b2Vec2 vertices_n[4];
			vertices_n[0].Set(0.0f, 0.0f);

			vertices_n[1].Set(2.0f, 0.0f);

			vertices_n[2].Set(2.0f, 2.0f);
			vertices_n[3].Set(0.0f, 2.0f);
			
		b2Vec2 vertices_n1[4];
			vertices_n1[0].Set(0.0f, 2.0f);

			vertices_n1[1].Set(15.0f, 2.0f);

			vertices_n1[2].Set(15.0f, 1.0f);
			vertices_n1[3].Set(0.0f, 1.0f);
		
		
		b2Body* body_n;
		
		b2PolygonShape polygon_n;
		polygon_n.Set(vertices_n,count);
		
		b2PolygonShape polygon_n1;
		polygon_n1.Set(vertices_n1,count);
		
		
		b2BodyDef bd_n;
		bd_n.position.Set(0.0f,20.0f);
		bd_n.type = b2_dynamicBody;
		body_n = m_world->CreateBody(&bd_n);
		b2FixtureDef *fd_n = new b2FixtureDef;
		fd_n->density = 0.1f;
		fd_n->shape = new b2PolygonShape;
		fd_n->shape = &polygon_n1;
		body_n->CreateFixture(fd_pb);
		fd_n->shape = &polygon_n;
		body_n->CreateFixture(fd_pb);
		
		
		//Spring between pullback and fixed body
		
		
		b2DistanceJointDef jointDef_pb;
		jointDef_pb.Initialize(body_pb, body3,b2Vec2(body_pb->GetWorldCenter().x-4.0f,body_pb->GetWorldCenter().y-1.0f) ,body3->GetWorldCenter());
		jointDef_pb.collideConnected = true;
		jointDef_pb.frequencyHz = 4000.0f;
		jointDef_pb.dampingRatio = 0.5f;
		m_world->CreateJoint(&jointDef_pb);
     */
      }
      
      
      
      
      
      /*! \par Another revolving platform
      * Lower platform<br>
      * ...Var: shape, Type: b2PolygonShape, Desc: Defines the shape of the platform.<br>
      * ...Var: bd, Type: b2BodyDef, Desc: Holds the data needed to create the platform.<br>
      * ...Var: body, Type: b2Body*, Desc: A pointer to the actual platform.<br>
      * ...Var: fd, Type: b2FixtureDef*, Desc: A pointer to the fixture of the platform.<br>
      * ...Var: bd2, Type: b2BodyDef, Desc: Holds the data needed to create the pivot.<br>
      * ...Var: body2, Type: b2Body*, Desc: A pointer to the actual pivot.<br>
      * Attaching rod<br>
      * ...Var: shape3, Type: b2PolygonShape, Desc: Sets the polygon associated with the rod.<br>
      * ...Var: bd3, Type: b2BodyDef, Desc: Holds the data needed to create the rod.<br>
      * ...Var: body3, Type:b2Body*, Desc: A pointer to the rod.<br>
      * ...Var: fd3, Type: b2FixtureDef*, Desc: A pointer to the fixture of the rod.<br>
      * Upper platform<br>
      * ...Var: shape4, Type: b2PolygonShape, Desc: Sets the polygon associated with the upper platform.<br>
      * ...Var: bd4, Type: b2BodyDef, Desc: Holds the data needed to create the platform.<br>
      * ...Var: body4, Type:b2Body*, Desc: A pointer to the platform.<br>
      * ...Var: fd4, Type: b2FixtureDef*, Desc: A pointer to the fixture of the platform.<br>
      * ...Var: bd5, Type: b2BodyDef, Desc: Holds the data needed to create the platform.<br>
      * A body attached to the middle of the attaching rod<br>
      * Var: b8, Type: b2Body*, Desc: A pointer to the body.
      * ...Var: shape, Type: b2PolygonShape, Desc: Sets the polygon associated with the body.<br>
      * ...Var: bd, Type: b2BodyDef, Desc: Holds the data needed to create the body.<br>
      * Bob of the second pendulum<br>
      * Var: b7, Type: b2Body*, Desc: A pointer to the bob.<br>
      * ...Var: shape, Type: b2PolygonShape, Desc: Sets the polygon associated with the bob.<br>
      * ...Var: bd, Type: b2BodyDef, Desc: Holds the data needed to create the bob.<br>
      * Joint between small body and bob<br>
      * ...Var: jd, Type: b2RevoluteJointDef, Desc: Joint between the small body and the rod.<br>
      * ...Var: anc, Type: b2Vec2, Desc: Coordinate to denote the anchor point.<br>
      * ...Var: jointDef, Type: b2RevoluteJointDef, Desc: Pivot around which the lower platform revolves.<br>
      * Motor is enabled for this pivot which will rotate the platform.<br>
      * Pivot around which the upper platform revolves<br>
      * ...Var: jointDef1, Type: b2RevoluteJointDef, Desc: Pivot around which the upper platform revolves.<br>
      * Joint between the small body and the attaching rod<br>
      * ...Var: jointDef7, Type: b2RevoluteJointDef, Desc: Defines the joint between small body and attaching rod.<br>
      * Attachment between the upper platform and attaching rod<br>
      * ...Var: jointDef2, Type: b2RevoluteJointDef, Desc: Defines the joint between upper platform and attaching rod.<br>
      * Attachment between the upper platform and attaching rod<br>
      * ...Var: jointDef3, Type: b2RevoluteJointDef, Desc: Defines the joint between lower platform and attaching rod.<br>
      * */
      /*
     {//Another revolving platform
     

     //Lower platform
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(20.0f, 30.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      //b2PolygonShape shape2;
      //shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(20.0f, 30.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);
      

      //Attaching rod
      b2PolygonShape shape3;
      shape3.SetAsBox(5.0f,0.2f);
      
      b2BodyDef bd3;
      bd3.position.Set(20.0f, 30.0f);
      bd3.type=b2_dynamicBody;
      b2Body* body3= m_world->CreateBody(&bd3);
      b2FixtureDef *fd3= new b2FixtureDef;
      fd3->density = 10.0f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape3;
      body3->CreateFixture(fd3);
      
      //Upper Platform
      b2PolygonShape shape4;
      shape4.SetAsBox(2.2f,0.2f);
      
      b2BodyDef bd4;
      bd4.position.Set(20.0f, 25.0f);
      bd4.type=b2_dynamicBody;
      b2Body* body4= m_world->CreateBody(&bd4);
      b2FixtureDef *fd4= new b2FixtureDef;
      fd4->density = 10.0f;
      fd4->shape = new b2PolygonShape;
      fd4->shape = &shape4;
      body4->CreateFixture(fd4);
      
      b2BodyDef bd5;
      bd5.position.Set(20.0f, 40.0f);
      b2Body* body5 = m_world->CreateBody(&bd5);

      
      //A body attached to the middle of the attaching rod
      b2Body* b8;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.position.Set(20.0f, 35.0f);
	bd.type=b2_dynamicBody;
	b8 = m_world->CreateBody(&bd);
	b8->CreateFixture(&shape, 10.0f);
      }
      

      //Bob of the second pendulum
     b2Body* b7;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.5f);
	
	b2BodyDef bd;
	bd.type=b2_dynamicBody;
	bd.position.Set(19.0f, 25.0f);
	b7 = m_world->CreateBody(&bd);
	b7->CreateFixture(&shape, 10.0f);
      }
 
    
      //Joint between small body and bob     
      b2RevoluteJointDef jd;
      b2Vec2 anc;
      anc.Set(20.0f, 35.0f);
      jd.Initialize(b7, b8, anc);
      m_world->CreateJoint(&jd);
      

      //Pivot around which the lower platform revolves.
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      jointDef.enableMotor=true;
      jointDef.motorSpeed=10;
      jointDef.maxMotorTorque=2000;
      m_world->CreateJoint(&jointDef);
 
      
      //Pivot around which upper platform revolves
      b2RevoluteJointDef jointDef1;
      jointDef1.bodyA = body5;
      jointDef1.bodyB = body4;
      jointDef1.localAnchorA.Set(0,0);
      jointDef1.localAnchorB.Set(0,0);
      jointDef1.collideConnected = false;
      m_world->CreateJoint(&jointDef1);

      
      //Joint between the small body and the attaching rod
      b2RevoluteJointDef jointDef7;
      jointDef7.bodyA = b8;
      jointDef7.bodyB = body3;
      jointDef7.localAnchorA.Set(0,0);
      jointDef7.localAnchorB.Set(0,0);
      jointDef7.collideConnected = false;
      m_world->CreateJoint(&jointDef7);

      
      //Attachment between the upper platform and attaching rod
      b2RevoluteJointDef jointDef2;
      jointDef2.bodyA = body4;
      jointDef2.bodyB = body3;
      jointDef2.localAnchorA.Set(-2,0);
      jointDef2.localAnchorB.Set(5,0);
      jointDef2.collideConnected = false;
      m_world->CreateJoint(&jointDef2);

      
      //Attachment between the lower platform and the attaching rod.
      b2RevoluteJointDef jointDef3;
      jointDef3.bodyA = body;
      jointDef3.bodyB = body3;
      jointDef3.localAnchorA.Set(2,0);
      jointDef3.localAnchorB.Set(-5,0);
      jointDef3.collideConnected = false;
      m_world->CreateJoint(&jointDef3);
            
      
      }*/

	/*! \par The heavy sphere on the platform
	 * Var: sbody, Type: b2Body*, Desc: A pointer to the sphere.<br>
	 * Var: circle, Type: b2CircleShape, Desc: To define the radius of the sphere.<br>
	 * Var: ballfd, Type: b2FixtureDef, Desc: To define the fixture of the sphere.<br>
	 * Var: ballbd, Type: b2BodyDef, Desc: Holds the data needed to create the sphere.<br>
	 * */

	
	
    //The heavy sphere on the platform
    /*
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }
	*/
	/*! \par The see-saw system at the bottom
	 * The triangle wedge<br>
	 * ...Var: sbody, Type: b2Body*, Desc: A pointer to the triangle wedge.<br>
	 * ...Var: poly, Type: b2PolygonShape, Desc: To describe the shape of the wedge.<br>
	 * ...Var: vertices, Type: b2Vec2[], Desc: Array of vertices to denote the corners of the wedge.<br>
	 * ...Var: wedgefd, Type: b2FixtureDef, Desc: Defines the fixture of the wedge.<br>
	 * ...Var: wedgebd, Type: b2BodyDef, Desc: Holds the data needed to create the wedge.<br>
	 * Another wedge kept on the ground<br>
	 * ...Var: sbody1, Type: b2Body*, Desc: A pointer to the actual wedge.<br>
     * ...Var: poly1, Type: b2PolygonShape, Desc: Defines the shape of the polygon corresponding the wedge.<br>
     * ...Var: vertices, Type: b2Vec2[], Desc: Array of vertices of the wedge.<br>
     * ...Var: wedgefd1, Type: b2FixtureDef, Desc: Gives the fixture to the wedge.<br>
     * ...Var: wedgebd1, Type: b2BodyDef, Desc: Holds the data needed to create the wedge.<br>
     * The plank on top of the wedge<br>
     * ...Var: shape, Type: b2PolygonShape, Desc: Describes the shape of the plank. <br>
     * ...Var: bd2, Type: b2bodyDef, Desc: Holds the definition of the plank.<br>
     * ...Var: fd2, Type: b2FixtureDef*, Desc: A pointer to the fixture of the plank.<br>
     * Var: jd, Type: b2RevoluteJointDef, Desc: The joint between plank and the wedge.<br>
     * Var: anchor, Type: b2Vec2, Desc: A coordinate denoting the anchor point.<br>
     * The light box on the right side of the see-saw<br>
     * ...Var: shape2, Type: b2PolyogonShape, Desc: Sets the shape of the box.<br>
     * ...Var: bd3, Type: b2BodyDef, Desc: Holds the data needed for creation of the box.<br>
     * ...Var: body3, Type: b2Body*, Desc: A pointer to the box.<br>
     * ...Var: fd3, Type: b2FixtureDef*, Desc: A pointer to the fixture of the box.<br>
     * The wedge on the left side of the plank<br>
     * ...Var: poly5, Type: b2PolygonShape, Desc: Defines the polygon associated with the wedge.<br>
     * ...Var: vertices5, Type: b2Vec2[], Desc: Array of vertices of the wedge.<br>
     * ...Var: wedgefd5, Type: b2FixtureDef, Desc: Defines the fixture of the wedge.<br>
     * ...Var: wedgebd5, Type: b2BodyDef, Desc: Holds the data needed to create the wedge.<br>  
	 * */
    //The see-saw system at the bottom
    /*
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
      //sbody->SetActive(true);
	  //sbody->ApplyLinearImpulse( b2Vec2(0,50000.0f), sbody->GetWorldCenter(),true );
      
      //The fixed wedge kept on the ground
      b2Body* sbody1;
      b2PolygonShape poly1;
      b2Vec2 vertices1[3];
      vertices1[0].Set(0,0);
      vertices1[1].Set(10,0);
      vertices1[2].Set(0,2);
      poly1.Set(vertices1, 3);
      b2FixtureDef wedgefd1;
      wedgefd1.shape = &poly1;
      wedgefd1.density = 1.0f;
      wedgefd1.friction = 0.0f;
      wedgefd1.restitution = 0.0f;
      b2BodyDef wedgebd1;
      wedgebd1.type=b2_dynamicBody;
      wedgebd1.position.Set(-30.0f, 0.0f);
      sbody1 = m_world->CreateBody(&wedgebd1);
      sbody1->CreateFixture(&wedgefd1);


      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);
      
  

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
 
      //The wedge on the left side of the plank
     b2Body* sbody5;
      b2PolygonShape poly5;
      b2Vec2 vertices5[3];
      vertices5[0].Set(10,0);
      vertices5[1].Set(0,5);
      vertices5[2].Set(0,0);
      poly5.Set(vertices5, 3);
      b2FixtureDef wedgefd5;
      wedgefd5.shape = &poly5;
      wedgefd5.density = 0.0071f;
      wedgefd5.friction = 0.01f;
      wedgefd5.restitution = 0.0f;
      b2BodyDef wedgebd5;
      wedgebd5.type = b2_dynamicBody;
      wedgebd5.position.Set(17.0f, 2.0f);
      sbody5 = m_world->CreateBody(&wedgebd5);
      sbody5->CreateFixture(&wedgefd5);
    }
    */
    
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.type = b2_dynamicBody;
      bd.position.Set(28.0f, 30.0f);
      temp = m_world->CreateBody(&bd);
      temp->CreateFixture(&shape, 0.0f);
    

  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
  /*void dominos_t::callme(){
	cout <<"here\n";
	cout <<temp->GetPosition().y << endl;
}

	void dominos_t::step(settings_t* settings){
		base_sim_t::step(settings);
		//cout << temp->GetPosition().y << endl;
	}*/
}



