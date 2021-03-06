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
      shape.Set(b2Vec2(-90.0f, -15.0f), b2Vec2(90.0f, -15.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
  
      
      
     ///////////////////////Varun 's part /////////////////////////////
     // lower rod
/*! \par lower rod
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to lower rod.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the lower rod.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to lower rod.<br>
     */ 
	{
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(2.5f, 19.8f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
     
         // diagonal rod
/*! \par diagonal rod
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to diagonal rod.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the diagonal rod.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to diagonal rod.<br>
     */          
    {
      b2PolygonShape shape;
	  shape.SetAsBox(sqrt(2.8f), 0.20f);

      b2BodyDef bd;
      //bd.type= b2_dynamicBody;
      bd.position.Set(9.7f, 19.0f-0.125f);
      bd.angle = 140.4f* b2_pi/180.0f;
      b2Body* ground = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->filter.categoryBits = 0x0003;
      //fd->filter.maskBits = 0x0003;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      ground->CreateFixture(&shape, 0.0f);

}
         //upper rod
/*! \par upper rod
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to upper rod.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the upper rod.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to upper rod.<br>
      * Var: vertices, Type: b2Vec2[], Desc: Array of vertices to denote the corners .<br>
     * Var:s , Type: b2PolygonShape,Desc: A shape using the vertices.<br>
      * Var: shape1, Type: b2PolygonShape, Desc: The shape corresponding to small object which keeps the bullet from going ahead in the barrel.<br>
     * Var: bd1, Type: b2BodyDef, Desc: The actual body denoting the upper rod.<br>
     * Var: ground1, Type: b2Body*, Desc: This is a pointer to upper rod.<br>
     */ 
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.0f, 0.25f);
      
      b2Vec2 vertices[3];
      vertices[0].Set(2.7f,0.15f);
      vertices[1].Set(3.7f,0.15f);
      vertices[2].Set(2.9f,-0.35f);
	  b2PolygonShape s;
	  s.Set(vertices,3);
	  
	  
      b2BodyDef bd;
      bd.position.Set(2.0f, 22.6f);
      b2Body* ground = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->filter.categoryBits = 0x0007;
      fd->filter.maskBits = 0x0001;
      //fd->filter.groupIndex = -1;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      ground->CreateFixture(fd);
      fd->shape = &s;
      ground->CreateFixture(fd);
      
      
      
      
      b2PolygonShape shape1;
      shape1.SetAsBox(0.2f, 0.25f);

      b2BodyDef bd1;
      bd1.position.Set(6.5f, 20.3f);
      b2Body* ground1 = m_world->CreateBody(&bd1);
      b2FixtureDef *fd1 = new b2FixtureDef;
      
      fd1->shape = new b2PolygonShape;
      fd1->shape = &shape1;
      ground1->CreateFixture(fd1);
      
      
      
    }
    
    
    //Extensions to the barrel's upper and lower rods
/*! \par Extensions to the barrel's upper rods
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to Extensions to the barrel's upper rod.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the Extensions to the barrel's upper rod.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to Extensions to the barrel's upper rod.<br>
     */ 
    {
		
	  b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-3.0f, 22.6f);
      b2Body* ground = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      ground->CreateFixture(fd);
      
  }
 /*! \par Extensions to the barrel's lower rod
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to Extensions to the barrel's lower rod.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the Extensions to the barrel's lower rod.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to Extensions to the barrel's lower rod.<br>
     */ 
  {
	
	      b2PolygonShape shape;
      shape.SetAsBox(3.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-5.9f, 19.8f);
      b2Body* ground = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      ground->CreateFixture(fd);
      
  }
    
	//bullet magazine
/*! \par Bullet Magazine
     * Var: shape2, Type: b2PolygonShape, Desc: The shape corresponding to bullet magazine.<br>
     * Var: bd3, Type: b2BodyDef, Desc: The actual body denoting the bullet magazine.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to Extensions to the barrel's lower rod.<br>
     * Var :jointDef2, Type: b2PrismaticJointDef , Desc: This defines the joints in the bullet magazine <br>
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to Extensions to the base.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting thebase.<br>
     * Var: ground1, Type: b2Body*, Desc: This is a pointer to base.<br>
     *Var : jointDef,Type:b2DistanceJointDef, Desc:This is distance joints function as spring in the bullet magazine <br>
     * Var : jointDef,Type:b2DistanceJointDef, Desc:This is distance joints function as spring in the bullet magazine <br>
     */ 
    {
	  b2PolygonShape shape2;
      shape2.SetAsBox(2.5f, 0.25f);
      b2BodyDef bd3;
      bd3.position.Set(14.0f, 12.5f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
      
      b2PrismaticJointDef jointDef2;
		b2Vec2 worldAxis(0.0f, 1.0f);
		jointDef2.Initialize(body3	, b1, body3->GetWorldCenter(), worldAxis);
		m_world->CreateJoint(&jointDef2);

    //base
    
      b2PolygonShape shape;
      shape.SetAsBox(3.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(14.0f, -2.8f);
      b2Body* ground1 = m_world->CreateBody(&bd);
      ground1->CreateFixture(&shape, 0.0f);
      b2Vec2 b1;
      b1.Set(13.0f,12.5f);
      b2Vec2 b2;
      b2.Set(13.0f,5.5f);
      b2Vec2 b3;
      b3.Set(15.0f,12.5f);
      b2Vec2 b4;
      b4.Set(15.0f,5.5f);
      
	 ///// These distance joints function as spring in the bullet magazine/////////// 
		b2DistanceJointDef jointDef;
		jointDef.Initialize(body3, ground1, b1, b2);
		jointDef.collideConnected = true;
		jointDef.frequencyHz = 6.5f;
		jointDef.dampingRatio = 2.0f;
		jointDef.length = 12.0f;
		m_world->CreateJoint(&jointDef);

		b2DistanceJointDef jointDef1;
		jointDef1.Initialize(body3, ground1, b3, b4);
		jointDef1.collideConnected = true;
		jointDef1.frequencyHz = 6.5f;
		jointDef1.dampingRatio = 2.0f;
		jointDef1.length = 12.0f;
		m_world->CreateJoint(&jointDef1);

    }
/*! \par Left Boundary
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to left boundary.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the left boundary.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to left boundary.<br>
     */ 

	//left boundary
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f, 10.0f);

      b2BodyDef bd;
      bd.position.Set(11.0f, 7.5f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    /*! \par Right Boundary
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to Right boundary.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the Right boundary.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to Right boundary.<br>
     */ 
    //right boundary
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f,10.5f);

      b2BodyDef bd;
      bd.position.Set(17.0f, 8.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    /*! \par shell of the bullet
     * Var: shape2, Type: b2PolygonShape, Desc: The shape corresponding to shell of the bullet.<br>
     * Var: bd3, Type: b2BodyDef, Desc: The actual body denoting the shell of the bullet.<br>
     * Var: fd3, Type: b2FixtureDef *, Desc: This is a pointer to shell of the bullet.<br>
     */ 
     
     /*! \par Triangular part of the bullet 
     * Var: poly5, Type: b2PolygonShape, Desc: The shape corresponding to triangular part of the bullet.<br>
     * * Var: vertices5, Type: b2Vec2[], Desc: Array of vertices to denote the  triangular part of the bullet.<br>
     * Var: wedgebd5, Type: b2BodyDef, Desc: The actual body denoting the triangular part  of the bullet.<br>
     * Var: wedgefd5, Type: b2FixtureDef *, Desc: This is a pointer to triangular part  of the bullet.<br>
     * 
     */ 
    
	for (int i = 0; i<3 ; i++){
		// The shell of the bullet////
	  b2PolygonShape shape2;
      shape2.SetAsBox(1.5f, 1.0f);
      b2BodyDef bd3;
      bd3.position.Set(14.5f, 17.5f - 2*i);
      bd3.type = b2_dynamicBody;
      body_bul[i] = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.2f;
      fd3->friction = 0.4f;
      fd3->restitution = 0.000001f;
      //fd3->filter.categoryBits = 0x0009;
      //fd3->filter.maskBits = 0x0006 | 0x0001;
      fd3->filter.groupIndex = i-1;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body_bul[i]->CreateFixture(fd3);
      int myint1 = 118+i;
      body_bul[i]->SetUserData((void*)myint1);
      
		//triangle which makes the actual flying part of the bullet////
      
      b2PolygonShape poly5;
      b2Vec2 vertices5[3];
      vertices5[0].Set(0.0f,0.0f);
      vertices5[1].Set(2.0f,0.9f);
      vertices5[2].Set(2.0f,-0.6f);
      poly5.Set(vertices5, 3);
      b2FixtureDef wedgefd5;
      wedgefd5.shape = &poly5;
      //wedgefd5.filter.categoryBits = 0x0002;
      //wedgefd5.filter.maskBits = 0x0002;
      wedgefd5.density = 0.1f;
      b2BodyDef wedgebd5;
      wedgebd5.type = b2_dynamicBody;
      wedgebd5.position.Set(11.0f, 17.5f - 2*i);
      body_bulhead[i] = m_world->CreateBody(&wedgebd5);
      body_bulhead[i]->CreateFixture(&wedgefd5);
      b2Vec2 a1;
      a1.Set(14.0f,18.5f - 2*i);
      b2Vec2 a2;
      a2.Set(14.0f,16.5f - 2*i);
      
	  jointDef_bul1.Initialize(body_bul[i], body_bulhead[i], a1);
	  jointDef_bul2.Initialize(body_bul[i], body_bulhead[i], a2);


	  //joint_1 and joint_2 are b2RevoluteJoint* type variables. Creating these 
	  //facilitates their destruction later when bullet is fired. //
	  joint_1[i] = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef_bul1); 
	  joint_2[i] = (b2RevoluteJoint*)m_world->CreateJoint(&jointDef_bul2);

}



       //The stopper for the hammer i.e. which holds the hammer back till the user presses trigger ****************
       /*! \par stopper for the hammer
     * Var: vertices_pb, Type: b2Vec2[], Desc: Array of vertices to denote the  stopper for the hammer.<br>
     * Var: vertices_pb1, Type: b2Vec2[], Desc: Array of vertices to denote the  stopper for the hammer.<br>
     * Var: vertices3, Type: b2Vec2[], Desc: Array of vertices to denote the  stopper for the hammer.<br>
     * Var:polygon_pb , Type:b2PolygonShape, Desc: The shape corresponding to stopper for the hammer <br>
     *  Var:polygon_pb1 , Type:b2PolygonShape, Desc: The shape corresponding to stopper for the hammer <br>
     *  Var:polygon3 , Type:b2PolygonShape, Desc: The shape corresponding to stopper for the hammer <br>
     * Var: bd_pd, Type: b2BodyDef, Desc: The actual body denoting the stopper of the hammer.<br>
     * Var: fd_pd, Type: b2FixtureDef *, Desc: This is a pointer to stopper of the hammer.<br>
     *  Var: bdx3, Type: b2BodyDef, Desc: The actual body denoting the spring part of the hammer.<br>
     * Var:jointDef3xy , Type:b2RevoluteJointDef ,Desc:The actual joint denoting the hammer joints<br>
     */
     
     /*! \par Small joining rod
      * Var: shapenw, Type: b2PolygonShape, Desc: The shape corresponding to Small joining rod.<br>
     * Var: bodynew, Type: b2BodyDef, Desc: The actual body denoting the Small joining rod.<br>
     *  Var: arod, Type: b2BodyDef, Desc: The actual body denoting the  small joint rod .<br>
     * Var:jointDefrod , Type:b2RevoluteJointDef ,Desc:The actual joint denoting thesmall  joint rod <br>
     */     
      {
		 
		
		
		b2Vec2 vertices_pb[4];
			vertices_pb[0].Set(-0.5f, 0.5f);

			vertices_pb[1].Set(0.5f, 0.5f);

			vertices_pb[2].Set(1.5f, -3.8f);
			vertices_pb[3].Set(0.5f, -3.8f);
			
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
		fd_pb->filter.groupIndex = -4;
		fd_pb->density = 0.1f;
		fd_pb->shape = new b2PolygonShape;
		fd_pb->shape = &polygon_pb1;
		bodyy->CreateFixture(fd_pb);
		fd_pb->shape = &polygon_pb;
		bodyy->CreateFixture(fd_pb);
		fd_pb->shape = &polygon3;
		bodyy->CreateFixture(fd_pb);
		int myint_bodyy=128;
		bodyy->SetUserData((void*)myint_bodyy);
		
		// This part functions as the spring which keeps the trigger tight.
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
      shaperod.SetAsBox(3.0f,0.5f);
      arod.position.Set(24.0f,10.0f);
      attachrod = m_world->CreateBody(&arod);
      b2FixtureDef *fd_arod = new b2FixtureDef;
      fd_arod->filter.groupIndex = -4;
      fd_arod->density = 0.2f;
      fd_arod->shape = new b2PolygonShape;
      fd_arod->shape = &shaperod;
      attachrod->CreateFixture(fd_arod);
      
      b2RevoluteJointDef jointDefrod;
      jointDefrod.bodyA = attachrod;
      jointDefrod.bodyB = bodyy;
      jointDefrod.localAnchorA.Set(2.8f,0.0f);
      jointDefrod.localAnchorB.Set(1.0f,-3.5f);
      jointDefrod.collideConnected = false;
      
      m_world->CreateJoint(&jointDefrod);
      
      //the small object just below the attaching rod///
      b2PolygonShape shapenew;
      shapenew.SetAsBox(0.2f, 0.2f);
      b2BodyDef bdnew;
      bdnew.position.Set(28.0f, 9.0f);
      b2Body* bodynew = m_world->CreateBody(&bdnew);
      b2FixtureDef *fdnew = new b2FixtureDef;
      fdnew->filter.groupIndex = -4;
      fdnew->density = 0.1f;
      fdnew->shape = new b2PolygonShape;
      fdnew->shape = &shapenew;
      bodynew->CreateFixture(fdnew);
      
     
		  
	  }
      
      
      
      
      
      
      
      ///////// The Trigger i.e the part which is available to the user to press/////////
      
       /*! \par The Trigger
      * Var: vertices_pb, Type: b2Vec2[], Desc: Array of vertices to denote  The Trigger.<br>
     * Var: vertices_pb1, Type: b2Vec2[], Desc: Array of vertices to denote The Trigger.<br>
     * Var: polygon_pb, Type: b2PolygonShape, Desc: The shape corresponding to The Trigger.<br>
     * Var: bd_pb, Type: b2BodyDef, Desc: The actual body denoting The Trigger.<br>
     *  Var: fd_pd, Type: b2FixtureDef *, Desc: This is a pointer to stopper The Trigger.<br>
     * Var: body2, Type: b2Body*, Desc: This is a pointer to The Trigger.<br>
     * Var:jointDefrod , Type:b2RevoluteJointDef ,Desc:The actual joint denoting the Trigger<br>
     * Var:jointDeft , Type:b2RevoluteJointDef ,Desc:The actual joint denoting the Trigger<br>
     */ 
      {
		  
		  b2Vec2 vertices_pb[4];
			vertices_pb[0].Set(-0.5f, 0.5f);

			vertices_pb[1].Set(0.5f, 0.5f);

			vertices_pb[2].Set(-1.5f, -2.5f);
			vertices_pb[3].Set(-2.5f, -2.5f);
			
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
		bd_pb.position.Set(25.0f,10.0f);
		bd_pb.type = b2_dynamicBody;
		body_t = m_world->CreateBody(&bd_pb);
		b2FixtureDef *fd_pb = new b2FixtureDef;
		fd_pb->filter.groupIndex = -4;
		fd_pb->density = 0.1f;
		fd_pb->shape = new b2PolygonShape;
		fd_pb->shape = &polygon_pb1;
		
		body_t->CreateFixture(fd_pb);
		fd_pb->shape = &polygon_pb;
		body_t->CreateFixture(fd_pb);
		
		
		b2BodyDef bd2;
      bd2.position.Set(26.0f, 6.8f);
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
      jointDefrod.localAnchorA.Set(-2.2f,0.0f);
      jointDefrod.localAnchorB.Set(-1.0f,3.5f);
      jointDefrod.collideConnected = false;
      
      m_world->CreateJoint(&jointDefrod);
		  
		  
	  }
      
      
      
      
      
      
      //The trigger mechanism
      
      /*! \par The trigger mechanism
      * Var: vertices, Type: b2Vec2[], Desc: Array of vertices to denote  The Trigger mechanism.<br>
     * Var: vertices1, Type: b2Vec2[], Desc: Array of vertices to denote The Trigger mechanism .<br>
     * Var: vertices2, Type: b2Vec2[], Desc: Array of vertices to denote The Trigger mechanism .<br>
      * Var: vertices3, Type: b2Vec2[], Desc: Array of vertices to denote The Trigger mechanism .<br>
       * Var: vertices5, Type: b2Vec2[], Desc: Array of vertices to denote The Trigger mechanism .<br>
        * Var: polygon, Type: b2PolygonShape, Desc: The shape corresponding to The Trigger mechanism.<br>
        * Var: polygon1, Type: b2PolygonShape, Desc: The shape corresponding to The Trigger mechanism.<br>
        * Var: polygon2, Type: b2PolygonShape, Desc: The shape corresponding to The Trigger mechanism.<br>
        * Var: polygon3, Type: b2PolygonShape, Desc: The shape corresponding to The Trigger mechanism.<br>
        * Var: polygon4, Type: b2PolygonShape, Desc: The shape corresponding to The Trigger mechanism.<br>
        * Var: polygon5, Type: b2PolygonShape, Desc: The shape corresponding to The Trigger mechanism.<br>
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to Right boundary.<br>
     * Var: bd, Type: b2BodyDef, Desc: The actual body denoting the Right boundary.<br>
     * Var: ground, Type: b2Body*, Desc: This is a pointer to Right boundary.<br>
     */ 
     
     /*! \par Lower Block
     * Var: shape, Type: b2PolygonShape, Desc: The shape corresponding to Lower Block.<br>
     * Var: bd3, Type: b2BodyDef, Desc: The actual body denoting the Lower Block.<br>
     * Var: fd3, Type: b2FixtureDef *, Desc: This is a pointer to Lower Block.<br>
     *   Var: body3, Type: b2body *, Desc: This is a pointer to Lower Block.<br>
     */ 
     
     
	   /*! \par Upper Block
     * Var: bd31, Type: b2BodyDef, Desc: The actual body denoting the Upper Block.<br>
     * Var: fd31, Type: b2FixtureDef *, Desc: This is a pointer to Upper Block.<br>
     *   Var: body31, Type: b2body *, Desc: This is a pointer to Upper Block.<br>
     */  
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
  
      
      
       //back stopper (stops the pullback mecahnism from breaking away*******************
      /*! \par back stopper (stops the pullback mecahnism from breaking away)
     * Var: shape2, Type: b2PolygonShape, Desc: The shape corresponding to back stopper<br>
     * Var: bd32, Type: b2BodyDef, Desc: The actual body denoting the back stopper.<br>
     * Var: fd32, Type: b2FixtureDef *, Desc: This is a pointer to back stopper.<br>
     *   Var: body32, Type: b2body *, Desc: This is a pointer to back stopper.<br>
     */  
      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 5.0f);
      
      b2BodyDef bd32;
      bd32.position.Set(44.0f, 24.0f);
      //bd3.type = b2_dynamicBody;
      b2Body* body32 = m_world->CreateBody(&bd32);
      b2FixtureDef *fd32 = new b2FixtureDef;
      //fd32->filter.categoryBits = 0x0002;
      //fd32->filter.maskBits = 0x0001;
      fd32->density = 0.1f;
      fd32->shape = new b2PolygonShape;
      fd32->shape = &shape2;
      body32->CreateFixture(fd32);
      //body32->SetActive(false);
 
     
		//The pullback mechanism  ************************
		
		/*! \par The pullback mechanism
		 * Variable: polygon_pb, Type: b2PolygonShape, Desc: Polygon made using thae array of verticies verticies_pb[]<br>
		 * Variable: polygon_pb1, Type: b2PolygonShape, Desc: Polygon made using thae array of verticies verticies_pb1[]<br>
		 * Variable: polygon_pb2, Type: b2PolygonShape, Desc: Polygon made using thae array of verticies verticies_pb2[]<br>
		 * Variable: polygon_pb3, Type: b2PolygonShape, Desc: Polygon made using thae array of verticies verticies_pb3[]<br>
		 * Variable: polygon_pb4, Type: b2PolygonShape, Desc: Polygon made using thae array of verticies verticies_pb4[]<br>
		 * Variable: bd_pb2, Type: b2BodyDef, Desc: The body definition of the pin hitting the bullet<br>
		 * Variable: fd_pb2, Type: b2FixtureDef; Desc: Fixture of the above body<br>
		 */
		
		
		b2Vec2 vertices_pb[3];
			vertices_pb[0].Set(0.0f, 0.0f);

			vertices_pb[1].Set(3.0f, 2.0f);

			//vertices_pb[2].Set(1.0f, 2.0f);
			vertices_pb[2].Set(0.0f, 2.0f);
			
			b2Vec2 vertices_pb2[4];
			vertices_pb2[0].Set(0.0f, 2.0f);

			vertices_pb2[1].Set(1.0f, 2.0f);

			vertices_pb2[2].Set(1.0f, 4.0f);
			vertices_pb2[3].Set(0.0f, 4.0f);
			
		b2Vec2 vertices_pb1[4];
			vertices_pb1[0].Set(0.0f, 2.0f);

			vertices_pb1[1].Set(23.0f, 2.0f);

			vertices_pb1[2].Set(23.0f, 1.0f);
			vertices_pb1[3].Set(0.0f, 1.0f);
			
		b2Vec2 vertices_pb3[4];
			vertices_pb3[0].Set(0.0f, 2.0f);

			vertices_pb3[1].Set(23.0f, 2.0f);

			vertices_pb3[2].Set(23.0f, 1.0f);
			vertices_pb3[3].Set(0.0f, 1.0f);
			
		b2Vec2 vertices_pb4[4];
			vertices_pb4[0].Set(0.0f, 1.65f);

			vertices_pb4[1].Set(-6.0f, 1.65f);

			vertices_pb4[2].Set(-6.0f, 1.05f);
			vertices_pb4[3].Set(0.0f, 1.05f);
		
		
		//b2Body* body_pb;
		
		b2PolygonShape polygon_pb;
		polygon_pb.Set(vertices_pb,3);
		
		b2PolygonShape polygon_pb1;
		polygon_pb1.Set(vertices_pb1,count);
		
		b2PolygonShape polygon_pb2;
		polygon_pb2.Set(vertices_pb2,count);
		
		
		b2PolygonShape polygon_pb3;
		polygon_pb3.Set(vertices_pb3,count);
		
		b2PolygonShape polygon_pb4;
		polygon_pb4.Set(vertices_pb4,count);
		
		
		
		// The bullet hitting pin
		b2BodyDef bd_pb2;
		bd_pb2.position.Set(15.5f,21.73f);
		bd_pb2.type = b2_dynamicBody;
		body_pb2 = m_world->CreateBody(&bd_pb2);
		b2FixtureDef *fd_pb2 = new b2FixtureDef;
		
		fd_pb2->density = 0.1f;
		fd_pb2->restitution = 0.9f;
		fd_pb2->shape = new b2PolygonShape;
		fd_pb2->shape = &polygon_pb3;
		body_pb2->CreateFixture(fd_pb2);
		
		int myint=108;
		body_pb2->SetUserData((void*)myint);
		/*!
		 * \par Rods in pullback
		 * variable: bd_pb, Type: b2BodyDef, Desc: Definiton of the lower ord of pullback mechanism<br>
		 * Variable: fd_pb, Type: b2FixtureDef; Desc: Fixture definition of the lower rod<br>
		 * Variable : body_pb, Type: b2Body, Desc: lower rod<br>
		 * variable: bd_pb1, Type: b2BodyDef, Desc: Definiton of the upper ord of pullback mechanism<br>
		 * Variable: fd_pb1, Type: b2FixtureDef; Desc: Fixture definition of the upper rod<br>
		 * Variable : body_pb1, Type: b2Body, Desc: upper rod<br>
		 * These use the above defined shapes polygon_pb,etc<br>
		 * Varible: myint_body_pb, Type: int, Desc: variable used for identifying body_pb<br>
		 */	
		
		
		// The lower rod in the pullback mechanism
		b2BodyDef bd_pb;
		bd_pb.position.Set(13.5f,20.0f);
		bd_pb.type = b2_dynamicBody;
		body_pb = m_world->CreateBody(&bd_pb);
		b2FixtureDef *fd_pb = new b2FixtureDef;
		//fd_pb->filter.categoryBits = 0x0004;
        //fd_pb->filter.maskBits = 0x0001;
        fd_pb->filter.groupIndex = -2;
		fd_pb->density = 0.1f;
		fd_pb->shape = new b2PolygonShape;
		fd_pb->shape = &polygon_pb1;
		body_pb->CreateFixture(fd_pb);
		fd_pb->shape = &polygon_pb;
		
		body_pb->CreateFixture(fd_pb);
		int myint_body_pb=138;
		body_pb->SetUserData((void*)myint_body_pb);
		
		
		

		// The upper rod in the pullback mechanism
		b2BodyDef bd_pb1;
		bd_pb1.position.Set(13.5f,22.0f);
		bd_pb1.type = b2_dynamicBody;
		body_pb1 = m_world->CreateBody(&bd_pb1);
		b2FixtureDef *fd_pb1 = new b2FixtureDef;
		fd_pb1->filter.categoryBits = 0x0006;
        fd_pb1->filter.maskBits = 0x0001;
        fd_pb1->filter.groupIndex = -1;
		fd_pb1->density = 0.1f;
		fd_pb1->shape = new b2PolygonShape;
		fd_pb1->shape = &polygon_pb1;
		body_pb1->CreateFixture(fd_pb1);
		fd_pb1->shape = &polygon_pb2;
		body_pb1->CreateFixture(fd_pb1);
		fd_pb1->shape = &polygon_pb4;
		body_pb1->CreateFixture(fd_pb1);
		
		
		/////The bullet stopper just above the magazine///////
		/*!
		 * \par bullet stopper<br>
		 * Var: bulstop, Type: b2BodyDef, Desc: Body definiton of the stopper<br>
		 * Var: bulletstopper, Type: b2Body*, Desc: the stopper<br>
		 * Var: fd_bulstop, Type: b2FixtureDef, Desc: Fixture definiton of the stopper<br>
		 * Var: s_bulstop, Type: b2PolygonShape, Desc: Shape of the stopper<br>
		 */
		b2BodyDef bulstop;
		bulstop.position.Set(15.5,20.0f);
		b2Body* bulletstopper = m_world->CreateBody(&bulstop);
		b2FixtureDef *fd_bulstop = new b2FixtureDef;
		fd_bulstop->filter.groupIndex = -2;
		b2PolygonShape *s_bulstop= new b2PolygonShape;
		s_bulstop->SetAsBox(2.0f,0.2f);
		fd_bulstop->shape = s_bulstop;
		bulletstopper->CreateFixture(fd_bulstop);
		
		
		
		// These distance joints act as springs to keep the pullback mechanism go forward if force on it becomes zero.
		/*!
		 * \par springs
		 * Var: joint4, Type: b2DistanceJointDef, Desc: A spring (distance joint) used in pullback mechanism to move it in forward direction when force is 0<br>
		 * Var: joint5, Type: b2DistanceJointDef, Desc: Second spring<br>
		 * Var: point6, Type: const b2Vec2, Desc: one end of joint4<br>
		 * Var: point7, Type: const b2Vec2, Desc: other end of joint4<br>
		 * Var: point8, Type: const b2Vec2, Desc: one end of joint5<br>
		 * Var: point9, Type: const b2Vec2, Desc: other end of joint5<br>
		 */
		b2DistanceJointDef joint4; 
		const b2Vec2 point6(13.5f, 20.5f); 
		const b2Vec2 point7(22.5f, 20.0f); 
		joint4.Initialize(body_pb, body3, point6, point7); 
		joint4.collideConnected = true; 
		joint4.frequencyHz = 1.1f; 
		joint4.dampingRatio =1.5f;
		joint4.length = 12.5f;
		 m_world->CreateJoint(&joint4);
		 
		 
		 b2DistanceJointDef joint5; 
		const b2Vec2 point8(13.5f, 24.5f); 
		const b2Vec2 point9(22.5f, 23.5f); 
		joint5.Initialize(body_pb1, body31, point8, point9); 
		joint5.collideConnected = true; 
		joint5.frequencyHz = 1.1f; 
		joint5.dampingRatio =1.5f;
		joint5.length = 12.5f;
		 m_world->CreateJoint(&joint5);
		 
		 
		 // These four joints ie. joint6 -joint 9 keep the hitting pin attached to the pullback mechanism's upper 
		 // and lower rods.
		 
		 /*!
		  * \par joints between hitting pin and rods
		  * Var: joint6 , Type: b2DistanceJointDef, Desc: used to attach body_pb1, body_pb2 i.e for lower block<br>
		  * Var: joint7 , Type: b2DistanceJointDef, Desc: used to attach body_pb, body_pb2 i.e for upper block<br>
		  * Var: joint8 , Type: b2DistanceJointDef, Desc: used to attach body_pb1, body_pb2 i.e for lower block<br>
		  * Var: joint9 , Type: b2DistanceJointDef, Desc: used to attach body_pb, body_pb2 i.e for upper block<br>
		  * Var: point10-point17, Type: const b2Vec2, Desc: Endpoints of above joints<br>
		  */
		 b2DistanceJointDef joint6; 
		const b2Vec2 point10(24.0f, 23.5f); 
		const b2Vec2 point11(16.5f, 23.2f); 
		joint6.Initialize(body_pb1, body_pb2, point10, point11); 
		joint6.collideConnected = true; 
		joint6.frequencyHz = 0.5f; 
		joint6.dampingRatio =0.8f;
		joint6.length = 10.0f;
		 m_world->CreateJoint(&joint6);
		 
		 
		 b2DistanceJointDef joint7; 
		const b2Vec2 point12(24.0f, 21.5f); 
		const b2Vec2 point13(16.5f, 23.2f); 
		joint7.Initialize(body_pb, body_pb2, point12, point13); 
		joint7.collideConnected = true; 
		joint7.frequencyHz = 0.5f; 
		joint7.dampingRatio =0.8f;
		joint7.length = 10.0f;
		 m_world->CreateJoint(&joint7);
		 
		 b2DistanceJointDef joint8; 
		const b2Vec2 point14(24.0f, 21.5f); 
		const b2Vec2 point15(31.5f, 23.2f); 
		joint8.Initialize(body_pb1, body_pb2, point14, point15); 
		joint8.collideConnected = true; 
		joint8.frequencyHz = 0.5f; 
		joint8.dampingRatio =0.8f;
		joint8.length = 10.0f;
		 m_world->CreateJoint(&joint8);
		 
		  b2DistanceJointDef joint9; 
		const b2Vec2 point16(24.0f, 23.5f); 
		const b2Vec2 point17(31.5f, 23.2f); 
		joint9.Initialize(body_pb, body_pb2, point16, point17); 
		joint9.collideConnected = true; 
		joint9.frequencyHz = 0.5f; 
		joint9.dampingRatio =0.8f;
		joint9.length = 10.0f;
		 m_world->CreateJoint(&joint9);
		 
		 
		 
		 //These three prismatic joints forbids the motion of the pullback
		 // mechanism in vertical axis.
		 /*!
		  * \par prismatic joints
		  * Var: jointDef_pb, Type: b2PrismaticJointDef, Desc: Restricts vertical motion for body_pb i.e upper body<br>
		  * Var: jointDef_pb1, Type: b2PrismaticJointDef, Desc: Restricts vertical motion for body_pb i.e lower body<br>
		  * Var: jointDef_pb2, Type: b2PrismaticJointDef, Desc: Restricts vertical motion for body_pb i.e hitting pin<br>
		  */
		 
		 b2PrismaticJointDef jointDef_pb;
		//b2Vec2 worldAxis(1.0f, 0.0f);
		jointDef_pb.bodyA = body_pb;
		jointDef_pb.bodyB = body3;
		jointDef_pb.localAnchorA.Set(0.0f,0.9f);
		jointDef_pb.localAnchorB.Set(0.0f,0.0f);
		//jointDef_pb2.Initialize(body_pb2, b1,body_pb2->GetWorldCenter(), worldAxis);
		m_world->CreateJoint(&jointDef_pb);
		
		b2PrismaticJointDef jointDef_pb1;
		//b2Vec2 worldAxis(1.0f, 0.0f);
		jointDef_pb1.bodyA = body_pb1;
		jointDef_pb1.bodyB = body31;
		jointDef_pb1.localAnchorA.Set(0.0f,2.2f);
		jointDef_pb1.localAnchorB.Set(0.0f,0.0f);
		//jointDef_pb2.Initialize(body_pb2, b1,body_pb2->GetWorldCenter(), worldAxis);
		m_world->CreateJoint(&jointDef_pb1);
		
		b2PrismaticJointDef jointDef_pb2;
		//b2Vec2 worldAxis(1.0f, 0.0f);
		jointDef_pb2.bodyA = body_pb2;
		jointDef_pb2.bodyB = body31;
		jointDef_pb2.localAnchorA.Set(0.0f,3.3f);
		jointDef_pb2.localAnchorB.Set(0.0f,0.0f);
		//jointDef_pb2.Initialize(body_pb2, b1,body_pb2->GetWorldCenter(), worldAxis);
		m_world->CreateJoint(&jointDef_pb2);
		 
		 
		 
		 
		 
		 
		 
	  // The hammer which hits the hitting pin once the trigger is pressed.
	  
	  /*!
	   * \par Hammer 
	   * Var: bdx, Type:b2BodyDef, Desc: body definition of hammer  <br>
	   * Var: bodyx, Type:b2Body, Desc: main body of hammer  <br>
	   * Var: fdx, Type: new b2FixtureDef, Desc: Fixture of the hammer<br>
	   */
	  b2BodyDef bdx;
      bdx.position.Set(34.0f, 20.0f);
      bdx.type = b2_dynamicBody;
      bodyx = m_world->CreateBody(&bdx);
      b2FixtureDef *fdx = new b2FixtureDef;
      fdx->density = 0.3f;
      fdx->shape = new b2PolygonShape;
      fdx->shape = &polygon;
      //fdx->restitution = 0.9f;
      bodyx->CreateFixture(fdx);
      //fdx->shape = &polygon1;
      //bodyx->CreateFixture(fdx);
      fdx->shape = &polygon2;
      bodyx->CreateFixture(fdx);

    
      // Small object which prevents the trigger stopper to go beyond certain distance.
      /*!
       * \par The object which prevents the trigger to go beyond certain distance
       * Var: shapenew, Type: b2PolygonShape, Desc: The shape of the body<br>
       * Var: bodynew,Type: b2Body*, Desc: the body of the object<br>
       * Var: fdnew, Type: b2FixtureDef*, Desc: the fixture of the body<br>  
       */
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
		
		
      //This is the part which defines where the hammer will be attached and the power 
      // with which it will strike at the hitting pin. Instead of spring motor has been
      // used to deliver the torque to the hammer.
      /*!
       * \par Motor delivering the torque
       * var:bd2, Type: b2BodyDef, Desc: Body definition of the body <br>
       * var:body2, Type: b2Body*, Desc: Main body of the body <br>
       * Var: jointDef, Type: b2RevoluteJointDef, Desc: The joint which generates the torque<br>
       */
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
      
      /////// The handle which the user holds while pressing the trigger////////////////
		/*!
	   * \par Handle
	   * Var: vertices[], Type: array of b2Vec2, Desc: verticies of the handle<br>
	   * Var: shape, Type: b2PolygonShape, Desc: shape of the handle<br>
	   * Var: body, Type:  b2Body*, Desc: Main body of the hammer<br>
	   * Var: fd, Type: b2FixtureDef, Desc: Fixture of the hammer<br>
	   */
      {
		  b2Vec2 vertices[4];
			vertices[0].Set(0.0f, 0.0f);

			vertices[1].Set(8.0f, 0.0f);

			vertices[2].Set(0.0f, 20.0f);
			vertices[3].Set(-8.0f, 20.0f);
			b2PolygonShape shape;
			shape.Set(vertices,4);
			b2BodyDef bd;
			  bd.position.Set(30.0f, -1.0f);
			  //bd.type = b2_dynamicBody;
			  b2Body* body = m_world->CreateBody(&bd);
			  b2FixtureDef *fd = new b2FixtureDef;
			  fd->filter.groupIndex = -4;
			  fd->density = 0.3f;
			  fd->shape = new b2PolygonShape;
			  fd->shape = &shape;
			  body->CreateFixture(fd);
		  
	  }
	  
	  /////////////// The bricks (just to make the simulation more interesting)///////////////
	  {
		  
		  /*!
		   * \par Bricks
		   * Var: shape, Type: b2PolygonShape, Desc: Shape of the bricks<br>
		   * Var: g,Type: b2Body*, Desc: ths body of the brick<br>
		   * Var: fd, Type: b2FixtureDef, Desc: the fixture of the brick<br>
		   */
		  b2PolygonShape shape;
				shape.SetAsBox(20.0f,2.0f);
				b2BodyDef bd;
			  bd.position.Set(-30.0f, 9.5f);
			  //bd.type = b2_dynamicBody;
			  b2Body* g = m_world->CreateBody(&bd);
			  b2FixtureDef *fd = new b2FixtureDef;
			  fd->density = 0.3f;
			  fd->shape = new b2PolygonShape;
			  fd->shape = &shape;
			  g->CreateFixture(fd);
			  b2Body* body[50];
		  for(int i = 0;i<10;i++){
			  for(int j =0 ;j <5;j++){
				  b2PolygonShape shape;
				  shape.SetAsBox(1.0f,1.0f);
				  b2BodyDef bd;
				  bd.position.Set(-30.0f+2.4*j, 12.0f+2.4*i);
				  bd.type = b2_dynamicBody;
				  body[5*i+j]= m_world->CreateBody(&bd);
				  b2FixtureDef *fd = new b2FixtureDef;
				  
				  fd->density = 0.001f;
				  fd->shape = new b2PolygonShape;
				  fd->shape = &shape;
				  body[5*i+j]->CreateFixture(fd);
				}
			}
	  
		}
      
      // calling the similar-to-constructor function for personalized contact listener instance.//
      myContactListenerInstance.MycontactListener(body_pb,body_pb1);
	  m_world->SetContactListener(&myContactListenerInstance);
     
    
     

  }
  // Defining the keyboard events//
  
  // 'q' loads the gun
  // 't' fires the bullet
  void dominos_t::keyboard(unsigned char key)
    {
        switch (key)
		{
		case 'q':
			body_pb->ApplyForce(b2Vec2(4000,0), body_pb->GetWorldCenter() ,true);
			body_pb1->ApplyForce(b2Vec2(4000,0), body_pb1->GetWorldCenter() ,true);
		break;
		case 't':
			for(int i =0 ;i<30;i++){
			body_t->ApplyForce(b2Vec2(100,0),b2Vec2(body_t->GetWorldCenter().x-1.0f,body_t->GetWorldCenter().y-4.0f),true);
			for( long i=0; i<pow(10,6); i++){}
			}
		break;
		case 'e':
			body_pb->ApplyForce(b2Vec2(-1000,0),body_pb->GetWorldCenter(),true);
		break;
		
		}
    }
  

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
  

}



