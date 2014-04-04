
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

#include "math.h"

namespace cs296

{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   * 
   */ 
  
  dominos_t::dominos_t()
  {
	 // ground
	 
	 b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    } 
	// lower rod  
	{
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-16.5f, 23.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    // diagonal rod
    {
		b2PolygonShape shape;
      shape.SetAsBox(sqrt(2.5f), 0.20f);
	
      b2BodyDef bd;
      bd.position.Set(-9.5f, 23.0f-0.125f);
      bd.angle = 150.4f* b2_pi/180.0f;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
		
	}
    //upper rod
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-15.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
      
    }
    
    //bullet stand
    {
		b2PolygonShape shape2;
      shape2.SetAsBox(2.5f, 0.25f);
      b2BodyDef bd3;
      bd3.position.Set(-5.0f, 14.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.1f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
      
      b2PrismaticJointDef jointDef2;
	b2Vec2 worldAxis(0.0f, 1.0f);
	jointDef2.Initialize(body3	, b1, body3->GetWorldCenter(), worldAxis);
	m_world->CreateJoint(&jointDef2);   
		
    //barell bottom
    
      b2PolygonShape shape;
      shape.SetAsBox(3.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-5.0f, 10.0f);
      b2Body* ground1 = m_world->CreateBody(&bd);
      ground1->CreateFixture(&shape, 0.0f);
      b2Vec2 b1;
      b1.Set(-6.0f,14.0f);
      b2Vec2 b2;
      b2.Set(-6.0f,10.0f); 
      b2Vec2 b3;
      b3.Set(-4.0f,14.0f); 
      b2Vec2 b4;
      b4.Set(-4.0f,10.0f);  
      
/*      float k = 5;	
      float y1 = ground1->GetWorldCenter().y;
      float y2 = body3->GetWorldCenter().y;
      
      body3 -> ApplyForce(b2Vec2(0,k*(y2-y1-4)) , body3->GetWorldCenter(),1); */
         
      b2DistanceJointDef jointDef;
		jointDef.Initialize(body3, ground1, b1, b2);
		jointDef.collideConnected = true;
		jointDef.frequencyHz = 1.0f;
		jointDef.dampingRatio = 0.5f;
		jointDef.length = 4.0f;
		m_world->CreateJoint(&jointDef);
		
		b2DistanceJointDef jointDef1;
		jointDef1.Initialize(body3, ground1, b3, b4);
		jointDef1.collideConnected = true;
		jointDef1.frequencyHz = 1.0f;
		jointDef1.dampingRatio = 0.5f;
		jointDef1.length = 4.0f;
		m_world->CreateJoint(&jointDef1);
		
    }
    //barell left
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f, 6.0f);
	
      b2BodyDef bd;
      bd.position.Set(-8.0f, 16.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    //barell right
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f, 6.0f);
	
      b2BodyDef bd;
      bd.position.Set(-2.0f, 16.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    
    //holder
    
    {
		b2Body* holder1;
      b2PolygonShape poly5;
      b2Vec2 vertices5[4];
      vertices5[0].Set(0,1);
      vertices5[1].Set(10,1);
      vertices5[2].Set(10,3);
      vertices5[3].Set(0,3);
      
      poly5.Set(vertices5, 4);
      b2FixtureDef wedgefd5;
      wedgefd5.shape = &poly5;
      wedgefd5.density = 0.002f;
      b2BodyDef wedgebd5;
      wedgebd5.linearVelocity.Set(5,0);
      wedgebd5.type = b2_dynamicBody;
      wedgebd5.position.Set(-7.5f + 10.0f, 40.0f);
      holder1 = m_world->CreateBody(&wedgebd5);
      holder1->CreateFixture(&wedgefd5);
      
      	b2Body* holder2;
      b2PolygonShape poly3;
      b2Vec2 vertices3[3];
      vertices3[0].Set(0,0);
      vertices3[1].Set(0,1);
      vertices3[2].Set(2,1);      
      poly3.Set(vertices3, 3);
      b2FixtureDef wedgefd3;
      wedgefd3.shape = &poly3;
      wedgefd3.density = 0.002f;
      b2BodyDef wedgebd3;
      wedgebd3.type = b2_dynamicBody;
      wedgebd3.position.Set(-7.5f + 10.0f, 40.0f);
      holder2 = m_world->CreateBody(&wedgebd3);
      holder2->CreateFixture(&wedgefd3);
      
      b2WeldJointDef jd;
      b2Vec2 joint1;
      joint1.Set(-7. + 10.0f,40.0f);
      jd.Initialize(holder1, holder2, joint1);
	m_world->CreateJoint(&jd);
	
	b2PrismaticJointDef jointDef;
	b2Vec2 worldAxis(1.0f, 0.0f);
	jointDef.Initialize(holder1, b1, holder1->GetWorldCenter(), worldAxis);
	m_world->CreateJoint(&jointDef);
  }
  // pusher 
  {
	  b2PolygonShape shape2;
      shape2.SetAsBox(5.0f, 0.25f);
      b2BodyDef bd3;
     bd3.linearVelocity.Set(-5,0);
      bd3.position.Set(20.0f, 22.5f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.03f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
	  b2PrismaticJointDef jointDef1;
		b2Vec2 worldAxis(1.0f, 0.0f);
		jointDef1.Initialize(body3, b1, body3->GetWorldCenter(), worldAxis);
		m_world->CreateJoint(&jointDef1);
  }
  
    
    //bullet 
    {
	/*	b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(30,15);
      bd->fixedRotation = true;
*/	
		for (int i = 0; i<5 ; i++){
// rectangle	
	  b2PolygonShape shape2;
      shape2.SetAsBox(1.5f, 1.0f);
      b2BodyDef bd3;
//     bd3.linearVelocity.Set(5,0);
      bd3.position.Set(-4.0f, 20.0f + 6*i);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.05f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
 //triangle     
      b2Body* sbody5;
      b2PolygonShape poly5;
      b2Vec2 vertices5[3];
      vertices5[0].Set(0,0);
      vertices5[1].Set(2,1);
      vertices5[2].Set(2,-1);
      poly5.Set(vertices5, 3);
      b2FixtureDef wedgefd5;
      wedgefd5.shape = &poly5;
      wedgefd5.density = 0.05f;
      b2BodyDef wedgebd5;
      wedgebd5.type = b2_dynamicBody;
      wedgebd5.position.Set(-7.5f, 20.0f + 6*i);
      sbody5 = m_world->CreateBody(&wedgebd5);
      sbody5->CreateFixture(&wedgefd5);
      b2Vec2 a1;
      a1.Set(-5.5f,21.0f + 6*i);
      b2Vec2 a2;
      a2.Set(-5.5f,19.0f + 6*i);
      
      //joints
      b2RevoluteJointDef jointDef1;
//      jointDef1.bodyA = body3;
 //     jointDef1.bodyB = sbody5;
   //   jointDef1.localAnchorB.Set(1,1);
     // jointDef1.localAnchorA.Set(-2,1);
		jointDef1.Initialize(body3, sbody5, a1);
		b2RevoluteJointDef jointDef2;
		jointDef2.Initialize(body3, sbody5, a2);
		m_world->CreateJoint(&jointDef2);
		m_world->CreateJoint(&jointDef1);
	}
		
/*      b2DistanceJointDef jointDef;
		jointDef.Initialize(body3, sbody5, worldAnchorOnBodyA, worldAnchorOnBodyB);
		jointDef.collideConnected = true; */
		
/*		b2Body* bullet = m_world->CreateBody(bd);
      bullet->CreateFixture(&wedgefd5);
      bullet->CreateFixture(fd3);  */
	}
    /*{
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
  }*/
    
    
	  
	  
	  
	  
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
