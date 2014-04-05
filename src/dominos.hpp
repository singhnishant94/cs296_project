/*
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

#ifndef _DOMINOS_HPP_
#define _DOMINOS_HPP_
//#include "cs296_base.hpp"

namespace cs296
{
	
	
	
	class MyContactListener : public b2ContactListener
     {
		public:
		int i;
		b2Body* temp;
		b2Body* temp1;
		b2Body* temp2;
		void MycontactListener(b2Body* x,b2Body* y,b2Body* z){i=0;temp = x;temp1=y;temp2=z;}
			   void BeginContact(b2Contact* contact) {
					  
							 //check if fixture A was a ball
					void *yo= contact->GetFixtureA()->GetBody()->GetUserData();
						void *lo= contact->GetFixtureB()->GetBody()->GetUserData();
					int a =*((int*)(&yo));
					int b=*((int*)(&lo));
						if (a==118){
							temp->ApplyLinearImpulse(b2Vec2(-1000,0),temp->GetWorldCenter(),true);
						for(int i = 0 ;i <20 ; i++){
							temp1->ApplyForce(b2Vec2(1000,0),temp1->GetWorldCenter(),true);
							temp2->ApplyForce(b2Vec2(1000,0),temp2->GetWorldCenter(),true);
						}
						std::cout<<"awesome\n";
					//for( long i=0; i<pow(10,8); i++){}
						}
					

			   }
		  
			   void EndContact(b2Contact* contact) {
		  /*
					void *yo= contact->GetFixtureA()->GetBody()->GetUserData();
					  void *lo= contact->GetFixtureB()->GetBody()->GetUserData();
					int a =*((int*)(&yo));
					int b=*((int*)(&lo));
					  if (a==109){
					  //for( long i=0; i<pow(10,8); i++){}
					  std::cout<<"awesome\n";
					 
					  }
					if(a==108 || b==108){
					std::cout<<"awesomemaxx\n";
					}*/
				}
     };
	
	
	
  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  class dominos_t : public base_sim_t
  {
  public:
	MyContactListener myContactListenerInstance;
    //b2Body* bodyx;
    //b2Body* temp;
    dominos_t();
    
    
    
    static base_sim_t* create()
    {
      return new dominos_t;
    }
    //void callme();
    //void step(settings_t*);
  };
}
  
#endif
