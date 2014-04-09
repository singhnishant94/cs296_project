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
namespace cs296
{	
	// These variables are needed in the step function so they had to be defined in this namespace.
	b2Body* body_bul[3];          // the shell of the bullet
    b2Body* body_bulhead[3];      // the triangle head of the bullet 
    b2Body* body_t;               // The trigger
	b2Body* body_pb;              // the pullback
    b2Body* body_pb1;             // the pullback
	bool casethrow;               // boolean to decide when to throw the shell 
	bool jointdestroy[3];		  // boolean to decide which joint to destroy
	bool reload;                  // boolean to decide when to reload
	int i;						  // this variable has been used in deciding how many times the loop will be run in
								  // step function to load the gun.
	b2RevoluteJoint* joint_1[3];  // first joint between bullet head and the shell
    b2RevoluteJoint* joint_2[3];  // second joint between bullet head and the shell
    
    bool automatic;
    int noofsteps=0;
    b2Body* curbul;
	
	class MyContactListener : public b2ContactListener
     {
		public:
		
		b2Body* temp1;
		b2Body* temp2;

		void MycontactListener(b2Body* y,b2Body* z)
					{
	
					temp1=y;
					temp2=z;
					
					}
			   void BeginContact(b2Contact* contact) {
					  
					void *bA= contact->GetFixtureA()->GetBody()->GetUserData();
					void *bB= contact->GetFixtureB()->GetBody()->GetUserData();
					int a =*((int*)(&bA));
					int b =*((int*)(&bB));
					// check if first body is one of the three bullets and second the hitting pin.
						if ((118<=a && a <121) && b==108){
							int k = a - 118;    // deciding which bullet
							
							reload = true;
							i = 0;
							
							casethrow = true;
							jointdestroy[k] = true;
						}
						if(a==138){         // The pullback hits the hammer while going back during loading.
							if(casethrow){
								casethrow =false;
							}
						}
					

			   }
		  
			   void EndContact(b2Contact* contact) {
		
				}
     };
	
	
	
  //! This is the class that sets up the Box2D simulation world
  //! Notice the public inheritance - why do we inherit the base_sim_t class?
  class dominos_t : public base_sim_t
  {
  public:
	MyContactListener myContactListenerInstance;
    b2Body* bodyx;                       // The hammer
    b2Body* temp;                      
    b2Body* bodyy;                      

    b2Body* body_pb2;                    // The hitting pin
    b2Body* attachrod;				     // The rod which attaches trigger to the stopper
    

    b2RevoluteJointDef jointDef_bul1;    // First joint between bullet head and the shell
    b2RevoluteJointDef jointDef_bul2;    // Second joint between bullet head and the shell

    void keyboard(unsigned char key);
    dominos_t();
    
    
    
    static base_sim_t* create()
    {
      return new dominos_t;
    }
  };
}
  
#endif
