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
	/*!
	 * \par Definitions
	 * body: body_bul[], Type: b2Body*, Desc: array of pointers to bodies of the bullets i.e shells<br>
	 * body: body_bulhead[], Type: b2Body*, Desc: array of pointers to heads of the bullets i.e triangular parts<br>
	 * body: body_pb, Type: b2Body*, Desc: Its the lower block of the pullback mechanism  <br>
	 * body: body_pb1, Type: b2Body*, Desc: Its the upper block of the pullback mechanism  <br>
	 * variable: casethrow, Type: bool, Desc: Its a boolean value to decide when to throw the shell out of the gun<br>
	 * variable: jointdestroy[], Type: boolean array, Desc: Its a boolean array to decide which joints to be broken <br>
	 * as joints between shell and rectangular part must be broken during the release of bullet<br>
	 * variable: reload, Type: bool, Desc: Its a bool to decide when to reload the gun<br>
	 * variable: i, Type: int, Desc: used in deciding how many times the loop will be run in step function to load the gun.<br>
	 * joint: joint_1[], Type: b2RevoluteJoint* , Desc: array of upper joints between bullet heads and shells<br>
	 * joint: joint_2[], Type: b2RevoluteJoint* , Desc: array of lower joints between bullet heads and shells<br>
	 * variable: automatic, Type: bool, Desc: Boolean to decide whether to run in auto mode<br>
	 * variable: noofsteps, Type: int, Desc: To count the no. of iterations<br>
	 * body: curbul, Type: b2Body*, Desc: Pointer to the current bullet<br>
 	 */
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
	
	/*! \par MyContactListener class
	 * Variable:temp1, Type: b2Body*,Desc: a variable defined in the class <br>
	 * Variable:temp2, Type: b2Body*,Desc: another variable defined in the class <br>
	 * Function: MycontactListener, Type: void, Desc: initializing function<br>
	 * Function: BeginContact, Type: void, Desc: Checks if one of the bullet is in contact with the hitting pin.<br>
	 * Then it finds the bullet and then pullback hits the hammer while going back during loading<br>
	 * Function: EndContact, Type: void, Desc: Ends the contact.<br>
	 * 
	 */
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
  
  /*!
   * \par class dominos_t<br>
   * Variable:bodyx, Type: b2Body*,Desc: Its the hammer <br>
   * Variable:body_pb2, Type: b2Body*,Desc: The hitting pin<br>
   * Variable:attachrod, Type: b2Body*,Desc: The rod attaching trigger and stopper<br>
   * Variable:body_t, Type: b2Body*,Desc: The trigger<br>
   * Variable:jointDef_bul1, Type: b2RevoluteJointDef,Desc: upper joint between bullet head and shell<br>
   * Variable:jointDef_bul2, Type: b2RevoluteJointDef,Desc: lower joint between bullet head and shell<br><br
   */
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
