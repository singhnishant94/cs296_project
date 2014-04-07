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


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>
#include <iostream>

#define	RAND_LIMIT 32767




namespace cs296
{
	extern bool jointdestroy;
	extern b2RevoluteJoint* joint_1;
	extern b2RevoluteJoint* joint_2;
	
	
	/**  The is the header file for cs296_base.cpp
     * This is the documentation block for the header file.
     * 
     */ 

  //! What is the difference between a class and a struct in C++?
  class base_sim_t;
  struct settings_t;
  
  //! Why do we use a typedef
  typedef base_sim_t* sim_create_fcn(); 

  //! Simulation settings. Some can be controlled in the GUI.
  struct settings_t
  {
	  
	 /*! \par The settings_t struct
	  * This struct defines all the basic variables required for displaying the simulation.<br>
	  * Their default values are also assigned in the struct.<br>
	  * var: view_center, type: b2vec2, Desc: the origin of our simulation w.r.t origin of Box2D.<br>
	  * var: hz, type: float32, Desc: timestep of the simulation.<br>
	  * It contains 2 int32 variables for velocity and postiton interations and also their default values.<br> 
	  * It contains another 11 int32 variables describing what should be shown in the simulations.<br>
	  * for example value of 1 is given for draw_shapes and draw_joints i.e they are shown in the GUI.<br>
	  * whereas AABBs are not drawn in the GUI.<br>
	  * Similarly there are variables showing stats,profiles and COMs.<br> 
	  * Variables to enable warm starting, sub-steping are also included which can be seen in simulation parameters of Box2D.<br>
	  * The variables with value 1 are intially visible.<br>
	  * We call this struct in constructor of base_sim_t to initialise settings.<br>
 	  */ 
    //! Notice the initialization of the class members in the constructor
    //! How is this happening?
    settings_t() :
      view_center(0.0f, 20.0f),
      hz(60.0f),
      velocity_iterations(8),
      position_iterations(3),
      draw_shapes(1),
      draw_joints(1),
      draw_AABBs(0),
      draw_pairs(0),
      draw_contact_points(0),
      draw_contact_normals(0),
      draw_contact_forces(0),
      draw_friction_forces(0),
      draw_COMs(0),
      draw_stats(0),
      draw_profile(0),
      enable_warm_starting(1),
      enable_continuous(1),
      enable_sub_stepping(0),
      pause(0),
      single_step(0)
    {}
    
    b2Vec2 view_center;
    float32 hz;
    int32 velocity_iterations;
    int32 position_iterations;
    int32 draw_shapes;
    int32 draw_joints;
    int32 draw_AABBs;
    int32 draw_pairs;
    int32 draw_contact_points;
    int32 draw_contact_normals;
    int32 draw_contact_forces;
    int32 draw_friction_forces;
    int32 draw_COMs;
    int32 draw_stats;
    int32 draw_profile;
    int32 enable_warm_starting;
    int32 enable_continuous;
    int32 enable_sub_stepping;
    int32 pause;
    int32 single_step;
  };
  
  struct sim_t
  
  
  {
	  /*! \par The sim_t struct
	   * This struct stores a name for the simulation and an instance from base_sim_t.
	   * It also contains a pointer to base_sim_t.
	   */ 
    const char *name;
    sim_create_fcn *create_fcn; //!< A pointer to base_sim_t.

    sim_t(const char *_name, sim_create_fcn *_create_fcn): 
      name(_name), create_fcn(_create_fcn) {;} //!< Instance of the dominos_t class
  };
  
  extern sim_t *sim;
  
  
  const int32 k_max_contact_points = 2048;  
  struct contact_point_t
  
  {
	  /*! \par The contact_point_t struct
	   * It used for contacts between 2 bodies.<br>
	   * var: fixtureA,fixtureB , type: b2fixture* , Desc: They store the fixtures of the bodies colliding.<br>
	   * var: normal , type: b2vec2 , Desc: Stores coordinates of normal force/vector.<br>
	   * var: position , type: b2vec2 , Desc: Stores coordinates of position of collision.<br>
	   * var: state , type: b2pointstate , Desc: Store states of the bodies (like dynamic).<br> 
	   */ 
    b2Fixture* fixtureA;
    b2Fixture* fixtureB;
    b2Vec2 normal;
    b2Vec2 position;
    b2PointState state;
  };
  
  class base_sim_t : public b2ContactListener
   
  {
	  /*! \par The base_sim_t class
	   * It contains functions for the simulator which take input from keyboard and mouse.<br>
	   * It creates the world object, adds all the objects to the world created by it. <br>
	   * It links the debug_draw_t class defined in render.cpp. <br>
	   * It implements the step function. It helps to take various settings from the GUI and pass it onto Box2d settings.<br>
	   * It stores all the information about the contacts before the solving time, and it can display the contact points.<br> 
	   */
  public:

    //temp = body_pb2;
    //void step(settings_t*);
    
    base_sim_t();

    //! Virtual destructors - amazing objects. Why are these necessary?
    virtual ~base_sim_t();    //!< Destructor of world object
    
    void set_text_line(int32 line) { m_text_line = line; }  //!< Used for printing strings
    void draw_title(int x, int y, const char *string);
    
    virtual void step(settings_t* settings);   //!< Changes the simulation view based on settings

    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }

    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }

    
    // Let derived tests know that a joint was destroyed.
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); }
    
    // Callbacks for derived classes.
    /*
    virtual void begin_contact(b2Contact* contact) {B2_NOT_USED(contact);}     //!< Called when 2 bodies undergo collision
    virtual void end_contact(b2Contact* contact) {B2_NOT_USED(contact);}   */   //!< end point of collision
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }

  //!How are protected members different from private memebers of a class in C++ ?
  protected:
  
  /*! \par
   * It defines ground and AABB world.<br>
   * var: m_points , type: vector of contact_point_t , Desc: Stores the contact points.<br>
   * m_debug_draw is called for drawing shapes.<br>
   */  

    //! What are Friend classes?
    friend class contact_listener_t;
    
    b2Body* m_ground_body;
    b2AABB m_world_AABB;
    contact_point_t m_points[k_max_contact_points];
    int32 m_point_count;

    debug_draw_t m_debug_draw;
    int32 m_text_line;
    b2World* m_world;     //!< It defines the wrold for simulation

    int32 m_step_count;
    
    b2Profile m_max_profile;
    b2Profile m_total_profile;
  };
  
 
  
  
}

#endif
