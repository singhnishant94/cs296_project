	CS296 Project : Simulation of a handgun using Box2D.

	Team Members: 
	Nishant Kumar Singh
	Tp Varun
	Haren Saga

	This is a simulation of a handgun using Box2D. It has been done as a part of our Software Systems laboratory course.
	We have tried our best to make it appear like a gun but more focus has been paid on the mechanism instead of the 
	appearance.

	Instructions to run the simulation:

	Manual mode:
	Just run the executable without any arguments i.e. ./mybins/cs296_24_exe
	Pressing keyboard 'q' for sometime will load the gun.
	Pressing 't' will fire the gun.
	
	Auto Mode:
	Run the executable as ./mybins/cs296_24_exe auto

	No gui mode:
	Run the executable as ./mybins/cs296_24_exe auto 3000
  	where 3000 is the number of iterations you want the program to run for. It can be varied.

	The gun is semi-automatic i.e. it performs all steps necessary to prepare the weapon to fire again after firing once.
	So, the gun has to be loaded only once in the beginning and it keeps firing on pressing trigger as long as there 
	are bullets in the magazine. 
