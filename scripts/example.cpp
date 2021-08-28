#include "fast_turtle.h" // The simulator library
#include <ros/ros.h> // The ROS library if you want to use it

int main()
{
  FastTurtle ft; // The simulator object
  ft.init_world(4, 0, 0,"square"); // Initializing the world
  // Adding a robot to the world (x=0,y=-1,orientation=-M_PI/2,radius=0.09m,cycle_time=1s,name="michelangelo")
  ft.add_turtlebot_burger(0, -1, -M_PI_2, 0.09, 1, "michelangelo"); 
  // Adding an obstacle (x=0, y=-2, radius=0.1m, type="round", dynamics = false)
  ft.add_obstacle(0, -2, 0.1, "round"); 
  // Defining some twist commands to act in the world
  float v = 0.1; // linear velocity
  float w = 0.0; // angular velocity
  // Define the observation object (where we collect the observations of the robot)
  Observation observation;

  // The main loop of the simulator
  // Only one robot acting and observing
  // The 0 argument below is because it is the only robot added until now 
  //and it is in the first element of the vector of robots in the world object
  while(1){
    // Robot 0 Acting with twist commands in the world 
    ft.act(v,w,0); 
    // Observation of the Robot 0 created above
    observation = ft.observe(0); 
    // Print the observation to the terminal for debugging
    observation.print(); 
  }

  return 0;
}
