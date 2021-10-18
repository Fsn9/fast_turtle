#include "fast_turtle.h" // The simulator library
#include <ros/ros.h> // The ROS library if you want to use it

int main()
{
  FastTurtle ft(1e-6); // The simulator object
  ft.init_world(4, 0, 0,"square"); // Initializing the world
  ft.add_turtlebot_burger(0, -1, -M_PI_2, 0.09, 20e-3, "michelangelo"); 
  ft.add_turtlebot_burger(0, -1, -M_PI_2, 0.09, 80e-3, "leonardo"); 
  ft.add_obstacle(0, -2, 0.1, "round"); 
  float v = 0.1; // linear velocity
  float w = 0.0; // angular velocity
  Observation observation; // observation object

  while(1){
    ft.act(v,w,0);
    observation = ft.observe(0); 
    observation.print_pose(); 
    observation = ft.observe(1);
    observation.print_pose(); 
    ft.sleep();
  }

  return 0;
}
