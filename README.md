# FastTurtle
**FastTurtle** is a ROS package that consists of a 2D simulator library for Turtlebot3 testing.

This ROS package has two functionalities:
* **Live Simulation** (launches a live simulation that listens to `geometry_msgs/Twist` (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html) messages commands. These commands control **Turtlebot3 Burger robots** (https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/).
* **Library** (can be used as C++ library for learning/training faster simulations)

## Documentation
### **Live Simulation** 
To date, the scene could contain:  

a) **A squared bounded world** with parameters (`float length, float xc, float yc, std::string type`), e.g:
```cpp
ft->init_world(4, 0, 0, "square");
```  
b) **Robots**: A maximum of four Turtlebot3 Burgers with parameters (`float x, float y, float orientation_radians, float radius, std::string name, float controller_period`), e.g:
 ```cpp
 ft->add_turtlebot_burger(0, -1, -M_PI_2, BURGER_RADIUS, "michelangelo", 0.1);
 ```
c) **Rounded obstacles**  with parameters (`float x, float y, float radius, std::string type`), e.g:
```cpp
ft->add_obstacle(0, -2, obstacle_radius, "round");
```  
You could take a look for the default implemented scene in the file `fast_turtle/src/live_simulation.cpp`  
#### Steps:
##### 1) Launch
* `roslaunch fast_turtle fast_turtle_live.launch`
##### 2) Interact
* **via Controller**: The live simulations listens to four topics (`\cmd_vel0`, `\cmd_vel1`,`\cmd_vel2` and `\cmd_vel3`). Each topic is waiting for a `geometry/Twist` message command. Each topic controls each robot added to the scene by order, respectively. 
There is a controller example already implemented (`fast_turtle/src/my_controller.cpp`) that listens and interacts with the robots in real-time. Edit `my_controller.cpp` as you like. This controller is already listening to the `\robots` topic where one has the real-time `pose` data and `laser_scan` data for each robot added in the the live scene. It is also capable of publishing `cmd_vel` twist commands.
* **via Teleoperation**: If you wish to **teleoperate** you can clone another package into your ROS Workspace called `turtlebot3_teleop` accessible at: https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop. By default, this package is publishing in the `cmd_vel` topic. Change the topic name in the file `turtlebot3_teleop/nodes/turtlebot3_teleop_key.py` file to one of the previously mentioned four topics.  
After clonning, you launch teleoperation with the following command: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

### **Library**
#### Steps:
##### 1) Inside the folder `fast_turtle/scripts`, create a `your_script_file_name.cpp` script file that uses the library. There is already an `example.cpp` script file in the `scripts` folder, as represented below (just to give you a big picture idea of how to use the library):
```cpp
// example.cpp
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
```

##### 2) Link your .cpp script file to the compiler
Inside the folder `fast_turtle/scripts` add the following lines to the end of the `CMakeLists.txt`
```cmake
add_executable(your_script_alias your_script_file_name.cpp)
target_link_libraries(your_script_alias ft ${catkin_LIBRARIES})
add_dependencies(your_script_alias fast_turtle_generate_messages_cpp)
```

##### 3) Build
Go to the `fast_turtle/scripts` folder via terminal and run the following commands
```sh
mkdir build
cd build
cmake ..
cmake --build .
```

##### 4) Run your script
After the last step, a folder `fast_turtle/scripts/exe` is created. Go the `exe` folder via terminal and run the following command:
`./your_script_alias`. Done!
