# FastTurtle
**FastTurtle** is a ROS package that consists of a 2D simulator library for Turtlebot3 testing.

This ROS package has two functionalities:
* **Live Simulation** (launches a live simulation that listens to `geometry_msgs/Twist` messages commands. More information about Twist message accessible at: http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html)
* **Library** (can be used as C++ library)

## Documentation
### Live Simulation
#### Steps:
##### 1) Launch
* `roslaunch fast_turtle fast_turtle_live.launch`
##### 2) Interact
* **via Publisher**: Publish `geometry_msgs/Twist` commands into `/cmd_vel` ROS topic available.
* **via Teleoperation**: If you wish to **teleoperate** you can clone another package into your ROS Workspace called `turtlebot3_teleop` accessible at: https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop. 
After clonning, you launch teleoperation with the following command: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

### Using the library
#### Steps:
##### 1) Inside the folder `fast_turtle/scripts`, create a `your_script_file_name.cpp` script file that uses the library. There is already an `example.cpp` script file in the `scripts` folder, as represented below (just to give you a big picture idea of how it works):
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
 ft.add_obstacle(0, -2, 0.1, "round", false); 
 // Defining some twist commands to act in the world
 float v = 0.1; //linear velocity
 float w = 0.0; //angular velocity
 // Define the observation object (where we collect the observations of the robot)
 Observation observation;
 
 // The main loop of the simulator
 while(1){
  ft.act(v,w); // Acting with twist commands in the world
  observation = ft.observe(0); // Observation of the robot 0 created in line 37 (the only robot stored in the vector of robots in the first position) 
  observation.print(); // Print the observation to the terminal for debugging
 }
}

  return 0;

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
