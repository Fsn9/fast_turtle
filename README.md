# FastTurtle
**FastTurtle** is a 2D simulator library for Turtlebot3 testing.

This ROS package has two functionalities:
* **Live Simulation** (launches a live simulation that listens to `geometry_msgs/Twist` messages commands. More information about Twist message accessible at: http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
* **Library** (can be used as C++ library)

## Documentation
### Live Simulation
#### To launch
* `roslaunch fast_turtle fast_turtle_live.launch`
#### To interact
* **via Publisher**: Publish `geometry_msgs/Twist` commands into `/cmd_vel` ROS topic available.
* **via Teleoperation**: If you wish to **teleoperate** you can clone another repository into your ROS Workspace called `turtlebot3_teleop` accessible at: https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop. 
After clonning, you launch teleoperation with the following command: `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
