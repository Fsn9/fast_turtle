#include <ros/ros.h>
// Listener messages
#include "fast_turtle/RobotData.h"
#include "fast_turtle/RobotDataArray.h"
// Publisher messages
#include "geometry_msgs/Twist.h"

ros::Publisher twist_pub;

void controller(float x, float y, float* vx, float* vy)
{
  /*
from dynamic_systems import Integrator

# Dynamics: States=[x] Control=[u] where x_dot=u
initial_state = [1.0];
initial_control = [0.0]
robot = Integrator(initial_state, initial_control)

dt = 0.01
sim_time = 10
time = []
u = []
x = []
xdv = []
k=2

    time.append(t)
    xd = math.sin(t)
    dot_xd=math.cos(t)

    # Control
    state = [x, y];
    u_control = [-k*(state[0]-xd)+dot_xd]
    
    # Send actuation commands
    robot.set_control(u_control)
    robot.actuate(dt)
*/
}

void listen_robot(const fast_turtle::RobotDataArray& msg)
{
    float x = msg.robots[0].pose.position.x;
    float y = msg.robots[0].pose.position.y;
    float vx = 0;
    float vy = 0;
    ROS_INFO("Listening (%f, %f)\n", x, y);
    geometry_msgs::Twist twist_msg;
    controller(x, y, &vx, &vy);
    twist_msg.linear.x = vx;
    twist_msg.linear.y = vy;
    twist_pub.publish(twist_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_controller2");
    ros::NodeHandle nh;
    twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_sd0", 100);
    ros::Subscriber sub = nh.subscribe("simple_drones", 10, listen_robot);
    ros::spin();
    return 0;
}
