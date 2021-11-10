#include <ros/ros.h>
// Listener messages
#include "fast_turtle/RobotData.h"
#include "fast_turtle/RobotDataArray.h"
// Publisher messages
#include "geometry_msgs/Twist.h"

ros::Publisher twist_pub;

void controller(float x, float y, float* vx, float* vy)
{
    if(x > 0) *vx = 0.2;
    else *vx = -0.2;
    if(y > 0) *vy = 0.2; 
    else *vy = -0.2;
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
