#include <ros/ros.h>
// Listener messages
#include "fast_turtle/RobotData.h"
#include "fast_turtle/RobotDataArray.h"
// Publisher messages
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    // Node initialization
    ros::init(argc, argv, "my_controller3");
    ros::NodeHandle nh;

    // Publisher and messages
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_sd0", 1000, true);
    fast_turtle::RobotDataArray msg;
    geometry_msgs::Twist twist_msg;
    
    // Commands
    std::vector<float> vels = {-1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int s = vels.size();

    // Robot positions
    float x = 0;
    float y = 0;

    // Sleep a little
    ros::Duration(0.1).sleep();

    // Publishing loop
    bool run = true;
    while(ros::ok())
    {
        if(run)
        {
            for(int i = 0; i < s; i++)
            {
                msg = *ros::topic::waitForMessage<fast_turtle::RobotDataArray>("/simple_drones");
                x = msg.robots[0].pose.position.x;
                y = msg.robots[0].pose.position.y;
                
                twist_msg.linear.x = vels[i];
                twist_msg.linear.y = 0.0;
                twist_pub.publish(twist_msg);
            }
            run = false;
        }
    }
    return 0;
}
