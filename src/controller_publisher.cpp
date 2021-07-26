#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
    // Init node
    ros::init(argc, argv, "controller_publisher");

    // Node object
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    // Rate
    //ros::Rate loop_rate(100);

    // Message object
    geometry_msgs::Twist cmd_vel_msg;

    while(ros::ok()){
        // Prepare message
        cmd_vel_msg.linear.x = 0.05*200;
        cmd_vel_msg.angular.z = 0.1*200;

        // Publish
        cmd_vel_pub.publish(cmd_vel_msg);

        // Spin (so callbacks work)
        ros::spinOnce();

        // Maintain rate
        //loop_rate.sleep();
    }
    
}