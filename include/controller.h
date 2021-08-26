#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include "fast_turtle.h"
#include "fast_turtle/RobotData.h"
#include "fast_turtle/RobotDataArray.h"

typedef struct robot_data{
    double pos_x; 
    double pos_y;
    double quat_x;
    double quat_y;
    double quat_z;
    double quat_w;
} robot_data;

class Controller{
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber listen_robot_subscriber_;
        ros::Publisher cmd_vel_publisher_;
        robot_data robot_data_{0,0,0,0,0,0};
        std::vector<float> robot_laser_scan_;
        int robot_idx_;
    public:
        Controller(int robot_idx);

        // Listens to /robots topic
        void listen_robot(const fast_turtle::RobotDataArray& robots_msg);

        // Function of deciding commands to be implemented
        virtual cmd_vel_ decide_commands() = 0;

        // Publishes commands into /cmd_vel##idx topic
        void publish_commands(double v, double w);
};
#endif // CONTROLLER_H