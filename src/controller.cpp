#include "controller.h"
#include "geometry_msgs/Twist.h"
#include "fast_turtle/RobotData.h"

Controller::Controller(int robot_idx)
{
    robot_idx_ = robot_idx;
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel" + std::to_string(robot_idx_), 1000);
    listen_robot_subscriber_ = nh_.subscribe("robots", 10, &Controller::listen_robot, this);
}

void Controller::publish_commands(double v, double w)
{
    if (v > MAX_LIN_VELOCITY) v = MAX_LIN_VELOCITY;
    if (w > MAX_ANG_VELOCITY) w = MAX_ANG_VELOCITY;
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = v;
    twist_msg.angular.z = w;
    cmd_vel_publisher_.publish(twist_msg);
}

void Controller::listen_robot(const fast_turtle::RobotDataArray& robots_msg)
{
    // Position (x,y)
    robot_data_.pos_x = robots_msg.robots[robot_idx_].pose.position.x;
    robot_data_.pos_y = robots_msg.robots[robot_idx_].pose.position.y;

    // Quaternion of orientation (x,y,z,w)
    robot_data_.quat_x = robots_msg.robots[robot_idx_].pose.orientation.x;
    robot_data_.quat_y = robots_msg.robots[robot_idx_].pose.orientation.y;
    robot_data_.quat_z = robots_msg.robots[robot_idx_].pose.orientation.z;
    robot_data_.quat_w = robots_msg.robots[robot_idx_].pose.orientation.w;

    // Laser scan distances (all the 360 ray distances)
    robot_laser_scan_ = robots_msg.robots[robot_idx_].scan.ranges;
}