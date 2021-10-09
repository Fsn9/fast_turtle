#include "controller.h"
#include "geometry_msgs/Twist.h"
#include "fast_turtle/RobotData.h"

Controller::Controller(int robot_idx, std::string topic_to_publish)
{
    robot_idx_ = robot_idx;
    cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>(topic_to_publish, 100);
}

robot_data Controller::get_robot_data()
{
    return robot_data_;
}

void Controller::listen(const fast_turtle::RobotDataArray& robots_msg)
{
    robot_data_.pos_x_ = robots_msg.robots[robot_idx_].pose.position.x;
    robot_data_.pos_y_ = robots_msg.robots[robot_idx_].pose.position.y;
    robot_data_.quat_x_ = robots_msg.robots[robot_idx_].pose.orientation.x;
    robot_data_.quat_y_ = robots_msg.robots[robot_idx_].pose.orientation.y;
    robot_data_.quat_z_ = robots_msg.robots[robot_idx_].pose.orientation.z;
    robot_data_.quat_w_ = robots_msg.robots[robot_idx_].pose.orientation.w;
    robot_data_.laser_scan_ = robots_msg.robots[robot_idx_].scan.ranges;
}

void Controller::set_subscriber(std::string topic_to_listen)
{
    listener_ = nh_.subscribe(topic_to_listen, 10, &Controller::listen, this);
}

SimpleDroneController::SimpleDroneController(int idx, std::string topic) : Controller(idx, topic)
{
    set_subscriber("simple_drones");
}

void SimpleDroneController::publish()
{
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = cmd_vx_;
    twist_msg.linear.y = cmd_vy_;
    cmd_vel_publisher_.publish(twist_msg);
}

void SimpleDroneController::update_commands(double vx, double vy)
{
    cmd_vx_ = vx;
    cmd_vy_ = vy;
}

TurtlebotBurgerController::TurtlebotBurgerController(int idx, std::string topic) : Controller(idx, topic)
{
    set_subscriber("turtlebot_burgers");
}

void TurtlebotBurgerController::publish()
{
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = cmd_v_;
    twist_msg.angular.z = cmd_w_;
    cmd_vel_publisher_.publish(twist_msg);
}

void TurtlebotBurgerController::update_commands(double v, double w)
{
    cmd_v_ = v;
    cmd_w_ = w;
}