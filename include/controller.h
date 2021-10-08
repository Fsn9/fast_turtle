#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include "fast_turtle.h"
#include "fast_turtle/RobotData.h"
#include "fast_turtle/RobotDataArray.h"

typedef struct robot_data{
    double pos_x_; 
    double pos_y_;
    double quat_x_;
    double quat_y_;
    double quat_z_;
    double quat_w_;
    std::vector<float> laser_scan_;
} robot_data;

class Controller{
    public:
        Controller(int robot_idx, std::string topic_to_publish);
        virtual void decide() = 0;
        virtual void publish() = 0;
    protected:
        robot_data get_robot_data();
        ros::Publisher cmd_vel_publisher_;
        virtual void update_commands(double, double) = 0;
    private:
        robot_data robot_data_{0,0,0,0,0,0};
        void listen(const fast_turtle::RobotDataArray& robots_msg);
        int robot_idx_;
        ros::NodeHandle nh_;
        ros::Subscriber listener_;
};

class SimpleDroneController : public Controller{
    public:
        SimpleDroneController(int idx, std::string topic) : Controller(idx, topic){}
        void publish() override;
    protected:
        void update_commands(double vx, double vy) override;
    private:
        float cmd_vx_{0}, cmd_vy_{0};
};

class TurtlebotBurgerController : public Controller{
    
    public:
        TurtlebotBurgerController(int idx, std::string topic) : Controller(idx, topic){}
        void publish() override;
    protected:
        void update_commands(double v, double w) override;
    private:
        float cmd_v_{0}, cmd_w_{0};
};
#endif // CONTROLLER_H