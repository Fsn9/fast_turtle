#ifndef FAST_TURTLE_H
#define FAST_TURTLE_H

#include "world.h"
#include <map>
#include <unistd.h> 
#include <chrono>

#define MAX_LIN_VELOCITY_TB_BURGER 0.22f
#define MAX_ANG_VELOCITY_TB_BURGER 2.84f
#define MAX_LIN_VELOCITY_SIMPLE_DRONE 1.f

typedef struct cmd_vel_tbb{
    double v;
    double w;
} cmd_vel_tbb;

typedef struct cmd_vel_sd{
    double vx;
    double vy;
} cmd_vel_sd;

class Observation;

class FastTurtle{
    private:
        World* w;
        std::vector<std::chrono::steady_clock::time_point> last_times_tb_robots; // keeps track of the controller last actuation times
        std::vector<std::chrono::steady_clock::time_point> last_times_simple_drones; // keeps track of the controller drones last actuation times
        unsigned int simulation_fps;
        double simulation_dt;
        std::vector<float> default_scan_;
        std::map<std::string, std::pair<double, double>> initial_positions_robots_;
    public:
        FastTurtle();
        FastTurtle(unsigned int simulation_fps);
        void init_world(double length, double xc, double yc, std::string type);
        void add_turtlebot_burger(double x, double y, double theta, double radius, std::string name, double controller_period = DEFAULT_CONTROLLER_PERIOD);
        void add_simple_drone(double x, double y, double height, double radius, std::string name, double controller_period = DEFAULT_CONTROLLER_PERIOD);
        void add_obstacle(double x, double y, double radius, std::string type_, bool dynamics) = delete;
        void add_obstacle(double x, double y, double radius, std::string type_);
        void add_wall(double x1, double y1, double x2, double y2, bool dynamics) = delete;
        void add_wall(double x1, double y1, double x2, double y2);
        void add_food_item(double x, double y, double radius, bool dynamics) = delete;
        void add_food_item(double x, double y, double radius);
        World* get_world();  
        std::vector<double> get_robot_position(std::string robot_name);
        std::vector<float> get_laser(std::string robot_name);
        std::vector<double> observe_robot_pose(int idx_robot);
        std::vector<float> observe_robot_lidar(int idx_robot);
        void act_turtlebot_burger(double v, double w, int idx_robot);
        void act_simple_drone(double vx, double vy, int idx_robot);
        Observation observe(int idx_robot);
        void sleep() = delete;
        void check_collisions();
        void reset_robots();
};

class Observation{
    private:
        std::vector<double> pose;
        std::vector<float> laser_data;
        int idx_robot;
    public:
        Observation();
        std::vector<double> get_pose();
        std::vector<float> get_laser_data();
        void print();
        void print_pose();
        void print_laser_data();
        Observation(std::vector<double> pose, std::vector<float> laser_data, int idx_robot);
};
#endif // FAST_TURTLE_H