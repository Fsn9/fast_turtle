#ifndef FAST_TURTLE_H
#define FAST_TURTLE_H

// ROS
// Simulator classes
#include "world.h"
// CPP API libraries
#include <unistd.h> 
#include <chrono>

#define MAX_LIN_VELOCITY 0.5f
#define MAX_ANG_VELOCITY 2.84f

// Command velocity
typedef struct cmd_vel_{
    double v;
    double w;
} cmd_vel_;

class Observation;

class FastTurtle{
    private:
        World* w;
        std::vector<std::chrono::steady_clock::time_point> last_times; // keeps track of the controller last actuation times
        unsigned int simulation_fps;
        double simulation_dt;
    public:
        FastTurtle();
        FastTurtle(unsigned int simulation_fps);
        void init_world(float length, float xc, float yc, std::string type);
        void add_turtlebot_burger(float x, float y, float theta, float radius, std::string name, float controller_period = DEFAULT_CONTROLLER_PERIOD);
        void add_obstacle(float x, float y, float radius, std::string type_, bool dynamics) = delete;
        void add_obstacle(float x, float y, float radius, std::string type_);
        void add_wall(float x1, float y1, float x2, float y2, bool dynamics) = delete;
        void add_wall(float x1, float y1, float x2, float y2);
        void add_food_item(float x, float y, float radius, bool dynamics) = delete;
        void add_food_item(float x, float y, float radius);
        World* get_world();  
        std::vector<float> observe_robot_pose(int idx_robot); // Get robot idx pose
        std::vector<float> observe_robot_lidar(int idx_robot); // Get robot idx lidar
        void act(float v, float w, int idx_robot); // Act with twist in the world
        Observation observe(int idx_robot);
        void sleep() = delete;
        void check_collisions();
};

class Observation{
    private:
        std::vector<float> pose;
        std::vector<float> laser_data;
        int idx_robot;
    public:
        Observation();
        std::vector<float> get_pose();
        std::vector<float> get_laser_data();
        void print();
        void print_pose();
        void print_laser_data();
        Observation(std::vector<float> pose, std::vector<float> laser_data, int idx_robot);
};
#endif // FAST_TURTLE_H