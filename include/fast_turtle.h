// ROS
// Simulator classes
#include "world.h"

class Observation;

#ifndef FAST_TURTLE_H
#define FAST_TURTLE_H
class FastTurtle{
    private:
        World* w;
    public:
        FastTurtle();
        void init_world(float length, float xc, float yc, std::string type);
        void add_turtlebot_burger(float x, float y, float theta, float radius, float dt, std::string name);
        void add_obstacle(float x, float y, float radius, std::string type_, bool dynamics);
        World* get_world();  
        std::vector<float> observe_robot_pose(int idx_robot); // Get robot idx pose
        std::vector<float> observe_robot_lidar(int idx_robot); // Get robot idx lidar
        void act(float v, float w, int idx_robot); // Act with twist in the world
        Observation observe(int idx_robot);
};
#endif

#ifndef OBSERVATION_H
#define OBSERVATION_H
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
#endif