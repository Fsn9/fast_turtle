// ROS
// Simulator classes
#include "world.h"

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
        int act(float v, float w); // Act with twist in the world
};