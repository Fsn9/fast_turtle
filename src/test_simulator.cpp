// Simulator classes
#include "fast_turtle.h"

int main(int argc, char **argv)
{
    FastTurtle ft;
    ft.init_world(8,0,0,"square");
    ft.add_turtlebot_burger(0, -1, -M_PI_2, 0.1, 1,"michelangelo");
    ft.add_obstacle(0, -2, 0.10, "round", false);
    ft.add_obstacle(0, 2, 0.10, "round", false);
    ft.add_obstacle(-1, -1, 0.10, "round", false);

    int robot_idx = 0;
    std::vector<float> pose = ft.observe_robot_pose(robot_idx);
    std::vector<float> lasers = ft.observe_robot_lidar(robot_idx);
    float x, y, th;
    x = pose[0];
    y = pose[1];
    th = pose[2];
    std::cout << ft.get_world()->get_burger(robot_idx)->tostring();
    ft.get_world()->get_burger(robot_idx)->get_lidar()->display_lasers();
    float v = 0.1;
    float w = 0.0;
    ft.act(v,w);
    return 0;
}
